#include "kpt/io/pcd_codec.hpp"
#include "kpt/cancellation.hpp"
#include "kpt/io/ascii_float_parser.hpp"

#include <spdlog/spdlog.h>

#include <algorithm>
#include <array>
#include <bit>
#include <cctype>
#include <charconv>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <limits>
#include <locale>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <unordered_set>
#include <utility>
#include <vector>

namespace kpt::io_detail {
namespace {

constexpr std::size_t kMaxHeaderBytes = 1024U * 1024U;
constexpr std::size_t kMaxHeaderLineBytes = 64U * 1024U;
constexpr std::size_t kMaxAsciiTokenBytes = 256U;
constexpr std::size_t kMaxFields = 4096U;
constexpr std::size_t kMaxPoints = 20000000U;
constexpr std::uint64_t kMaxBodyBytes = std::uint64_t{512} << 20U;
constexpr std::uint64_t kMaxCompressedWorkingSetBytes = std::uint64_t{768} << 20U;
constexpr std::size_t kMaximumEagerReserve = 1'000'000U;

enum class DataMode { Ascii, Binary, BinaryCompressed };

struct Field {
  std::string name;
  std::uint8_t size = 0;
  char type = '\0';
  std::size_t count = 1;
  std::size_t record_offset = 0;
  std::size_t soa_offset = 0;
};

struct Header {
  std::vector<Field> fields;
  std::size_t points = 0;
  std::size_t width = 0;
  std::size_t height = 1;
  std::array<float, 7> viewpoint{0.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F};
  std::size_t record_size = 0;
  DataMode mode = DataMode::Ascii;
  bool has_color = false;
  bool has_intensity = false;
  bool has_noise = false;
};

[[noreturn]] void fail(const std::filesystem::path &path,
                       const std::string &message) {
  const auto utf8 = path.generic_u8string();
  throw std::runtime_error("PCD parse error: " + message + ": " +
                           std::string(utf8.begin(), utf8.end()));
}

[[noreturn]] void writeFail(const std::filesystem::path &path,
                            const std::string &message) {
  const auto utf8 = path.generic_u8string();
  throw std::runtime_error("PCD write error: " + message + ": " +
                           std::string(utf8.begin(), utf8.end()));
}

std::vector<std::string_view> tokens(std::string_view line) {
  std::vector<std::string_view> result;
  std::size_t offset = 0;
  while (offset < line.size()) {
    while (offset < line.size() &&
           std::isspace(static_cast<unsigned char>(line[offset])) != 0)
      ++offset;
    if (offset == line.size())
      break;
    const auto begin = offset;
    while (offset < line.size() &&
           std::isspace(static_cast<unsigned char>(line[offset])) == 0)
      ++offset;
    result.push_back(line.substr(begin, offset - begin));
  }
  return result;
}

std::string pathText(const std::filesystem::path &path) {
  const auto utf8 = path.generic_u8string();
  return std::string(utf8.begin(), utf8.end());
}

bool readBoundedLine(std::istream &input, std::string &line,
                     std::size_t &total_bytes,
                     const std::filesystem::path &path) {
  line.clear();
  while (true) {
    const auto next = input.get();
    if (next == std::char_traits<char>::eof()) {
      if (input.bad())
        fail(path, "header read failed");
      return !line.empty();
    }
    if (total_bytes == kMaxHeaderBytes)
      fail(path, "header exceeds 1 MiB");
    ++total_bytes;
    if (next == '\n')
      return true;
    if (line.size() == kMaxHeaderLineBytes)
      fail(path, "header line exceeds 64 KiB");
    line.push_back(static_cast<char>(next));
  }
}

bool parseFloatingToken(std::string_view text, double &value) {
  const auto equalsIgnoreCase = [](std::string_view left,
                                   std::string_view right) {
    if (left.size() != right.size())
      return false;
    return std::equal(left.begin(), left.end(), right.begin(),
                      [](unsigned char lhs, unsigned char rhs) {
                        return std::tolower(lhs) == std::tolower(rhs);
                      });
  };
  if (equalsIgnoreCase(text, "nan") || equalsIgnoreCase(text, "+nan") ||
      equalsIgnoreCase(text, "-nan")) {
    value = std::numeric_limits<double>::quiet_NaN();
    if (!text.empty() && text.front() == '-')
      value = -value;
    return true;
  }
  if (equalsIgnoreCase(text, "inf") || equalsIgnoreCase(text, "+inf") ||
      equalsIgnoreCase(text, "infinity") ||
      equalsIgnoreCase(text, "+infinity")) {
    value = std::numeric_limits<double>::infinity();
    return true;
  }
  if (equalsIgnoreCase(text, "-inf") || equalsIgnoreCase(text, "-infinity")) {
    value = -std::numeric_limits<double>::infinity();
    return true;
  }
  const auto result = parseAsciiFloating(text, value);
  return result.ec == std::errc{} && result.ptr == text.data() + text.size();
}

double parseHeaderFloat(std::string_view text,
                        const std::filesystem::path &path,
                        std::string_view what) {
  double value = 0.0;
  if (!parseFloatingToken(text, value) || !std::isfinite(value))
    fail(path, "invalid " + std::string(what));
  return value;
}

std::size_t parseSize(std::string_view text, const std::filesystem::path &path,
                      std::string_view what) {
  std::uint64_t value = 0;
  const auto result =
      std::from_chars(text.data(), text.data() + text.size(), value);
  if (result.ec != std::errc{} || result.ptr != text.data() + text.size() ||
      value > std::numeric_limits<std::size_t>::max())
    fail(path, "invalid " + std::string(what));
  return static_cast<std::size_t>(value);
}

std::size_t checkedMultiply(std::size_t left, std::size_t right,
                            const std::filesystem::path &path,
                            std::string_view what) {
  if (right != 0 && left > std::numeric_limits<std::size_t>::max() / right)
    fail(path, std::string(what) + " overflow");
  return left * right;
}

std::size_t checkedAdd(std::size_t left, std::size_t right,
                       const std::filesystem::path &path,
                       std::string_view what) {
  if (left > std::numeric_limits<std::size_t>::max() - right)
    fail(path, std::string(what) + " overflow");
  return left + right;
}

bool validScalar(const Field &field) {
  if (field.type == 'F')
    return field.size == 4 || field.size == 8;
  if (field.type == 'I' || field.type == 'U')
    return field.size == 1 || field.size == 2 || field.size == 4 ||
           field.size == 8;
  return false;
}

bool isMapped(std::string_view name) {
  return name == "x" || name == "y" || name == "z" || name == "intensity" ||
         name == "reflectance" || name == "rgb" || name == "rgba" ||
         name == "r" || name == "g" || name == "b" || name == "red" ||
         name == "green" || name == "blue" || name == "noise" ||
         name == "is_noise" || name == "noise_class";
}

Header parseHeader(std::istream &input, const std::filesystem::path &path) {
  std::vector<std::string> names;
  std::vector<std::size_t> sizes;
  std::vector<char> types;
  std::vector<std::size_t> counts;
  std::size_t width = 0;
  std::size_t height = 1;
  std::size_t declared_points = 0;
  bool has_width = false;
  bool has_height = false;
  bool has_points = false;
  bool has_data = false;
  bool has_version = false;
  Header header;
  std::unordered_set<std::string> directives;

  std::string line;
  std::size_t header_bytes = 0;
  while (readBoundedLine(input, line, header_bytes, path)) {
    if (!line.empty() && line.back() == '\r')
      line.pop_back();
    auto row = tokens(line);
    if (row.empty() || row.front().starts_with('#'))
      continue;
    const auto &key = row.front();
    auto markDirective = [&](std::string canonical) {
      if (!directives.insert(std::move(canonical)).second)
      fail(path, "duplicate header directive " + std::string(key));
    };
    if (key == "FIELDS" || key == "FIELD") {
      markDirective("FIELDS");
      if (row.size() < 2)
        fail(path, "FIELDS is empty");
      names.clear();
      names.reserve(row.size() - 1U);
      for (std::size_t index = 1; index < row.size(); ++index)
        names.emplace_back(row[index]);
      if (names.size() > kMaxFields)
        fail(path, "too many fields");
    } else if (key == "SIZE") {
      markDirective("SIZE");
      sizes.clear();
      for (std::size_t index = 1; index < row.size(); ++index)
        sizes.push_back(parseSize(row[index], path, "SIZE"));
    } else if (key == "TYPE") {
      markDirective("TYPE");
      types.clear();
      for (std::size_t index = 1; index < row.size(); ++index) {
        if (row[index].size() != 1)
          fail(path, "invalid TYPE");
        types.push_back(row[index][0]);
      }
    } else if (key == "COUNT") {
      markDirective("COUNT");
      counts.clear();
      for (std::size_t index = 1; index < row.size(); ++index) {
        const auto value = parseSize(row[index], path, "COUNT");
        if (value == 0)
          fail(path, "COUNT must be positive");
        counts.push_back(value);
      }
    } else if (key == "WIDTH") {
      markDirective("WIDTH");
      if (row.size() != 2)
        fail(path, "invalid WIDTH");
      width = parseSize(row[1], path, "WIDTH");
      has_width = true;
    } else if (key == "HEIGHT") {
      markDirective("HEIGHT");
      if (row.size() != 2)
        fail(path, "invalid HEIGHT");
      height = parseSize(row[1], path, "HEIGHT");
      has_height = true;
    } else if (key == "POINTS") {
      markDirective("POINTS");
      if (row.size() != 2)
        fail(path, "invalid POINTS");
      declared_points = parseSize(row[1], path, "POINTS");
      has_points = true;
    } else if (key == "VIEWPOINT") {
      markDirective("VIEWPOINT");
      if (row.size() != 8)
        fail(path, "VIEWPOINT requires seven values");
      for (std::size_t index = 0; index < header.viewpoint.size(); ++index) {
        header.viewpoint[index] = static_cast<float>(
            parseHeaderFloat(row[index + 1], path, "VIEWPOINT"));
      }
    } else if (key == "VERSION") {
      markDirective("VERSION");
      if (row.size() != 2 || (row[1] != ".7" && row[1] != "0.7"))
        fail(path, "unsupported VERSION");
      has_version = true;
    } else if (key == "DATA") {
      markDirective("DATA");
      if (row.size() != 2)
        fail(path, "invalid DATA");
      if (row[1] == "ascii")
        header.mode = DataMode::Ascii;
      else if (row[1] == "binary")
        header.mode = DataMode::Binary;
      else if (row[1] == "binary_compressed")
        header.mode = DataMode::BinaryCompressed;
      else
        fail(path, "unsupported DATA encoding " + std::string(row[1]));
      has_data = true;
      break;
    }
  }

  if (!has_data)
    fail(path, "missing DATA");
  if (!has_version)
    fail(path, "missing VERSION");
  if (names.empty() || sizes.size() != names.size() ||
      types.size() != names.size())
    fail(path, "FIELDS/SIZE/TYPE lengths differ");
  if (counts.empty())
    counts.assign(names.size(), 1);
  if (counts.size() != names.size())
    fail(path, "FIELDS/COUNT lengths differ");

  if (has_width && has_height) {
    const auto product = checkedMultiply(width, height, path, "WIDTH*HEIGHT");
    if (has_points && product != declared_points)
      fail(path, "POINTS differs from WIDTH*HEIGHT");
    header.points = has_points ? declared_points : product;
  } else if (has_points) {
    header.points = declared_points;
  } else {
    fail(path, "missing POINTS or WIDTH/HEIGHT");
  }
  if (header.points > kMaxPoints)
    fail(path, "point count exceeds limit");
  header.width = has_width ? width : header.points;
  header.height = has_height ? height : 1;

  std::unordered_set<std::string> field_names;
  bool has_x = false;
  bool has_y = false;
  bool has_z = false;
  bool has_intensity = false;
  bool has_packed_rgb = false;
  bool has_red = false;
  bool has_green = false;
  bool has_blue = false;
  bool has_noise = false;
  std::size_t soa_offset = 0;
  for (std::size_t index = 0; index < names.size(); ++index) {
    Field field{names[index],       static_cast<std::uint8_t>(sizes[index]),
                types[index],       counts[index],
                header.record_size, soa_offset};
    if (sizes[index] > std::numeric_limits<std::uint8_t>::max() ||
        !validScalar(field))
      fail(path, "unsupported scalar type for field " + field.name);
    if (!field_names.insert(field.name).second)
      fail(path, "duplicate field " + field.name);
    if (isMapped(field.name) && field.count != 1)
      fail(path, "mapped field COUNT must be 1: " + field.name);
    if ((field.name == "rgb" || field.name == "rgba") &&
        (field.size != 4 || (field.type != 'F' && field.type != 'U')))
      fail(path, "packed RGB must be F32 or U32");
    if ((field.name == "noise" || field.name == "is_noise" ||
         field.name == "noise_class") &&
        (field.size != 1 || field.type != 'U'))
      fail(path, "noise must be U8");
    auto rejectAlias = [&](bool &seen, std::string_view canonical) {
      if (seen)
        fail(path, "duplicate mapped field " + std::string(canonical));
      seen = true;
    };
    if (field.name == "intensity" || field.name == "reflectance")
      rejectAlias(has_intensity, "intensity");
    else if (field.name == "rgb" || field.name == "rgba")
      rejectAlias(has_packed_rgb, "rgb");
    else if (field.name == "r" || field.name == "red")
      rejectAlias(has_red, "red");
    else if (field.name == "g" || field.name == "green")
      rejectAlias(has_green, "green");
    else if (field.name == "b" || field.name == "blue")
      rejectAlias(has_blue, "blue");
    else if (field.name == "noise" || field.name == "is_noise" ||
             field.name == "noise_class")
      rejectAlias(has_noise, "noise");
    const auto field_width =
        checkedMultiply(field.size, field.count, path, "field width");
    header.record_size =
        checkedAdd(header.record_size, field_width, path, "record size");
    const auto plane_size =
        checkedMultiply(field_width, header.points, path, "field plane size");
    soa_offset = checkedAdd(soa_offset, plane_size, path, "SoA size");
    has_x = has_x || field.name == "x";
    has_y = has_y || field.name == "y";
    has_z = has_z || field.name == "z";
    header.fields.push_back(std::move(field));
  }
  if (!has_x || !has_y || !has_z)
    fail(path, "x, y and z fields are required");
  header.has_color = has_packed_rgb || (has_red && has_green && has_blue);
  header.has_intensity = has_intensity;
  header.has_noise = has_noise;
  const auto body_size =
      checkedMultiply(header.record_size, header.points, path, "body size");
  if (static_cast<std::uint64_t>(body_size) > kMaxBodyBytes)
    fail(path, "body exceeds 512 MiB safety limit");
  return header;
}

std::uint64_t readUnsigned(const std::byte *bytes, std::size_t size) {
  std::uint64_t value = 0;
  for (std::size_t index = 0; index < size; ++index)
    value |=
        static_cast<std::uint64_t>(std::to_integer<unsigned char>(bytes[index]))
        << (index * 8U);
  return value;
}

std::int64_t signExtend(std::uint64_t value, std::size_t size) {
  const auto bits = size * 8U;
  if (bits == 64U)
    return std::bit_cast<std::int64_t>(value);
  const auto sign = std::uint64_t{1} << (bits - 1U);
  if ((value & sign) != 0)
    value |= (~std::uint64_t{0}) << bits;
  return std::bit_cast<std::int64_t>(value);
}

double numericValue(const Field &field, const std::byte *bytes) {
  const auto raw = readUnsigned(bytes, field.size);
  if (field.type == 'U')
    return static_cast<double>(raw);
  if (field.type == 'I')
    return static_cast<double>(signExtend(raw, field.size));
  if (field.size == 4) {
    const auto bits = static_cast<std::uint32_t>(raw);
    return static_cast<double>(std::bit_cast<float>(bits));
  }
  return std::bit_cast<double>(raw);
}

std::uint8_t colorValue(double value, const std::filesystem::path &path,
                        std::string_view field) {
  if (!std::isfinite(value) || value < 0.0 || value > 255.0 ||
      std::trunc(value) != value)
    fail(path, "invalid color value in " + std::string(field));
  return static_cast<std::uint8_t>(value);
}

struct DecodedPoint {
  PointT point;
  std::uint32_t packed_rgb = 0;
  bool has_packed = false;
  bool has_r = false;
  bool has_g = false;
  bool has_b = false;
};

float pointValue(double value, const std::filesystem::path &path,
                 std::string_view field) {
  constexpr auto maximum =
      static_cast<double>(std::numeric_limits<float>::max());
  if (!std::isfinite(value) || value < -maximum || value > maximum)
    fail(path, "invalid point value in " + std::string(field));
  return static_cast<float>(value);
}

void applyNumeric(DecodedPoint &decoded, const Field &field, double value,
                  std::uint32_t packed_bits,
                  const std::filesystem::path &path) {
  if (field.name == "x")
    decoded.point.x = pointValue(value, path, field.name);
  else if (field.name == "y")
    decoded.point.y = pointValue(value, path, field.name);
  else if (field.name == "z")
    decoded.point.z = pointValue(value, path, field.name);
  else if (field.name == "intensity" || field.name == "reflectance")
    decoded.point.intensity = pointValue(value, path, field.name);
  else if (field.name == "rgb" || field.name == "rgba") {
    decoded.packed_rgb =
        field.type == 'F' ? packed_bits : static_cast<std::uint32_t>(value);
    decoded.has_packed = true;
  } else if (field.name == "r" || field.name == "red") {
    decoded.point.r = colorValue(value, path, field.name);
    decoded.has_r = true;
  } else if (field.name == "g" || field.name == "green") {
    decoded.point.g = colorValue(value, path, field.name);
    decoded.has_g = true;
  } else if (field.name == "b" || field.name == "blue") {
    decoded.point.b = colorValue(value, path, field.name);
    decoded.has_b = true;
  } else if (field.name == "noise" || field.name == "is_noise" ||
             field.name == "noise_class") {
    decoded.point.noise = colorValue(value, path, field.name);
  }
}

PointT finishPoint(DecodedPoint decoded) {
  if (decoded.has_packed) {
    if (!decoded.has_r)
      decoded.point.r =
          static_cast<std::uint8_t>((decoded.packed_rgb >> 16U) & 0xffU);
    if (!decoded.has_g)
      decoded.point.g =
          static_cast<std::uint8_t>((decoded.packed_rgb >> 8U) & 0xffU);
    if (!decoded.has_b)
      decoded.point.b = static_cast<std::uint8_t>(decoded.packed_rgb & 0xffU);
  }
  return decoded.point;
}

double parseFloat(std::string_view text, const std::filesystem::path &path,
                  std::string_view field) {
  double value = 0.0;
  if (!parseFloatingToken(text, value))
    fail(path, "invalid ASCII value for field " + std::string(field));
  return value;
}

std::uint64_t parseUnsignedToken(std::string_view text,
                                 const std::filesystem::path &path,
                                 std::string_view field) {
  std::uint64_t value = 0;
  const auto result =
      std::from_chars(text.data(), text.data() + text.size(), value);
  if (result.ec != std::errc{} || result.ptr != text.data() + text.size())
    fail(path, "invalid unsigned value for field " + std::string(field));
  return value;
}

std::int64_t parseSignedToken(std::string_view text,
                              const std::filesystem::path &path,
                              std::string_view field) {
  std::int64_t value = 0;
  const auto result =
      std::from_chars(text.data(), text.data() + text.size(), value);
  if (result.ec != std::errc{} || result.ptr != text.data() + text.size())
    fail(path, "invalid signed value for field " + std::string(field));
  return value;
}

bool readAsciiToken(std::istream &input,
                    std::array<char, kMaxAsciiTokenBytes> &token,
                    std::size_t &token_size,
                    const std::filesystem::path &path, std::stop_token stop) {
  token_size = 0;
  std::size_t scanned_bytes = 0;
  while (true) {
    if ((scanned_bytes++ % 4096U) == 0U && stop.stop_requested())
      throw OperationCancelled();
    const auto next = input.peek();
    if (next == std::char_traits<char>::eof()) {
      if (input.bad())
        fail(path, "ASCII body read failed");
      return false;
    }
    if (std::isspace(static_cast<unsigned char>(next)) == 0)
      break;
    static_cast<void>(input.get());
  }
  while (true) {
    const auto next = input.peek();
    if (next == std::char_traits<char>::eof()) {
      if (input.bad())
        fail(path, "ASCII body read failed");
      return true;
    }
    if (std::isspace(static_cast<unsigned char>(next)) != 0)
      return true;
    if (token_size == token.size())
      fail(path, "ASCII token exceeds 256 bytes");
    token[token_size++] = static_cast<char>(input.get());
  }
}

void loadAsciiBody(std::istream &input, const Header &header,
                   const std::filesystem::path &path, PointCloudIRGB &cloud,
                   std::stop_token stop) {
  for (std::size_t point_index = 0; point_index < header.points;
       ++point_index) {
    if (stop.stop_requested())
      throw OperationCancelled();
    DecodedPoint decoded;
    std::array<char, kMaxAsciiTokenBytes> token_buffer{};
    std::size_t token_size = 0;
    for (const auto &field : header.fields) {
      for (std::size_t component = 0; component < field.count; ++component) {
        if ((component % 4096U) == 0U && stop.stop_requested())
          throw OperationCancelled();
        if (!readAsciiToken(input, token_buffer, token_size, path, stop))
          fail(path, "truncated ASCII body");
        const std::string_view token(token_buffer.data(), token_size);
        double value = 0.0;
        std::uint32_t packed_bits = 0;
        if (field.type == 'F') {
          value = parseFloat(token, path, field.name);
          if (field.size == 4)
            packed_bits =
                std::bit_cast<std::uint32_t>(static_cast<float>(value));
        } else if (field.type == 'U') {
          const auto raw = parseUnsignedToken(token, path, field.name);
          const auto max_value =
              field.size == 8 ? std::numeric_limits<std::uint64_t>::max()
                              : ((std::uint64_t{1} << (field.size * 8U)) - 1U);
          if (raw > max_value)
            fail(path, "unsigned value out of range for " + field.name);
          value = static_cast<double>(raw);
          packed_bits = static_cast<std::uint32_t>(raw);
        } else {
          const auto raw = parseSignedToken(token, path, field.name);
          const auto bits = field.size * 8U;
          const auto minimum = bits == 64U
                                   ? std::numeric_limits<std::int64_t>::min()
                                   : -(std::int64_t{1} << (bits - 1U));
          const auto maximum = bits == 64U
                                   ? std::numeric_limits<std::int64_t>::max()
                                   : ((std::int64_t{1} << (bits - 1U)) - 1);
          if (raw < minimum || raw > maximum)
            fail(path, "signed value out of range for " + field.name);
          value = static_cast<double>(raw);
          packed_bits = static_cast<std::uint32_t>(raw);
        }
        if (component == 0)
          applyNumeric(decoded, field, value, packed_bits, path);
      }
    }
    cloud.push_back(finishPoint(decoded));
  }
  std::array<char, kMaxAsciiTokenBytes> trailing{};
  std::size_t trailing_size = 0;
  if (readAsciiToken(input, trailing, trailing_size, path, stop))
    fail(path, "extra ASCII data after POINTS");
}

std::uint64_t remainingBytes(std::istream &input,
                             const std::filesystem::path &path) {
  const auto current = input.tellg();
  if (current < 0)
    fail(path, "cannot determine body offset");
  input.seekg(0, std::ios::end);
  const auto end = input.tellg();
  if (end < current)
    fail(path, "cannot determine file size");
  input.seekg(current);
  if (!input)
    fail(path, "cannot seek to body");
  return static_cast<std::uint64_t>(end - current);
}

void readExact(std::istream &input, std::byte *destination, std::size_t size,
               const std::filesystem::path &path, std::string_view what) {
  if (size == 0)
    return;
  input.read(reinterpret_cast<char *>(destination),
             static_cast<std::streamsize>(size));
  if (input.gcount() != static_cast<std::streamsize>(size))
    fail(path, "truncated " + std::string(what));
}

std::vector<std::byte> readVector(std::istream &input, std::size_t size,
                                  const std::filesystem::path &path,
                                  std::string_view what, std::stop_token stop) {
  if (static_cast<std::uint64_t>(size) > kMaxBodyBytes)
    fail(path, std::string(what) + " exceeds 512 MiB safety limit");
  if (stop.stop_requested())
    throw OperationCancelled();
  std::vector<std::byte> bytes(size);
  constexpr std::size_t chunk_size = 64U * 1024U;
  for (std::size_t offset = 0; offset < size;) {
    if (stop.stop_requested())
      throw OperationCancelled();
    const auto count = std::min(chunk_size, size - offset);
    readExact(input, bytes.data() + offset, count, path, what);
    offset += count;
  }
  return bytes;
}

void warnTrailing(const std::filesystem::path &path, std::uint64_t bytes) {
  if (bytes != 0)
    spdlog::warn("PCD {} has {} trailing bytes after declared POINTS; ignored",
                 pathText(path), bytes);
}

void decodeBinaryRange(const Header &header, std::span<const std::byte> bytes,
                       bool soa, std::size_t point_start,
                       std::size_t point_count,
                       const std::filesystem::path &path,
                       PointCloudIRGB &cloud, std::stop_token stop) {
  for (std::size_t local_index = 0; local_index < point_count;
       ++local_index) {
    if (stop.stop_requested())
      throw OperationCancelled();
    DecodedPoint decoded;
    const auto point_index = soa ? point_start + local_index : local_index;
    for (const auto &field : header.fields) {
      for (std::size_t component = 0; component < field.count; ++component) {
        if ((component % 4096U) == 0U && stop.stop_requested())
          throw OperationCancelled();
        const auto component_offset =
            checkedMultiply(component, field.size, path, "binary component");
        const auto field_offset =
            soa ? checkedAdd(
                      checkedAdd(field.soa_offset,
                                 checkedMultiply(point_index, field.size,
                                                 path, "binary point offset"),
                                 path, "binary field offset"),
                      checkedAdd(component_offset, 0, path,
                                 "binary component offset"),
                      path, "binary field offset")
                : checkedAdd(
                      checkedAdd(checkedMultiply(local_index, header.record_size,
                                                  path, "binary record offset"),
                                 field.record_offset, path,
                                 "binary field offset"),
                      component_offset, path, "binary component offset");
        if (field_offset > bytes.size() || field.size > bytes.size() - field_offset)
          fail(path, "truncated binary body");
        const auto *value_bytes = bytes.data() + field_offset;
        if (component == 0) {
          const auto value = numericValue(field, value_bytes);
          const auto packed = static_cast<std::uint32_t>(readUnsigned(
              value_bytes, std::min<std::size_t>(field.size, 4)));
          applyNumeric(decoded, field, value, packed, path);
        }
      }
    }
    cloud.push_back(finishPoint(decoded));
  }
}

void decodeBinaryStream(std::istream &input, const Header &header,
                        const std::filesystem::path &path,
                        PointCloudIRGB &cloud, std::stop_token stop) {
  if (stop.stop_requested())
    throw OperationCancelled();
  cloud.reserve(std::min(header.points, kMaximumEagerReserve));
  constexpr std::size_t chunk_bytes = 64U * 1024U;
  const auto records_per_chunk =
      std::max<std::size_t>(1, std::min(header.points,
                                        chunk_bytes / header.record_size));
  const auto bytes_per_chunk =
      checkedMultiply(records_per_chunk, header.record_size, path,
                      "binary chunk size");
  std::vector<std::byte> bytes(bytes_per_chunk);
  for (std::size_t point_start = 0; point_start < header.points;) {
    if (stop.stop_requested())
      throw OperationCancelled();
    const auto count = std::min(records_per_chunk, header.points - point_start);
    const auto byte_count =
        checkedMultiply(count, header.record_size, path, "binary chunk size");
    readExact(input, bytes.data(), byte_count, path, "binary body");
    decodeBinaryRange(header, {bytes.data(), byte_count}, false, point_start,
                      count, path, cloud, stop);
    point_start += count;
  }
}

void decodeBinary(const Header &header, const std::vector<std::byte> &bytes,
                  bool soa, const std::filesystem::path &path,
                  PointCloudIRGB &cloud, std::stop_token stop) {
  if (stop.stop_requested())
    throw OperationCancelled();
  cloud.reserve(std::min(header.points, kMaximumEagerReserve));
  decodeBinaryRange(header, bytes, soa, 0, header.points, path, cloud, stop);
}

std::uint32_t readU32(std::istream &input, const std::filesystem::path &path) {
  std::array<std::byte, 4> bytes{};
  input.read(reinterpret_cast<char *>(bytes.data()), bytes.size());
  if (!input)
    fail(path, "truncated compressed size prefix");
  return static_cast<std::uint32_t>(readUnsigned(bytes.data(), bytes.size()));
}

std::vector<std::byte> decompressLzf(const std::vector<std::byte> &input,
                                     std::size_t output_size,
                                     const std::filesystem::path &path,
                                     std::stop_token stop) {
  if (stop.stop_requested())
    throw OperationCancelled();
  std::vector<std::byte> output(output_size);
  std::size_t input_pos = 0;
  std::size_t output_pos = 0;
  std::size_t operation_count = 0;
  while (input_pos < input.size()) {
    if ((operation_count++ % 4096U) == 0U && stop.stop_requested())
      throw OperationCancelled();
    const auto control = std::to_integer<unsigned char>(input[input_pos++]);
    if (control < 32U) {
      const auto length = static_cast<std::size_t>(control) + 1U;
      if (length > input.size() - input_pos ||
          length > output.size() - output_pos)
        fail(path, "invalid LZF literal run");
      std::copy_n(input.begin() + static_cast<std::ptrdiff_t>(input_pos),
                  length,
                  output.begin() + static_cast<std::ptrdiff_t>(output_pos));
      input_pos += length;
      output_pos += length;
      continue;
    }
    std::size_t length = control >> 5U;
    std::size_t offset = (control & 0x1fU) << 8U;
    if (length == 7U) {
      if (input_pos == input.size())
        fail(path, "truncated LZF length");
      length += std::to_integer<unsigned char>(input[input_pos++]);
    }
    if (input_pos == input.size())
      fail(path, "truncated LZF back reference");
    offset += std::to_integer<unsigned char>(input[input_pos++]);
    length += 2U;
    if (offset + 1U > output_pos || length > output.size() - output_pos)
      fail(path, "invalid LZF back reference");
    auto reference = output_pos - offset - 1U;
    for (std::size_t index = 0; index < length; ++index)
      output[output_pos++] = output[reference++];
  }
  if (output_pos != output.size())
    fail(path, "LZF output size mismatch");
  return output;
}

void writeU32(std::ostream &output, std::uint32_t value) {
  std::array<char, 4> bytes{};
  for (std::size_t index = 0; index < bytes.size(); ++index)
    bytes[index] = static_cast<char>((value >> (index * 8U)) & 0xffU);
  output.write(bytes.data(), bytes.size());
}

void writeFloat(std::ostream &output, float value) {
  writeU32(output, std::bit_cast<std::uint32_t>(value));
}

} // namespace

void loadPcd(const std::filesystem::path &path, PointCloudIRGB &cloud,
             std::stop_token stop) {
  std::ifstream input(path, std::ios::binary);
  if (!input)
    fail(path, "cannot open file");
  bool has_color = false;
  bool has_intensity = false;
  loadPcd(input, path, cloud, has_color, has_intensity, stop);
}

void loadPcd(std::istream &input, const std::filesystem::path &path,
             PointCloudIRGB &cloud, bool &has_color, bool &has_intensity,
             std::stop_token stop) {
  const auto header = parseHeader(input, path);
  const bool parsed_has_color = header.has_color;
  const bool parsed_has_intensity = header.has_intensity;
  PointCloudIRGB parsed;
  if (header.mode == DataMode::Ascii) {
    loadAsciiBody(input, header, path, parsed, stop);
  } else if (header.mode == DataMode::Binary) {
    const auto byte_count =
        checkedMultiply(header.record_size, header.points, path, "body size");
    const auto remaining = remainingBytes(input, path);
    if (static_cast<std::uint64_t>(byte_count) > remaining)
      fail(path, "truncated binary body");
    decodeBinaryStream(input, header, path, parsed, stop);
    warnTrailing(path, remaining - static_cast<std::uint64_t>(byte_count));
  } else {
    const auto before_prefix = remainingBytes(input, path);
    if (before_prefix < 8)
      fail(path, "truncated compressed size prefix");
    const auto compressed_size = static_cast<std::size_t>(readU32(input, path));
    const auto uncompressed_size =
        static_cast<std::size_t>(readU32(input, path));
    const auto expected =
        checkedMultiply(header.record_size, header.points, path, "body size");
    if (uncompressed_size != expected)
      fail(path, "compressed uncompressed-size does not match schema");
    const auto payload_remaining = remainingBytes(input, path);
    if (static_cast<std::uint64_t>(compressed_size) > payload_remaining)
      fail(path, "truncated compressed body");
    const auto maximum_lzf_size =
        static_cast<std::uint64_t>(expected) +
        (static_cast<std::uint64_t>(expected) + 31U) / 32U + 16U;
    if (static_cast<std::uint64_t>(compressed_size) > maximum_lzf_size)
      fail(path, "compressed size exceeds LZF bound");
    const auto cloud_bytes = checkedMultiply(
        header.points, sizeof(PointT), path, "decoded cloud size");
    const auto working_set = static_cast<std::uint64_t>(compressed_size) +
                             static_cast<std::uint64_t>(uncompressed_size) +
                             static_cast<std::uint64_t>(cloud_bytes);
    if (working_set > kMaxCompressedWorkingSetBytes)
      fail(path, "compressed decode working set exceeds 768 MiB");
    const auto compressed =
        readVector(input, compressed_size, path, "compressed body", stop);
    const auto bytes = decompressLzf(compressed, uncompressed_size, path, stop);
    parsed.reserve(header.points);
    decodeBinary(header, bytes, true, path, parsed, stop);
    warnTrailing(path, payload_remaining -
                           static_cast<std::uint64_t>(compressed_size));
  }
  parsed.width = header.width;
  parsed.height = header.height;
  parsed.viewpoint = header.viewpoint;
  parsed.has_noise = header.has_noise;
  has_color = parsed_has_color;
  has_intensity = parsed_has_intensity;
  cloud = std::move(parsed);
}

void savePcd(const std::filesystem::path &path, const PointCloudIRGB &cloud) {
  std::ofstream output(path, std::ios::binary | std::ios::trunc);
  if (!output)
    writeFail(path, "cannot open file");
  savePcd(output, path, cloud);
  output.close();
  if (!output)
    writeFail(path, "close failed");
}

void savePcd(std::ostream &output, const std::filesystem::path &path,
             const PointCloudIRGB &cloud, std::stop_token stop) {
  if (cloud.size() > kMaxPoints)
    writeFail(path, "point count exceeds limit");
  const std::size_t record_size =
      5U * sizeof(float) + (cloud.has_noise ? sizeof(std::uint8_t) : 0U);
  if (cloud.size() > kMaxBodyBytes / record_size)
    writeFail(path, "body exceeds 512 MiB safety limit");
  auto width = cloud.width;
  auto height = cloud.height;
  const auto shape_valid =
      height != 0 &&
      width <= std::numeric_limits<std::size_t>::max() / height &&
      width * height == cloud.size();
  if (!shape_valid) {
    width = cloud.size();
    height = 1;
  }
  output.imbue(std::locale::classic());
  output << std::setprecision(std::numeric_limits<float>::max_digits10);
  output << "# .PCD v0.7 - Point Cloud Data file format\n"
            "VERSION 0.7\n"
         << (cloud.has_noise ? "FIELDS x y z rgb intensity noise\n"
                             : "FIELDS x y z rgb intensity\n")
         << (cloud.has_noise ? "SIZE 4 4 4 4 4 1\n"
                             : "SIZE 4 4 4 4 4\n")
         << (cloud.has_noise ? "TYPE F F F F F U\n"
                             : "TYPE F F F F F\n")
         << (cloud.has_noise ? "COUNT 1 1 1 1 1 1\n"
                             : "COUNT 1 1 1 1 1\n")
         << "WIDTH " << width << "\n"
         << "HEIGHT " << height << "\n"
         << "VIEWPOINT " << cloud.viewpoint[0] << ' ' << cloud.viewpoint[1]
         << ' ' << cloud.viewpoint[2] << ' ' << cloud.viewpoint[3] << ' '
         << cloud.viewpoint[4] << ' ' << cloud.viewpoint[5] << ' '
         << cloud.viewpoint[6] << "\n"
         << "POINTS " << cloud.size()
         << "\n"
            "DATA binary\n";
  std::size_t point_index = 0;
  for (const auto &point : cloud.points) {
    if ((point_index++ % 4096U) == 0U && stop.stop_requested())
      throw OperationCancelled();
    writeFloat(output, point.x);
    writeFloat(output, point.y);
    writeFloat(output, point.z);
    const auto rgb = (static_cast<std::uint32_t>(point.r) << 16U) |
                     (static_cast<std::uint32_t>(point.g) << 8U) |
                     static_cast<std::uint32_t>(point.b);
    writeU32(output, rgb);
    writeFloat(output, point.intensity);
    if (cloud.has_noise)
      output.put(static_cast<char>(point.noise));
  }
  if (!output)
    writeFail(path, "write failed");
}

} // namespace kpt::io_detail
