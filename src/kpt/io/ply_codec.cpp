#include "kpt/io/ply_codec.hpp"
#include "kpt/cancellation.hpp"

#include <array>
#include <bit>
#include <charconv>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <limits>
#include <locale>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <type_traits>
#include <utility>
#include <vector>

namespace kpt::io_detail {
namespace {

constexpr std::size_t maximum_header_bytes = 1U << 20U;
constexpr std::size_t maximum_header_line_bytes = 1U << 16U;
constexpr std::size_t maximum_ascii_token_bytes = 256;
constexpr std::size_t maximum_elements = 1024;
constexpr std::size_t maximum_properties_per_element = 1024;
constexpr std::size_t maximum_total_records = 20'000'000;
constexpr std::size_t maximum_vertex_records = 20'000'000;
constexpr std::size_t maximum_list_items = 20'000'000;
constexpr std::size_t maximum_decoded_scalars = 100'000'000;

enum class Encoding { Ascii, BinaryLittleEndian, BinaryBigEndian };
enum class ScalarType {
  Int8,
  UInt8,
  Int16,
  UInt16,
  Int32,
  UInt32,
  Float32,
  Float64
};

struct Property {
  std::string name;
  ScalarType value_type = ScalarType::Float32;
  bool is_list = false;
  ScalarType count_type = ScalarType::UInt8;
};

struct Element {
  std::string name;
  std::size_t count = 0;
  std::vector<Property> properties;
};

struct Header {
  Encoding encoding = Encoding::Ascii;
  std::vector<Element> elements;
};

[[noreturn]] void fail(const std::filesystem::path &path,
                       std::string_view detail) {
  const auto native = path.generic_u8string();
  throw std::runtime_error("parse error: PLY " + std::string(detail) + ": " +
                           std::string(native.begin(), native.end()));
}

void stripCarriageReturn(std::string &line) {
  if (!line.empty() && line.back() == '\r')
    line.pop_back();
}

bool readHeaderLine(std::istream &input, std::string &line,
                    std::size_t &header_bytes,
                    const std::filesystem::path &path) {
  line.clear();
  char character = '\0';
  while (input.get(character)) {
    if (header_bytes == maximum_header_bytes)
      fail(path, "header exceeds byte limit");
    ++header_bytes;
    if (character == '\n')
      return true;
    if (line.size() == maximum_header_line_bytes)
      fail(path, "header line exceeds length limit");
    line.push_back(character);
  }
  return !line.empty();
}

std::vector<std::string_view> words(const std::string &line) {
  std::vector<std::string_view> result;
  const std::string_view view(line);
  std::size_t cursor = 0;
  while (cursor < view.size()) {
    while (cursor < view.size() &&
           (view[cursor] == ' ' || view[cursor] == '\t'))
      ++cursor;
    const auto begin = cursor;
    while (cursor < view.size() && view[cursor] != ' ' && view[cursor] != '\t')
      ++cursor;
    if (begin != cursor)
      result.emplace_back(view.substr(begin, cursor - begin));
  }
  return result;
}

ScalarType parseType(std::string_view value,
                     const std::filesystem::path &path) {
  if (value == "char" || value == "int8")
    return ScalarType::Int8;
  if (value == "uchar" || value == "uint8")
    return ScalarType::UInt8;
  if (value == "short" || value == "int16")
    return ScalarType::Int16;
  if (value == "ushort" || value == "uint16")
    return ScalarType::UInt16;
  if (value == "int" || value == "int32")
    return ScalarType::Int32;
  if (value == "uint" || value == "uint32")
    return ScalarType::UInt32;
  if (value == "float" || value == "float32")
    return ScalarType::Float32;
  if (value == "double" || value == "float64")
    return ScalarType::Float64;
  fail(path, "unsupported scalar type '" + std::string(value) + "'");
}

bool isIntegral(ScalarType type) {
  return type != ScalarType::Float32 && type != ScalarType::Float64;
}

std::size_t scalarSize(ScalarType type) {
  switch (type) {
  case ScalarType::Int8:
  case ScalarType::UInt8:
    return 1;
  case ScalarType::Int16:
  case ScalarType::UInt16:
    return 2;
  case ScalarType::Int32:
  case ScalarType::UInt32:
  case ScalarType::Float32:
    return 4;
  case ScalarType::Float64:
    return 8;
  }
  return 0;
}

std::size_t parseCount(std::string_view token,
                       const std::filesystem::path &path) {
  std::uint64_t parsed = 0;
  const auto result =
      std::from_chars(token.data(), token.data() + token.size(), parsed);
  if (result.ec != std::errc{} || result.ptr != token.data() + token.size() ||
      parsed > std::numeric_limits<std::size_t>::max())
    fail(path, "invalid element count");
  return static_cast<std::size_t>(parsed);
}

Header readHeader(std::istream &input, const std::filesystem::path &path) {
  std::string line;
  std::size_t header_bytes = 0;
  if (!readHeaderLine(input, line, header_bytes, path))
    fail(path, "missing header");
  stripCarriageReturn(line);
  if (line != "ply")
    fail(path, "missing magic");

  Header header;
  bool saw_format = false;
  bool saw_end = false;
  std::size_t total_records = 0;
  Element *current = nullptr;
  while (readHeaderLine(input, line, header_bytes, path)) {
    stripCarriageReturn(line);
    const auto tokens = words(line);
    if (tokens.empty())
      continue;
    if (tokens[0] == "comment" || tokens[0] == "obj_info")
      continue;
    if (tokens[0] == "format") {
      if (saw_format || tokens.size() != 3 || tokens[2] != "1.0")
        fail(path, "invalid format declaration");
      if (tokens[1] == "ascii")
        header.encoding = Encoding::Ascii;
      else if (tokens[1] == "binary_little_endian")
        header.encoding = Encoding::BinaryLittleEndian;
      else if (tokens[1] == "binary_big_endian")
        header.encoding = Encoding::BinaryBigEndian;
      else
        fail(path, "unsupported format");
      saw_format = true;
      continue;
    }
    if (tokens[0] == "element") {
      if (!saw_format || tokens.size() != 3)
        fail(path, "invalid element declaration");
      if (header.elements.size() == maximum_elements)
        fail(path, "element count exceeds limit");
      const auto count = parseCount(tokens[2], path);
      if (tokens[1] == "vertex" && count > maximum_vertex_records)
        fail(path, "vertex count exceeds limit");
      if (count > maximum_total_records - total_records)
        fail(path, "total record count exceeds limit");
      total_records += count;
      header.elements.push_back({std::string(tokens[1]), count, {}});
      current = &header.elements.back();
      continue;
    }
    if (tokens[0] == "property") {
      if (current == nullptr)
        fail(path, "property without element");
      if (current->properties.size() == maximum_properties_per_element)
        fail(path, "property count exceeds limit");
      if (tokens.size() == 3) {
        current->properties.push_back({std::string(tokens[2]),
                                       parseType(tokens[1], path), false,
                                       ScalarType::UInt8});
      } else if (tokens.size() == 5 && tokens[1] == "list") {
        const auto count_type = parseType(tokens[2], path);
        if (!isIntegral(count_type))
          fail(path, "list count type must be integral");
        current->properties.push_back({std::string(tokens[4]),
                                       parseType(tokens[3], path), true,
                                       count_type});
      } else {
        fail(path, "invalid property declaration");
      }
      continue;
    }
    if (tokens[0] == "end_header") {
      if (tokens.size() != 1 || !saw_format)
        fail(path, "invalid end_header");
      saw_end = true;
      break;
    }
    fail(path, "unknown header directive '" + std::string(tokens[0]) + "'");
  }
  if (!saw_end)
    fail(path, "unterminated header");

  const Element *vertex = nullptr;
  for (const auto &element : header.elements) {
    if (element.name == "vertex") {
      if (vertex != nullptr)
        fail(path, "duplicate vertex element");
      vertex = &element;
    }
  }
  if (vertex == nullptr)
    fail(path, "missing vertex element");
  bool has_x = false;
  bool has_y = false;
  bool has_z = false;
  bool has_intensity = false;
  bool has_red = false;
  bool has_green = false;
  bool has_blue = false;
  const auto mark_mapped = [&](bool &seen, std::string_view canonical) {
    if (seen)
      fail(path, "duplicate mapped vertex property " + std::string(canonical));
    seen = true;
  };
  for (const auto &property : vertex->properties) {
    if (property.is_list)
      continue;
    if (property.name == "x")
      mark_mapped(has_x, "x");
    else if (property.name == "y")
      mark_mapped(has_y, "y");
    else if (property.name == "z")
      mark_mapped(has_z, "z");
    else if (property.name == "intensity")
      mark_mapped(has_intensity, "intensity");
    else if (property.name == "red" || property.name == "r")
      mark_mapped(has_red, "red");
    else if (property.name == "green" || property.name == "g")
      mark_mapped(has_green, "green");
    else if (property.name == "blue" || property.name == "b")
      mark_mapped(has_blue, "blue");
  }
  for (const auto required : {"x", "y", "z"}) {
    const bool found = required == std::string_view("x")   ? has_x
                       : required == std::string_view("y") ? has_y
                                                           : has_z;
    if (!found)
      fail(path, "vertex element missing " + std::string(required));
  }

  std::size_t fixed_scalar_reads = 0;
  for (const auto &element : header.elements) {
    if (!element.properties.empty() &&
        element.count > (maximum_decoded_scalars - fixed_scalar_reads) /
                            element.properties.size())
      fail(path, "minimum decoded scalar count exceeds limit");
    fixed_scalar_reads += element.count * element.properties.size();
  }
  return header;
}

template <typename T> T byteswapValue(T value) {
  static_assert(std::is_trivially_copyable_v<T>);
  auto bytes = std::bit_cast<std::array<std::byte, sizeof(T)>>(value);
  for (std::size_t left = 0, right = bytes.size() - 1; left < right;
       ++left, --right)
    std::swap(bytes[left], bytes[right]);
  return std::bit_cast<T>(bytes);
}

template <typename T>
T readBinaryValue(std::istream &input, bool swap,
                  const std::filesystem::path &path) {
  T value{};
  input.read(reinterpret_cast<char *>(&value), sizeof(value));
  if (!input)
    fail(path, "truncated binary payload");
  return swap && sizeof(T) > 1 ? byteswapValue(value) : value;
}

long double readBinaryScalar(std::istream &input, ScalarType type, bool swap,
                             const std::filesystem::path &path) {
  switch (type) {
  case ScalarType::Int8:
    return readBinaryValue<std::int8_t>(input, false, path);
  case ScalarType::UInt8:
    return readBinaryValue<std::uint8_t>(input, false, path);
  case ScalarType::Int16:
    return readBinaryValue<std::int16_t>(input, swap, path);
  case ScalarType::UInt16:
    return readBinaryValue<std::uint16_t>(input, swap, path);
  case ScalarType::Int32:
    return readBinaryValue<std::int32_t>(input, swap, path);
  case ScalarType::UInt32:
    return readBinaryValue<std::uint32_t>(input, swap, path);
  case ScalarType::Float32:
    return static_cast<long double>(
        readBinaryValue<float>(input, swap, path));
  case ScalarType::Float64:
    return static_cast<long double>(
        readBinaryValue<double>(input, swap, path));
  }
  fail(path, "invalid scalar type");
}

long double parseAsciiScalar(std::string_view token, ScalarType type,
                             const std::filesystem::path &path) {
  if (type == ScalarType::Float32 || type == ScalarType::Float64) {
    double value = 0;
    std::istringstream input{std::string(token)};
    input.imbue(std::locale::classic());
    input >> std::noskipws >> value;
    if (input.fail() ||
        input.rdbuf()->sgetc() != std::char_traits<char>::eof())
      fail(path, "invalid ASCII floating-point value");
    if (type == ScalarType::Float32) {
      if (std::isfinite(value) &&
          std::abs(value) >
              static_cast<double>(std::numeric_limits<float>::max()))
        fail(path, "ASCII float32 value out of range");
      return static_cast<long double>(static_cast<float>(value));
    }
    return static_cast<long double>(value);
  }
  if (type == ScalarType::UInt8 || type == ScalarType::UInt16 ||
      type == ScalarType::UInt32) {
    std::uint64_t value = 0;
    const auto result =
        std::from_chars(token.data(), token.data() + token.size(), value);
    if (result.ec != std::errc{} || result.ptr != token.data() + token.size())
      fail(path, "invalid ASCII unsigned integer");
    const auto maximum = type == ScalarType::UInt8
                             ? std::numeric_limits<std::uint8_t>::max()
                         : type == ScalarType::UInt16
                             ? std::numeric_limits<std::uint16_t>::max()
                             : std::numeric_limits<std::uint32_t>::max();
    if (value > maximum)
      fail(path, "ASCII unsigned integer out of range");
    return value;
  }
  std::int64_t value = 0;
  const auto result =
      std::from_chars(token.data(), token.data() + token.size(), value);
  if (result.ec != std::errc{} || result.ptr != token.data() + token.size())
    fail(path, "invalid ASCII signed integer");
  const auto minimum =
      type == ScalarType::Int8    ? std::numeric_limits<std::int8_t>::min()
      : type == ScalarType::Int16 ? std::numeric_limits<std::int16_t>::min()
                                  : std::numeric_limits<std::int32_t>::min();
  const auto maximum =
      type == ScalarType::Int8    ? std::numeric_limits<std::int8_t>::max()
      : type == ScalarType::Int16 ? std::numeric_limits<std::int16_t>::max()
                                  : std::numeric_limits<std::int32_t>::max();
  if (value < minimum || value > maximum)
    fail(path, "ASCII signed integer out of range");
  return value;
}

std::string readAsciiToken(std::istream &input,
                           const std::filesystem::path &path,
                           std::stop_token stop) {
  std::string token;
  char character = '\0';
  std::size_t scanned_bytes = 0;
  while (input.get(character)) {
    if ((scanned_bytes++ % 4096U) == 0U && stop.stop_requested())
      throw OperationCancelled();
    if (character != ' ' && character != '\t' && character != '\r' &&
        character != '\n') {
      token.push_back(character);
      break;
    }
  }
  if (token.empty())
    fail(path, "truncated ASCII payload");
  while (input.get(character)) {
    if (character == ' ' || character == '\t' || character == '\r' ||
        character == '\n')
      return token;
    if (token.size() == maximum_ascii_token_bytes)
      fail(path, "ASCII token exceeds length limit");
    token.push_back(character);
  }
  return token;
}

long double readScalar(std::istream &input, ScalarType type, Encoding encoding,
                       const std::filesystem::path &path,
                       std::stop_token stop) {
  if (encoding == Encoding::Ascii)
    return parseAsciiScalar(readAsciiToken(input, path, stop), type, path);
  const bool file_little = encoding == Encoding::BinaryLittleEndian;
  const bool host_little = std::endian::native == std::endian::little;
  return readBinaryScalar(input, type, file_little != host_little, path);
}

std::size_t listCount(long double value, ScalarType type,
                      const std::filesystem::path &path) {
  if (!isIntegral(type) || !std::isfinite(value) || value < 0 ||
      value > static_cast<long double>(std::numeric_limits<std::size_t>::max()))
    fail(path, "invalid list count");
  const auto count = static_cast<std::size_t>(value);
  if (count > maximum_list_items)
    fail(path, "list count exceeds limit");
  return count;
}

void assignVertex(PointT &point, std::string_view name, long double value,
                  const std::filesystem::path &path) {
  const auto as_float = [&] {
    if (std::isfinite(value) &&
        std::abs(value) >
            static_cast<long double>(std::numeric_limits<float>::max()))
      fail(path, std::string(name) + " value exceeds float range");
    return static_cast<float>(value);
  };
  if (name == "x")
    point.x = as_float();
  else if (name == "y")
    point.y = as_float();
  else if (name == "z")
    point.z = as_float();
  else if (name == "intensity")
    point.intensity = as_float();
  else if (name == "red" || name == "r") {
    if (!std::isfinite(value) || value < 0 || value > 255 ||
        std::trunc(value) != value)
      fail(path, "red value out of range");
    point.r = static_cast<std::uint8_t>(value);
  } else if (name == "green" || name == "g") {
    if (!std::isfinite(value) || value < 0 || value > 255 ||
        std::trunc(value) != value)
      fail(path, "green value out of range");
    point.g = static_cast<std::uint8_t>(value);
  } else if (name == "blue" || name == "b") {
    if (!std::isfinite(value) || value < 0 || value > 255 ||
        std::trunc(value) != value)
      fail(path, "blue value out of range");
    point.b = static_cast<std::uint8_t>(value);
  }
}

struct WorkBudget {
  std::size_t remaining_scalars = maximum_decoded_scalars;

  void consume(std::size_t count, const std::filesystem::path &path) {
    if (count > remaining_scalars)
      fail(path, "decoded scalar count exceeds limit");
    remaining_scalars -= count;
  }
};

void consumeElement(std::istream &input, const Element &element,
                    Encoding encoding, PointCloudIRGB &cloud,
                    WorkBudget &budget, const std::filesystem::path &path,
                    std::stop_token stop) {
  const bool vertices = element.name == "vertex";
  if (vertices) {
    if (element.count > cloud.points.max_size() - cloud.size())
      fail(path, "vertex count exceeds container capacity");
    // Header counts are untrusted. Avoid a huge eager allocation before the
    // payload proves that those records actually exist.
    constexpr std::size_t maximum_eager_reserve = 1'000'000;
    if (element.count <= maximum_eager_reserve) {
      if (stop.stop_requested())
        throw OperationCancelled();
      cloud.reserve(cloud.size() + element.count);
    }
  }

  for (std::size_t record = 0; record < element.count; ++record) {
    if ((record % 4096U) == 0U && stop.stop_requested())
      throw OperationCancelled();
    PointT point{};
    for (const auto &property : element.properties) {
      if (!property.is_list) {
        budget.consume(1, path);
        const auto value =
            readScalar(input, property.value_type, encoding, path, stop);
        if (vertices)
          assignVertex(point, property.name, value, path);
        continue;
      }
      budget.consume(1, path);
      const auto count = listCount(
          readScalar(input, property.count_type, encoding, path, stop),
          property.count_type, path);
      if (encoding != Encoding::Ascii) {
        const auto item_size = scalarSize(property.value_type);
        if (count > std::numeric_limits<std::size_t>::max() / item_size)
          fail(path, "list byte count overflow");
      }
      budget.consume(count, path);
      for (std::size_t item = 0; item < count; ++item) {
        if ((item % 4096U) == 0U && stop.stop_requested())
          throw OperationCancelled();
        static_cast<void>(
            readScalar(input, property.value_type, encoding, path, stop));
      }
    }
    if (vertices)
      cloud.push_back(point);
  }
}

template <typename T> void writeLittleEndian(std::ostream &output, T value) {
  if constexpr (sizeof(T) > 1) {
    if (std::endian::native == std::endian::big)
      value = byteswapValue(value);
  }
  output.write(reinterpret_cast<const char *>(&value), sizeof(value));
}

} // namespace

void loadPly(const std::filesystem::path &path, PointCloudIRGB &cloud,
             std::stop_token stop) {
  std::ifstream input(path, std::ios::binary);
  if (!input) {
    const auto native = path.generic_u8string();
    throw std::runtime_error("file not found: " +
                             std::string(native.begin(), native.end()));
  }

  const auto header = readHeader(input, path);
  PointCloudIRGB parsed;
  WorkBudget budget;
  for (const auto &element : header.elements)
    consumeElement(input, element, header.encoding, parsed, budget, path, stop);
  char trailing = '\0';
  if (header.encoding == Encoding::Ascii) {
    std::size_t trailing_bytes = 0;
    while (input.get(trailing)) {
      if ((trailing_bytes++ % 4096U) == 0U && stop.stop_requested())
        throw OperationCancelled();
      if (trailing != ' ' && trailing != '\t' && trailing != '\r' &&
          trailing != '\n')
        fail(path, "extra ASCII data after declared elements");
    }
  } else if (input.get(trailing)) {
    fail(path, "extra binary data after declared elements");
  }
  cloud = std::move(parsed);
}

void savePly(const std::filesystem::path &path, const PointCloudIRGB &cloud) {
  std::ofstream output(path, std::ios::binary | std::ios::trunc);
  const auto native = path.generic_u8string();
  const std::string display_path(native.begin(), native.end());
  if (!output)
    throw std::runtime_error("cannot write: " + display_path);
  savePly(output, path, cloud);
  output.close();
  if (!output)
    throw std::runtime_error("write error: PLY close: " + display_path);
}

void savePly(std::ostream &output, const std::filesystem::path &path,
             const PointCloudIRGB &cloud, std::stop_token stop) {
  const auto native = path.generic_u8string();
  const std::string display_path(native.begin(), native.end());
  constexpr std::size_t written_properties = 7;
  if (cloud.size() > maximum_vertex_records ||
      cloud.size() > maximum_total_records ||
      cloud.size() > maximum_decoded_scalars / written_properties)
    throw std::runtime_error("write error: PLY point count exceeds reader "
                             "limits: " +
                             display_path);

  output << "ply\n"
            "format binary_little_endian 1.0\n"
            "comment generated by kitti_pointcloud_tools\n"
            "element vertex "
         << cloud.size()
         << "\n"
            "property float x\n"
            "property float y\n"
            "property float z\n"
            "property uchar red\n"
            "property uchar green\n"
            "property uchar blue\n"
            "property float intensity\n"
            "end_header\n";
  if (!output)
    throw std::runtime_error("write error: PLY header: " + display_path);

  std::size_t point_index = 0;
  for (const auto &point : cloud) {
    if ((point_index++ % 4096U) == 0U && stop.stop_requested())
      throw OperationCancelled();
    writeLittleEndian(output, point.x);
    writeLittleEndian(output, point.y);
    writeLittleEndian(output, point.z);
    writeLittleEndian(output, point.r);
    writeLittleEndian(output, point.g);
    writeLittleEndian(output, point.b);
    writeLittleEndian(output, point.intensity);
  }
  if (!output)
    throw std::runtime_error("write error: PLY payload: " + display_path);
  output.flush();
  if (!output)
    throw std::runtime_error("write error: PLY flush: " + display_path);
}

} // namespace kpt::io_detail
