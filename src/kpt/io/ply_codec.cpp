#include "kpt/io/ply_codec.hpp"

#include <array>
#include <bit>
#include <charconv>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <string>
#include <string_view>
#include <type_traits>
#include <utility>
#include <vector>

namespace kpt::io_detail {
namespace {

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
  if (!std::getline(input, line))
    fail(path, "missing header");
  stripCarriageReturn(line);
  if (line != "ply")
    fail(path, "missing magic");

  Header header;
  bool saw_format = false;
  bool saw_end = false;
  Element *current = nullptr;
  while (std::getline(input, line)) {
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
      header.elements.push_back(
          {std::string(tokens[1]), parseCount(tokens[2], path), {}});
      current = &header.elements.back();
      continue;
    }
    if (tokens[0] == "property") {
      if (current == nullptr)
        fail(path, "property without element");
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
  for (const auto required : {"x", "y", "z"}) {
    bool found = false;
    for (const auto &property : vertex->properties) {
      if (!property.is_list && property.name == required) {
        found = true;
        break;
      }
    }
    if (!found)
      fail(path, "vertex element missing " + std::string(required));
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
    return readBinaryValue<float>(input, swap, path);
  case ScalarType::Float64:
    return readBinaryValue<double>(input, swap, path);
  }
  fail(path, "invalid scalar type");
}

long double parseAsciiScalar(std::string_view token, ScalarType type,
                             const std::filesystem::path &path) {
  if (type == ScalarType::Float32 || type == ScalarType::Float64) {
    double value = 0;
    const auto result =
        std::from_chars(token.data(), token.data() + token.size(), value);
    if (result.ec != std::errc{} || result.ptr != token.data() + token.size())
      fail(path, "invalid ASCII floating-point value");
    return value;
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
                           const std::filesystem::path &path) {
  std::string token;
  if (!(input >> token))
    fail(path, "truncated ASCII payload");
  return token;
}

long double readScalar(std::istream &input, ScalarType type, Encoding encoding,
                       const std::filesystem::path &path) {
  if (encoding == Encoding::Ascii)
    return parseAsciiScalar(readAsciiToken(input, path), type, path);
  const bool file_little = encoding == Encoding::BinaryLittleEndian;
  const bool host_little = std::endian::native == std::endian::little;
  return readBinaryScalar(input, type, file_little != host_little, path);
}

std::size_t listCount(long double value, ScalarType type,
                      const std::filesystem::path &path) {
  if (!isIntegral(type) || !std::isfinite(value) || value < 0 ||
      value > static_cast<long double>(std::numeric_limits<std::size_t>::max()))
    fail(path, "invalid list count");
  return static_cast<std::size_t>(value);
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
    if (!std::isfinite(value) || value < 0 || value > 255)
      fail(path, "red value out of range");
    point.r = static_cast<std::uint8_t>(value);
  } else if (name == "green" || name == "g") {
    if (!std::isfinite(value) || value < 0 || value > 255)
      fail(path, "green value out of range");
    point.g = static_cast<std::uint8_t>(value);
  } else if (name == "blue" || name == "b") {
    if (!std::isfinite(value) || value < 0 || value > 255)
      fail(path, "blue value out of range");
    point.b = static_cast<std::uint8_t>(value);
  }
}

void consumeElement(std::istream &input, const Element &element,
                    Encoding encoding, PointCloudIRGB &cloud,
                    const std::filesystem::path &path) {
  const bool vertices = element.name == "vertex";
  if (vertices) {
    if (element.count > cloud.points.max_size() - cloud.size())
      fail(path, "vertex count exceeds container capacity");
    // Header counts are untrusted. Avoid a huge eager allocation before the
    // payload proves that those records actually exist.
    constexpr std::size_t maximum_eager_reserve = 1'000'000;
    if (element.count <= maximum_eager_reserve)
      cloud.reserve(cloud.size() + element.count);
  }

  for (std::size_t record = 0; record < element.count; ++record) {
    PointT point{};
    for (const auto &property : element.properties) {
      if (!property.is_list) {
        const auto value =
            readScalar(input, property.value_type, encoding, path);
        if (vertices)
          assignVertex(point, property.name, value, path);
        continue;
      }
      const auto count =
          listCount(readScalar(input, property.count_type, encoding, path),
                    property.count_type, path);
      if (encoding != Encoding::Ascii) {
        const auto item_size = scalarSize(property.value_type);
        if (count > std::numeric_limits<std::size_t>::max() / item_size)
          fail(path, "list byte count overflow");
      }
      for (std::size_t item = 0; item < count; ++item)
        static_cast<void>(
            readScalar(input, property.value_type, encoding, path));
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

void loadPly(const std::filesystem::path &path, PointCloudIRGB &cloud) {
  std::ifstream input(path, std::ios::binary);
  if (!input) {
    const auto native = path.generic_u8string();
    throw std::runtime_error("file not found: " +
                             std::string(native.begin(), native.end()));
  }

  const auto header = readHeader(input, path);
  PointCloudIRGB parsed;
  for (const auto &element : header.elements)
    consumeElement(input, element, header.encoding, parsed, path);
  cloud = std::move(parsed);
}

void savePly(const std::filesystem::path &path, const PointCloudIRGB &cloud) {
  std::ofstream output(path, std::ios::binary | std::ios::trunc);
  const auto native = path.generic_u8string();
  const std::string display_path(native.begin(), native.end());
  if (!output)
    throw std::runtime_error("cannot write: " + display_path);

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

  for (const auto &point : cloud) {
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
}

} // namespace kpt::io_detail
