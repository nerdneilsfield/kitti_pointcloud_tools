#include "kpt/io/io.hpp"
#include "kpt/io/pcd_codec.hpp"
#include "kpt/io/ply_codec.hpp"
#include "platform/native_file.hpp"
#include "platform/utf8_path.hpp"

#include <spdlog/spdlog.h>

#include <algorithm>
#include <bit>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <limits>
#include <locale>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace kpt {
namespace {

constexpr std::size_t kMaxPointCount = 20000000U;
constexpr std::size_t kMaxTextLineBytes = 64U * 1024U;

PointCloudIRGBPtr makeCloud() { return std::make_shared<PointCloudIRGB>(); }

std::string displayPath(const std::filesystem::path &path) {
  auto converted = platform::pathToUtf8(path);
  return converted ? std::move(converted).value() : "<invalid-native-path>";
}

std::uint32_t fromLittleEndian(std::uint32_t value) {
  if constexpr (std::endian::native == std::endian::big) {
    return ((value & 0x000000FFU) << 24U) | ((value & 0x0000FF00U) << 8U) |
           ((value & 0x00FF0000U) >> 8U) | ((value & 0xFF000000U) >> 24U);
  }
  return value;
}

std::uint32_t toLittleEndian(std::uint32_t value) {
  return fromLittleEndian(value);
}

float readLittleFloat(std::istream &input, const std::filesystem::path &path) {
  std::uint32_t bits = 0;
  input.read(reinterpret_cast<char *>(&bits), sizeof(bits));
  if (!input)
    throw std::runtime_error("parse error: truncated bin record: " +
                             displayPath(path));
  return std::bit_cast<float>(fromLittleEndian(bits));
}

void writeLittleFloat(std::ostream &output, float value,
                      const std::filesystem::path &path) {
  auto bits = toLittleEndian(std::bit_cast<std::uint32_t>(value));
  output.write(reinterpret_cast<const char *>(&bits), sizeof(bits));
  if (!output)
    throw std::runtime_error("write error: " + displayPath(path));
}

void loadBin(const std::filesystem::path &path, PointCloudIRGB &cloud) {
  std::ifstream input(path, std::ios::binary | std::ios::ate);
  if (!input)
    throw std::runtime_error("file not found: " + displayPath(path));
  const auto end = input.tellg();
  if (end < 0)
    throw std::runtime_error("parse error: cannot determine bin size: " +
                             displayPath(path));
  constexpr std::uintmax_t record_size = 4U * sizeof(float);
  const auto byte_count = static_cast<std::uintmax_t>(end);
  if (byte_count % record_size != 0)
    throw std::runtime_error("parse error: bin size not multiple of 16: " +
                             displayPath(path));
  const auto point_count = byte_count / record_size;
  if (point_count > kMaxPointCount)
    throw std::runtime_error("parse error: bin point count exceeds limit: " +
                             displayPath(path));
  cloud.reserve(static_cast<std::size_t>(point_count));
  input.seekg(0, std::ios::beg);
  for (std::uintmax_t index = 0; index < point_count; ++index) {
    PointT point;
    point.x = readLittleFloat(input, path);
    point.y = readLittleFloat(input, path);
    point.z = readLittleFloat(input, path);
    point.intensity = readLittleFloat(input, path);
    cloud.push_back(point);
  }
}

bool readBoundedLine(std::istream &input, std::string &line,
                     const std::filesystem::path &path) {
  line.clear();
  std::size_t input_bytes = 0;
  char character = '\0';
  while (input.get(character)) {
    ++input_bytes;
    if (input_bytes > kMaxTextLineBytes) {
      throw std::runtime_error("parse error: text line exceeds 64 KiB: " +
                               displayPath(path));
    }
    if (character == '\n') {
      if (!line.empty() && line.back() == '\r')
        line.pop_back();
      return true;
    }
    line.push_back(character);
  }
  if (input.bad())
    throw std::runtime_error("read error: " + displayPath(path));
  if (!line.empty() && line.back() == '\r')
    line.pop_back();
  return !line.empty();
}

std::size_t expectedColumns(Format format) {
  switch (format) {
  case Format::XYZ:
    return 3;
  case Format::XYZI:
    return 4;
  case Format::XYZRGB:
    return 6;
  case Format::XYZRGBI:
    return 7;
  default:
    throw std::runtime_error("not a delimited point format");
  }
}

std::uint8_t parseColor(float value, const std::string &line) {
  if (!std::isfinite(value) || value < 0.0F || value > 255.0F ||
      std::trunc(value) != value) {
    throw std::runtime_error("RGB value must be an integer in [0,255]: " +
                             line);
  }
  return static_cast<std::uint8_t>(value);
}

void loadAscii(const std::filesystem::path &path, Format format,
               PointCloudIRGB &cloud) {
  std::ifstream input(path);
  if (!input)
    throw std::runtime_error("file not found: " + displayPath(path));
  input.imbue(std::locale::classic());
  const auto columns = expectedColumns(format);
  std::string line;
  std::size_t line_number = 0;
  std::size_t warning_count = 0;
  while (readBoundedLine(input, line, path)) {
    ++line_number;
    if (line_number == 1 && line.starts_with("\xEF\xBB\xBF"))
      line.erase(0, 3);
    const auto first =
        std::find_if_not(line.begin(), line.end(), [](unsigned char character) {
          return std::isspace(character) != 0;
        });
    if (first == line.end() || *first == '#')
      continue;
    std::istringstream row(line);
    row.imbue(std::locale::classic());
    std::vector<float> values;
    values.reserve(columns + 1U);
    float value = 0.0F;
    while (values.size() <= columns && row >> value)
      values.push_back(value);
    if (values.size() != columns || !row.eof()) {
      if (++warning_count <= 50) {
        if (values.size() > columns) {
          spdlog::warn("skip {}:{}: more than {} numeric columns",
                       displayPath(path), line_number, columns);
        } else if (values.size() == columns) {
          spdlog::warn("skip {}:{}: trailing non-numeric token",
                       displayPath(path), line_number);
        } else {
          spdlog::warn("skip {}:{}: expected {} numeric columns, got {}",
                       displayPath(path), line_number, columns, values.size());
        }
      }
      continue;
    }

    PointT point;
    point.x = values[0];
    point.y = values[1];
    point.z = values[2];
    try {
      if (format == Format::XYZI) {
        point.intensity = values[3];
      } else if (format == Format::XYZRGB || format == Format::XYZRGBI) {
        point.r = parseColor(values[3], line);
        point.g = parseColor(values[4], line);
        point.b = parseColor(values[5], line);
        if (format == Format::XYZRGBI)
          point.intensity = values[6];
      }
    } catch (const std::runtime_error &error) {
      if (++warning_count <= 50)
        spdlog::warn("skip {}:{}: {}", displayPath(path), line_number,
                     error.what());
      continue;
    }
    cloud.push_back(point);
    if (cloud.size() > kMaxPointCount) {
      throw std::runtime_error("parse error: text point count exceeds limit: " +
                               displayPath(path));
    }
  }
  if (warning_count > 50)
    spdlog::warn("... {} more skipped lines", warning_count - 50);
}

void saveBin(const std::filesystem::path &path, const PointCloudIRGB &cloud) {
  if (cloud.size() > kMaxPointCount)
    throw std::runtime_error("write error: point count exceeds limit: " +
                             displayPath(path));
  std::ofstream output(path, std::ios::binary);
  if (!output)
    throw std::runtime_error("cannot write: " + displayPath(path));
  for (const auto &point : cloud.points) {
    writeLittleFloat(output, point.x, path);
    writeLittleFloat(output, point.y, path);
    writeLittleFloat(output, point.z, path);
    writeLittleFloat(output, point.intensity, path);
  }
  output.flush();
  output.close();
  if (!output)
    throw std::runtime_error("write error: " + displayPath(path));
}

void saveAscii(const std::filesystem::path &path, const PointCloudIRGB &cloud,
               Format format) {
  static_cast<void>(expectedColumns(format));
  if (cloud.size() > kMaxPointCount)
    throw std::runtime_error("write error: point count exceeds limit: " +
                             displayPath(path));
  std::ofstream output(path);
  if (!output)
    throw std::runtime_error("cannot write: " + displayPath(path));
  output.imbue(std::locale::classic());
  output << std::setprecision(std::numeric_limits<float>::max_digits10);
  for (const auto &point : cloud.points) {
    output << point.x << ' ' << point.y << ' ' << point.z;
    switch (format) {
    case Format::XYZ:
      break;
    case Format::XYZI:
      output << ' ' << point.intensity;
      break;
    case Format::XYZRGB:
    case Format::XYZRGBI:
      output << ' ' << static_cast<unsigned>(point.r) << ' '
             << static_cast<unsigned>(point.g) << ' '
             << static_cast<unsigned>(point.b);
      if (format == Format::XYZRGBI)
        output << ' ' << point.intensity;
      break;
    default:
      break;
    }
    output << '\n';
  }
  if (!output)
    throw std::runtime_error("write error: " + displayPath(path));
  output.flush();
  output.close();
  if (!output)
    throw std::runtime_error("write error: " + displayPath(path));
}

std::filesystem::path
temporaryOutputPath(const std::filesystem::path &destination) {
  static thread_local std::mt19937_64 generator(std::random_device{}());
  auto filename = destination.filename();
  filename += ".kpt-tmp-" + std::to_string(generator());
  return destination.parent_path() / filename;
}

} // namespace

PointCloudIRGBPtr load(const std::filesystem::path &path) {
  std::error_code status_error;
  const bool exists = std::filesystem::exists(path, status_error);
  if (status_error) {
    throw std::runtime_error("cannot inspect input " + displayPath(path) +
                             ": " + status_error.message());
  }
  if (!exists)
    throw std::runtime_error("file not found: " + displayPath(path));
  auto cloud = makeCloud();
  const auto format = detect(path);
  switch (format) {
  case Format::Bin:
    loadBin(path, *cloud);
    break;
  case Format::PCD:
    io_detail::loadPcd(path, *cloud);
    break;
  case Format::PLY:
    io_detail::loadPly(path, *cloud);
    break;
  default:
    loadAscii(path, format, *cloud);
    break;
  }
  return cloud;
}

void save(const std::filesystem::path &path, const PointCloudIRGB &cloud,
          std::optional<Format> ascii_flavor) {
  const auto format = detect(path);
  if (ascii_flavor && *ascii_flavor != format) {
    throw std::runtime_error(
        "explicit output format does not match file extension: " +
        displayPath(path));
  }
  const auto temporary = temporaryOutputPath(path);
  try {
    switch (format) {
    case Format::Bin:
      saveBin(temporary, cloud);
      break;
    case Format::PCD:
      io_detail::savePcd(temporary, cloud);
      break;
    case Format::PLY:
      io_detail::savePly(temporary, cloud);
      break;
    default:
      saveAscii(temporary, cloud, format);
      break;
    }
    auto replaced = platform::replaceFileAtomically(temporary, path);
    if (!replaced) {
      throw std::runtime_error(
          "output publication or durability check failed for " +
          displayPath(path) + ": " + replaced.error().message);
    }
    for (const auto &warning : replaced.value().post_commit_warnings) {
      spdlog::warn("{}: {}", warning.message, warning.system_error.message());
    }
  } catch (...) {
    std::error_code ignored;
    std::filesystem::remove(temporary, ignored);
    throw;
  }
  spdlog::debug("saved {} points to {}", cloud.size(), displayPath(path));
}

} // namespace kpt
