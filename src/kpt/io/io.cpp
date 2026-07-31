#include "kpt/io/io.hpp"
#include "kpt/cancellation.hpp"
#include "kpt/io/pcd_codec.hpp"
#include "kpt/io/ply_codec.hpp"
#include "platform/native_file.hpp"
#include "platform/utf8_path.hpp"

#include <spdlog/spdlog.h>

#include <algorithm>
#include <array>
#include <bit>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <limits>
#include <locale>
#include <random>
#include <sstream>
#include <stdexcept>
#include <streambuf>
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

class NativeOutputStreamBuf final : public std::streambuf {
public:
  explicit NativeOutputStreamBuf(platform::NativeOutputFile &output)
      : output_(output) {
    setp(buffer_.data(), buffer_.data() + buffer_.size());
  }

  [[nodiscard]] const std::optional<platform::PlatformError> &error() const {
    return error_;
  }

protected:
  std::streamsize xsputn(const char *data, std::streamsize size) override {
    if (size <= 0)
      return 0;
    std::streamsize copied = 0;
    while (copied < size) {
      if (pptr() == epptr() && !flushBuffer())
        break;
      const auto available = epptr() - pptr();
      const auto count = std::min(available, size - copied);
      std::memcpy(pptr(), data + copied, static_cast<std::size_t>(count));
      pbump(static_cast<int>(count));
      copied += count;
    }
    return copied;
  }

  int_type overflow(int_type character) override {
    if (!flushBuffer())
      return traits_type::eof();
    if (traits_type::eq_int_type(character, traits_type::eof()))
      return traits_type::not_eof(character);
    *pptr() = traits_type::to_char_type(character);
    pbump(1);
    return traits_type::not_eof(character);
  }

  int sync() override { return flushBuffer() ? 0 : -1; }

private:
  bool flushBuffer() {
    const auto size = static_cast<std::size_t>(pptr() - pbase());
    if (size == 0)
      return !error_;
    auto written =
        output_.write({reinterpret_cast<const std::uint8_t *>(pbase()), size});
    setp(buffer_.data(), buffer_.data() + buffer_.size());
    if (!written) {
      error_ = std::move(written).error();
      return false;
    }
    return true;
  }

  platform::NativeOutputFile &output_;
  std::array<char, 64U * 1024U> buffer_{};
  std::optional<platform::PlatformError> error_;
};

std::unique_ptr<platform::NativeOutputFile>
openCloudOutput(const std::filesystem::path &destination) {
  static thread_local std::mt19937_64 generator(std::random_device{}());
  for (int attempt = 0; attempt < 64; ++attempt) {
    auto candidate = destination;
    candidate += ".kpt-tmp-" + std::to_string(generator());
    auto opened = platform::openNativeOutputExclusively(candidate);
    if (!opened)
      throw std::system_error(opened.error().system_error,
                              opened.error().message);
    auto output = std::move(opened).value();
    if (output)
      return output;
  }
  throw std::runtime_error("cannot reserve unique temporary output: " +
                           displayPath(destination));
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

void loadBin(const std::filesystem::path &path, PointCloudIRGB &cloud,
             std::stop_token stop) {
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
  if (stop.stop_requested())
    throw OperationCancelled();
  cloud.reserve(static_cast<std::size_t>(point_count));
  input.seekg(0, std::ios::beg);
  for (std::uintmax_t index = 0; index < point_count; ++index) {
    if ((index % 4096U) == 0U && stop.stop_requested())
      throw OperationCancelled();
    PointT point;
    point.x = readLittleFloat(input, path);
    point.y = readLittleFloat(input, path);
    point.z = readLittleFloat(input, path);
    point.intensity = readLittleFloat(input, path);
    cloud.push_back(point);
  }
}

void loadBin(std::istream &input, std::size_t byte_count,
             const std::filesystem::path &path, PointCloudIRGB &cloud,
             std::stop_token stop) {
  constexpr std::size_t record_size = 4U * sizeof(float);
  if (byte_count % record_size != 0)
    throw std::runtime_error("parse error: bin size not multiple of 16: " +
                             displayPath(path));
  const auto point_count = byte_count / record_size;
  if (point_count > kMaxPointCount)
    throw std::runtime_error("parse error: bin point count exceeds limit: " +
                             displayPath(path));
  cloud.reserve(point_count);
  for (std::size_t index = 0; index < point_count; ++index) {
    if ((index % 4096U) == 0U && stop.stop_requested())
      throw OperationCancelled();
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

void loadAscii(std::istream &input, const std::filesystem::path &path,
               Format format, PointCloudIRGB &cloud, std::stop_token stop);

void loadAscii(const std::filesystem::path &path, Format format,
               PointCloudIRGB &cloud, std::stop_token stop) {
  std::ifstream input(path);
  if (!input)
    throw std::runtime_error("file not found: " + displayPath(path));
  loadAscii(input, path, format, cloud, stop);
}

void loadAscii(std::istream &input, const std::filesystem::path &path,
               Format format, PointCloudIRGB &cloud, std::stop_token stop) {
  input.imbue(std::locale::classic());
  const auto columns = expectedColumns(format);
  std::string line;
  std::size_t line_number = 0;
  std::size_t warning_count = 0;
  while (readBoundedLine(input, line, path)) {
    ++line_number;
    if (stop.stop_requested())
      throw OperationCancelled();
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
    if (cloud.size() >= kMaxPointCount) {
      throw std::runtime_error("parse error: text point count exceeds limit: " +
                               displayPath(path));
    }
    cloud.push_back(point);
  }
  if (warning_count > 50)
    spdlog::warn("... {} more skipped lines", warning_count - 50);
}

class MemoryStreamBuf final : public std::streambuf {
public:
  explicit MemoryStreamBuf(std::span<const std::byte> bytes) {
    if (bytes.size() >
        static_cast<std::size_t>(std::numeric_limits<std::ptrdiff_t>::max()))
      throw std::length_error("memory input exceeds stream address range");
    begin_ = bytes.empty()
                 ? &empty_
                 : const_cast<char *>(
                       reinterpret_cast<const char *>(bytes.data()));
    end_ = begin_ + bytes.size();
    setg(begin_, begin_, end_);
  }

protected:
  pos_type seekoff(off_type offset, std::ios_base::seekdir direction,
                   std::ios_base::openmode mode) override {
    if ((mode & std::ios_base::in) == 0)
      return pos_type(off_type(-1));
    off_type origin = 0;
    if (direction == std::ios_base::beg) {
      origin = 0;
    } else if (direction == std::ios_base::cur) {
      origin = gptr() - begin_;
    } else if (direction == std::ios_base::end) {
      origin = end_ - begin_;
    } else {
      return pos_type(off_type(-1));
    }
    const off_type length = end_ - begin_;
    if ((offset > 0 && origin > length - offset) ||
        (offset < 0 &&
         (offset == std::numeric_limits<off_type>::min() ||
          origin < -offset))) {
      return pos_type(off_type(-1));
    }
    return seekpos(origin + offset, mode);
  }

  pos_type seekpos(pos_type position, std::ios_base::openmode mode) override {
    if ((mode & std::ios_base::in) == 0)
      return pos_type(off_type(-1));
    const auto offset = static_cast<off_type>(position);
    if (offset < 0 || offset > end_ - begin_)
      return pos_type(off_type(-1));
    setg(begin_, begin_ + offset, end_);
    return position;
  }

private:
  char *begin_ = nullptr;
  char *end_ = nullptr;
  char empty_ = '\0';
};

void saveBin(std::ostream &output, const std::filesystem::path &path,
             const PointCloudIRGB &cloud, std::stop_token stop) {
  if (cloud.size() > kMaxPointCount)
    throw std::runtime_error("write error: point count exceeds limit: " +
                             displayPath(path));
  std::size_t point_index = 0;
  for (const auto &point : cloud.points) {
    if ((point_index++ % 4096U) == 0U && stop.stop_requested())
      throw OperationCancelled();
    writeLittleFloat(output, point.x, path);
    writeLittleFloat(output, point.y, path);
    writeLittleFloat(output, point.z, path);
    writeLittleFloat(output, point.intensity, path);
  }
  if (!output)
    throw std::runtime_error("write error: " + displayPath(path));
}

void saveAscii(std::ostream &output, const std::filesystem::path &path,
               const PointCloudIRGB &cloud, Format format,
               std::stop_token stop) {
  static_cast<void>(expectedColumns(format));
  if (cloud.size() > kMaxPointCount)
    throw std::runtime_error("write error: point count exceeds limit: " +
                             displayPath(path));
  output.imbue(std::locale::classic());
  output << std::setprecision(std::numeric_limits<float>::max_digits10);
  std::size_t point_index = 0;
  for (const auto &point : cloud.points) {
    if ((point_index++ % 4096U) == 0U && stop.stop_requested())
      throw OperationCancelled();
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
}

} // namespace

PointCloudIRGBPtr load(const std::filesystem::path &path,
                       std::stop_token stop) {
  if (stop.stop_requested())
    throw OperationCancelled();
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
    loadBin(path, *cloud, stop);
    break;
  case Format::PCD:
    io_detail::loadPcd(path, *cloud, stop);
    break;
  case Format::PLY:
    io_detail::loadPly(path, *cloud, stop);
    break;
  default:
    loadAscii(path, format, *cloud, stop);
    break;
  }
  return cloud;
}

DecodedCloud decode(std::span<const std::byte> bytes,
                    std::string_view source_name, std::stop_token stop) {
  if (stop.stop_requested())
    throw OperationCancelled();
  const auto path = std::filesystem::path(std::string(source_name));
  const auto format = detect(path);
  MemoryStreamBuf buffer(bytes);
  std::istream input(&buffer);
  auto cloud = makeCloud();
  CloudSchema schema;
  switch (format) {
  case Format::Bin:
    schema.has_intensity = true;
    loadBin(input, bytes.size(), path, *cloud, stop);
    break;
  case Format::PCD:
    io_detail::loadPcd(input, path, *cloud, schema.has_color,
                       schema.has_intensity, stop);
    schema.has_noise = cloud->has_noise;
    break;
  case Format::PLY:
    io_detail::loadPly(input, path, *cloud, schema.has_color,
                       schema.has_intensity, stop);
    break;
  case Format::XYZ:
    loadAscii(input, path, format, *cloud, stop);
    break;
  case Format::XYZI:
    schema.has_intensity = true;
    loadAscii(input, path, format, *cloud, stop);
    break;
  case Format::XYZRGB:
    schema.has_color = true;
    loadAscii(input, path, format, *cloud, stop);
    break;
  case Format::XYZRGBI:
    schema.has_color = true;
    schema.has_intensity = true;
    loadAscii(input, path, format, *cloud, stop);
    break;
  }
  return {std::move(cloud), schema};
}

CloudWriteStatus saveAtomic(const std::filesystem::path &path,
                            const PointCloudIRGB &cloud, bool overwrite,
                            std::optional<Format> ascii_flavor,
                            std::stop_token stop) {
  if (stop.stop_requested())
    throw OperationCancelled();
  const auto format = detect(path);
  if (ascii_flavor && *ascii_flavor != format) {
    throw std::runtime_error(
        "explicit output format does not match file extension: " +
        displayPath(path));
  }
  auto native_output = openCloudOutput(path);
  NativeOutputStreamBuf buffer(*native_output);
  std::ostream output(&buffer);
  try {
    switch (format) {
    case Format::Bin:
      saveBin(output, path, cloud, stop);
      break;
    case Format::PCD:
      io_detail::savePcd(output, path, cloud, stop);
      break;
    case Format::PLY:
      io_detail::savePly(output, path, cloud, stop);
      break;
    default:
      saveAscii(output, path, cloud, format, stop);
      break;
    }
    output.flush();
    if (buffer.error()) {
      throw std::system_error(buffer.error()->system_error,
                              buffer.error()->message);
    }
    if (!output)
      throw std::runtime_error("write error: " + displayPath(path));
    if (stop.stop_requested())
      throw OperationCancelled();
    auto published = native_output->publish(path, overwrite);
    if (!published) {
      throw std::system_error(
          published.error().system_error,
          "output publication or durability check failed for " +
              displayPath(path) + ": " + published.error().message);
    }
    if (!published.value().published)
      return CloudWriteStatus::Skipped;
    for (const auto &warning : published.value().post_commit_warnings) {
      spdlog::warn("{}: {}", warning.message, warning.system_error.message());
    }
  } catch (...) {
    if (buffer.error()) {
      const auto error = *buffer.error();
      native_output.reset();
      throw std::system_error(error.system_error, error.message);
    }
    native_output.reset();
    throw;
  }
  spdlog::debug("saved {} points to {}", cloud.size(), displayPath(path));
  return CloudWriteStatus::Written;
}

void save(const std::filesystem::path &path, const PointCloudIRGB &cloud,
          std::optional<Format> ascii_flavor) {
  static_cast<void>(saveAtomic(path, cloud, true, ascii_flavor));
}

} // namespace kpt
