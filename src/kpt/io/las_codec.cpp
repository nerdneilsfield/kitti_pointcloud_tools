#include "kpt/io/las_codec.hpp"
#include "kpt/cancellation.hpp"
#include "platform/utf8_path.hpp"

#include <spdlog/spdlog.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <istream>
#include <limits>
#include <ostream>
#include <stdexcept>
#include <string>

namespace kpt::io_detail {
namespace {

std::string displayPath(const std::filesystem::path &path) {
  auto converted = platform::pathToUtf8(path);
  return converted ? std::move(converted).value() : "<invalid-native-path>";
}

#pragma pack(push, 1)
struct LasHeader12 {
  char signature[4];          // "LASF"
  std::uint16_t file_source_id;
  std::uint16_t global_encoding;
  std::uint8_t project_id[16];
  std::uint8_t version_major;
  std::uint8_t version_minor;
  char system_identifier[32];
  char generating_software[32];
  std::uint16_t creation_day;
  std::uint16_t creation_year;
  std::uint16_t header_size;
  std::uint32_t offset_to_point_data;
  std::uint32_t num_vlrs;
  std::uint8_t point_format;
  std::uint16_t point_record_length;
  std::uint32_t num_points;
  std::uint32_t points_by_return[5];
  double scale_x, scale_y, scale_z;
  double offset_x, offset_y, offset_z;
  double max_x, min_x, max_y, min_y, max_z, min_z;
};
static_assert(sizeof(LasHeader12) == 227);

struct LasPoint0 {
  std::int32_t x, y, z;
  std::uint16_t intensity;
  std::uint8_t return_info;
  std::uint8_t classification;
  std::int8_t scan_angle_rank;
  std::uint8_t user_data;
  std::uint16_t point_source_id;
};
static_assert(sizeof(LasPoint0) == 20);

struct LasRGB {
  std::uint16_t r, g, b;
};
static_assert(sizeof(LasRGB) == 6);
#pragma pack(pop)

void readHeader(std::istream &input, LasHeader12 &header,
                const std::filesystem::path &path) {
  input.read(reinterpret_cast<char *>(&header), sizeof(header));
  if (!input || input.gcount() != sizeof(header))
    throw std::runtime_error("LAS parse error: truncated header: " +
                             displayPath(path));
  if (std::memcmp(header.signature, "LASF", 4) != 0)
    throw std::runtime_error("LAS parse error: bad signature: " +
                             displayPath(path));
  if (header.version_major != 1 ||
      (header.version_minor != 2 && header.version_minor != 4 &&
       header.version_minor != 3 && header.version_minor != 1)) {
    throw std::runtime_error("LAS parse error: unsupported version " +
                             std::to_string(header.version_major) + "." +
                             std::to_string(header.version_minor) + ": " +
                             displayPath(path));
  }
}

void skipVlrs(std::istream &input, std::uint32_t count,
              const std::filesystem::path &path) {
  for (std::uint32_t i = 0; i < count; ++i) {
    std::uint16_t reserved;
    char user_id[16];
    std::uint16_t record_id;
    std::uint16_t length;
    char description[32];
    input.read(reinterpret_cast<char *>(&reserved), 2);
    input.read(reinterpret_cast<char *>(user_id), 16);
    input.read(reinterpret_cast<char *>(&record_id), 2);
    input.read(reinterpret_cast<char *>(&length), 2);
    input.read(reinterpret_cast<char *>(description), 32);
    if (!input)
      throw std::runtime_error("LAS parse error: truncated VLR: " +
                               displayPath(path));
    std::vector<char> payload(length);
    input.read(payload.data(), length);
    if (!input)
      throw std::runtime_error("LAS parse error: truncated VLR payload: " +
                               displayPath(path));
  }
}

void loadPoints(std::istream &input, const LasHeader12 &header,
                PointCloudIRGB &cloud, bool &has_color,
                bool &has_intensity, std::stop_token stop,
                const std::filesystem::path &path) {
  const std::uint8_t fmt = header.point_format;
  const bool has_rgb = (fmt == 2 || fmt == 3 || fmt == 5);
  const bool has_gps = (fmt == 1 || fmt == 3 || fmt == 4 || fmt == 5);
  const std::uint16_t rec_len = header.point_record_length;
  const std::uint32_t count = header.num_points;
  const double sx = header.scale_x != 0.0 ? header.scale_x : 0.01;
  const double sy = header.scale_y != 0.0 ? header.scale_y : 0.01;
  const double sz = header.scale_z != 0.0 ? header.scale_z : 0.01;
  const double ox = header.offset_x;
  const double oy = header.offset_y;
  const double oz = header.offset_z;

  has_color = has_rgb;
  has_intensity = true;

  cloud.clear();
  cloud.reserve(count);
  cloud.has_noise = true;

  const std::size_t base_size = sizeof(LasPoint0);
  const std::size_t gps_size = has_gps ? sizeof(double) : 0;
  const std::size_t rgb_size = has_rgb ? sizeof(LasRGB) : 0;
  const std::size_t expected = base_size + gps_size + rgb_size;
  if (rec_len < expected) {
    throw std::runtime_error("LAS parse error: record length " +
                             std::to_string(rec_len) + " < expected " +
                             std::to_string(expected) + ": " +
                             displayPath(path));
  }
  const std::size_t padding = rec_len - expected;

  std::vector<char> buf(rec_len);
  for (std::uint32_t i = 0; i < count; ++i) {
    if (stop.stop_requested())
      throw OperationCancelled();
    input.read(buf.data(), rec_len);
    if (!input || input.gcount() != rec_len)
      throw std::runtime_error("LAS parse error: truncated point record " +
                               std::to_string(i) + ": " + displayPath(path));

    LasPoint0 p0;
    std::memcpy(&p0, buf.data(), sizeof(p0));

    PointT pt;
    pt.x = static_cast<float>(static_cast<double>(p0.x) * sx + ox);
    pt.y = static_cast<float>(static_cast<double>(p0.y) * sy + oy);
    pt.z = static_cast<float>(static_cast<double>(p0.z) * sz + oz);
    pt.intensity = static_cast<float>(p0.intensity) / 65535.0F;
    if (pt.intensity > 1.0F)
      pt.intensity = 1.0F;
    pt.noise = (p0.classification == 7 || p0.classification == 18) ? 1 : 0;

    if (has_rgb) {
      LasRGB rgb;
      std::memcpy(&rgb, buf.data() + base_size + gps_size, sizeof(rgb));
      pt.r = static_cast<std::uint8_t>(rgb.r >> 8);
      pt.g = static_cast<std::uint8_t>(rgb.g >> 8);
      pt.b = static_cast<std::uint8_t>(rgb.b >> 8);
    }

    cloud.points.push_back(pt);
  }
  cloud.width = cloud.points.size();
  cloud.height = 1;
}

void writeHeader(std::ostream &output, const PointCloudIRGB &cloud,
                 const std::filesystem::path &path) {
  bool any_rgb = false;
  double min_x = std::numeric_limits<double>::max();
  double min_y = std::numeric_limits<double>::max();
  double min_z = std::numeric_limits<double>::max();
  double max_x = std::numeric_limits<double>::lowest();
  double max_y = std::numeric_limits<double>::lowest();
  double max_z = std::numeric_limits<double>::lowest();

  for (const auto &p : cloud.points) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
      continue;
    min_x = std::min(min_x, static_cast<double>(p.x));
    min_y = std::min(min_y, static_cast<double>(p.y));
    min_z = std::min(min_z, static_cast<double>(p.z));
    max_x = std::max(max_x, static_cast<double>(p.x));
    max_y = std::max(max_y, static_cast<double>(p.y));
    max_z = std::max(max_z, static_cast<double>(p.z));
    if (p.r != 0 || p.g != 0 || p.b != 0)
      any_rgb = true;
  }
  if (cloud.points.empty()) {
    min_x = min_y = min_z = 0;
    max_x = max_y = max_z = 0;
  }

  const std::uint8_t point_format = any_rgb ? 2 : 0;
  const std::uint16_t rec_len = any_rgb ? 26 : 20;

  LasHeader12 header{};
  std::memcpy(header.signature, "LASF", 4);
  header.version_major = 1;
  header.version_minor = 2;
  std::strncpy(header.system_identifier, "kpt", 32);
  std::strncpy(header.generating_software, "kitti_pointcloud_tools", 32);
  header.header_size = sizeof(header);
  header.offset_to_point_data = sizeof(header);
  header.num_vlrs = 0;
  header.point_format = point_format;
  header.point_record_length = rec_len;
  header.num_points = static_cast<std::uint32_t>(cloud.size());
  header.scale_x = 0.01;
  header.scale_y = 0.01;
  header.scale_z = 0.01;
  header.offset_x = std::floor(min_x);
  header.offset_y = std::floor(min_y);
  header.offset_z = std::floor(min_z);
  header.max_x = max_x;
  header.min_x = min_x;
  header.max_y = max_y;
  header.min_y = min_y;
  header.max_z = max_z;
  header.min_z = min_z;

  output.write(reinterpret_cast<const char *>(&header), sizeof(header));
  if (!output)
    throw std::runtime_error("LAS write error: header: " + displayPath(path));
}

void writePoints(std::ostream &output, const PointCloudIRGB &cloud,
                 std::stop_token stop, const std::filesystem::path &path) {
  bool any_rgb = false;
  for (const auto &p : cloud.points) {
    if (p.r != 0 || p.g != 0 || p.b != 0) {
      any_rgb = true;
      break;
    }
  }

  const double sx = 0.01;
  const double sy = 0.01;
  const double sz = 0.01;

  double off_x = std::numeric_limits<double>::max();
  double off_y = std::numeric_limits<double>::max();
  double off_z = std::numeric_limits<double>::max();
  for (const auto &p : cloud.points) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
      continue;
    off_x = std::min(off_x, static_cast<double>(p.x));
    off_y = std::min(off_y, static_cast<double>(p.y));
    off_z = std::min(off_z, static_cast<double>(p.z));
  }
  if (cloud.points.empty()) {
    off_x = off_y = off_z = 0;
  }

  for (const auto &p : cloud.points) {
    if (stop.stop_requested())
      throw OperationCancelled();

    LasPoint0 p0{};
    p0.x = static_cast<std::int32_t>(std::round(
        (static_cast<double>(p.x) - std::floor(off_x)) / sx));
    p0.y = static_cast<std::int32_t>(std::round(
        (static_cast<double>(p.y) - std::floor(off_y)) / sy));
    p0.z = static_cast<std::int32_t>(std::round(
        (static_cast<double>(p.z) - std::floor(off_z)) / sz));
    p0.intensity = static_cast<std::uint16_t>(
        std::clamp(static_cast<float>(p.intensity), 0.0F, 1.0F) * 65535.0F);
    p0.classification = (cloud.has_noise && p.noise != 0) ? 7 : 0;

    output.write(reinterpret_cast<const char *>(&p0), sizeof(p0));
    if (!output)
      throw std::runtime_error("LAS write error: point data: " +
                               displayPath(path));

    if (any_rgb) {
      LasRGB rgb{};
      rgb.r = static_cast<std::uint16_t>(p.r) << 8;
      rgb.g = static_cast<std::uint16_t>(p.g) << 8;
      rgb.b = static_cast<std::uint16_t>(p.b) << 8;
      output.write(reinterpret_cast<const char *>(&rgb), sizeof(rgb));
      if (!output)
        throw std::runtime_error("LAS write error: RGB: " + displayPath(path));
    }
  }
}

} // namespace

void loadLas(const std::filesystem::path &path, PointCloudIRGB &cloud,
             bool &has_color, bool &has_intensity, std::stop_token stop) {
  std::ifstream input(path, std::ios::binary);
  if (!input)
    throw std::runtime_error("file not found: " + displayPath(path));
  loadLas(input, path, cloud, has_color, has_intensity, stop);
}

void loadLas(std::istream &input, const std::filesystem::path &path,
             PointCloudIRGB &cloud, bool &has_color, bool &has_intensity,
             std::stop_token stop) {
  LasHeader12 header;
  readHeader(input, header, path);
  if (header.offset_to_point_data > sizeof(header))
    skipVlrs(input, header.num_vlrs, path);
  input.seekg(header.offset_to_point_data, std::ios::beg);
  loadPoints(input, header, cloud, has_color, has_intensity, stop, path);
}

void saveLas(std::ostream &output, const std::filesystem::path &path,
             const PointCloudIRGB &cloud, std::stop_token stop) {
  writeHeader(output, cloud, path);
  writePoints(output, cloud, stop, path);
}

} // namespace kpt::io_detail
