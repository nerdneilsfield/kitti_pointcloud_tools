#include "kpt/io/obj_codec.hpp"
#include "kpt/cancellation.hpp"
#include "kpt/io/codec_limits.hpp"
#include "kpt/io/ascii_float_parser.hpp"
#include "platform/utf8_path.hpp"

#include <spdlog/spdlog.h>

#include <cmath>
#include <sstream>
#include <stdexcept>
#include <string>

namespace kpt::io_detail {
namespace {

std::string displayPath(const std::filesystem::path &path) {
  auto converted = platform::pathToUtf8(path);
  return converted ? std::move(converted).value() : "<invalid-native-path>";
}

bool readBoundedLine(std::istream &input, std::string &line,
                     const std::filesystem::path &path) {
  line.clear();
  char value = '\0';
  while (input.get(value)) {
    if (value == '\n')
      return true;
    if (line.size() == kMaxTextLineBytes)
      throw std::runtime_error("OBJ parse error: line exceeds 64 KiB: " +
                               displayPath(path));
    line.push_back(value);
  }
  if (input.bad())
    throw std::runtime_error("OBJ read error: " + displayPath(path));
  return !line.empty();
}

bool containsNonFiniteNumber(std::string_view line) {
  while (!line.empty()) {
    while (!line.empty() && (line.front() == ' ' || line.front() == '\t'))
      line.remove_prefix(1);
    const auto end = line.find_first_of(" \t\r");
    const auto token = line.substr(0, end);
    double value = 0.0;
    const auto parsed = parseAsciiFloating(token, value);
    if (parsed.ec == std::errc{} && parsed.ptr == token.data() + token.size() &&
        !std::isfinite(value))
      return true;
    if (end == std::string_view::npos)
      break;
    line.remove_prefix(end);
  }
  return false;
}

} // namespace

void loadObj(std::istream &input, const std::filesystem::path &path,
             PointCloudIRGB &cloud, bool &has_color, std::stop_token stop) {
  std::string line;
  std::size_t line_number = 0;

  PointCloudIRGB parsed;
  parsed.has_noise = false;
  bool parsed_has_color = false;

  while (readBoundedLine(input, line, path)) {
    ++line_number;
    if (line_number % 10000 == 0 && stop.stop_requested())
      throw OperationCancelled();

    auto sv = std::string_view(line);
    while (!sv.empty() && (sv.front() == ' ' || sv.front() == '\t' ||
                           sv.front() == '\r'))
      sv.remove_prefix(1);
    if (sv.empty() || sv.front() == '#')
      continue;

    if (sv.size() < 2 || sv[0] != 'v' || (sv[1] != ' ' && sv[1] != '\t'))
      continue;

    sv.remove_prefix(2);
    if (containsNonFiniteNumber(sv))
      throw std::runtime_error("OBJ parse error: non-finite point value: " +
                               displayPath(path));
    std::istringstream ss{std::string(sv)};
    double x, y, z;
    if (!(ss >> x >> y >> z))
      continue;
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z))
      throw std::runtime_error("OBJ parse error: non-finite position: " +
                               displayPath(path));

    PointT pt;
    pt.x = static_cast<float>(x);
    pt.y = static_cast<float>(y);
    pt.z = static_cast<float>(z);
    if (!std::isfinite(pt.x) || !std::isfinite(pt.y) ||
        !std::isfinite(pt.z))
      throw std::runtime_error("OBJ parse error: position exceeds float "
                               "range: " +
                               displayPath(path));

    double r = 0, g = 0, b = 0;
    if (ss >> r >> g >> b) {
      if (!std::isfinite(r) || !std::isfinite(g) || !std::isfinite(b))
        throw std::runtime_error("OBJ parse error: non-finite color: " +
                                 displayPath(path));
      if (r > 1.0 || g > 1.0 || b > 1.0) {
        pt.r = static_cast<std::uint8_t>(std::clamp(r, 0.0, 255.0));
        pt.g = static_cast<std::uint8_t>(std::clamp(g, 0.0, 255.0));
        pt.b = static_cast<std::uint8_t>(std::clamp(b, 0.0, 255.0));
      } else {
        pt.r = static_cast<std::uint8_t>(std::clamp(r * 255.0, 0.0, 255.0));
        pt.g = static_cast<std::uint8_t>(std::clamp(g * 255.0, 0.0, 255.0));
        pt.b = static_cast<std::uint8_t>(std::clamp(b * 255.0, 0.0, 255.0));
      }
      parsed_has_color = true;
    }

    if (parsed.size() == kMaxPointCount)
      throw std::runtime_error("OBJ parse error: point count exceeds limit: " +
                               displayPath(path));
    parsed.points.push_back(pt);
  }

  parsed.width = parsed.points.size();
  parsed.height = 1;
  cloud = std::move(parsed);
  has_color = parsed_has_color;
}

void saveObj(std::ostream &output, const std::filesystem::path &path,
             const PointCloudIRGB &cloud, std::stop_token stop) {
  if (cloud.size() > kMaxPointCount)
    throw std::runtime_error("OBJ write error: point count exceeds limit: " +
                             displayPath(path));
  output << "# Exported by kitti_pointcloud_tools\n";
  output << "# " << cloud.size() << " vertices\n";
  output << std::setprecision(6);
  for (std::size_t i = 0; i < cloud.points.size(); ++i) {
    if (i % 10000 == 0 && stop.stop_requested())
      throw OperationCancelled();
    const auto &p = cloud.points[i];
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
      throw std::runtime_error("OBJ write error: non-finite point: " +
                               displayPath(path));
    output << "v " << p.x << ' ' << p.y << ' ' << p.z;
    output << ' ' << (p.r / 255.0F) << ' ' << (p.g / 255.0F) << ' '
           << (p.b / 255.0F);
    output << '\n';
  }
  if (!output)
    throw std::runtime_error("OBJ write error: data: " + displayPath(path));
}

} // namespace kpt::io_detail
