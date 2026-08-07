#include "kpt/io/obj_codec.hpp"
#include "kpt/cancellation.hpp"
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

} // namespace

void loadObj(std::istream &input, const std::filesystem::path &path,
             PointCloudIRGB &cloud, bool &has_color, std::stop_token stop) {
  std::string line;
  std::size_t line_number = 0;

  cloud.clear();
  cloud.has_noise = false;
  has_color = false;

  while (std::getline(input, line)) {
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
    std::istringstream ss{std::string(sv)};
    double x, y, z;
    if (!(ss >> x >> y >> z))
      continue;

    PointT pt;
    pt.x = static_cast<float>(x);
    pt.y = static_cast<float>(y);
    pt.z = static_cast<float>(z);

    double r = 0, g = 0, b = 0;
    if (ss >> r >> g >> b) {
      if (r > 1.0 || g > 1.0 || b > 1.0) {
        pt.r = static_cast<std::uint8_t>(std::clamp(r, 0.0, 255.0));
        pt.g = static_cast<std::uint8_t>(std::clamp(g, 0.0, 255.0));
        pt.b = static_cast<std::uint8_t>(std::clamp(b, 0.0, 255.0));
      } else {
        pt.r = static_cast<std::uint8_t>(std::clamp(r * 255.0, 0.0, 255.0));
        pt.g = static_cast<std::uint8_t>(std::clamp(g * 255.0, 0.0, 255.0));
        pt.b = static_cast<std::uint8_t>(std::clamp(b * 255.0, 0.0, 255.0));
      }
      has_color = true;
    }

    cloud.points.push_back(pt);
  }

  cloud.width = cloud.points.size();
  cloud.height = 1;
}

void saveObj(std::ostream &output, const std::filesystem::path &path,
             const PointCloudIRGB &cloud, std::stop_token stop) {
  output << "# Exported by kitti_pointcloud_tools\n";
  output << "# " << cloud.size() << " vertices\n";
  output << std::setprecision(6);
  for (std::size_t i = 0; i < cloud.points.size(); ++i) {
    if (i % 10000 == 0 && stop.stop_requested())
      throw OperationCancelled();
    const auto &p = cloud.points[i];
    output << "v " << p.x << ' ' << p.y << ' ' << p.z;
    output << ' ' << (p.r / 255.0F) << ' ' << (p.g / 255.0F) << ' '
           << (p.b / 255.0F);
    output << '\n';
  }
}

} // namespace kpt::io_detail
