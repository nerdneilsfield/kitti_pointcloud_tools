#include "kpt/io/pts_codec.hpp"
#include "kpt/cancellation.hpp"
#include "platform/utf8_path.hpp"

#include <spdlog/spdlog.h>

#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>

namespace kpt::io_detail {
namespace {

std::string displayPath(const std::filesystem::path &path) {
  auto converted = platform::pathToUtf8(path);
  return converted ? std::move(converted).value() : "<invalid-native-path>";
}

struct PtsRow {
  float vals[7];
  int count;
};

bool parsePtsLine(std::string_view line, PtsRow &out) {
  out.count = 0;
  std::istringstream ss{std::string(line)};
  for (int i = 0; i < 7; ++i) {
    double v;
    if (!(ss >> v))
      break;
    out.vals[i] = static_cast<float>(v);
    ++out.count;
  }
  return out.count >= 3;
}

} // namespace

void loadPts(std::istream &input, const std::filesystem::path &path,
             PointCloudIRGB &cloud, bool &has_color, bool &has_intensity,
             std::stop_token stop) {
  std::string line;
  std::size_t line_number = 0;
  std::size_t skipped = 0;
  bool first_line = true;

  cloud.clear();
  cloud.has_noise = false;

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

    PtsRow row;
    if (!parsePtsLine(sv, row)) {
      if (++skipped <= 50)
        spdlog::warn("skip {}:{}: need >= 3 numeric columns",
                     displayPath(path), line_number);
      continue;
    }

    if (first_line) {
      first_line = false;
      if (row.count == 1) {
        continue;
      }
      has_intensity = (row.count >= 4);
      has_color = (row.count >= 6);
    }

    PointT pt;
    pt.x = row.vals[0];
    pt.y = row.vals[1];
    pt.z = row.vals[2];
    if (row.count >= 4)
      pt.intensity = row.vals[3];
    if (row.count >= 7) {
      pt.r = static_cast<std::uint8_t>(
          std::clamp(row.vals[4], 0.0F, 255.0F));
      pt.g = static_cast<std::uint8_t>(
          std::clamp(row.vals[5], 0.0F, 255.0F));
      pt.b = static_cast<std::uint8_t>(
          std::clamp(row.vals[6], 0.0F, 255.0F));
    }
    cloud.points.push_back(pt);
  }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  if (skipped > 50)
    spdlog::warn("... {} more skipped lines in {}", skipped - 50,
                 displayPath(path));
}

void savePts(std::ostream &output, const std::filesystem::path &path,
             const PointCloudIRGB &cloud, std::stop_token stop) {
  output << cloud.size() << '\n';
  output << std::setprecision(6);
  for (std::size_t i = 0; i < cloud.points.size(); ++i) {
    if (i % 10000 == 0 && stop.stop_requested())
      throw OperationCancelled();
    const auto &p = cloud.points[i];
    output << p.x << ' ' << p.y << ' ' << p.z;
    output << ' ' << p.intensity;
    output << ' ' << static_cast<int>(p.r) << ' '
           << static_cast<int>(p.g) << ' '
           << static_cast<int>(p.b);
    output << '\n';
  }
}

} // namespace kpt::io_detail
