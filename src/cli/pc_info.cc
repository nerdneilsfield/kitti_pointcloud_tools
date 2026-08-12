#include "kpt/io/conversion_options.hpp"
#include "kpt/io/io.hpp"
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <numeric>
#include <popl.hpp>
#include <spdlog/spdlog.h>

namespace {

struct CloudStats {
  std::size_t total_points = 0;
  std::size_t finite_points = 0;
  std::size_t non_finite_points = 0;
  std::size_t noise_points = 0;
  std::size_t has_rgb = 0;
  std::size_t has_intensity = 0;

  double min_x = std::numeric_limits<double>::max();
  double min_y = std::numeric_limits<double>::max();
  double min_z = std::numeric_limits<double>::max();
  double max_x = std::numeric_limits<double>::lowest();
  double max_y = std::numeric_limits<double>::lowest();
  double max_z = std::numeric_limits<double>::lowest();

  double mean_x = 0;
  double mean_y = 0;
  double mean_z = 0;

  double cen_x = 0;
  double cen_y = 0;
  double cen_z = 0;

  double range_x = 0;
  double range_y = 0;
  double range_z = 0;

  double min_intensity = std::numeric_limits<double>::max();
  double max_intensity = std::numeric_limits<double>::lowest();
  double mean_intensity = 0;
  std::size_t finite_intensity = 0;

  double density = 0;
  double volume = 0;
};

CloudStats computeStats(const kpt::PointCloudIRGB &cloud) {
  CloudStats s;
  s.total_points = cloud.size();
  s.has_rgb = 0;
  s.has_intensity = 0;

  double sum_x = 0, sum_y = 0, sum_z = 0;
  double sum_i = 0;

  for (const auto &p : cloud.points) {
    const bool finite =
        std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
    if (!finite) {
      ++s.non_finite_points;
      continue;
    }
    ++s.finite_points;

    s.min_x = std::min(s.min_x, static_cast<double>(p.x));
    s.min_y = std::min(s.min_y, static_cast<double>(p.y));
    s.min_z = std::min(s.min_z, static_cast<double>(p.z));
    s.max_x = std::max(s.max_x, static_cast<double>(p.x));
    s.max_y = std::max(s.max_y, static_cast<double>(p.y));
    s.max_z = std::max(s.max_z, static_cast<double>(p.z));

    sum_x += p.x;
    sum_y += p.y;
    sum_z += p.z;

    if (p.r != 0 || p.g != 0 || p.b != 0)
      ++s.has_rgb;

    if (std::isfinite(p.intensity) && p.intensity != 0.0F) {
      ++s.has_intensity;
      ++s.finite_intensity;
      s.min_intensity =
          std::min(s.min_intensity, static_cast<double>(p.intensity));
      s.max_intensity =
          std::max(s.max_intensity, static_cast<double>(p.intensity));
      sum_i += p.intensity;
    }

    if (cloud.has_noise && p.noise != 0)
      ++s.noise_points;
  }

  if (s.finite_points > 0) {
    s.mean_x = sum_x / s.finite_points;
    s.mean_y = sum_y / s.finite_points;
    s.mean_z = sum_z / s.finite_points;
    s.cen_x = (s.min_x + s.max_x) * 0.5;
    s.cen_y = (s.min_y + s.max_y) * 0.5;
    s.cen_z = (s.min_z + s.max_z) * 0.5;
    s.range_x = s.max_x - s.min_x;
    s.range_y = s.max_y - s.min_y;
    s.range_z = s.max_z - s.min_z;
    s.volume = s.range_x * s.range_y * s.range_z;
    if (s.volume > 0)
      s.density = s.finite_points / s.volume;
    if (s.finite_intensity > 0)
      s.mean_intensity = sum_i / s.finite_intensity;
  } else {
    s.min_x = s.min_y = s.min_z = 0;
    s.max_x = s.max_y = s.max_z = 0;
  }

  return s;
}

void printStats(const std::string &filename, const kpt::PointCloudIRGB &cloud,
                const CloudStats &s) {
  const char *sep =
      "============================================================";

  std::cout << sep << '\n';
  std::cout << "  Point Cloud Info: " << filename << '\n';
  std::cout << sep << '\n';
  std::cout << '\n';

  std::cout << "--- Summary ---\n";
  std::cout << "  Total points:      " << s.total_points << '\n';
  std::cout << "  Finite points:     " << s.finite_points << '\n';
  std::cout << "  Non-finite points: " << s.non_finite_points << '\n';
  std::cout << "  Noise points:      " << s.noise_points
            << (cloud.has_noise ? "" : " (noise field absent)") << '\n';
  std::cout << "  Nonzero RGB:       " << s.has_rgb << " / " << s.finite_points
            << '\n';
  std::cout << "  Nonzero intensity: " << s.has_intensity << " / "
            << s.finite_points << '\n';
  std::cout << '\n';

  if (s.finite_points == 0) {
    std::cout << "  (no finite points to analyze)\n";
    std::cout << '\n' << sep << '\n';
    return;
  }

  std::cout << std::fixed << std::setprecision(4);

  std::cout << "--- Bounding Box (AABB) ---\n";
  std::cout << "           " << std::setw(14) << "X" << std::setw(16) << "Y"
            << std::setw(16) << "Z\n";
  std::cout << "  Min:    " << std::setw(14) << s.min_x << std::setw(16)
            << s.min_y << std::setw(16) << s.min_z << '\n';
  std::cout << "  Max:    " << std::setw(14) << s.max_x << std::setw(16)
            << s.max_y << std::setw(16) << s.max_z << '\n';
  std::cout << "  Size:   " << std::setw(14) << s.range_x << std::setw(16)
            << s.range_y << std::setw(16) << s.range_z << '\n';
  std::cout << "  Center: " << std::setw(14) << s.cen_x << std::setw(16)
            << s.cen_y << std::setw(16) << s.cen_z << '\n';
  std::cout << "  Mean:   " << std::setw(14) << s.mean_x << std::setw(16)
            << s.mean_y << std::setw(16) << s.mean_z << '\n';
  std::cout << '\n';

  std::cout << "--- Dimensions ---\n";
  std::cout << "  Range X: " << s.range_x << " m\n";
  std::cout << "  Range Y: " << s.range_y << " m\n";
  std::cout << "  Range Z: " << s.range_z << " m\n";
  std::cout << "  Volume:  " << s.volume << " m^3\n";
  std::cout << "  Density: " << s.density << " points/m^3\n";
  std::cout << '\n';

  if (s.finite_intensity > 0) {
    std::cout << "--- Intensity ---\n";
    std::cout << "  Min:  " << s.min_intensity << '\n';
    std::cout << "  Max:  " << s.max_intensity << '\n';
    std::cout << "  Mean: " << s.mean_intensity << '\n';
    std::cout << '\n';
  }

  std::cout << sep << '\n';
}

} // namespace

int main(int argc, char *argv[]) {
  popl::OptionParser op("kpt_info: point cloud statistics and info");
  auto help = op.add<popl::Switch>("h", "help", "help");
  auto log_level = op.add<popl::Value<int>>("l", "log-level",
                                            "0=err 1=warn 2=info 3=debug", 2);
  try {
    op.parse(argc, argv);
  } catch (const std::exception &error) {
    spdlog::error("argument error: {}", error.what());
    return 2;
  }
  if (help->is_set()) {
    std::cout << op << "\n";
    return 0;
  }
  try {
    kpt::io::validateLogLevel(log_level->value());
  } catch (const std::invalid_argument &error) {
    spdlog::error("{}", error.what());
    return 1;
  }
  switch (log_level->value()) {
  case 0:
    spdlog::set_level(spdlog::level::err);
    break;
  case 1:
    spdlog::set_level(spdlog::level::warn);
    break;
  case 2:
    spdlog::set_level(spdlog::level::info);
    break;
  case 3:
    spdlog::set_level(spdlog::level::debug);
    break;
  }

  auto positional = op.non_option_args();
  if (positional.empty()) {
    std::cerr << "usage: kpt_info <file> [file2 ...]\n";
    return 1;
  }

  bool failed = false;
  for (const auto &path : positional) {
    try {
      spdlog::info("loading {}", path);
      auto cloud = kpt::load(path);
      const auto stats = computeStats(*cloud);
      printStats(path, *cloud, stats);
    } catch (const std::exception &e) {
      spdlog::error("{}: {}", path, e.what());
      failed = true;
    }
  }

  return failed ? 1 : 0;
}
