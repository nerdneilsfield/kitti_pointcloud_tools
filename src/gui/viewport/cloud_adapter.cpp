#include "gui/viewport/cloud_adapter.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

namespace kpt::gui {
namespace {

void computeIntensityPercentiles(const std::vector<float> &values,
                                 float &lo, float &hi) {
  if (values.empty()) {
    lo = 0.0F;
    hi = 1.0F;
    return;
  }
  auto sorted = values;
  const std::size_t n = sorted.size();
  const std::size_t idx_lo = static_cast<std::size_t>(
      static_cast<double>(n - 1) * 0.05);
  const std::size_t idx_hi = static_cast<std::size_t>(
      static_cast<double>(n - 1) * 0.90);
  if (idx_lo == idx_hi) {
    lo = sorted.front();
    hi = sorted.back();
    return;
  }
  std::nth_element(sorted.begin(), sorted.begin() + idx_lo, sorted.end());
  lo = sorted[idx_lo];
  std::nth_element(sorted.begin() + idx_lo, sorted.begin() + idx_hi,
                   sorted.end());
  hi = sorted[idx_hi];
  if (lo >= hi) {
    lo = sorted.front();
    hi = sorted.back();
  }
}

CloudBounds finishBounds(const Eigen::Vector3f &minimum,
                         const Eigen::Vector3f &maximum,
                         std::size_t finite_points, float intensity_min,
                         float intensity_max, float intensity_lo,
                         float intensity_hi, bool has_noise,
                         std::size_t noise_points) {
  CloudBounds bounds{};
  bounds.finite_points = finite_points;
  bounds.has_noise = has_noise;
  bounds.noise_points = noise_points;
  if (finite_points == 0)
    return bounds;

  const Eigen::Vector3d minimum_double = minimum.cast<double>();
  const Eigen::Vector3d extent = maximum.cast<double>() - minimum_double;
  const Eigen::Vector3d center = minimum_double + extent * 0.5;

  bounds.minimum = minimum;
  bounds.maximum = maximum;
  bounds.center = center.cast<float>();
  const double radius = extent.norm() * 0.5;
  bounds.radius = radius > 0.0 ? radius : 0.001;
  bounds.z_min = minimum.z();
  bounds.z_max = maximum.z();
  if (intensity_min <= intensity_max) {
    bounds.intensity_min = intensity_min;
    bounds.intensity_max = intensity_max;
    bounds.intensity_p1 = intensity_lo;
    bounds.intensity_p99 = intensity_hi;
  }
  return bounds;
}

} // namespace

CloudBounds calculateBounds(const PointCloudIRGB &cloud) {
  Eigen::Vector3f minimum =
      Eigen::Vector3f::Constant(std::numeric_limits<float>::max());
  Eigen::Vector3f maximum =
      Eigen::Vector3f::Constant(std::numeric_limits<float>::lowest());
  float intensity_min = std::numeric_limits<float>::max();
  float intensity_max = std::numeric_limits<float>::lowest();
  std::vector<float> intensities;
  std::size_t finite_points = 0;
  std::size_t noise_points = 0;

  for (const auto &point : cloud) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
        !std::isfinite(point.z)) {
      continue;
    }
    const Eigen::Vector3f position(point.x, point.y, point.z);
    minimum = minimum.cwiseMin(position);
    maximum = maximum.cwiseMax(position);
    if (std::isfinite(point.intensity)) {
      intensity_min = std::min(intensity_min, point.intensity);
      intensity_max = std::max(intensity_max, point.intensity);
      intensities.push_back(point.intensity);
    }
    ++finite_points;
    if (cloud.has_noise && point.noise != 0)
      ++noise_points;
  }

  float lo, hi;
  computeIntensityPercentiles(intensities, lo, hi);

  return finishBounds(minimum, maximum, finite_points, intensity_min,
                      intensity_max, lo, hi, cloud.has_noise, noise_points);
}

std::shared_ptr<const ViewportCloudSnapshot>
makeViewportCloudSnapshot(const PointCloudIRGBConstPtr &cloud,
                          std::uint64_t request_generation) {
  auto snapshot = std::make_shared<ViewportCloudSnapshot>();
  snapshot->revision = request_generation;
  if (request_generation == 0 || !cloud) {
    return snapshot;
  }
  snapshot->bounds.has_noise = cloud->has_noise;

  snapshot->vertices.reserve(cloud->size());
  Eigen::Vector3f minimum =
      Eigen::Vector3f::Constant(std::numeric_limits<float>::max());
  Eigen::Vector3f maximum =
      Eigen::Vector3f::Constant(std::numeric_limits<float>::lowest());
  float intensity_min = std::numeric_limits<float>::max();
  float intensity_max = std::numeric_limits<float>::lowest();
  std::vector<float> intensities;
  std::size_t noise_points = 0;

  for (const auto &point : *cloud) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
        !std::isfinite(point.z)) {
      continue;
    }

    const Eigen::Vector3f position(point.x, point.y, point.z);
    minimum = minimum.cwiseMin(position);
    maximum = maximum.cwiseMax(position);
    if (std::isfinite(point.intensity)) {
      intensity_min = std::min(intensity_min, point.intensity);
      intensity_max = std::max(intensity_max, point.intensity);
      intensities.push_back(point.intensity);
    }

    snapshot->vertices.push_back(
        {position,
         {static_cast<float>(point.r) / 255.0F,
          static_cast<float>(point.g) / 255.0F,
          static_cast<float>(point.b) / 255.0F},
         std::isfinite(point.intensity) ? point.intensity : 0.0F,
         cloud->has_noise && point.noise != 0 ? 1.0F : 0.0F});
    if (cloud->has_noise && point.noise != 0)
      ++noise_points;
  }

  if (snapshot->vertices.empty()) {
    return snapshot;
  }

  float lo, hi;
  computeIntensityPercentiles(intensities, lo, hi);

  snapshot->bounds =
      finishBounds(minimum, maximum, snapshot->vertices.size(), intensity_min,
                   intensity_max, lo, hi, cloud->has_noise, noise_points);
  return snapshot;
}

} // namespace kpt::gui
