#include "gui/viewport/cloud_adapter.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iterator>
#include <limits>
#include <vector>

namespace kpt::gui {
namespace {

void computeIntensityPercentiles(const std::vector<float> &values, float &lo,
                                 float &hi) {
  if (values.empty()) {
    lo = 0.0F;
    hi = 1.0F;
    return;
  }
  auto sorted = values;
  const std::size_t n = sorted.size();
  const std::size_t idx_lo =
      static_cast<std::size_t>(static_cast<double>(n - 1) * 0.05);
  const std::size_t idx_hi =
      static_cast<std::size_t>(static_cast<double>(n - 1) * 0.90);
  if (idx_lo == idx_hi) {
    lo = sorted.front();
    hi = sorted.back();
    return;
  }
  const auto lo_it = std::next(
      sorted.begin(), static_cast<std::ptrdiff_t>(idx_lo));
  const auto hi_it = std::next(
      sorted.begin(), static_cast<std::ptrdiff_t>(idx_hi));
  std::nth_element(sorted.begin(), lo_it, sorted.end());
  lo = sorted[idx_lo];
  std::nth_element(lo_it, hi_it, sorted.end());
  hi = sorted[idx_hi];
  if (lo >= hi) {
    lo = sorted.front();
    hi = sorted.back();
  }
}

// Builds a 256-entry cumulative distribution table over [value_min, value_max].
// cdf[i] is the fraction of samples strictly below the i-th bin edge, mapped to
// [0, 1]. Returns false when the distribution is degenerate (empty or a single
// distinct value); in that case cdf is filled with an identity ramp so callers
// can safely sample it and fall back to the linear path via the valid flag.
bool buildIntensityCdf(const std::vector<float> &sorted_values, float value_min,
                       float value_max, std::array<float, 256> &cdf) {
  for (std::size_t i = 0; i < cdf.size(); ++i)
    cdf[i] = static_cast<float>(i) / static_cast<float>(cdf.size() - 1);
  if (sorted_values.empty() || !(value_min < value_max))
    return false;
  const double span =
      static_cast<double>(value_max) - static_cast<double>(value_min);
  const std::size_t n = sorted_values.size();
  // One pass: each bin edge's cumulative count = number of samples < edge.
  // Lower bound search keeps the ramp monotonic non-decreasing.
  auto it = sorted_values.begin();
  for (std::size_t i = 0; i < cdf.size(); ++i) {
    const double edge =
        static_cast<double>(value_min) +
        span * static_cast<double>(i) / static_cast<double>(cdf.size() - 1);
    // cdf[i] = fraction of samples with value <= edge. upper_bound gives the
    // first position strictly greater than edge, so its offset is that count.
    it = std::upper_bound(it, sorted_values.end(), static_cast<float>(edge));
    const std::size_t count =
        static_cast<std::size_t>(std::distance(sorted_values.begin(), it));
    cdf[i] =
        static_cast<float>(static_cast<double>(count) / static_cast<double>(n));
  }
  cdf.back() = 1.0F;
  return true;
}

CloudBounds finishBounds(const Eigen::Vector3f &minimum,
                         const Eigen::Vector3f &maximum,
                         const Eigen::Vector3d &position_sum,
                         std::size_t finite_points, float intensity_min,
                         float intensity_max, float intensity_lo,
                         float intensity_hi,
                         const std::array<float, 256> &intensity_cdf,
                         bool intensity_cdf_valid, bool has_noise,
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
  bounds.centroid =
      (position_sum / static_cast<double>(finite_points)).cast<float>();
  bounds.center = center.cast<float>();
  const double radius = extent.norm() * 0.5;
  bounds.radius = radius > 0.0 ? radius : 0.001;
  bounds.z_min = minimum.z();
  bounds.z_max = maximum.z();
  if (intensity_min <= intensity_max) {
    bounds.intensity_min = intensity_min;
    bounds.intensity_max = intensity_max;
    bounds.intensity_p05 = intensity_lo;
    bounds.intensity_p90 = intensity_hi;
    bounds.intensity_cdf = intensity_cdf;
    bounds.intensity_cdf_valid = intensity_cdf_valid;
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
  Eigen::Vector3d position_sum = Eigen::Vector3d::Zero();

  for (const auto &point : cloud) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
        !std::isfinite(point.z)) {
      continue;
    }
    const Eigen::Vector3f position(point.x, point.y, point.z);
    minimum = minimum.cwiseMin(position);
    maximum = maximum.cwiseMax(position);
    position_sum += position.cast<double>();
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

  std::sort(intensities.begin(), intensities.end());
  std::array<float, 256> intensity_cdf{};
  const bool intensity_cdf_valid = buildIntensityCdf(
      intensities, intensity_min, intensity_max, intensity_cdf);

  return finishBounds(minimum, maximum, position_sum, finite_points, intensity_min,
                      intensity_max, lo, hi, intensity_cdf, intensity_cdf_valid,
                      cloud.has_noise, noise_points);
}

std::shared_ptr<const ViewportCloudSnapshot>
makeViewportCloudSnapshot(const PointCloudIRGBConstPtr &cloud,
                          std::uint64_t request_generation,
                          std::stop_token stop) {
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
  Eigen::Vector3d position_sum = Eigen::Vector3d::Zero();

  std::size_t visited = 0;
  for (const auto &point : *cloud) {
    if ((visited++ % 4096U) == 0U && stop.stop_requested())
      return snapshot;
    if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
        !std::isfinite(point.z)) {
      continue;
    }

    const Eigen::Vector3f position(point.x, point.y, point.z);
    minimum = minimum.cwiseMin(position);
    maximum = maximum.cwiseMax(position);
    position_sum += position.cast<double>();
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

  if (stop.stop_requested())
    return snapshot;
  float lo, hi;
  computeIntensityPercentiles(intensities, lo, hi);

  std::sort(intensities.begin(), intensities.end());
  if (stop.stop_requested())
    return snapshot;
  std::array<float, 256> intensity_cdf{};
  const bool intensity_cdf_valid = buildIntensityCdf(
      intensities, intensity_min, intensity_max, intensity_cdf);

  snapshot->bounds =
      finishBounds(minimum, maximum, position_sum, snapshot->vertices.size(), intensity_min,
                   intensity_max, lo, hi, intensity_cdf, intensity_cdf_valid,
                   cloud->has_noise, noise_points);
  constexpr std::size_t kMaximumPickingCandidates = 100'000U;
  const auto picking_count =
      std::min(snapshot->vertices.size(), kMaximumPickingCandidates);
  snapshot->picking_vertices.reserve(picking_count);
  for (std::size_t index = 0; index < picking_count; ++index) {
    const auto source = (index * snapshot->vertices.size()) / picking_count;
    snapshot->picking_vertices.push_back(snapshot->vertices[source]);
  }
  return snapshot;
}

} // namespace kpt::gui
