#include "gui/viewport/cloud_adapter.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace kpt::gui {

CloudBounds calculateBounds(const PointCloudIRGB &cloud) {
  // Iterate directly over the cloud; avoid the full-point deep copy that the
  // snapshot path would perform just to discard it.
  Eigen::Vector3f minimum =
      Eigen::Vector3f::Constant(std::numeric_limits<float>::max());
  Eigen::Vector3f maximum =
      Eigen::Vector3f::Constant(std::numeric_limits<float>::lowest());
  float intensity_min = std::numeric_limits<float>::max();
  float intensity_max = std::numeric_limits<float>::lowest();
  std::size_t finite_points = 0;

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
    }
    ++finite_points;
  }

  CloudBounds bounds{};
  bounds.finite_points = finite_points;
  if (finite_points == 0)
    return bounds;
  bounds.minimum = minimum;
  bounds.maximum = maximum;
  bounds.center = (minimum + maximum) * 0.5F;
  bounds.radius = std::max((maximum - minimum).norm() * 0.5F, 0.001F);
  bounds.z_min = minimum.z();
  bounds.z_max = maximum.z();
  if (intensity_min <= intensity_max) {
    bounds.intensity_min = intensity_min;
    bounds.intensity_max = intensity_max;
  }
  return bounds;
}

std::shared_ptr<const ViewportCloudSnapshot>
makeViewportCloudSnapshot(const PointCloudIRGBConstPtr &cloud,
                          std::uint64_t request_generation) {
  auto snapshot = std::make_shared<ViewportCloudSnapshot>();
  snapshot->revision = request_generation;
  if (request_generation == 0 || !cloud) {
    return snapshot;
  }

  snapshot->vertices.reserve(cloud->size());
  Eigen::Vector3f minimum =
      Eigen::Vector3f::Constant(std::numeric_limits<float>::max());
  Eigen::Vector3f maximum =
      Eigen::Vector3f::Constant(std::numeric_limits<float>::lowest());
  float intensity_min = std::numeric_limits<float>::max();
  float intensity_max = std::numeric_limits<float>::lowest();

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
    }

    snapshot->vertices.push_back(
        {position,
         {static_cast<float>(point.r) / 255.0F,
          static_cast<float>(point.g) / 255.0F,
          static_cast<float>(point.b) / 255.0F},
         std::isfinite(point.intensity) ? point.intensity : 0.0F});
  }

  auto &bounds = snapshot->bounds;
  bounds.finite_points = snapshot->vertices.size();
  if (snapshot->vertices.empty()) {
    return snapshot;
  }

  bounds.minimum = minimum;
  bounds.maximum = maximum;
  bounds.center = (minimum + maximum) * 0.5F;
  bounds.radius = std::max((maximum - minimum).norm() * 0.5F, 0.001F);
  bounds.z_min = minimum.z();
  bounds.z_max = maximum.z();
  if (intensity_min <= intensity_max) {
    bounds.intensity_min = intensity_min;
    bounds.intensity_max = intensity_max;
  }
  return snapshot;
}

} // namespace kpt::gui
