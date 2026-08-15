#include "gui/roi_filter.hpp"

#include <cmath>
#include <stdexcept>

namespace kpt::gui {
namespace {

[[nodiscard]] bool finiteAffine(const Eigen::Affine3d &transform) noexcept {
  const Eigen::Vector4d expected_bottom_row{0.0, 0.0, 0.0, 1.0};
  return transform.matrix().allFinite() &&
         (transform.matrix().row(3).array() ==
          expected_bottom_row.transpose().array())
             .all();
}

[[nodiscard]] bool finitePointTPosition(const Eigen::Vector3d &point) noexcept {
  for (Eigen::Index axis = 0; axis < point.size(); ++axis) {
    const float coordinate = static_cast<float>(point[axis]);
    if (!std::isfinite(coordinate)) {
      return false;
    }
  }
  return true;
}

} // namespace

PointCloudIRGB filterCloudToWorldRoi(const PointCloudIRGB &local_cloud,
                                     const Eigen::Affine3d &local_to_world,
                                     const RoiBox &world_roi) {
  if (!finiteAffine(local_to_world)) {
    throw std::invalid_argument("ROI filter requires a finite affine transform");
  }

  PointCloudIRGB filtered = local_cloud;
  filtered.points.clear();
  filtered.reserve(local_cloud.size());

  for (const PointT &local_point : local_cloud.points) {
    const Eigen::Vector3d local{static_cast<double>(local_point.x),
                                static_cast<double>(local_point.y),
                                static_cast<double>(local_point.z)};
    const auto world = transformLocalToWorld(local, local_to_world);
    if (!world.has_value() || !world_roi.contains(*world) ||
        !finitePointTPosition(*world)) {
      continue;
    }

    PointT transformed = local_point;
    transformed.x = static_cast<float>((*world).x());
    transformed.y = static_cast<float>((*world).y());
    transformed.z = static_cast<float>((*world).z());
    filtered.points.push_back(transformed);
  }

  filtered.width = filtered.points.size();
  filtered.height = 1;
  return filtered;
}

} // namespace kpt::gui
