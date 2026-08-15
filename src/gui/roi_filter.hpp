#pragma once

#include "gui/scene/scene.hpp"

#include <Eigen/Geometry>

namespace kpt::gui {

// Converts accepted layer-local points to world coordinates. The ROI is a
// closed world-space AABB: every face is included. Invalid input positions and
// transforms whose result cannot be represented as finite PointT coordinates
// are omitted. A non-finite or projective transform is a programming error and
// throws std::invalid_argument rather than silently exporting corrupted data.
//
// The result preserves each accepted point's RGB, intensity and noise fields,
// along with cloud.has_noise and viewpoint metadata. It is unorganized
// (width == size(), height == 1) because ROI selection can create holes.
[[nodiscard]] PointCloudIRGB filterCloudToWorldRoi(
    const PointCloudIRGB &local_cloud,
    const Eigen::Affine3d &local_to_world,
    const RoiBox &world_roi);

} // namespace kpt::gui
