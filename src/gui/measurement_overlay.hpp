#pragma once

#include "gui/scene/scene.hpp"
#include "gui/viewport/render_types.hpp"

#include <Eigen/Core>

#include <vector>

namespace kpt::gui {

// Normalized viewport-space primitives for a persistent world-space
// Measurement.  They deliberately contain no ImGui types so visibility and
// projection semantics remain unit-testable independently from a renderer.
struct MeasurementOverlayMarker {
  MeasurementId measurement_id = 0;
  Eigen::Vector2f normalized_position = Eigen::Vector2f::Zero();
  bool second_endpoint = false;
  bool detached = false;
  bool pending = false;
};

struct MeasurementOverlaySegment {
  MeasurementId measurement_id = 0;
  Eigen::Vector2f first_normalized_position = Eigen::Vector2f::Zero();
  Eigen::Vector2f second_normalized_position = Eigen::Vector2f::Zero();
  double distance = 0.0;
  bool detached = false;
};

struct MeasurementOverlay {
  std::vector<MeasurementOverlayMarker> markers;
  std::vector<MeasurementOverlaySegment> segments;
};

// World-space line primitives for the viewport render pass. Unlike the ImGui
// labels built from MeasurementOverlay, these are present in the offscreen
// framebuffer and therefore survive native/WebGL screenshot capture. Their
// visibility policy is intentionally identical to buildMeasurementOverlay().
[[nodiscard]] std::vector<ViewportLineVertex>
buildMeasurementRenderGuides(const Scene &scene, const ViewportFrame &frame);

// Resolves stored world points directly: later layer transform edits cannot
// move an annotation. Hidden source layers suppress their endpoint, deleted
// sources remain as detached history, and the Scene's closed world-space ROI
// suppresses points outside the same visual filter used by point rendering.
[[nodiscard]] MeasurementOverlay buildMeasurementOverlay(
    const Scene &scene, const ViewportFrame &frame);

// World-space wireframe of an oriented ROI box, for the viewport render pass
// and the in-progress extrusion preview alike. Returns empty when the box is
// unset or would project to non-finite vertices.
[[nodiscard]] std::vector<ViewportLineVertex>
buildOrientedRoiGuides(const std::optional<OrientedRoi> &roi,
                       const ViewportFrame &frame);

// World-space wireframe of an axis-aligned ROI box, for face-drag editing.
[[nodiscard]] std::vector<ViewportLineVertex>
buildRoiBoxGuides(const std::optional<RoiBox> &roi, const ViewportFrame &frame);

} // namespace kpt::gui
