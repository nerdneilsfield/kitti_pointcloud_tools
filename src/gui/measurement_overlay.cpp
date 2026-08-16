#include "gui/measurement_overlay.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <string>
#include <string_view>

namespace kpt::gui {
namespace {

struct ProjectedEndpoint {
  Eigen::Vector2f normalized_position = Eigen::Vector2f::Zero();
  bool detached = false;
};

[[nodiscard]] std::optional<Eigen::Vector2f>
projectWorldPoint(const Eigen::Vector3d &world_point,
                  const ViewportFrame &frame) noexcept {
  constexpr float kClipTolerance = 1.0e-5F;
  constexpr double kFloatLimit =
      static_cast<double>(std::numeric_limits<float>::max());
  if (!world_point.allFinite() || !frame.view_projection.allFinite() ||
      !frame.world_origin.allFinite() || !std::isfinite(frame.world_scale) ||
      frame.world_scale <= 0.0F ||
      (world_point.array().abs() > kFloatLimit).any()) {
    return std::nullopt;
  }

  const Eigen::Vector3f render_local =
      (world_point.cast<float>() - frame.world_origin) * frame.world_scale;
  if (!render_local.allFinite()) {
    return std::nullopt;
  }
  const Eigen::Vector4f clip = frame.view_projection *
                               Eigen::Vector4f(render_local.x(),
                                               render_local.y(),
                                               render_local.z(), 1.0F);
  if (!clip.allFinite() || clip.w() <= kClipTolerance) {
    return std::nullopt;
  }
  const Eigen::Vector3f ndc = clip.head<3>() / clip.w();
  if (!ndc.allFinite() || ndc.x() < -1.0F - kClipTolerance ||
      ndc.x() > 1.0F + kClipTolerance || ndc.y() < -1.0F - kClipTolerance ||
      ndc.y() > 1.0F + kClipTolerance || ndc.z() < -1.0F - kClipTolerance ||
      ndc.z() > 1.0F + kClipTolerance) {
    return std::nullopt;
  }
  return Eigen::Vector2f{
      std::clamp(ndc.x() * 0.5F + 0.5F, 0.0F, 1.0F),
      std::clamp(0.5F - ndc.y() * 0.5F, 0.0F, 1.0F)};
}

[[nodiscard]] std::optional<ProjectedEndpoint>
projectVisibleEndpoint(const Scene &scene, std::string_view source_key,
                       const Eigen::Vector3d &world_point,
                       const ViewportFrame &frame) {
  if (const auto &roi = scene.roi(); roi && !roi->contains(world_point)) {
    return std::nullopt;
  }
  const CloudLayer *layer = scene.findLayerBySourceKey(std::string(source_key));
  if (layer != nullptr && !layer->visible()) {
    return std::nullopt;
  }
  const auto projected = projectWorldPoint(world_point, frame);
  if (!projected) {
    return std::nullopt;
  }
  return ProjectedEndpoint{*projected, layer == nullptr};
}

void appendMarker(MeasurementOverlay &overlay, const Measurement &measurement,
                  const ProjectedEndpoint &endpoint, bool second_endpoint,
                  bool pending) {
  overlay.markers.push_back({measurement.id(), endpoint.normalized_position,
                             second_endpoint, endpoint.detached, pending});
}

} // namespace

MeasurementOverlay buildMeasurementOverlay(const Scene &scene,
                                           const ViewportFrame &frame) {
  MeasurementOverlay overlay;
  for (const Measurement &measurement : scene.measurements()) {
    const auto first = projectVisibleEndpoint(scene, measurement.firstSourceKey(),
                                              measurement.firstWorld(), frame);
    if (first) {
      appendMarker(overlay, measurement, *first, false,
                   !measurement.secondWorld().has_value());
    }

    if (!measurement.secondWorld() || !measurement.secondSourceKey()) {
      continue;
    }
    const auto second = projectVisibleEndpoint(
        scene, *measurement.secondSourceKey(), *measurement.secondWorld(), frame);
    if (second) {
      appendMarker(overlay, measurement, *second, true, false);
    }
    // A segment is meaningful only when both endpoints survived exactly the
    // same layer/ROI/frustum visibility policy as their markers.
    if (first && second) {
      const auto distance = measurement.distance();
      if (distance && std::isfinite(*distance)) {
        overlay.segments.push_back({measurement.id(), first->normalized_position,
                                    second->normalized_position, *distance,
                                    first->detached || second->detached});
      }
    }
  }
  return overlay;
}

} // namespace kpt::gui
