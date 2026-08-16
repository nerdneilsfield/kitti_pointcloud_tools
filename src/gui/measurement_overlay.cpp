#include "gui/measurement_overlay.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <optional>
#include <string>
#include <string_view>

namespace kpt::gui {
namespace {

struct ProjectedEndpoint {
  Eigen::Vector2f normalized_position = Eigen::Vector2f::Zero();
  Eigen::Vector3f world_position = Eigen::Vector3f::Zero();
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
  return ProjectedEndpoint{*projected, world_point.cast<float>(),
                           layer == nullptr};
}

void appendMarker(MeasurementOverlay &overlay, const Measurement &measurement,
                  const ProjectedEndpoint &endpoint, bool second_endpoint,
                  bool pending) {
  overlay.markers.push_back({measurement.id(), endpoint.normalized_position,
                             second_endpoint, endpoint.detached, pending});
}

void appendGuideLine(std::vector<ViewportLineVertex> &guides,
                     const Eigen::Vector3f &first,
                     const Eigen::Vector3f &second,
                     const Eigen::Vector3f &colour) {
  if (!first.allFinite() || !second.allFinite())
    return;
  guides.push_back({first, colour});
  guides.push_back({second, colour});
}

void appendGuideMarker(std::vector<ViewportLineVertex> &guides,
                       const ProjectedEndpoint &endpoint, float radius,
                       const Eigen::Vector3f &colour) {
  if (!std::isfinite(radius) || radius <= 0.0F)
    return;
  // Three axes keep an asterisk visible from every camera direction while
  // retaining a single depth-tested line primitive contract.
  const std::array<Eigen::Vector3f, 3> axes = {
      Eigen::Vector3f{1.0F, 0.0F, 0.0F},
      Eigen::Vector3f{0.0F, 1.0F, 0.0F},
      Eigen::Vector3f{0.0F, 0.0F, 1.0F},
  };
  for (const Eigen::Vector3f &axis : axes) {
    appendGuideLine(guides, endpoint.world_position - axis * radius,
                    endpoint.world_position + axis * radius, colour);
  }
}

const Eigen::Vector3f &guideColour(bool detached, bool pending) {
  static const Eigen::Vector3f attached{0.36F, 0.82F, 1.0F};
  static const Eigen::Vector3f detached_colour{1.0F, 0.68F, 0.25F};
  static const Eigen::Vector3f pending_colour{1.0F, 0.84F, 0.30F};
  if (detached)
    return detached_colour;
  return pending ? pending_colour : attached;
}

} // namespace

std::vector<ViewportLineVertex>
buildMeasurementRenderGuides(const Scene &scene, const ViewportFrame &frame) {
  std::vector<ViewportLineVertex> guides;
  if (!std::isfinite(frame.world_scale) || frame.world_scale <= 0.0F)
    return guides;

  // ViewportModel rebases distance to roughly one. A constant normalized
  // radius therefore stays legible across source coordinate scales.
  constexpr float marker_radius_normalized = 0.015F;
  const float marker_radius = marker_radius_normalized / frame.world_scale;
  if (!std::isfinite(marker_radius) || marker_radius <= 0.0F)
    return guides;

  guides.reserve(scene.measurements().size() * 14U);
  for (const Measurement &measurement : scene.measurements()) {
    const auto first = projectVisibleEndpoint(scene, measurement.firstSourceKey(),
                                              measurement.firstWorld(), frame);
    if (first) {
      appendGuideMarker(guides, *first, marker_radius,
                        guideColour(first->detached,
                                    !measurement.secondWorld().has_value()));
    }

    if (!measurement.secondWorld() || !measurement.secondSourceKey())
      continue;
    const auto second = projectVisibleEndpoint(
        scene, *measurement.secondSourceKey(), *measurement.secondWorld(), frame);
    if (second)
      appendGuideMarker(guides, *second, marker_radius,
                        guideColour(second->detached, false));
    // A segment is meaningful only if both source endpoints survive the same
    // layer, ROI, and frustum policy as their interactive labels.
    if (first && second) {
      appendGuideLine(guides, first->world_position, second->world_position,
                      guideColour(first->detached || second->detached, false));
    }
  }
  return guides;
}

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
