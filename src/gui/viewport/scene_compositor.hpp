#pragma once

#include "gui/scene/render_adapter.hpp"

#include <memory>
#include <optional>
#include <stop_token>

namespace kpt::gui {

// CPU-side bridge from the review Scene to renderer-owned layer buffers. It
// applies each layer's local-to-world transform and preserves the
// SceneRenderAdapter's deterministic opaque/transparent ordering. Opacity is
// carried as a separate draw value; it is never baked against a background.
struct SceneCompositeOptions {
  // Retained only for source compatibility with callers that used the former
  // flattened compositor. Layered rendering ignores it by design.
  Eigen::Vector3f background = Eigen::Vector3f::Zero();
  std::optional<LayerId> only_layer;
};

// Produces the concrete native multi-pass payload. `camera_cloud` contains at
// most 100,000 actual world-space ROI samples plus exact aggregate bounds for
// fit/picking; it is never a full duplicate of GPU layer payloads.
[[nodiscard]] std::shared_ptr<const LayeredViewportSnapshot>
composeLayeredSceneViewportSnapshot(const LayerRenderList &render_list,
                                    std::uint64_t revision,
                                    const SceneCompositeOptions &options = {},
                                    std::stop_token stop = {});

// Builds only the bounded, real point sample used for camera fitting.  This is
// useful for "Fit active" without flattening/rendering every visible layer.
// Samples are chosen after the same transform + closed world ROI predicate as
// native layer uploads, so its 95th-percentile FOV cannot be dominated by
// rejected AABB corners.
[[nodiscard]] std::shared_ptr<const ViewportCloudSnapshot>
composeSceneFitViewportSnapshot(const LayerRenderList &render_list,
                                std::uint64_t revision,
                                const SceneCompositeOptions &options = {},
                                std::stop_token stop = {});

// Compatibility helper for camera probes and old single-cloud callers. It
// deliberately does not emulate alpha by mixing colours with `background`.
// Native review drawing must use composeLayeredSceneViewportSnapshot.
[[nodiscard]] std::shared_ptr<const ViewportCloudSnapshot>
composeSceneViewportSnapshot(const LayerRenderList &render_list,
                             std::uint64_t revision,
                             const SceneCompositeOptions &options = {},
                             std::stop_token stop = {});

} // namespace kpt::gui
