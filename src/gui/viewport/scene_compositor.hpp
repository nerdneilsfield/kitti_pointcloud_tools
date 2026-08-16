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

// Produces the concrete native multi-pass payload. `camera_cloud` contains the
// same world-space points for fit/picking only; it is not the render source.
[[nodiscard]] std::shared_ptr<const LayeredViewportSnapshot>
composeLayeredSceneViewportSnapshot(
    const LayerRenderList &render_list, std::uint64_t revision,
    const SceneCompositeOptions &options = {}, std::stop_token stop = {});

// Compatibility helper for camera probes and old single-cloud callers. It
// deliberately does not emulate alpha by mixing colours with `background`.
// Native review drawing must use composeLayeredSceneViewportSnapshot.
[[nodiscard]] std::shared_ptr<const ViewportCloudSnapshot>
composeSceneViewportSnapshot(const LayerRenderList &render_list,
                             std::uint64_t revision,
                             const SceneCompositeOptions &options = {},
                             std::stop_token stop = {});

} // namespace kpt::gui
