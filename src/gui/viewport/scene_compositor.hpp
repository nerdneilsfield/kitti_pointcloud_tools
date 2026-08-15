#pragma once

#include "gui/scene/render_adapter.hpp"

#include <memory>
#include <optional>

namespace kpt::gui {

// CPU-side bridge from the review Scene to the established single-cloud
// viewport.  It applies each layer's local-to-world transform and keeps the
// SceneRenderAdapter's deterministic opaque/transparent ordering.  The
// existing renderer receives one world-space snapshot, so this bridge is also
// the one place where per-layer colour is baked for legacy backends.
//
// Opacity is intentionally approximated against `background` here.  True
// per-layer alpha requires persistent backend buffers and a multi-pass render
// API; callers must not mistake this compatibility bridge for OIT.
struct SceneCompositeOptions {
  Eigen::Vector3f background = Eigen::Vector3f::Zero();
  std::optional<LayerId> only_layer;
};

[[nodiscard]] std::shared_ptr<const ViewportCloudSnapshot>
composeSceneViewportSnapshot(const LayerRenderList &render_list,
                             std::uint64_t revision,
                             const SceneCompositeOptions &options = {});

} // namespace kpt::gui
