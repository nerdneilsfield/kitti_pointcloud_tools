#pragma once

#include "gui/scene/scene.hpp"
#include "gui/viewport/cloud_adapter.hpp"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <unordered_map>
#include <vector>

namespace kpt::gui {

// Bounds in review-scene world coordinates.  They are calculated from each
// layer's local bounds and finite affine transform; no renderer-local origin
// rebasing leaks through this API.
struct WorldBounds {
  Eigen::Vector3d minimum = Eigen::Vector3d::Zero();
  Eigen::Vector3d maximum = Eigen::Vector3d::Zero();
  Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
  double radius = 0.0;
  std::size_t finite_points = 0;
};

// A deterministic uniform selection over a source vertex buffer.  Backends
// can upload only selected vertices without making a second CPU cloud copy.
struct LayerVertexSelection {
  std::size_t source_vertex_count = 0;
  std::size_t retained_vertex_count = 0;

  [[nodiscard]] bool fullResolution() const noexcept {
    return source_vertex_count == retained_vertex_count;
  }
  [[nodiscard]] std::optional<std::size_t>
  sourceIndex(std::size_t retained_index) const noexcept;
};

enum class LayerDetail { Full, UniformLod, Deferred };

enum class LayerPickScope { AllVisibleLayers, ActiveLayerOnly };

// One stable item per Scene layer.  Hidden/unloaded layers remain represented
// for layer-panel state, while draw orders contain only visible selected items.
struct LayerRenderItem {
  LayerId layer_id = 0;
  std::string source_key;
  std::shared_ptr<const ViewportCloudSnapshot> snapshot;
  Eigen::Affine3d local_to_world = Eigen::Affine3d::Identity();
  LayerStyle style;
  bool visible = true;
  LayerDetail detail = LayerDetail::Deferred;
  LayerVertexSelection vertex_selection;
  std::size_t picking_candidate_count = 0;
  std::uint64_t estimated_gpu_bytes = 0;
};

// `opaque_draw_order` and `transparent_draw_order` index `layers`.  The
// latter is back-to-front by transformed bounds centre along camera_forward.
struct LayerRenderList {
  std::vector<LayerRenderItem> layers;
  std::vector<std::size_t> opaque_draw_order;
  std::vector<std::size_t> transparent_draw_order;
  std::optional<WorldBounds> visible_world_bounds;
  // Active fit intentionally works even when that selected layer is hidden.
  std::optional<WorldBounds> active_world_bounds;
  LayerPickScope pick_scope = LayerPickScope::AllVisibleLayers;
  std::uint64_t estimated_gpu_bytes = 0;
};

// Cross-backend admission policy.  Platform code optionally provides available
// RAM; no portable VRAM query is required.  Tests/integrators can set the
// explicit budget without pretending it is a VRAM measurement.
struct LayerAdmissionConfig {
  static constexpr std::uint64_t kFallbackGpuBudgetBytes = 512ULL * 1024ULL * 1024ULL;
  static constexpr std::uint64_t kMaximumGpuBudgetBytes = 1024ULL * 1024ULL * 1024ULL;

  std::optional<std::uint64_t> available_system_memory_bytes;
  std::optional<std::uint64_t> explicit_gpu_budget_bytes;

  [[nodiscard]] std::uint64_t resolvedGpuBudgetBytes() const noexcept;
};

struct SceneRenderOptions {
  LayerAdmissionConfig admission;
  Eigen::Vector3d camera_position = Eigen::Vector3d::Zero();
  Eigen::Vector3d camera_forward = -Eigen::Vector3d::UnitZ();
};

// Resolves a local single-viewport hit at the Scene boundary.  Measurements
// must consume `world_position`, never the compatibility `world_position`
// member on PickResult (which is still viewport-local for legacy callers).
struct LayerPickResult {
  LayerId layer_id = 0;
  std::string source_key;
  Eigen::Vector3f local_position = Eigen::Vector3f::Zero();
  Eigen::Vector3d world_position = Eigen::Vector3d::Zero();
  float intensity = 0.0F;
  float noise = 0.0F;
};

// Pure Scene-to-render adapter.  It owns no renderer and performs no GPU work;
// the native app can keep its existing single-cloud path until a backend adopts
// LayerRenderList.  Snapshots arrive from the existing async cloud adapter.
class SceneRenderAdapter {
public:
  static constexpr std::size_t kMaximumAllLayerPickLayers = 4;
  static constexpr std::size_t kMaximumPickingCandidatesPerLayer = 100'000U;

  // Older asynchronous results cannot replace a newer accepted snapshot.
  [[nodiscard]] bool
  acceptSnapshot(LayerId layer_id,
                 std::shared_ptr<const ViewportCloudSnapshot> snapshot);
  void removeSnapshot(LayerId layer_id) noexcept;
  void pruneMissingLayers(const Scene &scene) noexcept;

  [[nodiscard]] LayerRenderList
  build(const Scene &scene,
        const SceneRenderOptions &options = {}) const;

  [[nodiscard]] static std::optional<WorldBounds>
  transformBounds(const CloudBounds &local_bounds,
                  const Eigen::Affine3d &local_to_world) noexcept;
  [[nodiscard]] static std::optional<LayerPickResult>
  resolvePick(const Scene &scene, LayerId layer_id,
              const PickResult &local_pick);

private:
  std::unordered_map<LayerId, std::shared_ptr<const ViewportCloudSnapshot>>
      snapshots_;
};

} // namespace kpt::gui
