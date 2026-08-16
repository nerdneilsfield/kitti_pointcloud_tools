#include "gui/scene/render_adapter.hpp"

#include "kpt/cancellation.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <utility>

namespace kpt::gui {
namespace {

[[nodiscard]] bool finite(const Eigen::Vector3d &value) noexcept {
  return value.allFinite();
}

[[nodiscard]] std::uint64_t saturatingVertexBytes(std::size_t count) noexcept {
  constexpr std::uint64_t vertex_bytes = sizeof(ViewportVertex);
  if (count > std::numeric_limits<std::uint64_t>::max() / vertex_bytes) {
    return std::numeric_limits<std::uint64_t>::max();
  }
  return static_cast<std::uint64_t>(count) * vertex_bytes;
}

[[nodiscard]] std::size_t saturatingAdd(std::size_t left,
                                        std::size_t right) noexcept {
  if (left > std::numeric_limits<std::size_t>::max() - right) {
    return std::numeric_limits<std::size_t>::max();
  }
  return left + right;
}

[[nodiscard]] std::size_t budgetVertexCount(std::uint64_t bytes) noexcept {
  constexpr std::uint64_t vertex_bytes = sizeof(ViewportVertex);
  const auto count = bytes / vertex_bytes;
  return count > std::numeric_limits<std::size_t>::max()
             ? std::numeric_limits<std::size_t>::max()
             : static_cast<std::size_t>(count);
}

[[nodiscard]] std::optional<WorldBounds>
combineBounds(const std::vector<WorldBounds> &bounds) noexcept {
  if (bounds.empty()) {
    return std::nullopt;
  }

  WorldBounds combined;
  combined.minimum = bounds.front().minimum;
  combined.maximum = bounds.front().maximum;
  Eigen::Vector3d centroid_sum = Eigen::Vector3d::Zero();
  std::size_t point_count = 0;
  for (const auto &item : bounds) {
    combined.minimum = combined.minimum.cwiseMin(item.minimum);
    combined.maximum = combined.maximum.cwiseMax(item.maximum);
    centroid_sum += item.centroid * static_cast<double>(item.finite_points);
    point_count = saturatingAdd(point_count, item.finite_points);
  }
  if (point_count == 0 || !finite(centroid_sum)) {
    return std::nullopt;
  }
  combined.finite_points = point_count;
  combined.centroid = centroid_sum / static_cast<double>(point_count);
  const auto extent = combined.maximum - combined.minimum;
  combined.radius = extent.norm() * 0.5;
  return finite(combined.centroid) && std::isfinite(combined.radius)
             ? std::optional<WorldBounds>{combined}
             : std::nullopt;
}

[[nodiscard]] bool isTransparent(const LayerStyle &style) noexcept {
  return style.opacity < 1.0F;
}

[[nodiscard]] std::optional<WorldBounds> eligibleWorldBounds(
    const ViewportCloudSnapshot &snapshot,
    const Eigen::Affine3d &local_to_world,
    const std::optional<RoiBox> &world_roi, std::stop_token stop) {
  WorldBounds result;
  result.minimum = Eigen::Vector3d::Constant(
      std::numeric_limits<double>::infinity());
  result.maximum = Eigen::Vector3d::Constant(
      -std::numeric_limits<double>::infinity());
  Eigen::Vector3d centroid_sum = Eigen::Vector3d::Zero();

  std::size_t source_index = 0;
  for (const ViewportVertex &vertex : snapshot.vertices) {
    if ((source_index++ % 4096U) == 0U && stop.stop_requested()) {
      throw OperationCancelled();
    }
    const auto world = transformLocalToWorld(vertex.position.cast<double>(),
                                             local_to_world);
    if (!world.has_value() || (world_roi && !world_roi->contains(*world))) {
      continue;
    }
    result.minimum = result.minimum.cwiseMin(*world);
    result.maximum = result.maximum.cwiseMax(*world);
    centroid_sum += *world;
    result.finite_points = saturatingAdd(result.finite_points, 1);
  }
  if (result.finite_points == 0 || !finite(centroid_sum)) {
    return std::nullopt;
  }
  result.centroid = centroid_sum / static_cast<double>(result.finite_points);
  result.radius = (result.maximum - result.minimum).norm() * 0.5;
  return finite(result.minimum) && finite(result.maximum) &&
                 finite(result.centroid) && std::isfinite(result.radius)
             ? std::optional<WorldBounds>{result}
             : std::nullopt;
}

} // namespace

std::optional<std::size_t>
LayerVertexSelection::sourceIndex(std::size_t retained_index) const noexcept {
  if (requiresEligibilityScan() || source_vertex_count == 0 ||
      retained_vertex_count == 0 ||
      retained_index >= retained_vertex_count) {
    return std::nullopt;
  }

  // quotient/remainder form avoids the ordinary product overflow for normal
  // source sizes.  The long-double fallback covers theoretical size_t limits;
  // point-cloud codec limits are much smaller than either representation.
  const auto quotient = source_vertex_count / retained_vertex_count;
  const auto remainder = source_vertex_count % retained_vertex_count;
  if (remainder != 0 &&
      retained_index > std::numeric_limits<std::size_t>::max() / remainder) {
    return std::min(source_vertex_count - 1, static_cast<std::size_t>(
        (static_cast<long double>(retained_index) *
         static_cast<long double>(source_vertex_count)) /
        static_cast<long double>(retained_vertex_count)));
  }
  return quotient * retained_index +
         (remainder * retained_index) / retained_vertex_count;
}

std::uint64_t LayerAdmissionConfig::resolvedGpuBudgetBytes() const noexcept {
  if (explicit_gpu_budget_bytes.has_value()) {
    return std::min(*explicit_gpu_budget_bytes, kMaximumGpuBudgetBytes);
  }
  if (!available_system_memory_bytes.has_value() ||
      *available_system_memory_bytes == 0) {
    return kFallbackGpuBudgetBytes;
  }
  return std::min(*available_system_memory_bytes / 4U,
                  kMaximumGpuBudgetBytes);
}

bool SceneRenderAdapter::acceptSnapshot(
    LayerId layer_id, std::shared_ptr<const ViewportCloudSnapshot> snapshot) {
  if (layer_id == 0 || !snapshot) {
    return false;
  }
  const auto existing = snapshots_.find(layer_id);
  if (existing != snapshots_.end() && existing->second &&
      snapshot->revision < existing->second->revision) {
    return false;
  }
  snapshots_[layer_id] = std::move(snapshot);
  return true;
}

bool SceneRenderAdapter::hasSnapshot(LayerId layer_id) const noexcept {
  return snapshots_.contains(layer_id);
}

void SceneRenderAdapter::removeSnapshot(LayerId layer_id) noexcept {
  snapshots_.erase(layer_id);
}

void SceneRenderAdapter::clearSnapshots() noexcept { snapshots_.clear(); }

void SceneRenderAdapter::pruneMissingLayers(const Scene &scene) noexcept {
  std::erase_if(snapshots_, [&scene](const auto &entry) {
    return scene.findLayer(entry.first) == nullptr;
  });
}

SceneRenderSnapshot SceneRenderAdapter::capture(const Scene &scene) const {
  SceneRenderSnapshot result;
  const auto &scene_layers = scene.layers();
  result.layers.reserve(scene_layers.size());
  result.world_roi = scene.roi();
  result.active_layer_id = scene.activeLayer();
  for (const CloudLayer &layer : scene_layers) {
    SceneRenderSource source;
    source.layer_id = layer.id();
    source.source_key = layer.sourceKey();
    source.local_to_world = layer.localToWorld();
    source.style = layer.style();
    source.visible = layer.visible();
    if (const auto snapshot = snapshots_.find(layer.id());
        snapshot != snapshots_.end()) {
      source.snapshot = snapshot->second;
    }
    result.layers.push_back(std::move(source));
  }
  return result;
}

LayerRenderList SceneRenderAdapter::build(
    const Scene &scene, const SceneRenderOptions &options) const {
  return build(capture(scene), options);
}

LayerRenderList SceneRenderAdapter::build(
    const SceneRenderSnapshot &scene, const SceneRenderOptions &options,
    std::stop_token stop) {
  LayerRenderList result;
  result.layers.reserve(scene.layers.size());
  std::vector<std::optional<WorldBounds>> item_bounds;
  item_bounds.reserve(scene.layers.size());
  std::vector<WorldBounds> visible_bounds;
  visible_bounds.reserve(scene.layers.size());

  std::vector<std::size_t> visible_loaded_indices;
  visible_loaded_indices.reserve(scene.layers.size());
  std::size_t full_vertex_count = 0;

  for (const auto &layer : scene.layers) {
    if (stop.stop_requested()) {
      throw OperationCancelled();
    }
    LayerRenderItem item;
    item.layer_id = layer.layer_id;
    item.source_key = layer.source_key;
    item.local_to_world = layer.local_to_world;
    item.style = layer.style;
    item.world_roi = scene.world_roi;
    item.visible = layer.visible;
    item.snapshot = layer.snapshot;
    if (item.snapshot) {
      item.vertex_selection.source_vertex_count = item.snapshot->vertices.size();
      // Bounds, LOD, and fit all observe exactly the same world-space ROI
      // eligibility rule.  Transforming only the old AABB would include
      // rejected points and selecting before filtering could leave a small
      // ROI preview empty even when it contains valid points.
      const bool needs_bounds =
          item.visible || scene.active_layer_id == item.layer_id;
      const auto bounds = needs_bounds
                              ? eligibleWorldBounds(*item.snapshot,
                                                    item.local_to_world,
                                                    item.world_roi, stop)
                              : std::optional<WorldBounds>{};
      item.eligible_world_bounds = bounds;
      item_bounds.push_back(bounds);
      if (bounds.has_value()) {
        item.vertex_selection.eligible_vertex_count = bounds->finite_points;
        item.picking_candidate_count = std::min(
            bounds->finite_points, kMaximumPickingCandidatesPerLayer);
        if (item.visible) {
          visible_bounds.push_back(*bounds);
        }
        if (scene.active_layer_id == item.layer_id) {
          result.active_world_bounds = bounds;
        }
      }
      if (item.visible && item.vertex_selection.eligible_vertex_count != 0) {
        visible_loaded_indices.push_back(result.layers.size());
        full_vertex_count = saturatingAdd(
            full_vertex_count, item.vertex_selection.eligible_vertex_count);
      }
    } else {
      item_bounds.push_back(std::nullopt);
    }
    result.layers.push_back(std::move(item));
  }

  result.visible_world_bounds = combineBounds(visible_bounds);

  std::size_t budget_vertices =
      budgetVertexCount(options.admission.resolvedGpuBudgetBytes());
  if (options.maximum_render_vertices.has_value()) {
    budget_vertices = std::min(budget_vertices,
                               *options.maximum_render_vertices);
  }
  if (full_vertex_count <= budget_vertices) {
    for (const auto index : visible_loaded_indices) {
      auto &item = result.layers[index];
      item.vertex_selection.retained_vertex_count =
          item.vertex_selection.eligible_vertex_count;
      item.detail = LayerDetail::Full;
    }
  } else if (budget_vertices != 0) {
    // First grant one vertex per layer while possible.  Then distribute the
    // remaining budget proportionally, retaining every layer whenever budget
    // permits.  This is deterministic and requires no fixed layer-count cap.
    const auto granted_layers =
        std::min(budget_vertices, visible_loaded_indices.size());
    for (std::size_t index = 0; index < granted_layers; ++index) {
      result.layers[visible_loaded_indices[index]]
          .vertex_selection.retained_vertex_count = 1;
    }

    std::size_t remaining_budget = budget_vertices - granted_layers;
    std::size_t total_extra_vertices = 0;
    for (const auto index : visible_loaded_indices) {
      const auto eligible =
          result.layers[index].vertex_selection.eligible_vertex_count;
      if (eligible > 1) {
        total_extra_vertices = saturatingAdd(total_extra_vertices, eligible - 1);
      }
    }
    if (remaining_budget != 0 && total_extra_vertices != 0) {
      std::size_t distributed = 0;
      for (const auto index : visible_loaded_indices) {
        auto &selection = result.layers[index].vertex_selection;
        if (selection.retained_vertex_count == 0 ||
            selection.eligible_vertex_count <= 1) {
          continue;
        }
        const auto share = static_cast<std::size_t>(
            (static_cast<long double>(selection.eligible_vertex_count - 1) *
             static_cast<long double>(remaining_budget)) /
            static_cast<long double>(total_extra_vertices));
        const auto extra = std::min(share,
                                    selection.eligible_vertex_count -
                                        selection.retained_vertex_count);
        selection.retained_vertex_count += extra;
        distributed = saturatingAdd(distributed, extra);
      }
      remaining_budget -= std::min(remaining_budget, distributed);

      // Sum of floor shares leaves fewer than one vertex per participating
      // layer.  This bounded pass consumes the exact remainder without a
      // point-count-sized loop.
      for (const auto index : visible_loaded_indices) {
        if (remaining_budget == 0) {
          break;
        }
        auto &selection = result.layers[index].vertex_selection;
        if (selection.retained_vertex_count < selection.eligible_vertex_count) {
          ++selection.retained_vertex_count;
          --remaining_budget;
        }
      }
    }
    for (const auto index : visible_loaded_indices) {
      auto &item = result.layers[index];
      item.detail = item.vertex_selection.retained_vertex_count == 0
                        ? LayerDetail::Deferred
                        : (item.vertex_selection.fullResolution()
                               ? LayerDetail::Full
                               : LayerDetail::UniformLod);
    }
  }

  const bool valid_camera = finite(options.camera_position) &&
                            finite(options.camera_forward) &&
                            options.camera_forward.squaredNorm() > 0.0;
  for (std::size_t index = 0; index < result.layers.size(); ++index) {
    auto &item = result.layers[index];
    if (!item.visible || item.vertex_selection.retained_vertex_count == 0) {
      continue;
    }
    item.estimated_gpu_bytes =
        saturatingVertexBytes(item.vertex_selection.retained_vertex_count);
    if (std::numeric_limits<std::uint64_t>::max() - result.estimated_gpu_bytes <
        item.estimated_gpu_bytes) {
      result.estimated_gpu_bytes = std::numeric_limits<std::uint64_t>::max();
    } else {
      result.estimated_gpu_bytes += item.estimated_gpu_bytes;
    }
    if (isTransparent(item.style)) {
      result.transparent_draw_order.push_back(index);
    } else {
      result.opaque_draw_order.push_back(index);
    }
  }

  if (valid_camera) {
    const auto depth = [&item_bounds, &options](std::size_t index) {
      if (!item_bounds[index].has_value()) {
        return -std::numeric_limits<double>::infinity();
      }
      return options.camera_forward.dot(item_bounds[index]->centroid -
                                        options.camera_position);
    };
    std::stable_sort(result.transparent_draw_order.begin(),
                     result.transparent_draw_order.end(),
                     [&depth](std::size_t left, std::size_t right) {
                       return depth(left) > depth(right);
                     });
  }

  const auto visible_layer_count = static_cast<std::size_t>(std::count_if(
      result.layers.begin(), result.layers.end(), [](const LayerRenderItem &item) {
        return item.visible;
      }));
  result.pick_scope = visible_layer_count > kMaximumAllLayerPickLayers
                          ? LayerPickScope::ActiveLayerOnly
                          : LayerPickScope::AllVisibleLayers;
  return result;
}

std::optional<WorldBounds> SceneRenderAdapter::transformBounds(
    const CloudBounds &local_bounds,
    const Eigen::Affine3d &local_to_world) noexcept {
  if (local_bounds.finite_points == 0 ||
      !local_bounds.minimum.allFinite() || !local_bounds.maximum.allFinite() ||
      !local_bounds.centroid.allFinite()) {
    return std::nullopt;
  }

  const Eigen::Vector3d local_min = local_bounds.minimum.cast<double>();
  const Eigen::Vector3d local_max = local_bounds.maximum.cast<double>();
  Eigen::Vector3d world_min =
      Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
  Eigen::Vector3d world_max =
      Eigen::Vector3d::Constant(-std::numeric_limits<double>::infinity());
  for (unsigned corner = 0; corner < 8; ++corner) {
    const Eigen::Vector3d local{
        (corner & 1U) != 0U ? local_max.x() : local_min.x(),
        (corner & 2U) != 0U ? local_max.y() : local_min.y(),
        (corner & 4U) != 0U ? local_max.z() : local_min.z(),
    };
    const auto world = transformLocalToWorld(local, local_to_world);
    if (!world.has_value()) {
      return std::nullopt;
    }
    world_min = world_min.cwiseMin(*world);
    world_max = world_max.cwiseMax(*world);
  }
  const auto centroid = transformLocalToWorld(
      local_bounds.centroid.cast<double>(), local_to_world);
  if (!centroid.has_value()) {
    return std::nullopt;
  }

  WorldBounds result;
  result.minimum = world_min;
  result.maximum = world_max;
  result.centroid = *centroid;
  result.radius = (world_max - world_min).norm() * 0.5;
  result.finite_points = local_bounds.finite_points;
  return finite(result.minimum) && finite(result.maximum) &&
                 finite(result.centroid) && std::isfinite(result.radius)
             ? std::optional<WorldBounds>{result}
             : std::nullopt;
}

std::optional<LayerPickResult> SceneRenderAdapter::resolvePick(
    const Scene &scene, LayerId layer_id, const PickResult &local_pick) {
  const auto *layer = scene.findLayer(layer_id);
  if (layer == nullptr || !local_pick.cloud_position.allFinite()) {
    return std::nullopt;
  }
  const auto world = transformLocalToWorld(
      local_pick.cloud_position.cast<double>(), layer->localToWorld());
  if (!world.has_value()) {
    return std::nullopt;
  }
  return LayerPickResult{layer_id, layer->sourceKey(),
                         local_pick.cloud_position, *world,
                         local_pick.intensity, local_pick.noise};
}

} // namespace kpt::gui
