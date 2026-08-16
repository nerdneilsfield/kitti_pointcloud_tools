#include "gui/viewport/scene_compositor.hpp"

#include <algorithm>
#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

namespace kpt::gui {
namespace {

[[nodiscard]] Eigen::Vector3f clampColour(Eigen::Vector3f value) noexcept {
  return value.array().max(0.0F).min(1.0F).matrix();
}

void hashValue(std::uint64_t &state, std::uint64_t value) noexcept {
  // FNV-1a is sufficient here: this is an in-memory GPU-cache revision, not
  // a persistent content address. Include every draw-affecting scalar so a
  // camera-only transparent-order refresh can retain existing buffers.
  state ^= value;
  state *= 1099511628211ULL;
}

void hashFloat(std::uint64_t &state, float value) noexcept {
  hashValue(state, std::bit_cast<std::uint32_t>(value));
}

void hashDouble(std::uint64_t &state, double value) noexcept {
  hashValue(state, std::bit_cast<std::uint64_t>(value));
}

[[nodiscard]] std::uint64_t
layerContentRevision(const LayerRenderItem &item) noexcept {
  std::uint64_t state = 1469598103934665603ULL;
  hashValue(state, item.snapshot ? item.snapshot->revision : 0);
  hashValue(state, item.vertex_selection.source_vertex_count);
  hashValue(state, item.vertex_selection.eligible_vertex_count);
  hashValue(state, item.vertex_selection.retained_vertex_count);
  for (int row = 0; row < 4; ++row) {
    for (int column = 0; column < 4; ++column)
      hashDouble(state, item.local_to_world.matrix()(row, column));
  }
  const LayerStyle &style = item.style;
  hashValue(state, static_cast<std::uint64_t>(style.color_by));
  hashValue(state, static_cast<std::uint64_t>(style.color_map));
  hashFloat(state, style.point_size);
  hashFloat(state, style.opacity);
  hashFloat(state, style.scalar_min);
  hashFloat(state, style.scalar_max);
  for (int component = 0; component < 3; ++component) {
    hashFloat(state, style.fixed_color[component]);
    hashFloat(state, style.noise_color[component]);
  }
  hashValue(state, style.highlight_noise ? 1U : 0U);
  hashValue(state, style.intensity_equalize ? 1U : 0U);
  hashValue(state, item.world_roi.has_value() ? 1U : 0U);
  if (item.world_roi) {
    for (int component = 0; component < 3; ++component) {
      hashDouble(state, item.world_roi->minimum()[component]);
      hashDouble(state, item.world_roi->maximum()[component]);
    }
  }
  return state == 0 ? 1 : state;
}

[[nodiscard]] Eigen::Vector3f turbo(float value) noexcept {
  const float x = std::clamp(value, 0.0F, 1.0F);
  const Eigen::Vector4f v4{1.0F, x, x * x, x * x * x};
  const Eigen::Vector2f v2 = v4.tail<2>() * (x * x * x * x);
  const Eigen::Vector4f red{0.13572138F, 4.61539260F, -42.66032258F,
                             132.13108234F};
  const Eigen::Vector4f green{0.09140261F, 2.19418839F, 4.84296658F,
                               -14.18503333F};
  const Eigen::Vector4f blue{0.10667330F, 12.64194608F, -60.58204836F,
                              110.36276771F};
  const Eigen::Vector2f red2{-152.94239396F, 59.28637943F};
  const Eigen::Vector2f green2{4.27729857F, 2.82956604F};
  const Eigen::Vector2f blue2{-89.90310912F, 27.34824973F};
  return clampColour(Eigen::Vector3f{red.dot(v4) + red2.dot(v2),
                                     green.dot(v4) + green2.dot(v2),
                                     blue.dot(v4) + blue2.dot(v2)});
}

[[nodiscard]] float normalized(float value, float minimum,
                               float maximum) noexcept {
  if (!std::isfinite(value) || !std::isfinite(minimum) ||
      !std::isfinite(maximum) || !(minimum < maximum)) {
    return 0.5F;
  }
  return std::clamp((value - minimum) / (maximum - minimum), 0.0F, 1.0F);
}

[[nodiscard]] Eigen::Vector3f styledColour(const ViewportVertex &vertex,
                                            const LayerRenderItem &item) noexcept {
  const LayerStyle &style = item.style;
  Eigen::Vector3f colour = vertex.color;
  if (style.highlight_noise && vertex.noise > 0.5F) {
    colour = style.noise_color;
  } else {
    switch (style.color_by) {
    case ColorBy::RGB:
    case ColorBy::Label:
      colour = vertex.color;
      break;
    case ColorBy::None:
      colour = style.fixed_color;
      break;
    case ColorBy::Intensity:
      colour = turbo(normalized(vertex.intensity, style.scalar_min,
                                style.scalar_max));
      break;
    case ColorBy::Z:
      colour = turbo(normalized(vertex.position.z(),
                                item.snapshot->bounds.z_min,
                                item.snapshot->bounds.z_max));
      break;
    }
  }
  return clampColour(colour);
}

[[nodiscard]] std::optional<ViewportVertex>
worldVertex(const ViewportVertex &local, const LayerRenderItem &item,
            bool apply_compatibility_style) {
  const auto world = transformLocalToWorld(local.position.cast<double>(),
                                           item.local_to_world);
  if (!world || !world->allFinite() ||
      (world->array().abs() >
       static_cast<double>(std::numeric_limits<float>::max()))
          .any()) {
    return std::nullopt;
  }
  // ROI is a closed world-space box.  Do the finite local-to-world transform
  // once and use the same predicate for GPU payload, camera fit, picking
  // candidates, and the legacy probe snapshot.
  if (item.world_roi && !item.world_roi->contains(*world))
    return std::nullopt;
  ViewportVertex world_vertex = local;
  world_vertex.position = world->cast<float>();
  if (apply_compatibility_style)
    world_vertex.color = styledColour(local, item);
  if (!world_vertex.position.allFinite() || !world_vertex.color.allFinite())
    return std::nullopt;
  return world_vertex;
}

void addItemVertices(const LayerRenderItem &item,
                     std::vector<ViewportVertex> &out,
                     bool apply_compatibility_style = false) {
  if (!item.snapshot || !item.visible ||
      item.vertex_selection.retained_vertex_count == 0) {
    return;
  }
  const auto &vertices = item.snapshot->vertices;
  const auto count = item.vertex_selection.retained_vertex_count;
  out.reserve(out.size() + count);
  if (!item.vertex_selection.requiresEligibilityScan()) {
    for (std::size_t retained = 0; retained < count; ++retained) {
      const auto source = item.vertex_selection.sourceIndex(retained);
      if (!source || *source >= vertices.size())
        continue;
      if (const auto vertex = worldVertex(vertices[*source], item,
                                          apply_compatibility_style)) {
        out.push_back(*vertex);
      }
    }
    return;
  }

  // A ROI can accept an arbitrarily sparse subset.  Select uniform ranks
  // *after* eligibility rather than selecting local vertices first and then
  // discarding them.  This second bounded scan avoids storing a potentially
  // enormous index vector alongside every CPU cloud.
  const auto eligible = item.vertex_selection.eligible_vertex_count;
  if (eligible == 0)
    return;
  std::size_t accepted = 0;
  std::size_t retained = 0;
  for (const ViewportVertex &local : vertices) {
    const auto vertex = worldVertex(local, item, apply_compatibility_style);
    if (!vertex)
      continue;
    const std::size_t wanted_rank =
        (retained * eligible) / count;
    if (accepted == wanted_rank) {
      out.push_back(*vertex);
      ++retained;
      if (retained == count)
        return;
    }
    ++accepted;
  }
}

void setBounds(ViewportCloudSnapshot &snapshot) {
  if (snapshot.vertices.empty())
    return;
  Eigen::Vector3f minimum = Eigen::Vector3f::Constant(
      std::numeric_limits<float>::max());
  Eigen::Vector3f maximum = Eigen::Vector3f::Constant(
      std::numeric_limits<float>::lowest());
  Eigen::Vector3d sum = Eigen::Vector3d::Zero();
  float intensity_min = std::numeric_limits<float>::max();
  float intensity_max = std::numeric_limits<float>::lowest();
  std::size_t noise_points = 0;
  std::size_t finite = 0;
  for (const ViewportVertex &vertex : snapshot.vertices) {
    if (!vertex.position.allFinite())
      continue;
    minimum = minimum.cwiseMin(vertex.position);
    maximum = maximum.cwiseMax(vertex.position);
    sum += vertex.position.cast<double>();
    if (std::isfinite(vertex.intensity)) {
      intensity_min = std::min(intensity_min, vertex.intensity);
      intensity_max = std::max(intensity_max, vertex.intensity);
    }
    noise_points += vertex.noise > 0.5F ? 1U : 0U;
    ++finite;
  }
  if (finite == 0)
    return;
  CloudBounds &bounds = snapshot.bounds;
  bounds.minimum = minimum;
  bounds.maximum = maximum;
  bounds.center = (minimum.cast<double>() + maximum.cast<double>())
                      .cast<float>() * 0.5F;
  bounds.centroid = (sum / static_cast<double>(finite)).cast<float>();
  bounds.radius = std::max(0.001, (maximum - minimum).cast<double>().norm() *
                                       0.5);
  bounds.finite_points = finite;
  bounds.intensity_min = intensity_min;
  bounds.intensity_max = intensity_max;
  bounds.intensity_p05 = intensity_min;
  bounds.intensity_p90 = intensity_max;
  bounds.z_min = minimum.z();
  bounds.z_max = maximum.z();
  bounds.has_noise = noise_points != 0;
  bounds.noise_points = noise_points;
}

[[nodiscard]] ViewportLayerDraw
layerDrawState(const LayerRenderItem &item, const CloudBounds &world_bounds) {
  ViewportLayerDraw draw;
  draw.layer_id = item.layer_id;
  draw.style.color_by = item.style.color_by;
  draw.style.color_map = item.style.color_map;
  draw.style.point_size = item.style.point_size;
  draw.style.scalar_min = item.style.scalar_min;
  draw.style.scalar_max = item.style.scalar_max;
  draw.style.fixed_color = item.style.fixed_color;
  draw.style.noise_color = item.style.noise_color;
  draw.style.highlight_noise = item.style.highlight_noise;
  draw.style.intensity_equalize = item.style.intensity_equalize;
  // Z is evaluated from transformed world positions by the shader. Its range
  // must therefore be world-space too, rather than the layer-local range.
  if (draw.style.color_by == ColorBy::Z &&
      std::isfinite(world_bounds.z_min) &&
      std::isfinite(world_bounds.z_max)) {
    draw.style.scalar_min = world_bounds.z_min;
    draw.style.scalar_max = world_bounds.z_max;
  }
  if (item.snapshot) {
    draw.intensity_cdf = item.snapshot->bounds.intensity_cdf;
    draw.intensity_cdf_valid = item.snapshot->bounds.intensity_cdf_valid;
  }
  draw.opacity = std::clamp(item.style.opacity, 0.0F, 1.0F);
  return draw;
}

[[nodiscard]] std::vector<ViewportVertex>
worldVertices(const LayerRenderItem &item) {
  std::vector<ViewportVertex> vertices;
  addItemVertices(item, vertices);
  return vertices;
}

void appendPickingCandidates(ViewportCloudSnapshot &snapshot) {
  constexpr std::size_t kMaximumPickingCandidates = 100'000U;
  const std::size_t count =
      std::min(kMaximumPickingCandidates, snapshot.vertices.size());
  snapshot.picking_vertices.clear();
  snapshot.picking_vertices.reserve(count);
  for (std::size_t index = 0; index < count; ++index) {
    const std::size_t source = (index * snapshot.vertices.size()) / count;
    snapshot.picking_vertices.push_back(snapshot.vertices[source]);
  }
}

} // namespace

std::shared_ptr<const LayeredViewportSnapshot>
composeLayeredSceneViewportSnapshot(const LayerRenderList &render_list,
                                    std::uint64_t revision,
                                    const SceneCompositeOptions &options) {
  auto result = std::make_shared<LayeredViewportSnapshot>();
  result->revision = revision;
  auto camera_cloud = std::make_shared<ViewportCloudSnapshot>();
  camera_cloud->revision = revision;
  if (revision == 0) {
    result->camera_cloud = std::move(camera_cloud);
    return result;
  }

  const auto append = [&render_list, &options, &result, &camera_cloud, revision](
                          const std::vector<std::size_t> &order,
                          std::vector<ViewportLayerSnapshot> &destination) {
    for (const std::size_t index : order) {
      if (index >= render_list.layers.size())
        continue;
      const LayerRenderItem &item = render_list.layers[index];
      if (options.only_layer && item.layer_id != *options.only_layer)
        continue;
      ViewportLayerSnapshot layer;
      layer.vertices = worldVertices(item);
      if (layer.vertices.empty())
        continue;
      ViewportCloudSnapshot layer_cloud;
      layer_cloud.vertices = layer.vertices;
      setBounds(layer_cloud);
      layer.draw = layerDrawState(item, layer_cloud.bounds);
      layer.revision = layerContentRevision(item);
      camera_cloud->vertices.insert(camera_cloud->vertices.end(),
                                    layer.vertices.begin(), layer.vertices.end());
      destination.push_back(std::move(layer));
    }
  };
  append(render_list.opaque_draw_order, result->opaque_layers);
  append(render_list.transparent_draw_order, result->transparent_layers);
  setBounds(*camera_cloud);
  appendPickingCandidates(*camera_cloud);
  result->camera_cloud = std::move(camera_cloud);
  return result;
}

std::shared_ptr<const ViewportCloudSnapshot>
composeSceneViewportSnapshot(const LayerRenderList &render_list,
                             std::uint64_t revision,
                             const SceneCompositeOptions &options) {
  auto snapshot = std::make_shared<ViewportCloudSnapshot>();
  snapshot->revision = revision;
  if (revision == 0)
    return snapshot;

  const auto append = [&render_list, &options, snapshot](
                          const std::vector<std::size_t> &order) {
    for (const std::size_t index : order) {
      if (index >= render_list.layers.size())
        continue;
      const LayerRenderItem &item = render_list.layers[index];
      if (options.only_layer && item.layer_id != *options.only_layer)
        continue;
      addItemVertices(item, snapshot->vertices, true);
    }
  };
  append(render_list.opaque_draw_order);
  append(render_list.transparent_draw_order);
  setBounds(*snapshot);

  appendPickingCandidates(*snapshot);
  return snapshot;
}

} // namespace kpt::gui
