#include "gui/viewport/scene_compositor.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

namespace kpt::gui {
namespace {

[[nodiscard]] Eigen::Vector3f clampColour(Eigen::Vector3f value) noexcept {
  return value.array().max(0.0F).min(1.0F).matrix();
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
  for (std::size_t retained = 0; retained < count; ++retained) {
    const auto source = item.vertex_selection.sourceIndex(retained);
    if (!source || *source >= vertices.size())
      continue;
    const ViewportVertex &local = vertices[*source];
    const auto world = transformLocalToWorld(local.position.cast<double>(),
                                             item.local_to_world);
    if (!world || !world->allFinite() ||
        (world->array().abs() >
         static_cast<double>(std::numeric_limits<float>::max()))
            .any()) {
      continue;
    }
    ViewportVertex world_vertex = local;
    world_vertex.position = world->cast<float>();
    if (apply_compatibility_style)
      world_vertex.color = styledColour(local, item);
    if (!world_vertex.position.allFinite() || !world_vertex.color.allFinite())
      continue;
    out.push_back(world_vertex);
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
      layer.revision = revision;
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
