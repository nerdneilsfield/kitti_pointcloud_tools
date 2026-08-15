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

[[nodiscard]] Eigen::Vector3f bakedColour(const ViewportVertex &vertex,
                                           const LayerRenderItem &item,
                                           const Eigen::Vector3f &background) noexcept {
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
  const float opacity = std::clamp(style.opacity, 0.0F, 1.0F);
  return clampColour(background * (1.0F - opacity) + colour * opacity);
}

void addItemVertices(const LayerRenderItem &item,
                     std::vector<ViewportVertex> &out,
                     const Eigen::Vector3f &background) {
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
    world_vertex.color = bakedColour(local, item, background);
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

} // namespace

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
      addItemVertices(item, snapshot->vertices, options.background);
    }
  };
  append(render_list.opaque_draw_order);
  append(render_list.transparent_draw_order);
  setBounds(*snapshot);

  constexpr std::size_t kMaximumPickingCandidates = 100'000U;
  const std::size_t count =
      std::min(kMaximumPickingCandidates, snapshot->vertices.size());
  snapshot->picking_vertices.reserve(count);
  for (std::size_t index = 0; index < count; ++index) {
    const std::size_t source = (index * snapshot->vertices.size()) / count;
    snapshot->picking_vertices.push_back(snapshot->vertices[source]);
  }
  return snapshot;
}

} // namespace kpt::gui
