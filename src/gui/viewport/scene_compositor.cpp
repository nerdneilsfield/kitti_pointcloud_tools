#include "gui/viewport/scene_compositor.hpp"

#include "kpt/cancellation.hpp"

#include <algorithm>
#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <span>
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
layerContentRevision(const LayerRenderItem &item,
                     const ViewportLayerDraw &draw) noexcept {
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
  hashValue(state, static_cast<std::uint64_t>(style.intensity_range_mode));
  // A shared scale can change one layer's draw state when a different layer
  // becomes visible.  Hash resolved payload, not only persisted layer state.
  hashFloat(state, draw.style.scalar_min);
  hashFloat(state, draw.style.scalar_max);
  hashValue(state, draw.style.intensity_equalize ? 1U : 0U);
  hashValue(state, draw.intensity_cdf_valid ? 1U : 0U);
  if (draw.intensity_cdf_valid) {
    for (const float value : draw.intensity_cdf) {
      hashFloat(state, value);
    }
  }
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

struct IntensityRange {
  float minimum = 0.0F;
  float maximum = 1.0F;
};

[[nodiscard]] bool validIntensityRange(float minimum, float maximum) noexcept {
  return std::isfinite(minimum) && std::isfinite(maximum) && minimum < maximum;
}

[[nodiscard]] bool validIntensityInterval(float minimum,
                                          float maximum) noexcept {
  return std::isfinite(minimum) && std::isfinite(maximum) && minimum <= maximum;
}

// The persisted schema permits a closed scalar interval. GPU normalization
// needs a non-zero denominator, so preserve an exact degenerate Manual/shared
// value in scene state while using a tiny finite draw-only window around it.
[[nodiscard]] IntensityRange drawableIntensityRange(float value) noexcept {
  if (!std::isfinite(value)) {
    return {};
  }
  const float padding = std::max(1.0F, std::abs(value) * 1.0e-6F);
  const float minimum = value - padding;
  const float maximum = value + padding;
  if (validIntensityRange(minimum, maximum)) {
    return {minimum, maximum};
  }
  const float lower =
      std::nextafter(value, -std::numeric_limits<float>::infinity());
  const float upper =
      std::nextafter(value, std::numeric_limits<float>::infinity());
  if (validIntensityRange(lower, upper)) {
    return {lower, upper};
  }
  return {};
}

[[nodiscard]] std::optional<IntensityRange>
robustIntensityRange(const CloudBounds &bounds) noexcept {
  if (validIntensityRange(bounds.intensity_p05, bounds.intensity_p90)) {
    return IntensityRange{bounds.intensity_p05, bounds.intensity_p90};
  }
  if (validIntensityRange(bounds.intensity_min, bounds.intensity_max)) {
    return IntensityRange{bounds.intensity_min, bounds.intensity_max};
  }
  return std::nullopt;
}

[[nodiscard]] std::optional<IntensityRange>
sharedIntensityRange(const CloudBounds &bounds) noexcept {
  if (validIntensityRange(bounds.intensity_p05, bounds.intensity_p90)) {
    return IntensityRange{bounds.intensity_p05, bounds.intensity_p90};
  }
  if (validIntensityRange(bounds.intensity_min, bounds.intensity_max)) {
    return IntensityRange{bounds.intensity_min, bounds.intensity_max};
  }
  if (validIntensityInterval(bounds.intensity_p05, bounds.intensity_p90)) {
    return IntensityRange{bounds.intensity_p05, bounds.intensity_p90};
  }
  if (validIntensityInterval(bounds.intensity_min, bounds.intensity_max)) {
    return IntensityRange{bounds.intensity_min, bounds.intensity_max};
  }
  return std::nullopt;
}

[[nodiscard]] float sampleIntensityCdf(const std::array<float, 256> &cdf,
                                       float coordinate) noexcept {
  const float clamped = std::clamp(coordinate, 0.0F, 1.0F);
  const float scaled = clamped * static_cast<float>(cdf.size() - 1U);
  const auto lower = static_cast<std::size_t>(scaled);
  const auto upper = std::min(lower + 1U, cdf.size() - 1U);
  const float fraction = scaled - static_cast<float>(lower);
  return std::clamp(cdf[lower] + (cdf[upper] - cdf[lower]) * fraction, 0.0F,
                    1.0F);
}

[[nodiscard]] std::array<float, 256>
resampleIntensityCdf(const std::array<float, 256> &source,
                     const IntensityRange source_range,
                     const IntensityRange destination_range) noexcept {
  std::array<float, 256> result{};
  for (std::size_t index = 0; index < result.size(); ++index) {
    const float fraction =
        static_cast<float>(index) / static_cast<float>(result.size() - 1U);
    const float value =
        destination_range.minimum +
        (destination_range.maximum - destination_range.minimum) * fraction;
    result[index] = sampleIntensityCdf(
        source, normalized(value, source_range.minimum, source_range.maximum));
  }
  return result;
}

[[nodiscard]] Eigen::Vector3f
styledColour(const ViewportVertex &vertex,
             const ViewportLayerDraw &draw) noexcept {
  const ViewportStyle &style = draw.style;
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
    case ColorBy::Intensity: {
      float scalar =
          normalized(vertex.intensity, style.scalar_min, style.scalar_max);
      if (style.intensity_equalize && draw.intensity_cdf_valid) {
        scalar = sampleIntensityCdf(draw.intensity_cdf, scalar);
      }
      colour = turbo(scalar);
    } break;
    case ColorBy::Z:
      colour = turbo(
          normalized(vertex.position.z(), style.scalar_min, style.scalar_max));
      break;
    }
  }
  return clampColour(colour);
}

[[nodiscard]] std::optional<ViewportVertex>
worldVertex(const ViewportVertex &local, const LayerRenderItem &item,
            const ViewportLayerDraw *compatibility_draw = nullptr) {
  const auto world =
      transformLocalToWorld(local.position.cast<double>(), item.local_to_world);
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
  if (compatibility_draw != nullptr)
    world_vertex.color = styledColour(world_vertex, *compatibility_draw);
  if (!world_vertex.position.allFinite() || !world_vertex.color.allFinite())
    return std::nullopt;
  return world_vertex;
}

void addItemVertices(const LayerRenderItem &item,
                     std::vector<ViewportVertex> &out,
                     const ViewportLayerDraw *compatibility_draw = nullptr,
                     std::stop_token stop = {}) {
  if (!item.snapshot || !item.visible ||
      item.vertex_selection.retained_vertex_count == 0) {
    return;
  }
  const auto &vertices = item.snapshot->vertices;
  const auto count = item.vertex_selection.retained_vertex_count;
  out.reserve(out.size() + count);
  if (!item.vertex_selection.requiresEligibilityScan()) {
    for (std::size_t retained = 0; retained < count; ++retained) {
      if ((retained % 4096U) == 0U && stop.stop_requested())
        throw OperationCancelled();
      const auto source = item.vertex_selection.sourceIndex(retained);
      if (!source || *source >= vertices.size())
        continue;
      if (const auto vertex =
              worldVertex(vertices[*source], item, compatibility_draw)) {
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
  std::size_t source_index = 0;
  for (const ViewportVertex &local : vertices) {
    if ((source_index++ % 4096U) == 0U && stop.stop_requested())
      throw OperationCancelled();
    const auto vertex = worldVertex(local, item, compatibility_draw);
    if (!vertex)
      continue;
    const std::size_t wanted_rank = (retained * eligible) / count;
    if (accepted == wanted_rank) {
      out.push_back(*vertex);
      ++retained;
      if (retained == count)
        return;
    }
    ++accepted;
  }
}

[[nodiscard]] CloudBounds
calculateVertexBounds(std::span<const ViewportVertex> vertices) {
  CloudBounds bounds;
  if (vertices.empty()) {
    return bounds;
  }
  Eigen::Vector3f minimum =
      Eigen::Vector3f::Constant(std::numeric_limits<float>::max());
  Eigen::Vector3f maximum =
      Eigen::Vector3f::Constant(std::numeric_limits<float>::lowest());
  Eigen::Vector3d sum = Eigen::Vector3d::Zero();
  float intensity_min = std::numeric_limits<float>::max();
  float intensity_max = std::numeric_limits<float>::lowest();
  std::size_t noise_points = 0;
  std::size_t finite = 0;
  for (const ViewportVertex &vertex : vertices) {
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
  if (finite == 0) {
    return bounds;
  }
  bounds.minimum = minimum;
  bounds.maximum = maximum;
  bounds.center =
      (minimum.cast<double>() + maximum.cast<double>()).cast<float>() * 0.5F;
  bounds.centroid = (sum / static_cast<double>(finite)).cast<float>();
  bounds.radius =
      std::max(0.001, (maximum - minimum).cast<double>().norm() * 0.5);
  bounds.finite_points = finite;
  bounds.intensity_min = intensity_min;
  bounds.intensity_max = intensity_max;
  bounds.intensity_p05 = intensity_min;
  bounds.intensity_p90 = intensity_max;
  bounds.z_min = minimum.z();
  bounds.z_max = maximum.z();
  bounds.has_noise = noise_points != 0;
  bounds.noise_points = noise_points;
  return bounds;
}

void setBounds(ViewportCloudSnapshot &snapshot) {
  if (snapshot.vertices.empty()) {
    return;
  }
  snapshot.bounds = calculateVertexBounds(snapshot.vertices);
}

constexpr std::size_t kMaximumCameraFitSamples = 100'000U;

[[nodiscard]] std::size_t saturatingAdd(std::size_t left,
                                        std::size_t right) noexcept {
  return left > std::numeric_limits<std::size_t>::max() - right
             ? std::numeric_limits<std::size_t>::max()
             : left + right;
}

[[nodiscard]] std::optional<WorldBounds>
combineFitBounds(const std::vector<const LayerRenderItem *> &items) {
  std::optional<WorldBounds> result;
  Eigen::Vector3d centroid_sum = Eigen::Vector3d::Zero();
  std::size_t count = 0;
  for (const LayerRenderItem *item : items) {
    if (item == nullptr || !item->eligible_world_bounds) {
      continue;
    }
    const WorldBounds &bounds = *item->eligible_world_bounds;
    if (!bounds.minimum.allFinite() || !bounds.maximum.allFinite() ||
        !bounds.centroid.allFinite() || bounds.finite_points == 0 ||
        !std::isfinite(bounds.radius)) {
      continue;
    }
    if (!result) {
      result = bounds;
    } else {
      result->minimum = result->minimum.cwiseMin(bounds.minimum);
      result->maximum = result->maximum.cwiseMax(bounds.maximum);
    }
    centroid_sum += bounds.centroid * static_cast<double>(bounds.finite_points);
    count = saturatingAdd(count, bounds.finite_points);
  }
  if (!result || count == 0 || !centroid_sum.allFinite()) {
    return std::nullopt;
  }
  result->centroid = centroid_sum / static_cast<double>(count);
  result->finite_points = count;
  result->radius = (result->maximum - result->minimum).norm() * 0.5;
  return result->minimum.allFinite() && result->maximum.allFinite() &&
                 result->centroid.allFinite() && std::isfinite(result->radius)
             ? result
             : std::nullopt;
}

void setBoundsFromWorldBounds(ViewportCloudSnapshot &snapshot,
                              const WorldBounds &world_bounds) {
  constexpr double float_limit =
      static_cast<double>(std::numeric_limits<float>::max());
  if (!world_bounds.minimum.allFinite() || !world_bounds.maximum.allFinite() ||
      !world_bounds.centroid.allFinite() ||
      (world_bounds.minimum.array() > world_bounds.maximum.array()).any() ||
      (world_bounds.minimum.array().abs() > float_limit).any() ||
      (world_bounds.maximum.array().abs() > float_limit).any() ||
      (world_bounds.centroid.array().abs() > float_limit).any() ||
      !std::isfinite(world_bounds.radius) || world_bounds.finite_points == 0) {
    return;
  }
  CloudBounds bounds;
  bounds.minimum = world_bounds.minimum.cast<float>();
  bounds.maximum = world_bounds.maximum.cast<float>();
  bounds.centroid = world_bounds.centroid.cast<float>();
  bounds.center =
      ((world_bounds.minimum + world_bounds.maximum) * 0.5).cast<float>();
  bounds.radius = std::max(world_bounds.radius, 0.001);
  bounds.z_min = bounds.minimum.z();
  bounds.z_max = bounds.maximum.z();
  bounds.finite_points = world_bounds.finite_points;
  snapshot.bounds = bounds;
}

[[nodiscard]] std::size_t uniformRank(std::size_t index, std::size_t total,
                                      std::size_t selected) noexcept {
  if (total == 0 || selected == 0 || index >= selected) {
    return total;
  }
  const auto quotient = total / selected;
  const auto remainder = total % selected;
  if (remainder != 0 &&
      index > std::numeric_limits<std::size_t>::max() / remainder) {
    return std::min(total - 1, static_cast<std::size_t>(
                                   (static_cast<long double>(index) *
                                    static_cast<long double>(total)) /
                                   static_cast<long double>(selected)));
  }
  return std::min(total - 1, quotient * index + (remainder * index) / selected);
}

void appendFitSamples(const LayerRenderItem &item, std::size_t sample_count,
                      std::vector<ViewportVertex> &out, std::stop_token stop) {
  if (!item.snapshot || sample_count == 0 ||
      item.vertex_selection.eligible_vertex_count == 0) {
    return;
  }
  const auto eligible = item.vertex_selection.eligible_vertex_count;
  const auto &vertices = item.snapshot->vertices;
  if (!item.vertex_selection.requiresEligibilityScan()) {
    for (std::size_t selected = 0; selected < sample_count; ++selected) {
      if ((selected % 4096U) == 0U && stop.stop_requested()) {
        throw OperationCancelled();
      }
      const auto source = uniformRank(selected, eligible, sample_count);
      if (source >= vertices.size()) {
        continue;
      }
      if (const auto vertex = worldVertex(vertices[source], item)) {
        out.push_back(*vertex);
      }
    }
    return;
  }

  // A sparse ROI changes the local index domain.  Select uniform ranks after
  // acceptance, keeping the fit sample bounded without storing all accepted
  // source indices alongside the cloud.
  std::size_t accepted = 0;
  std::size_t selected = 0;
  for (std::size_t source = 0; source < vertices.size(); ++source) {
    if ((source % 4096U) == 0U && stop.stop_requested()) {
      throw OperationCancelled();
    }
    const auto vertex = worldVertex(vertices[source], item);
    if (!vertex) {
      continue;
    }
    const std::size_t wanted = uniformRank(selected, eligible, sample_count);
    if (accepted == wanted) {
      out.push_back(*vertex);
      ++selected;
      if (selected == sample_count) {
        return;
      }
    }
    ++accepted;
  }
}

[[nodiscard]] std::shared_ptr<const ViewportCloudSnapshot>
makeCameraCloud(const std::vector<const LayerRenderItem *> &items,
                std::uint64_t revision, std::stop_token stop) {
  auto snapshot = std::make_shared<ViewportCloudSnapshot>();
  snapshot->revision = revision;
  if (revision == 0 || stop.stop_requested()) {
    if (stop.stop_requested()) {
      throw OperationCancelled();
    }
    return snapshot;
  }
  const auto bounds = combineFitBounds(items);
  if (!bounds) {
    return snapshot;
  }

  struct Allocation {
    const LayerRenderItem *item = nullptr;
    std::size_t eligible = 0;
    std::size_t samples = 0;
  };
  std::vector<Allocation> allocations;
  allocations.reserve(items.size());
  std::size_t total_eligible = 0;
  for (const LayerRenderItem *item : items) {
    if (item == nullptr || !item->snapshot || !item->eligible_world_bounds ||
        item->vertex_selection.eligible_vertex_count == 0) {
      continue;
    }
    const std::size_t eligible = item->vertex_selection.eligible_vertex_count;
    allocations.push_back({item, eligible, 0});
    total_eligible = saturatingAdd(total_eligible, eligible);
  }
  if (allocations.empty() || total_eligible == 0) {
    return snapshot;
  }

  const std::size_t sample_budget =
      std::min(total_eligible, kMaximumCameraFitSamples);
  std::size_t assigned = 0;
  for (Allocation &allocation : allocations) {
    allocation.samples =
        sample_budget == total_eligible
            ? allocation.eligible
            : std::min(allocation.eligible,
                       static_cast<std::size_t>(
                           (static_cast<long double>(allocation.eligible) *
                            static_cast<long double>(sample_budget)) /
                           static_cast<long double>(total_eligible)));
    assigned = saturatingAdd(assigned, allocation.samples);
  }
  // Flooring proportional shares leaves fewer than one sample per allocation.
  // This short deterministic pass makes the cap exact without point-sized
  // auxiliary storage.
  for (std::size_t cursor = 0; assigned < sample_budget && !allocations.empty();
       ++cursor) {
    Allocation &allocation = allocations[cursor % allocations.size()];
    if (allocation.samples < allocation.eligible) {
      ++allocation.samples;
      ++assigned;
    }
  }

  snapshot->vertices.reserve(sample_budget);
  for (const Allocation &allocation : allocations) {
    appendFitSamples(*allocation.item, allocation.samples, snapshot->vertices,
                     stop);
  }
  if (snapshot->vertices.empty()) {
    return snapshot;
  }
  setBoundsFromWorldBounds(*snapshot, *bounds);
  return snapshot;
}

[[nodiscard]] std::vector<const LayerRenderItem *>
fitItemsFor(const LayerRenderList &render_list,
            const SceneCompositeOptions &options,
            bool require_render_selection) {
  std::vector<const LayerRenderItem *> result;
  if (options.only_layer) {
    for (const LayerRenderItem &item : render_list.layers) {
      if (item.layer_id != *options.only_layer ||
          (require_render_selection &&
           item.vertex_selection.retained_vertex_count == 0)) {
        continue;
      }
      result.push_back(&item);
      break;
    }
    return result;
  }
  // Camera fitting has no painter-order dependency.  Iterate the complete
  // layer list, not draw orders: a visible layer can be Deferred under an
  // ultra-small transient GPU cap and must still be included in "Fit visible".
  for (const LayerRenderItem &item : render_list.layers) {
    if (!item.visible || (require_render_selection &&
                          item.vertex_selection.retained_vertex_count == 0)) {
      continue;
    }
    result.push_back(&item);
  }
  return result;
}

[[nodiscard]] std::optional<IntensityRange>
sharedVisibleIntensityRange(const LayerRenderList &render_list) noexcept {
  std::optional<IntensityRange> shared;
  for (const LayerRenderItem &item : render_list.layers) {
    if (!item.visible || item.style.color_by != ColorBy::Intensity ||
        !item.snapshot) {
      continue;
    }
    const auto range = sharedIntensityRange(item.snapshot->bounds);
    if (!range) {
      continue;
    }
    if (!shared) {
      shared = *range;
    } else {
      shared->minimum = std::min(shared->minimum, range->minimum);
      shared->maximum = std::max(shared->maximum, range->maximum);
    }
  }
  if (!shared) {
    return std::nullopt;
  }
  if (validIntensityRange(shared->minimum, shared->maximum)) {
    return shared;
  }
  return drawableIntensityRange(shared->minimum);
}

[[nodiscard]] ViewportLayerDraw
layerDrawState(const LayerRenderItem &item,
               const std::optional<IntensityRange> &shared_visible_range) {
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
  if (item.snapshot) {
    draw.intensity_cdf = item.snapshot->bounds.intensity_cdf;
    draw.intensity_cdf_valid = item.snapshot->bounds.intensity_cdf_valid;

    const CloudBounds &bounds = item.snapshot->bounds;
    const auto robust = robustIntensityRange(bounds);
    const auto automatic = sharedIntensityRange(bounds);
    const IntensityRange raw{bounds.intensity_min, bounds.intensity_max};
    const IntensityRange manual{item.style.scalar_min, item.style.scalar_max};

    if (item.style.color_by == ColorBy::Intensity &&
        shared_visible_range.has_value()) {
      // Shared-visible is intentionally a linear comparison scale. It never
      // writes persisted Manual/Equalize values, and ROI is not consulted:
      // dragging a crop must not make colours jump.
      draw.style.scalar_min = shared_visible_range->minimum;
      draw.style.scalar_max = shared_visible_range->maximum;
      draw.style.intensity_equalize = false;
      draw.intensity_cdf_valid = false;
    } else if (item.style.color_by == ColorBy::Intensity) {
      if (item.style.intensity_range_mode == IntensityRangeMode::Manual &&
          validIntensityInterval(manual.minimum, manual.maximum)) {
        const IntensityRange resolved_manual =
            validIntensityRange(manual.minimum, manual.maximum)
                ? manual
                : drawableIntensityRange(manual.minimum);
        draw.style.scalar_min = resolved_manual.minimum;
        draw.style.scalar_max = resolved_manual.maximum;
        if (draw.style.intensity_equalize && draw.intensity_cdf_valid &&
            validIntensityRange(raw.minimum, raw.maximum) &&
            validIntensityRange(manual.minimum, manual.maximum)) {
          draw.intensity_cdf =
              resampleIntensityCdf(draw.intensity_cdf, raw, manual);
        } else {
          draw.style.intensity_equalize = false;
          draw.intensity_cdf_valid = false;
        }
      } else if (draw.style.intensity_equalize && draw.intensity_cdf_valid &&
                 validIntensityRange(raw.minimum, raw.maximum)) {
        // Auto equalization follows the complete raw domain used to build the
        // CDF, exactly matching ViewportModel's single-cloud path.
        draw.style.scalar_min = raw.minimum;
        draw.style.scalar_max = raw.maximum;
      } else if (robust) {
        // Auto linear maps a robust P05--P90 range. Degenerate CDFs fall
        // back here, retaining useful contrast without pretending to equalize.
        draw.style.scalar_min = robust->minimum;
        draw.style.scalar_max = robust->maximum;
        draw.style.intensity_equalize = false;
        draw.intensity_cdf_valid = false;
      } else if (automatic) {
        // A constant cloud has a meaningful scalar value even though it has no
        // intra-layer contrast. Keep it centred in a tiny draw-only window
        // rather than silently falling back to the persisted 0--1 defaults.
        const IntensityRange resolved_automatic =
            validIntensityRange(automatic->minimum, automatic->maximum)
                ? *automatic
                : drawableIntensityRange(automatic->minimum);
        draw.style.scalar_min = resolved_automatic.minimum;
        draw.style.scalar_max = resolved_automatic.maximum;
        draw.style.intensity_equalize = false;
        draw.intensity_cdf_valid = false;
      } else {
        draw.style.intensity_equalize = false;
        draw.intensity_cdf_valid = false;
      }
    }
  }
  draw.opacity = std::clamp(item.style.opacity, 0.0F, 1.0F);
  return draw;
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
                                    const SceneCompositeOptions &options,
                                    std::stop_token stop) {
  auto result = std::make_shared<LayeredViewportSnapshot>();
  result->revision = revision;
  if (revision == 0) {
    auto camera_cloud = std::make_shared<ViewportCloudSnapshot>();
    camera_cloud->revision = revision;
    result->camera_cloud = std::move(camera_cloud);
    return result;
  }

  std::vector<const LayerRenderItem *> fit_items;
  fit_items.reserve(render_list.opaque_draw_order.size() +
                    render_list.transparent_draw_order.size());

  const auto shared_visible_range =
      render_list.intensity_scale_mode == IntensityScaleMode::SharedVisible
          ? sharedVisibleIntensityRange(render_list)
          : std::optional<IntensityRange>{};
  const auto append = [&render_list, &options, &result, &fit_items, revision,
                       &shared_visible_range,
                       stop](const std::vector<std::size_t> &order,
                             std::vector<ViewportLayerSnapshot> &destination) {
    std::size_t order_index = 0;
    for (const std::size_t index : order) {
      if ((order_index++ % 4096U) == 0U && stop.stop_requested())
        throw OperationCancelled();
      if (index >= render_list.layers.size())
        continue;
      const LayerRenderItem &item = render_list.layers[index];
      if (options.only_layer && item.layer_id != *options.only_layer)
        continue;
      ViewportLayerSnapshot layer;
      addItemVertices(item, layer.vertices, nullptr, stop);
      if (layer.vertices.empty())
        continue;
      layer.draw = layerDrawState(item, shared_visible_range);
      layer.revision = layerContentRevision(item, layer.draw);
      fit_items.push_back(&item);
      destination.push_back(std::move(layer));
    }
  };
  append(render_list.opaque_draw_order, result->opaque_layers);
  append(render_list.transparent_draw_order, result->transparent_layers);
  result->camera_cloud = makeCameraCloud(fit_items, revision, stop);
  return result;
}

std::shared_ptr<const ViewportCloudSnapshot> composeSceneFitViewportSnapshot(
    const LayerRenderList &render_list, std::uint64_t revision,
    const SceneCompositeOptions &options, std::stop_token stop) {
  return makeCameraCloud(fitItemsFor(render_list, options, false), revision,
                         stop);
}

std::shared_ptr<const ViewportCloudSnapshot> composeSceneViewportSnapshot(
    const LayerRenderList &render_list, std::uint64_t revision,
    const SceneCompositeOptions &options, std::stop_token stop) {
  auto snapshot = std::make_shared<ViewportCloudSnapshot>();
  snapshot->revision = revision;
  if (revision == 0)
    return snapshot;

  const auto shared_visible_range =
      render_list.intensity_scale_mode == IntensityScaleMode::SharedVisible
          ? sharedVisibleIntensityRange(render_list)
          : std::optional<IntensityRange>{};

  const auto append = [&render_list, &options, snapshot, &shared_visible_range,
                       stop](const std::vector<std::size_t> &order) {
    std::size_t order_index = 0;
    for (const std::size_t index : order) {
      if ((order_index++ % 4096U) == 0U && stop.stop_requested())
        throw OperationCancelled();
      if (index >= render_list.layers.size())
        continue;
      const LayerRenderItem &item = render_list.layers[index];
      if (options.only_layer && item.layer_id != *options.only_layer)
        continue;
      const auto draw = layerDrawState(item, shared_visible_range);
      addItemVertices(item, snapshot->vertices, &draw, stop);
    }
  };
  append(render_list.opaque_draw_order);
  append(render_list.transparent_draw_order);
  setBounds(*snapshot);

  appendPickingCandidates(*snapshot);
  return snapshot;
}

} // namespace kpt::gui
