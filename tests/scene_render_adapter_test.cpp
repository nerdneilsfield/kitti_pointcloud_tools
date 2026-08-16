#include "gui/scene/render_adapter.hpp"
#include "gui/viewport/model.hpp"
#include "gui/viewport/scene_compositor.hpp"
#include "kpt/cancellation.hpp"

#include <catch2/catch.hpp>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <stop_token>
#include <string>
#include <utility>
#include <vector>

namespace {

using kpt::PointCloudIRGB;
using kpt::PointCloudIRGBConstPtr;
using kpt::PointT;
using kpt::gui::LayerAdmissionConfig;
using kpt::gui::LayerDetail;
using kpt::gui::LayerPickScope;
using kpt::gui::PickResult;
using kpt::gui::Scene;
using kpt::gui::SceneRenderAdapter;
using kpt::gui::SceneRenderOptions;

PointCloudIRGBConstPtr
makeCloud(std::size_t point_count,
          Eigen::Vector3f first = Eigen::Vector3f::Zero(),
          Eigen::Vector3f step = Eigen::Vector3f::UnitX()) {
  auto cloud = std::make_shared<PointCloudIRGB>();
  cloud->points.reserve(point_count);
  for (std::size_t index = 0; index < point_count; ++index) {
    const auto position = first + step * static_cast<float>(index);
    PointT point;
    point.x = position.x();
    point.y = position.y();
    point.z = position.z();
    point.intensity = static_cast<float>(index);
    cloud->points.push_back(point);
  }
  cloud->width = cloud->points.size();
  return cloud;
}

std::shared_ptr<const kpt::gui::ViewportCloudSnapshot>
snapshot(const PointCloudIRGBConstPtr &cloud, std::uint64_t revision) {
  return kpt::gui::makeViewportCloudSnapshot(cloud, revision);
}

Eigen::Affine3d translate(double x, double y, double z) {
  Eigen::Affine3d transform = Eigen::Affine3d::Identity();
  transform.translation() = Eigen::Vector3d{x, y, z};
  return transform;
}

TEST_CASE(
    "scene render adapter keeps layer state and exposes world fit bounds") {
  Scene scene;
  const auto first_cloud = makeCloud(3, {0.0F, 0.0F, 0.0F}, {1.0F, 1.0F, 1.0F});
  const auto second_cloud =
      makeCloud(2, {0.0F, 0.0F, 0.0F}, {2.0F, 0.0F, 0.0F});
  const auto first = scene.addLayer("first", first_cloud);
  const auto second = scene.addLayer("second", second_cloud);
  REQUIRE(scene.setLayerTransform(first, translate(10.0, 0.0, 0.0)));
  REQUIRE(scene.setLayerTransform(second, translate(20.0, 0.0, 0.0)));
  REQUIRE(scene.setLayerVisible(second, false));
  REQUIRE(scene.setActiveLayer(second));
  scene.setRoi(kpt::gui::RoiBox({9.0, -1.0, -1.0}, {13.0, 3.0, 3.0}));

  kpt::gui::LayerStyle transparent_style;
  transparent_style.opacity = 0.5F;
  REQUIRE(scene.setLayerStyle(second, transparent_style));

  SceneRenderAdapter adapter;
  REQUIRE(adapter.acceptSnapshot(first, snapshot(first_cloud, 1)));
  REQUIRE(adapter.acceptSnapshot(second, snapshot(second_cloud, 1)));
  const auto list = adapter.build(scene);

  REQUIRE(list.layers.size() == 2);
  REQUIRE(list.layers[0].source_key == "opaque:first");
  REQUIRE(list.layers[0].visible);
  REQUIRE(list.layers[0].local_to_world.isApprox(translate(10.0, 0.0, 0.0)));
  REQUIRE(list.layers[0].detail == LayerDetail::Full);
  REQUIRE(list.layers[0].world_roi.has_value());
  REQUIRE(list.layers[0].world_roi->contains(Eigen::Vector3d{10.0, 0.0, 0.0}));
  REQUIRE_FALSE(list.layers[1].visible);
  REQUIRE(list.layers[1].style.opacity == Approx(0.5F));
  REQUIRE((list.opaque_draw_order == std::vector<std::size_t>{0}));
  REQUIRE(list.transparent_draw_order.empty());

  REQUIRE(list.visible_world_bounds.has_value());
  REQUIRE(list.visible_world_bounds->minimum.isApprox(
      Eigen::Vector3d{10.0, 0.0, 0.0}));
  REQUIRE(list.visible_world_bounds->maximum.isApprox(
      Eigen::Vector3d{12.0, 2.0, 2.0}));
  REQUIRE(list.visible_world_bounds->centroid.isApprox(
      Eigen::Vector3d{11.0, 1.0, 1.0}));
  // Active fit observes the same closed world ROI as rendering.  The hidden
  // active layer is entirely outside the ROI, so it has no fit bounds.
  REQUIRE_FALSE(list.active_world_bounds.has_value());
}

TEST_CASE(
    "scene render adapter allocates deterministic uniform LOD and pick scope") {
  Scene scene;
  SceneRenderAdapter adapter;
  constexpr std::size_t layer_count = 5;
  for (std::size_t index = 0; index < layer_count; ++index) {
    const auto cloud = makeCloud(10, {static_cast<float>(index), 0.0F, 0.0F});
    const auto id = scene.addLayer("lod-" + std::to_string(index), cloud);
    REQUIRE(adapter.acceptSnapshot(id, snapshot(cloud, 1)));
  }

  SceneRenderOptions options;
  options.admission.explicit_gpu_budget_bytes =
      10 * sizeof(kpt::gui::ViewportVertex);
  const auto list = adapter.build(scene, options);

  REQUIRE(list.pick_scope == LayerPickScope::ActiveLayerOnly);
  REQUIRE(list.estimated_gpu_bytes == 10 * sizeof(kpt::gui::ViewportVertex));
  for (const auto &item : list.layers) {
    REQUIRE(item.detail == LayerDetail::UniformLod);
    REQUIRE(item.vertex_selection.source_vertex_count == 10);
    REQUIRE(item.vertex_selection.retained_vertex_count == 2);
    REQUIRE(item.vertex_selection.sourceIndex(0) == 0);
    REQUIRE(item.vertex_selection.sourceIndex(1) == 5);
    REQUIRE_FALSE(item.vertex_selection.sourceIndex(2).has_value());
  }
}

TEST_CASE("scene render adapter orders transparent layers back to front") {
  Scene scene;
  const auto cloud = makeCloud(1);
  const auto opaque = scene.addLayer("opaque", cloud);
  const auto near_layer = scene.addLayer("near", cloud);
  const auto far_layer = scene.addLayer("far", cloud);
  REQUIRE(scene.setLayerTransform(near_layer, translate(0.0, 0.0, -2.0)));
  REQUIRE(scene.setLayerTransform(far_layer, translate(0.0, 0.0, -8.0)));
  kpt::gui::LayerStyle transparent_style;
  transparent_style.opacity = 0.25F;
  REQUIRE(scene.setLayerStyle(near_layer, transparent_style));
  REQUIRE(scene.setLayerStyle(far_layer, transparent_style));

  SceneRenderAdapter adapter;
  REQUIRE(adapter.acceptSnapshot(opaque, snapshot(cloud, 1)));
  REQUIRE(adapter.acceptSnapshot(near_layer, snapshot(cloud, 1)));
  REQUIRE(adapter.acceptSnapshot(far_layer, snapshot(cloud, 1)));

  SceneRenderOptions options;
  options.camera_position = Eigen::Vector3d::Zero();
  options.camera_forward = -Eigen::Vector3d::UnitZ();
  const auto list = adapter.build(scene, options);

  REQUIRE((list.opaque_draw_order == std::vector<std::size_t>{0}));
  REQUIRE((list.transparent_draw_order == std::vector<std::size_t>{2, 1}));
}

TEST_CASE("layered compositor preserves alpha separate from point colours") {
  auto red = std::make_shared<PointCloudIRGB>();
  red->push_back(PointT{0.0F, 0.0F, 0.0F, 255, 0, 0, 0, 0.0F});
  auto blue = std::make_shared<PointCloudIRGB>();
  blue->push_back(PointT{0.0F, 0.0F, -1.0F, 0, 0, 255, 0, 0.0F});
  Scene scene;
  const auto opaque = scene.addLayer("opaque", red);
  const auto transparent = scene.addLayer("transparent", blue);
  kpt::gui::LayerStyle transparent_style;
  transparent_style.color_by = kpt::ColorBy::RGB;
  transparent_style.opacity = 0.25F;
  REQUIRE(scene.setLayerStyle(transparent, transparent_style));

  SceneRenderAdapter adapter;
  REQUIRE(adapter.acceptSnapshot(opaque, snapshot(red, 1)));
  REQUIRE(adapter.acceptSnapshot(transparent, snapshot(blue, 1)));
  SceneRenderOptions options;
  options.camera_forward = -Eigen::Vector3d::UnitZ();
  const auto list = adapter.build(scene, options);
  kpt::gui::SceneCompositeOptions composite_options;
  composite_options.background = Eigen::Vector3f{0.8F, 0.6F, 0.4F};
  const auto layered =
      kpt::gui::composeLayeredSceneViewportSnapshot(list, 9, composite_options);

  REQUIRE(layered->revision == 9);
  REQUIRE(layered->camera_cloud);
  REQUIRE(layered->camera_cloud->revision == 9);
  REQUIRE(layered->opaque_layers.size() == 1);
  REQUIRE(layered->transparent_layers.size() == 1);
  REQUIRE(layered->opaque_layers.front().vertices.front().color.isApprox(
      Eigen::Vector3f{1.0F, 0.0F, 0.0F}));
  REQUIRE(layered->transparent_layers.front().vertices.front().color.isApprox(
      Eigen::Vector3f{0.0F, 0.0F, 1.0F}));
  REQUIRE(layered->transparent_layers.front().draw.opacity == Approx(0.25F));
  REQUIRE(layered->transparent_layers.front().draw.style.color_by ==
          kpt::ColorBy::RGB);

  // A camera-only order refresh receives a new scene revision for the camera
  // model, while backend uploads retain layer-local buffers by content
  // revision. Editing a layer style must invalidate that buffer payload.
  const auto reordered = kpt::gui::composeLayeredSceneViewportSnapshot(
      list, 10, composite_options);
  REQUIRE(reordered->camera_cloud->revision == 10);
  REQUIRE(reordered->transparent_layers.front().revision ==
          layered->transparent_layers.front().revision);
  transparent_style.opacity = 0.5F;
  REQUIRE(scene.setLayerStyle(transparent, transparent_style));
  const auto edited = kpt::gui::composeLayeredSceneViewportSnapshot(
      adapter.build(scene, options), 11, composite_options);
  REQUIRE(edited->transparent_layers.front().revision !=
          layered->transparent_layers.front().revision);
}

TEST_CASE("layered compositor applies closed world ROI after transforms") {
  const auto cloud = makeCloud(2, {0.0F, 0.0F, 0.0F}, {2.0F, 0.0F, 0.0F});
  Scene scene;
  const auto layer = scene.addLayer("roi", cloud);
  REQUIRE(scene.setLayerTransform(layer, translate(10.0, 0.0, 0.0)));
  scene.setRoi(kpt::gui::RoiBox{Eigen::Vector3d{10.0, 0.0, 0.0},
                                Eigen::Vector3d{10.0, 0.0, 0.0}});
  SceneRenderAdapter adapter;
  REQUIRE(adapter.acceptSnapshot(layer, snapshot(cloud, 1)));
  const auto layered =
      kpt::gui::composeLayeredSceneViewportSnapshot(adapter.build(scene), 11);

  REQUIRE(layered->opaque_layers.size() == 1);
  REQUIRE(layered->opaque_layers.front().vertices.size() == 1);
  REQUIRE(layered->opaque_layers.front().vertices.front().position.isApprox(
      Eigen::Vector3f{10.0F, 0.0F, 0.0F}));
  REQUIRE(layered->camera_cloud->vertices.size() == 1);
}

TEST_CASE("layered compositor preserves imported world Z scalar ranges") {
  const auto cloud = makeCloud(2, {0.0F, 0.0F, 1.0F},
                               {0.0F, 0.0F, 2.0F});
  Scene scene;
  const auto layer = scene.addLayer("z-range", cloud);
  REQUIRE(scene.setLayerTransform(layer, translate(0.0, 0.0, 100.0)));
  kpt::gui::LayerStyle style;
  style.color_by = kpt::ColorBy::Z;
  style.scalar_min = 100.0F;
  style.scalar_max = 104.0F;
  REQUIRE(scene.setLayerStyle(layer, style));

  SceneRenderAdapter adapter;
  REQUIRE(adapter.acceptSnapshot(layer, snapshot(cloud, 1)));
  const auto list = adapter.build(scene);
  const auto layered =
      kpt::gui::composeLayeredSceneViewportSnapshot(list, 3);

  REQUIRE(layered->opaque_layers.size() == 1);
  const auto &draw = layered->opaque_layers.front().draw;
  REQUIRE(draw.style.color_by == kpt::ColorBy::Z);
  REQUIRE(draw.style.scalar_min == Approx(100.0F));
  REQUIRE(draw.style.scalar_max == Approx(104.0F));

  // Legacy single-cloud callers style transformed Z coordinates with the
  // same imported range, rather than silently reverting to local bounds.
  const auto compatibility = kpt::gui::composeSceneViewportSnapshot(list, 4);
  REQUIRE(compatibility->vertices.size() == 2);
  REQUIRE_FALSE(compatibility->vertices[0].color.isApprox(
      compatibility->vertices[1].color));
}

TEST_CASE("scene render adapter applies ROI before LOD and fit bounds") {
  const auto cloud = makeCloud(10, {0.0F, 0.0F, 0.0F}, {1.0F, 0.0F, 0.0F});
  Scene scene;
  const auto layer = scene.addLayer("roi-lod", cloud);
  // Keep only a sparse middle interval.  A pre-ROI uniform 2-point selection
  // would choose indices 0 and 5 and incorrectly miss index 4.
  scene.setRoi(kpt::gui::RoiBox{Eigen::Vector3d{4.0, 0.0, 0.0},
                                Eigen::Vector3d{5.0, 0.0, 0.0}});
  SceneRenderAdapter adapter;
  REQUIRE(adapter.acceptSnapshot(layer, snapshot(cloud, 1)));

  SceneRenderOptions options;
  options.maximum_render_vertices = 1;
  const auto list = adapter.build(scene, options);
  REQUIRE(list.visible_world_bounds.has_value());
  REQUIRE(list.visible_world_bounds->minimum.isApprox(
      Eigen::Vector3d{4.0, 0.0, 0.0}));
  REQUIRE(list.visible_world_bounds->maximum.isApprox(
      Eigen::Vector3d{5.0, 0.0, 0.0}));
  REQUIRE(list.layers.front().vertex_selection.source_vertex_count == 10);
  REQUIRE(list.layers.front().vertex_selection.eligible_vertex_count == 2);
  REQUIRE(list.layers.front().vertex_selection.retained_vertex_count == 1);
  REQUIRE(list.layers.front().vertex_selection.requiresEligibilityScan());
  REQUIRE_FALSE(list.layers.front().vertex_selection.sourceIndex(0));

  const auto layered = kpt::gui::composeLayeredSceneViewportSnapshot(list, 7);
  REQUIRE(layered->camera_cloud);
  // GPU admission retains one LOD point, but camera framing samples actual
  // accepted ROI points rather than inheriting that lossy upload selection.
  REQUIRE(layered->camera_cloud->vertices.size() == 2);
  REQUIRE(layered->camera_cloud->bounds.centroid.isApprox(
      Eigen::Vector3f{4.5F, 0.0F, 0.0F}));
}

TEST_CASE("layered camera fit samples actual bounded ROI geometry",
          "[scene][roi]") {
  constexpr std::size_t accepted_points = 100'001U;
  auto mutable_cloud = std::make_shared<PointCloudIRGB>();
  mutable_cloud->points.reserve(accepted_points + 2U);
  for (std::size_t index = 0; index < accepted_points; ++index) {
    PointT point;
    point.x = static_cast<float>(index);
    mutable_cloud->push_back(std::move(point));
  }
  // These are deliberately extreme AABB corners.  They must neither appear
  // in fit candidates nor distort the 95th-percentile FOV after ROI.
  mutable_cloud->push_back(PointT{-1'000'000.0F, 0.0F, 0.0F});
  mutable_cloud->push_back(PointT{1'000'000.0F, 0.0F, 0.0F});
  mutable_cloud->width = mutable_cloud->points.size();
  const PointCloudIRGBConstPtr cloud = std::move(mutable_cloud);

  Scene scene;
  const auto layer = scene.addLayer("bounded-fit", cloud);
  scene.setRoi(kpt::gui::RoiBox{{0.0, -1.0, -1.0}, {100'000.0, 1.0, 1.0}});
  SceneRenderAdapter adapter;
  REQUIRE(adapter.acceptSnapshot(layer, snapshot(cloud, 1)));

  SceneRenderOptions options;
  const auto list = adapter.build(scene, options);
  REQUIRE(list.layers.front().vertex_selection.retained_vertex_count ==
          accepted_points);
  const auto layered = kpt::gui::composeLayeredSceneViewportSnapshot(list, 2);
  REQUIRE(layered->camera_cloud);
  REQUIRE(layered->opaque_layers.front().vertices.size() == accepted_points);
  REQUIRE(layered->camera_cloud->vertices.size() == 100'000U);
  REQUIRE(std::all_of(
      layered->camera_cloud->vertices.begin(),
      layered->camera_cloud->vertices.end(), [](const auto &vertex) {
        return vertex.position.x() >= 0.0F && vertex.position.x() <= 100'000.0F;
      }));
  REQUIRE(layered->camera_cloud->bounds.centroid.isApprox(
      Eigen::Vector3f{50'000.0F, 0.0F, 0.0F}));

  kpt::gui::ViewportModel model;
  model.setCloud(layered->camera_cloud, kpt::gui::CameraUpdate::Fit);
  static_cast<void>(model.frame({1280, 720}));
  const auto camera = model.cameraSnapshot();
  REQUIRE(camera.target.isApprox(Eigen::Vector3d{50'000.0, 0.0, 0.0}));
  REQUIRE(camera.fov_y_degrees >= 35.0F);
  REQUIRE(camera.fov_y_degrees <= 75.0F);

  // Active fit remains useful for an intentionally hidden reference layer;
  // it must still use real transformed/ROI points, not synthesize AABB
  // corners or require a visible GPU payload.
  REQUIRE(scene.setLayerVisible(layer, false));
  REQUIRE(scene.setActiveLayer(layer));
  const auto hidden_list = adapter.build(scene, options);
  kpt::gui::SceneCompositeOptions active_only;
  active_only.only_layer = layer;
  const auto active_fit = kpt::gui::composeSceneFitViewportSnapshot(
      hidden_list, 3, active_only);
  REQUIRE(active_fit->vertices.size() == 100'000U);
  REQUIRE(active_fit->bounds.centroid.isApprox(
      Eigen::Vector3f{50'000.0F, 0.0F, 0.0F}));
}

TEST_CASE("visible fit includes deferred layers outside GPU admission",
          "[scene]") {
  const auto cloud = makeCloud(3, {10.0F, 0.0F, 0.0F},
                               {2.0F, 0.0F, 0.0F});
  Scene scene;
  const auto layer = scene.addLayer("deferred-fit", cloud);
  SceneRenderAdapter adapter;
  REQUIRE(adapter.acceptSnapshot(layer, snapshot(cloud, 1)));

  SceneRenderOptions options;
  options.maximum_render_vertices = 0;
  const auto list = adapter.build(scene, options);
  REQUIRE(list.layers.front().detail == LayerDetail::Deferred);
  REQUIRE(list.layers.front().vertex_selection.retained_vertex_count == 0);
  const auto fit = kpt::gui::composeSceneFitViewportSnapshot(list, 2);
  REQUIRE(fit->vertices.size() == 3);
  REQUIRE(fit->bounds.centroid.isApprox(Eigen::Vector3f{12.0F, 0.0F, 0.0F}));
}

TEST_CASE("scene render snapshot is worker-safe and ROI build observes stop",
          "[scene][roi]") {
  const auto cloud = makeCloud(16'384);
  Scene scene;
  const auto layer = scene.addLayer("cancellable-roi", cloud);
  scene.setRoi(kpt::gui::RoiBox{Eigen::Vector3d{0.0, -1.0, -1.0},
                                Eigen::Vector3d{16'383.0, 1.0, 1.0}});
  SceneRenderAdapter adapter;
  REQUIRE(adapter.acceptSnapshot(layer, snapshot(cloud, 1)));
  const auto captured = adapter.capture(scene);
  REQUIRE(captured.layers.size() == 1);
  REQUIRE(captured.layers.front().snapshot);

  std::stop_source stop;
  stop.request_stop();
  REQUIRE_THROWS_AS(SceneRenderAdapter::build(captured, {}, stop.get_token()),
                    kpt::OperationCancelled);
}

TEST_CASE("scene compositor observes a cancelled review generation",
          "[scene][roi]") {
  const auto cloud = makeCloud(16'384);
  Scene scene;
  const auto layer = scene.addLayer("cancel-compose", cloud);
  SceneRenderAdapter adapter;
  REQUIRE(adapter.acceptSnapshot(layer, snapshot(cloud, 1)));
  const auto list = adapter.build(scene);

  std::stop_source stop;
  stop.request_stop();
  REQUIRE_THROWS_AS(kpt::gui::composeLayeredSceneViewportSnapshot(
                        list, 2, {}, stop.get_token()),
                    kpt::OperationCancelled);
  REQUIRE_THROWS_AS(
      kpt::gui::composeSceneViewportSnapshot(list, 2, {}, stop.get_token()),
      kpt::OperationCancelled);
}

TEST_CASE(
    "scene render adapter resolves local picks and rejects stale snapshots") {
  Scene scene;
  const auto cloud = makeCloud(3);
  const auto layer = scene.addLayer("scan", cloud);
  REQUIRE(scene.setLayerTransform(layer, translate(100.0, -2.0, 3.0)));

  SceneRenderAdapter adapter;
  REQUIRE(adapter.acceptSnapshot(layer, snapshot(cloud, 4)));
  REQUIRE_FALSE(adapter.acceptSnapshot(layer, snapshot(cloud, 3)));

  PickResult local_pick;
  local_pick.cloud_position = {1.0F, 2.0F, 3.0F};
  local_pick.world_position = {-999.0F, -999.0F, -999.0F};
  local_pick.intensity = 7.0F;
  local_pick.noise = 1.0F;
  const auto resolved = adapter.resolvePick(scene, layer, local_pick);
  REQUIRE(resolved.has_value());
  REQUIRE(resolved->source_key == "opaque:scan");
  REQUIRE(resolved->local_position.isApprox(Eigen::Vector3f{1.0F, 2.0F, 3.0F}));
  REQUIRE(resolved->world_position.isApprox(Eigen::Vector3d{101.0, 0.0, 6.0}));
  REQUIRE(resolved->intensity == Approx(7.0F));
  REQUIRE_FALSE(adapter.resolvePick(scene, layer + 1, local_pick).has_value());

  REQUIRE(scene.removeLayer(layer));
  adapter.pruneMissingLayers(scene);
  const auto empty = adapter.build(scene);
  REQUIRE(empty.layers.empty());
}

TEST_CASE("layer admission budget follows RAM policy and style rejects invalid "
          "values") {
  LayerAdmissionConfig admission;
  REQUIRE(admission.resolvedGpuBudgetBytes() ==
          LayerAdmissionConfig::kFallbackGpuBudgetBytes);
  admission.available_system_memory_bytes = 1024ULL * 1024ULL * 1024ULL;
  REQUIRE(admission.resolvedGpuBudgetBytes() == 256ULL * 1024ULL * 1024ULL);
  admission.available_system_memory_bytes = 16ULL * 1024ULL * 1024ULL * 1024ULL;
  REQUIRE(admission.resolvedGpuBudgetBytes() ==
          LayerAdmissionConfig::kMaximumGpuBudgetBytes);
  admission.explicit_gpu_budget_bytes = 2ULL * 1024ULL * 1024ULL * 1024ULL;
  REQUIRE(admission.resolvedGpuBudgetBytes() ==
          LayerAdmissionConfig::kMaximumGpuBudgetBytes);

  Scene scene;
  const auto layer = scene.addLayer("style");
  kpt::gui::LayerStyle invalid_style;
  invalid_style.opacity = 1.1F;
  REQUIRE_FALSE(kpt::gui::isValidLayerStyle(invalid_style));
  REQUIRE_THROWS(scene.setLayerStyle(layer, invalid_style));
  REQUIRE(scene.findLayer(layer)->style().opacity == Approx(1.0F));
}

} // namespace
