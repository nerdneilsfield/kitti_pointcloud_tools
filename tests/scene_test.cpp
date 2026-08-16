#include "gui/scene/scene.hpp"

#include <catch2/catch.hpp>

#include <filesystem>
#include <limits>
#include <stdexcept>
#include <string>

namespace {

using kpt::gui::RoiBox;
using kpt::gui::Scene;

Eigen::Vector3d point(double x, double y, double z) { return {x, y, z}; }

TEST_CASE("scene allocates monotonic runtime IDs for stable source keys") {
  Scene scene;

  const auto first = scene.addLayer("scan-a");
  const auto second = scene.addLayer("scan-b");

  REQUIRE(first == 1);
  REQUIRE(second == 2);
  REQUIRE(scene.removeLayer(first));
  REQUIRE(scene.addLayer("scan-c") == 3);
  REQUIRE(scene.findLayerBySourceKey("scan-b")->id() == second);
  REQUIRE_THROWS_AS(scene.addLayer("scan-b"), std::invalid_argument);
  REQUIRE_THROWS_AS(scene.addLayer(""), std::invalid_argument);
}

TEST_CASE("path source keys resolve relative paths against an explicit base") {
  const std::filesystem::path base{"/review/session"};
  const auto relative =
      kpt::gui::pathSourceKey("captures/../scan.kpt", base);
  const auto absolute =
      kpt::gui::pathSourceKey("/review/session/scan.kpt", "/ignored/base");
  const auto dotted_absolute =
      kpt::gui::pathSourceKey("/review/session/./scan.kpt", {});

  REQUIRE(relative == "path:/review/session/scan.kpt");
  REQUIRE(relative == absolute);
  REQUIRE(relative == dotted_absolute);
  REQUIRE(kpt::gui::isCanonicalSourceKey(relative));
  REQUIRE_FALSE(kpt::gui::isCanonicalSourceKey(
      "path:/review/session/captures/../scan.kpt"));
  REQUIRE_THROWS_AS(kpt::gui::pathSourceKey("scan.kpt", "relative/base"),
                    std::invalid_argument);
  REQUIRE_THROWS_AS(kpt::gui::pathSourceKey({}, base), std::invalid_argument);
}

TEST_CASE("scene canonicalizes legacy opaque keys and rejects path aliases") {
  Scene scene;
  const auto opaque = kpt::gui::opaqueSourceKey("stream:42");
  REQUIRE(opaque == "opaque:stream:42");
  REQUIRE(kpt::gui::isCanonicalSourceKey(opaque));
  REQUIRE(kpt::gui::isCanonicalSourceKey("opaque:stream/camera-42"));
  REQUIRE_FALSE(kpt::gui::isCanonicalSourceKey("opaque:bad\nkey"));
  REQUIRE_THROWS_AS(kpt::gui::opaqueSourceKey("bad\nkey"),
                    std::invalid_argument);

  constexpr std::string_view hashed =
      "sha256:0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef";
  REQUIRE(kpt::gui::isCanonicalSourceKey(hashed));
  const auto hashed_layer = scene.addLayer(std::string{hashed});
  REQUIRE(scene.findLayer(hashed_layer)->sourceKey() == hashed);
  REQUIRE_FALSE(kpt::gui::isCanonicalSourceKey(
      "sha256:0123456789ABCDEF0123456789abcdef0123456789abcdef0123456789abcdef"));
  REQUIRE_THROWS_AS(scene.addLayer("sha256:not-a-digest"),
                    std::invalid_argument);
  REQUIRE(kpt::gui::isCanonicalSourceKey("path:C:/review/scan.xyz"));
  REQUIRE_FALSE(kpt::gui::isCanonicalSourceKey("path:C:\\review\\scan.xyz"));

  const auto path = kpt::gui::pathSourceKey("scan.kpt", "/review/session");
  static_cast<void>(scene.addLayer(path));
  REQUIRE_THROWS_AS(scene.addLayer("path:/review/session/./scan.kpt"),
                    std::invalid_argument);
  REQUIRE_THROWS_AS(scene.addLayer(path), std::invalid_argument);

  const auto legacy = scene.addLayer("stream:42");
  REQUIRE(scene.findLayer(legacy)->sourceKey() == opaque);
  REQUIRE(scene.findLayerBySourceKey("stream:42")->id() == legacy);
  REQUIRE_THROWS_AS(scene.addLayer("stream:42"), std::invalid_argument);
  REQUIRE_THROWS_AS(kpt::gui::opaqueSourceKey(""), std::invalid_argument);
}

TEST_CASE("scene owns the active layer and repairs it after removal") {
  Scene scene;
  const auto first = scene.addLayer("scan-a");
  const auto second = scene.addLayer("scan-b");
  REQUIRE(scene.activeLayer() == first);
  REQUIRE(scene.setActiveLayer(second));
  REQUIRE(scene.activeLayer() == second);
  REQUIRE_FALSE(scene.setActiveLayer(999));
  REQUIRE(scene.activeLayer() == second);
  REQUIRE(scene.removeLayer(second));
  REQUIRE(scene.activeLayer() == first);
  REQUIRE(scene.removeLayer(first));
  REQUIRE_FALSE(scene.activeLayer().has_value());
}

TEST_CASE("clearing layers removes stale active state but retains measurement history") {
  Scene scene;
  const auto first = scene.addLayer("scan-a");
  static_cast<void>(scene.addMeasurement("scan-a", point(1.0, 2.0, 3.0)));

  scene.clearLayers();

  REQUIRE(scene.layers().empty());
  REQUIRE_FALSE(scene.activeLayer().has_value());
  REQUIRE(scene.measurements().size() == 1);
  REQUIRE(scene.measurementDetached(scene.measurements().front()));

  const auto replacement = scene.addLayer("scan-b");
  REQUIRE(replacement > first);
  REQUIRE(scene.activeLayer() == replacement);
}

TEST_CASE("share import reset drops document state without reusing runtime IDs") {
  Scene scene;
  const auto original_layer = scene.addLayer("scan-a");
  static_cast<void>(scene.addMeasurement("scan-a", point(1.0, 2.0, 3.0)));
  scene.setRoi(RoiBox({0.0, 0.0, 0.0}, {1.0, 1.0, 1.0}));

  scene.resetForImport();

  REQUIRE(scene.layers().empty());
  REQUIRE(scene.measurements().empty());
  REQUIRE_FALSE(scene.roi().has_value());
  REQUIRE_FALSE(scene.activeLayer().has_value());
  REQUIRE_FALSE(scene.undo());
  REQUIRE(scene.addLayer("scan-b") > original_layer);
}

TEST_CASE("share import can establish a history root after hydration") {
  Scene scene;
  const auto layer = scene.addLayer("scan-a");
  REQUIRE(scene.setLayerVisible(layer, false));
  REQUIRE(scene.undo());
  REQUIRE(scene.setLayerVisible(layer, false));

  scene.clearHistory();

  REQUIRE_FALSE(scene.undo());
  REQUIRE_FALSE(scene.redo());
  REQUIRE_FALSE(scene.findLayer(layer)->visible());
}

TEST_CASE("inspection settings replace bookmarks by their stable name") {
  kpt::gui::InspectionSettings settings;
  kpt::gui::CameraSnapshot first;
  first.distance = 2.0;
  settings.saveBookmark({"overview", first});
  auto second = first;
  second.distance = 4.0;
  settings.saveBookmark({"overview", second});

  REQUIRE(settings.bookmarks().size() == 1);
  REQUIRE(settings.findBookmark("overview")->camera().distance == Approx(4.0));
  REQUIRE(settings.removeBookmark("overview"));
  REQUIRE_FALSE(settings.removeBookmark("overview"));
  REQUIRE_THROWS_AS(kpt::gui::CameraBookmark("", first), std::invalid_argument);
}

TEST_CASE("ROI is finite closed world-space AABB") {
  RoiBox roi({-1.0, -2.0, -3.0}, {4.0, 5.0, 6.0});

  REQUIRE(roi.contains({-1.0, -2.0, -3.0}));
  REQUIRE(roi.contains({4.0, 5.0, 6.0}));
  REQUIRE_FALSE(roi.contains({4.001, 5.0, 6.0}));
  REQUIRE_FALSE(roi.contains({std::numeric_limits<double>::quiet_NaN(), 0.0,
                              0.0}));
  REQUIRE_THROWS_AS(RoiBox({1.0, 0.0, 0.0}, {0.0, 1.0, 1.0}),
                    std::invalid_argument);
  REQUIRE_THROWS_AS(RoiBox({0.0, 0.0, 0.0},
                            {std::numeric_limits<double>::infinity(), 1.0,
                             1.0}),
                    std::invalid_argument);
}

TEST_CASE("ROI filters local points after a finite forward affine transform") {
  RoiBox roi({0.0, 0.0, 0.0}, {2.0, 2.0, 2.0});
  Eigen::Affine3d translate = Eigen::Affine3d::Identity();
  translate.translation() = point(1.0, 1.0, 1.0);

  REQUIRE(roi.containsTransformedLocal(point(-1.0, -1.0, -1.0), translate));
  REQUIRE(roi.containsTransformedLocal(point(1.0, 1.0, 1.0), translate));
  REQUIRE_FALSE(roi.containsTransformedLocal(point(1.001, 1.0, 1.0), translate));
  REQUIRE(kpt::gui::transformLocalToWorld(point(0.0, 0.0, 0.0), translate)
              ->isApprox(point(1.0, 1.0, 1.0)));

  Eigen::Affine3d non_finite = Eigen::Affine3d::Identity();
  non_finite.matrix()(0, 0) = std::numeric_limits<double>::quiet_NaN();
  REQUIRE_FALSE(roi.containsTransformedLocal(point(0.0, 0.0, 0.0), non_finite));
  REQUIRE_FALSE(kpt::gui::transformLocalToWorld(point(0.0, 0.0, 0.0),
                                                 non_finite));

  Eigen::Affine3d projective = Eigen::Affine3d::Identity();
  projective.matrix()(3, 0) = 0.5;
  REQUIRE_FALSE(roi.containsTransformedLocal(point(0.0, 0.0, 0.0), projective));
  Scene scene;
  const auto layer_id = scene.addLayer("scan-a");
  REQUIRE_THROWS_AS(scene.setLayerTransform(layer_id, projective),
                    std::invalid_argument);
}

TEST_CASE("measurements retain immutable world points when their layer changes") {
  Scene scene;
  const auto layer_id = scene.addLayer("scan-a");
  const auto measurement_id =
      scene.addMeasurement("scan-a", point(1.0, 2.0, 3.0),
                           point(4.0, 6.0, 3.0));

  const auto *layer = scene.findLayer(layer_id);
  REQUIRE(layer != nullptr);
  Eigen::Affine3d translated = Eigen::Affine3d::Identity();
  translated.translation() = point(100.0, 0.0, 0.0);
  REQUIRE(scene.setLayerTransform(layer_id, translated));

  const auto &measurement = scene.measurements().front();
  REQUIRE(measurement.id() == measurement_id);
  REQUIRE(measurement.firstWorld().isApprox(point(1.0, 2.0, 3.0)));
  REQUIRE(measurement.distance() == Approx(5.0));
  REQUIRE_FALSE(scene.measurementDetached(measurement));
  REQUIRE(scene.removeLayer(layer_id));
  REQUIRE(scene.measurementDetached(measurement));
  REQUIRE(measurement.firstWorld().isApprox(point(1.0, 2.0, 3.0)));
}

TEST_CASE("measurement picking completes pending IDs without duplicate records") {
  Scene scene;
  static_cast<void>(scene.addLayer("scan-a"));

  const auto first = scene.beginMeasurement("scan-a", point(1.0, 2.0, 3.0));
  REQUIRE(scene.measurements().size() == 1);
  REQUIRE_FALSE(scene.measurements().front().secondWorld().has_value());
  REQUIRE(scene.completeMeasurement(first, point(4.0, 2.0, 3.0)));
  REQUIRE(scene.measurements().size() == 1);
  REQUIRE(scene.measurements().front().id() == first);
  REQUIRE(scene.measurements().front().distance() == Approx(3.0));

  const auto second = scene.beginMeasurement("scan-a", point(5.0, 0.0, 0.0));
  REQUIRE(second > first);
  REQUIRE(scene.measurements().size() == 2);
  REQUIRE_FALSE(scene.measurements().back().secondWorld().has_value());
  REQUIRE(scene.completeMeasurement(second, point(5.0, 4.0, 0.0)));
  REQUIRE(scene.measurements().size() == 2);
  REQUIRE(scene.measurements().back().id() == second);
  REQUIRE(scene.measurements().back().distance() == Approx(4.0));
  REQUIRE_FALSE(scene.completeMeasurement(second, point(5.0, 5.0, 0.0)));
}

TEST_CASE("clearing measurements is undoable without losing completed IDs") {
  Scene scene;
  const auto first = scene.addMeasurement("scan-a", point(0.0, 0.0, 0.0),
                                          point(1.0, 0.0, 0.0));
  const auto second = scene.beginMeasurement("scan-a", point(2.0, 0.0, 0.0));
  REQUIRE(scene.completeMeasurement(second, point(3.0, 0.0, 0.0)));
  REQUIRE(scene.clearMeasurements());
  REQUIRE(scene.measurements().empty());
  REQUIRE_FALSE(scene.clearMeasurements());

  REQUIRE(scene.undo());
  REQUIRE(scene.measurements().size() == 2);
  REQUIRE(scene.measurements()[0].id() == first);
  REQUIRE(scene.measurements()[1].id() == second);
  REQUIRE(scene.measurements()[1].distance() == Approx(1.0));
  REQUIRE(scene.redo());
  REQUIRE(scene.measurements().empty());
}

TEST_CASE("layer identity is immutable while scene owns safe edits") {
  Scene scene;
  const auto layer_id = scene.addLayer("stable-source");
  const auto *layer = scene.findLayer(layer_id);
  REQUIRE(layer->id() == layer_id);
  REQUIRE(layer->sourceKey() == "opaque:stable-source");
  REQUIRE(layer->visible());

  REQUIRE(scene.setLayerVisible(layer_id, false));
  REQUIRE_FALSE(scene.findLayer(layer_id)->visible());
  REQUIRE_FALSE(scene.setLayerVisible(layer_id + 1, true));
  REQUIRE_THROWS_AS(scene.setLayerTransform(
                        layer_id,
                        Eigen::Affine3d(Eigen::Matrix4d::Constant(
                            std::numeric_limits<double>::quiet_NaN()))),
                    std::invalid_argument);
  REQUIRE(scene.findLayer(layer_id)->localToWorld().isApprox(
      Eigen::Affine3d::Identity()));
}

TEST_CASE("unresolved share layers hydrate without replacing review identity") {
  Scene scene;
  const auto layer_id = scene.addLayer("path:/review/session/scan.pcd");
  Eigen::Affine3d transform = Eigen::Affine3d::Identity();
  transform.translation() = point(5.0, 6.0, 7.0);
  REQUIRE(scene.setLayerTransform(layer_id, transform));
  REQUIRE(scene.setLayerVisible(layer_id, false));

  auto cloud = std::make_shared<kpt::PointCloudIRGB>();
  cloud->points.push_back({});
  REQUIRE(scene.setLayerCloud(layer_id, cloud));
  const auto *hydrated = scene.findLayer(layer_id);
  REQUIRE(hydrated != nullptr);
  REQUIRE(hydrated->sourceKey() == "path:/review/session/scan.pcd");
  REQUIRE(hydrated->cloud() == cloud);
  REQUIRE(hydrated->localToWorld().isApprox(transform));
  REQUIRE_FALSE(hydrated->visible());

  REQUIRE(scene.undo());
  REQUIRE_FALSE(scene.findLayer(layer_id)->cloud());
  REQUIRE(scene.redo());
  REQUIRE(scene.findLayer(layer_id)->cloud() == cloud);
  REQUIRE_FALSE(scene.setLayerCloud(layer_id + 100, cloud));
}

TEST_CASE("import hydration establishes a non-undoable history root") {
  Scene scene;
  scene.resetForImport();
  const auto layer_id = scene.addLayer("path:/review/session/scan.pcd");
  scene.clearHistory();

  auto cloud = std::make_shared<kpt::PointCloudIRGB>();
  cloud->points.push_back({});
  REQUIRE(scene.hydrateLayerCloud(layer_id, cloud));
  REQUIRE(scene.findLayer(layer_id)->cloud() == cloud);
  REQUIRE_FALSE(scene.undo());
  REQUIRE(scene.findLayer(layer_id)->cloud() == cloud);
  REQUIRE_FALSE(scene.hydrateLayerCloud(layer_id + 100, cloud));
}

TEST_CASE("measurements reject missing source keys and non-finite world points") {
  REQUIRE_THROWS_AS(kpt::gui::Measurement(1, "", {0.0, 0.0, 0.0}),
                    std::invalid_argument);
  REQUIRE_THROWS_AS(kpt::gui::Measurement(
                        1, "scan-a",
                        {std::numeric_limits<double>::infinity(), 0.0, 0.0}),
                    std::invalid_argument);
}

TEST_CASE("undo stack executes, branches, and retains at most 100 commands") {
  kpt::gui::UndoStack stack;
  int value = 0;
  const auto increment = [&value] {
    return kpt::gui::UndoStack::Command{
        [&value] { --value; }, [&value] { ++value; }};
  };

  stack.execute(increment());
  stack.execute(increment());
  REQUIRE(value == 2);
  REQUIRE(stack.undo());
  REQUIRE(value == 1);
  REQUIRE(stack.redo());
  REQUIRE(value == 2);
  REQUIRE(stack.undo());
  stack.execute(increment());
  REQUIRE_FALSE(stack.redo());

  for (int index = 0; index < 100; ++index) {
    stack.execute(increment());
  }
  REQUIRE(stack.undoCount() == kpt::gui::UndoStack::kCapacity);
  REQUIRE(stack.redoCount() == 0);
  REQUIRE(value == 102);
}

TEST_CASE("undo stack has fixed command capacity before executing callbacks") {
  kpt::gui::UndoStack stack;
  int value = 0;
  for (int index = 0; index < static_cast<int>(kpt::gui::UndoStack::kCapacity);
       ++index) {
    stack.execute({[&value] { --value; }, [&value] { ++value; }});
  }
  REQUIRE(value == static_cast<int>(kpt::gui::UndoStack::kCapacity));
  REQUIRE(stack.undoCount() == kpt::gui::UndoStack::kCapacity);
}

TEST_CASE("undo stack rejects incomplete commands without changing history") {
  kpt::gui::UndoStack stack;
  REQUIRE_THROWS_AS(stack.execute({{}, [] {}}), std::invalid_argument);
  REQUIRE(stack.undoCount() == 0);
  REQUIRE_FALSE(stack.undo());
  REQUIRE_FALSE(stack.redo());
}

TEST_CASE("undo and redo retain history when a callback throws") {
  kpt::gui::UndoStack stack;
  bool fail_undo = true;
  stack.execute({[&fail_undo] {
                   if (fail_undo) {
                     throw std::runtime_error("undo failure");
                   }
                 },
                 [] {}});

  REQUIRE_THROWS_AS(stack.undo(), std::runtime_error);
  REQUIRE(stack.undoCount() == 1);
  REQUIRE(stack.redoCount() == 0);
  fail_undo = false;
  REQUIRE(stack.undo());

  kpt::gui::UndoStack redo_stack;
  bool fail_redo = false;
  redo_stack.execute({[] {}, [&fail_redo] {
                       if (fail_redo) {
                         throw std::runtime_error("redo failure");
                       }
                     }});
  REQUIRE(redo_stack.undo());
  fail_redo = true;
  REQUIRE_THROWS_AS(redo_stack.redo(), std::runtime_error);
  REQUIRE(redo_stack.undoCount() == 0);
  REQUIRE(redo_stack.redoCount() == 1);
}

TEST_CASE("scene review edits are one transactional undo record") {
  Scene scene;
  const auto layer = scene.addLayer("scan-a");
  const auto original_style = scene.findLayer(layer)->style();

  REQUIRE(scene.beginTransaction());
  REQUIRE(scene.transactionActive());
  Eigen::Affine3d moved = Eigen::Affine3d::Identity();
  moved.translation() = point(4.0, -2.0, 8.0);
  REQUIRE(scene.setLayerTransform(layer, moved));
  REQUIRE(scene.setLayerVisible(layer, false));
  auto style = original_style;
  style.opacity = 0.35F;
  style.point_size = 6.0F;
  REQUIRE(scene.setLayerStyle(layer, style));
  scene.setRoi(RoiBox({-1.0, -2.0, -3.0}, {10.0, 20.0, 30.0}));
  REQUIRE(scene.commitTransaction());
  REQUIRE_FALSE(scene.transactionActive());

  const auto *edited = scene.findLayer(layer);
  REQUIRE_FALSE(edited->visible());
  REQUIRE(edited->localToWorld().isApprox(moved));
  REQUIRE(edited->style().opacity == Approx(0.35F));
  REQUIRE(scene.roi().has_value());

  REQUIRE(scene.undo());
  const auto *restored = scene.findLayer(layer);
  REQUIRE(restored->visible());
  REQUIRE(restored->localToWorld().isApprox(Eigen::Affine3d::Identity()));
  REQUIRE(restored->style().opacity == Approx(original_style.opacity));
  REQUIRE_FALSE(scene.roi().has_value());

  REQUIRE(scene.redo());
  REQUIRE_FALSE(scene.findLayer(layer)->visible());
  REQUIRE(scene.findLayer(layer)->localToWorld().isApprox(moved));
  REQUIRE(scene.roi()->maximum().isApprox(point(10.0, 20.0, 30.0)));
}

TEST_CASE("scene layer mutations retain identity across undo and redo") {
  Scene scene;
  const auto first = scene.addLayer("scan-a");
  const auto second = scene.addLayer("scan-b");
  REQUIRE(scene.setActiveLayer(second));

  REQUIRE(scene.removeLayer(second));
  REQUIRE(scene.findLayer(second) == nullptr);
  REQUIRE(scene.activeLayer() == first);
  REQUIRE(scene.undo());
  REQUIRE(scene.findLayer(second) != nullptr);
  REQUIRE(scene.findLayer(second)->sourceKey() == "opaque:scan-b");
  REQUIRE(scene.activeLayer() == second);
  REQUIRE(scene.redo());
  REQUIRE(scene.findLayer(second) == nullptr);
  REQUIRE(scene.activeLayer() == first);
}

TEST_CASE("measurement endpoints can belong to different review layers") {
  Scene scene;
  const auto first_layer = scene.addLayer("scan-a");
  const auto second_layer = scene.addLayer("scan-b");
  const auto measurement =
      scene.beginMeasurement("scan-a", point(1.0, 2.0, 3.0));

  REQUIRE(scene.completeMeasurement(measurement, "scan-b",
                                    point(4.0, 6.0, 3.0)));
  const auto &stored = scene.measurements().front();
  REQUIRE(stored.firstSourceKey() == "opaque:scan-a");
  REQUIRE(stored.secondSourceKey() == "opaque:scan-b");
  REQUIRE(stored.distance() == Approx(5.0));
  REQUIRE_FALSE(scene.measurementDetached(stored));

  REQUIRE(scene.removeLayer(second_layer));
  REQUIRE(scene.measurementDetached(stored));
  REQUIRE(scene.undo());
  REQUIRE_FALSE(scene.measurementDetached(stored));
  REQUIRE(scene.removeLayer(first_layer));
  REQUIRE(scene.measurementDetached(stored));
}

} // namespace
