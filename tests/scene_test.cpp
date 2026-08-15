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

} // namespace
