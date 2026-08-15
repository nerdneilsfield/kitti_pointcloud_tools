#include "gui/scene/scene.hpp"

#include <catch2/catch.hpp>

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

TEST_CASE("layer identity is immutable while scene owns safe edits") {
  Scene scene;
  const auto layer_id = scene.addLayer("stable-source");
  const auto *layer = scene.findLayer(layer_id);
  REQUIRE(layer->id() == layer_id);
  REQUIRE(layer->sourceKey() == "stable-source");
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
