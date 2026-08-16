#include "gui/measurement_overlay.hpp"

#include <catch2/catch.hpp>

#include <Eigen/Geometry>

#include <optional>

namespace {

using kpt::gui::MeasurementOverlay;
using kpt::gui::RoiBox;
using kpt::gui::Scene;
using kpt::gui::ViewportFrame;

ViewportFrame identityFrame() {
  ViewportFrame frame;
  frame.view_projection = Eigen::Matrix4f::Identity();
  frame.world_origin = Eigen::Vector3f::Zero();
  frame.world_scale = 1.0F;
  return frame;
}

TEST_CASE("measurement overlay projects immutable world points", "[measurement][overlay]") {
  Scene scene;
  const auto layer = scene.addLayer("scan-a");
  const auto id = scene.addMeasurement(
      "scan-a", Eigen::Vector3d{0.0, 0.0, 0.0},
      std::optional<Eigen::Vector3d>{Eigen::Vector3d{0.5, 0.0, 0.0}});

  MeasurementOverlay overlay =
      kpt::gui::buildMeasurementOverlay(scene, identityFrame());
  REQUIRE(overlay.markers.size() == 2);
  REQUIRE(overlay.segments.size() == 1);
  CHECK(overlay.markers[0].measurement_id == id);
  CHECK(overlay.markers[0].normalized_position.isApprox(
      Eigen::Vector2f{0.5F, 0.5F}));
  CHECK(overlay.markers[1].normalized_position.isApprox(
      Eigen::Vector2f{0.75F, 0.5F}));
  CHECK_FALSE(overlay.segments.front().detached);

  Eigen::Affine3d moved = Eigen::Affine3d::Identity();
  moved.translation() = Eigen::Vector3d{100.0, 0.0, 0.0};
  REQUIRE(scene.setLayerTransform(layer, moved));
  overlay = kpt::gui::buildMeasurementOverlay(scene, identityFrame());
  REQUIRE(overlay.markers.size() == 2);
  // A transform affects the source cloud, never a completed world-space pick.
  CHECK(overlay.markers[0].normalized_position.isApprox(
      Eigen::Vector2f{0.5F, 0.5F}));
}

TEST_CASE("measurement overlay follows layer visibility, ROI, and detached history",
          "[measurement][overlay]") {
  Scene scene;
  const auto layer = scene.addLayer("scan-a");
  static_cast<void>(scene.addMeasurement(
      "scan-a", Eigen::Vector3d{0.0, 0.0, 0.0},
      std::optional<Eigen::Vector3d>{Eigen::Vector3d{0.5, 0.0, 0.0}}));

  REQUIRE(scene.setLayerVisible(layer, false));
  CHECK(kpt::gui::buildMeasurementOverlay(scene, identityFrame()).markers.empty());

  REQUIRE(scene.setLayerVisible(layer, true));
  REQUIRE(scene.removeLayer(layer));
  auto overlay = kpt::gui::buildMeasurementOverlay(scene, identityFrame());
  REQUIRE(overlay.markers.size() == 2);
  REQUIRE(overlay.segments.size() == 1);
  CHECK(overlay.markers[0].detached);
  CHECK(overlay.markers[1].detached);
  CHECK(overlay.segments[0].detached);

  // ROI is a closed world-space predicate, including for retained detached
  // history. Only P2 survives this box, so no misleading partial segment.
  scene.setRoi(RoiBox({0.25, -1.0, -1.0}, {1.0, 1.0, 1.0}));
  overlay = kpt::gui::buildMeasurementOverlay(scene, identityFrame());
  REQUIRE(overlay.markers.size() == 1);
  CHECK(overlay.markers.front().second_endpoint);
  CHECK(overlay.segments.empty());
}

TEST_CASE("pending measurement overlay marks its first endpoint", "[measurement][overlay]") {
  Scene scene;
  static_cast<void>(scene.addLayer("scan-a"));
  static_cast<void>(scene.beginMeasurement("scan-a", {0.0, 0.0, 0.0}));

  const auto overlay = kpt::gui::buildMeasurementOverlay(scene, identityFrame());
  REQUIRE(overlay.markers.size() == 1);
  CHECK(overlay.markers.front().pending);
  CHECK_FALSE(overlay.markers.front().second_endpoint);
  CHECK(overlay.segments.empty());
}

} // namespace
