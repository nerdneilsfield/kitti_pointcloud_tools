#include "gui/scene/scene.hpp"

#include <catch2/catch.hpp>

#include <limits>
#include <stdexcept>

namespace {

using kpt::gui::RoiBox;
using kpt::gui::Scene;

TEST_CASE("scene allocates monotonic runtime IDs for stable source keys") {
  Scene scene;

  const auto first = scene.addLayer("scan-a");
  const auto second = scene.addLayer("scan-b");

  REQUIRE(first == 1);
  REQUIRE(second == 2);
  REQUIRE(scene.removeLayer(first));
  REQUIRE(scene.addLayer("scan-c") == 3);
  REQUIRE(scene.findLayerBySourceKey("scan-b")->id == second);
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

} // namespace
