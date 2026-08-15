#include "gui/roi_filter.hpp"

#include <catch2/catch.hpp>

#include <limits>
#include <stdexcept>

namespace {

kpt::PointT point(float x, float y, float z, std::uint8_t noise = 0,
                  float intensity = 0.0F) {
  kpt::PointT value;
  value.x = x;
  value.y = y;
  value.z = z;
  value.r = 12;
  value.g = 34;
  value.b = 56;
  value.noise = noise;
  value.intensity = intensity;
  return value;
}

TEST_CASE("ROI filter keeps closed world bounds and point attributes") {
  kpt::PointCloudIRGB cloud;
  cloud.points = {
      point(-1.0F, 0.0F, 0.0F, 7, 0.25F),
      point(1.0F, 2.0F, 3.0F, 8, 0.75F),
      point(1.001F, 2.0F, 3.0F, 9, 1.0F),
      point(std::numeric_limits<float>::quiet_NaN(), 0.0F, 0.0F),
      point(0.0F, std::numeric_limits<float>::infinity(), 0.0F),
  };
  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.viewpoint = {9.0F, 8.0F, 7.0F, 1.0F, 0.0F, 0.0F, 0.0F};
  cloud.has_noise = true;

  const kpt::gui::RoiBox roi({-1.0, 0.0, 0.0}, {1.0, 2.0, 3.0});
  const auto filtered = kpt::gui::filterCloudToWorldRoi(
      cloud, Eigen::Affine3d::Identity(), roi);

  REQUIRE(filtered.size() == 2);
  REQUIRE(filtered.width == 2);
  REQUIRE(filtered.height == 1);
  REQUIRE(filtered.has_noise);
  REQUIRE(filtered.viewpoint == cloud.viewpoint);
  REQUIRE(filtered.points[0].x == Approx(-1.0F));
  REQUIRE(filtered.points[0].noise == 7);
  REQUIRE(filtered.points[0].intensity == Approx(0.25F));
  REQUIRE(filtered.points[0].r == 12);
  REQUIRE(filtered.points[1].x == Approx(1.0F));
  REQUIRE(filtered.points[1].y == Approx(2.0F));
  REQUIRE(filtered.points[1].z == Approx(3.0F));
  REQUIRE(filtered.points[1].noise == 8);
  REQUIRE(filtered.points[1].intensity == Approx(0.75F));
}

TEST_CASE("ROI filter tests world ROI after forward layer transform") {
  kpt::PointCloudIRGB cloud;
  cloud.points = {point(-1.0F, 0.0F, 0.0F), point(1.0F, 0.0F, 0.0F)};
  Eigen::Affine3d transform = Eigen::Affine3d::Identity();
  transform.translation() = Eigen::Vector3d{2.0, 3.0, 4.0};
  const kpt::gui::RoiBox roi({1.0, 3.0, 4.0}, {1.0, 3.0, 4.0});

  const auto filtered =
      kpt::gui::filterCloudToWorldRoi(cloud, transform, roi);

  REQUIRE(filtered.size() == 1);
  REQUIRE(filtered.points.front().x == Approx(1.0F));
  REQUIRE(filtered.points.front().y == Approx(3.0F));
  REQUIRE(filtered.points.front().z == Approx(4.0F));
}

TEST_CASE("ROI filter rejects invalid transforms and omits float-overflow worlds") {
  kpt::PointCloudIRGB cloud;
  cloud.points = {point(1.0F, 0.0F, 0.0F)};
  const kpt::gui::RoiBox all_world({-std::numeric_limits<double>::max(), -1.0, -1.0},
                                    {std::numeric_limits<double>::max(), 1.0, 1.0});

  Eigen::Affine3d non_finite = Eigen::Affine3d::Identity();
  non_finite.matrix()(0, 0) = std::numeric_limits<double>::quiet_NaN();
  REQUIRE_THROWS_AS(kpt::gui::filterCloudToWorldRoi(cloud, non_finite, all_world),
                    std::invalid_argument);

  Eigen::Affine3d projective = Eigen::Affine3d::Identity();
  projective.matrix()(3, 0) = 1.0;
  REQUIRE_THROWS_AS(kpt::gui::filterCloudToWorldRoi(cloud, projective, all_world),
                    std::invalid_argument);

  Eigen::Affine3d huge_translation = Eigen::Affine3d::Identity();
  huge_translation.translation().x() = std::numeric_limits<double>::max();
  REQUIRE(kpt::gui::filterCloudToWorldRoi(cloud, huge_translation, all_world)
              .empty());
}

} // namespace
