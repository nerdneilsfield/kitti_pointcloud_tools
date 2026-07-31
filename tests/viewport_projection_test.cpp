#include "gui/viewport/model.hpp"

#include <catch2/catch.hpp>

#include <Eigen/Core>

#include <cmath>
#include <cstdint>
#include <memory>

namespace {

std::shared_ptr<const kpt::gui::ViewportCloudSnapshot>
unitCloud(std::uint64_t revision) {
  auto snapshot = std::make_shared<kpt::gui::ViewportCloudSnapshot>();
  snapshot->revision = revision;
  snapshot->bounds.minimum = Eigen::Vector3f::Constant(-1.0F);
  snapshot->bounds.maximum = Eigen::Vector3f::Constant(1.0F);
  snapshot->bounds.center = Eigen::Vector3f::Zero();
  snapshot->bounds.radius = 1.0F;
  snapshot->bounds.finite_points = 1;
  snapshot->vertices.push_back(
      {Eigen::Vector3f::Zero(), Eigen::Vector3f::Ones(), 0.0F});
  return snapshot;
}

float projectedDepth(const kpt::gui::ViewportFrame &frame,
                     const Eigen::Vector3f &world) {
  const Eigen::Vector3f local =
      (world - frame.world_origin) * frame.world_scale;
  const Eigen::Vector4f clip =
      frame.view_projection *
      Eigen::Vector4f(local.x(), local.y(), local.z(), 1.0F);
  REQUIRE(std::isfinite(clip.w()));
  REQUIRE(std::abs(clip.w()) > 1e-6F);
  return clip.z() / clip.w();
}

} // namespace

TEST_CASE("projection preserves near-to-far depth and clipping direction",
          "[viewport_model][projection]") {
  kpt::gui::ViewportModel model;
  model.setCloud(unitCloud(1));
  model.setView(kpt::gui::CameraPreset::Front);
  const auto frame = model.frame({800, 600});

  // CloudCompare front view places the eye at -Y, looking along +Y.
  constexpr float distance = 2.8F;
  constexpr float near_plane = distance * 0.001F;
  constexpr float far_plane = distance + 8.0F;
  const auto pointAtEyeDistance = [](float eye_distance) {
    return Eigen::Vector3f(0.0F, -distance + eye_distance, 0.0F);
  };

  const float near_inside =
      projectedDepth(frame, pointAtEyeDistance(near_plane * 1.01F));
  const float center = projectedDepth(frame, Eigen::Vector3f::Zero());
  const float far_inside =
      projectedDepth(frame, pointAtEyeDistance(far_plane * 0.99F));
  REQUIRE(near_inside >= -1.0F);
  REQUIRE(near_inside < center);
  REQUIRE(center < far_inside);
  REQUIRE(far_inside <= 1.0F);

  const float before_near =
      projectedDepth(frame, pointAtEyeDistance(near_plane * 0.5F));
  const float beyond_far =
      projectedDepth(frame, pointAtEyeDistance(far_plane * 1.01F));
  REQUIRE(before_near < -1.0F);
  REQUIRE(beyond_far > 1.0F);
}
