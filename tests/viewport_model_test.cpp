#include "gui/viewport/cloud_adapter.hpp"
#include "gui/viewport/model.hpp"
#include "gui/viewport/renderer.hpp"

#include <catch2/catch.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <utility>
#include <vector>

namespace {

using kpt::gui::CameraUpdate;
using kpt::gui::CameraSnapshot;
using kpt::gui::CloudBounds;
using kpt::gui::ViewportCloudSnapshot;
using kpt::gui::ViewportModel;
using kpt::gui::ViewportVertex;

constexpr kpt::gui::PixelExtent kSquareExtent{600, 600};

std::shared_ptr<const ViewportCloudSnapshot>
snapshot(std::uint64_t revision, const Eigen::Vector3f &minimum,
         const Eigen::Vector3f &maximum) {
  auto value = std::make_shared<ViewportCloudSnapshot>();
  value->revision = revision;
  value->bounds.minimum = minimum;
  value->bounds.maximum = maximum;
  value->bounds.centroid = (minimum + maximum) * 0.5F;
  value->bounds.center = (minimum + maximum) * 0.5F;
  value->bounds.radius = std::max((maximum - minimum).norm() * 0.5F, 0.001F);
  value->bounds.z_min = minimum.z();
  value->bounds.z_max = maximum.z();
  value->bounds.intensity_min = 2.0F;
  value->bounds.intensity_max = 8.0F;
  value->bounds.intensity_p05 = 2.0F;
  value->bounds.intensity_p90 = 8.0F;
  value->bounds.finite_points = 1;
  value->vertices.push_back({value->bounds.center, Eigen::Vector3f::Ones(),
                             value->bounds.intensity_min});
  return value;
}

bool differs(const Eigen::Matrix4f &left, const Eigen::Matrix4f &right) {
  return !left.isApprox(right, 1e-5F);
}

bool differs(const kpt::gui::ViewportFrame &left,
             const kpt::gui::ViewportFrame &right) {
  return differs(left.view_projection, right.view_projection) ||
         !left.world_origin.isApprox(right.world_origin, 1e-5F) ||
         std::abs(left.world_scale - right.world_scale) > 1e-5F;
}

Eigen::Vector3f screenAxis(const Eigen::Matrix4f &matrix, int row) {
  return matrix.block<1, 3>(row, 0).transpose().normalized();
}

Eigen::Vector2f screenPosition(const kpt::gui::ViewportFrame &frame,
                               const Eigen::Vector3f &position,
                               kpt::gui::PixelExtent extent) {
  const Eigen::Vector3f local =
      (position - frame.world_origin) * frame.world_scale;
  const Eigen::Vector4f clip =
      frame.view_projection *
      Eigen::Vector4f(local.x(), local.y(), local.z(), 1.0F);
  const Eigen::Vector3f ndc = clip.head<3>() / clip.w();
  return {(ndc.x() * 0.5F + 0.5F) * static_cast<float>(extent.width),
          (0.5F - ndc.y() * 0.5F) * static_cast<float>(extent.height)};
}

class FakeFrameContext final : public kpt::gui::FrameContext {
public:
  [[nodiscard]] kpt::gui::BackendKind backendKind() const noexcept override {
    return kpt::gui::BackendKind::OpenGL;
  }
};

class FakeRenderer final : public kpt::gui::ViewportRenderer {
public:
  kpt::Result<void, kpt::gui::RendererError>
  upload(std::span<const ViewportVertex> vertices,
         std::uint64_t revision) override {
    uploaded.assign(vertices.begin(), vertices.end());
    uploaded_revision = revision;
    return {};
  }

  kpt::Result<void, kpt::gui::RendererError>
  resize(kpt::gui::PixelExtent physical_pixels) override {
    size = physical_pixels;
    return {};
  }

  kpt::Result<void, kpt::gui::RendererError>
  render(const kpt::gui::ViewportFrame &,
         kpt::gui::FrameContext &context) override {
    rendered_backend = context.backendKind();
    return {};
  }

  [[nodiscard]] kpt::gui::ViewportTexture texture() const override {
    return {};
  }

  [[nodiscard]] kpt::gui::PixelExtent extent() const override { return size; }

  [[nodiscard]] kpt::gui::BackendKind backendKind() const noexcept override {
    return kpt::gui::BackendKind::OpenGL;
  }

  std::vector<ViewportVertex> uploaded;
  std::uint64_t uploaded_revision = 0;
  kpt::gui::PixelExtent size;
  kpt::gui::BackendKind rendered_backend = kpt::gui::BackendKind::Metal;
};

} // namespace

TEST_CASE("cloud adapter filters non-finite points and computes bounds",
          "[viewport_model]") {
  auto cloud = std::make_shared<kpt::PointCloudIRGB>();
  cloud->has_noise = true;

  kpt::PointT first{};
  first.x = -2.0F;
  first.y = 1.0F;
  first.z = 3.0F;
  first.r = 255;
  first.g = 128;
  first.b = 0;
  first.intensity = 2.0F;
  cloud->push_back(first);

  kpt::PointT second{};
  second.x = 4.0F;
  second.y = 5.0F;
  second.z = -1.0F;
  second.intensity = 8.0F;
  second.noise = 2;
  cloud->push_back(second);

  kpt::PointT invalid_position{};
  invalid_position.x = std::numeric_limits<float>::quiet_NaN();
  invalid_position.intensity = -100.0F;
  cloud->push_back(invalid_position);

  kpt::PointT invalid_intensity{};
  invalid_intensity.x = 1.0F;
  invalid_intensity.y = 2.0F;
  invalid_intensity.z = 0.0F;
  invalid_intensity.intensity = std::numeric_limits<float>::infinity();
  cloud->push_back(invalid_intensity);

  const auto result = kpt::gui::makeViewportCloudSnapshot(cloud, 7);
  REQUIRE(result->revision == 7);
  REQUIRE(result->vertices.size() == 3);
  REQUIRE(result->bounds.finite_points == 3);
  REQUIRE(result->bounds.minimum.isApprox(Eigen::Vector3f(-2.0F, 1.0F, -1.0F)));
  REQUIRE(result->bounds.maximum.isApprox(Eigen::Vector3f(4.0F, 5.0F, 3.0F)));
  REQUIRE(result->bounds.center.isApprox(Eigen::Vector3f(1.0F, 3.0F, 1.0F)));
  REQUIRE(result->bounds.centroid.isApprox(Eigen::Vector3f(1.0F, 8.0F / 3.0F,
                                                            2.0F / 3.0F)));
  REQUIRE(result->bounds.radius > 0.0F);
  REQUIRE(result->bounds.z_min == -1.0F);
  REQUIRE(result->bounds.z_max == 3.0F);
  REQUIRE(result->bounds.intensity_min == 2.0F);
  REQUIRE(result->bounds.intensity_max == 8.0F);
  REQUIRE(result->bounds.has_noise);
  REQUIRE(result->bounds.noise_points == 1);
  REQUIRE(result->vertices.front().color.x() == 1.0F);
  REQUIRE(result->vertices[1].noise == 1.0F);
  REQUIRE(result->vertices.back().intensity == 0.0F);
}

TEST_CASE("empty and generation zero snapshots are benign",
          "[viewport_model]") {
  const auto empty = kpt::gui::makeViewportCloudSnapshot(
      std::make_shared<kpt::PointCloudIRGB>(), 3);
  REQUIRE(empty->revision == 3);
  REQUIRE(empty->vertices.empty());
  REQUIRE(empty->bounds.finite_points == 0);
  REQUIRE(empty->bounds.center.isZero());
  REQUIRE(empty->bounds.radius == 1.0F);

  auto cloud = std::make_shared<kpt::PointCloudIRGB>();
  cloud->push_back(kpt::PointT{});
  const auto none = kpt::gui::makeViewportCloudSnapshot(cloud, 0);
  REQUIRE(none->revision == 0);
  REQUIRE(none->vertices.empty());

  ViewportModel model;
  model.setCloud(none);
  REQUIRE(model.cloudRevision() == 0);
  REQUIRE_FALSE(model.cloud());
  REQUIRE(model.bounds().finite_points == 0);
}

TEST_CASE("fit centers the finite-point centroid and uses bounded automatic FOV",
          "[viewport_model][camera]") {
  auto cloud = std::make_shared<kpt::PointCloudIRGB>();
  for (int index = 0; index < 19; ++index) {
    kpt::PointT point{};
    point.x = static_cast<float>(index % 3) - 1.0F;
    point.y = static_cast<float>(index % 5) - 2.0F;
    cloud->push_back(point);
  }
  kpt::PointT outlier{};
  outlier.x = 100.0F;
  cloud->push_back(outlier);

  const auto fitted_cloud = kpt::gui::makeViewportCloudSnapshot(cloud, 10);
  REQUIRE_FALSE(fitted_cloud->bounds.centroid.isApprox(fitted_cloud->bounds.center));

  ViewportModel model;
  model.setCloud(fitted_cloud);
  const auto frame = model.frame(kSquareExtent);
  const auto screen = screenPosition(frame, fitted_cloud->bounds.centroid,
                                     kSquareExtent);
  REQUIRE(screen.x() == Approx(kSquareExtent.width * 0.5F).margin(1.0e-3F));
  REQUIRE(screen.y() == Approx(kSquareExtent.height * 0.5F).margin(1.0e-3F));
  REQUIRE(frame.fov_y_degrees >= 35.0F);
  REQUIRE(frame.fov_y_degrees <= 75.0F);
}

TEST_CASE("finite float extremes produce finite derived bounds and camera",
          "[viewport_model][bounds]") {
  auto cloud = std::make_shared<kpt::PointCloudIRGB>();
  kpt::PointT minimum{};
  minimum.x = minimum.y = minimum.z = std::numeric_limits<float>::lowest();
  kpt::PointT maximum{};
  maximum.x = maximum.y = maximum.z = std::numeric_limits<float>::max();
  cloud->push_back(minimum);
  cloud->push_back(maximum);

  const auto result = kpt::gui::makeViewportCloudSnapshot(cloud, 8);
  REQUIRE(result->bounds.finite_points == 2);
  REQUIRE(result->bounds.center.allFinite());
  REQUIRE(std::isfinite(result->bounds.radius));
  REQUIRE(result->bounds.radius > 0.0F);
  REQUIRE(result->bounds.radius >
          static_cast<double>(std::numeric_limits<float>::max()));

  ViewportModel model;
  model.setCloud(result);
  kpt::gui::ViewportStyle guide_style;
  guide_style.show_coordinate_axes = true;
  guide_style.show_scale_grid = true;
  model.setStyle(guide_style);
  const auto extreme_frame = model.frame(kSquareExtent);
  REQUIRE(extreme_frame.view_projection.allFinite());
  REQUIRE(extreme_frame.world_origin.allFinite());
  REQUIRE(extreme_frame.world_scale >= std::numeric_limits<float>::min());
  REQUIRE(std::isfinite(extreme_frame.grid_spacing));
  REQUIRE(extreme_frame.grid_spacing > 0.0F);
  REQUIRE(std::all_of(
      extreme_frame.guides.begin(), extreme_frame.guides.end(),
      [](const auto &vertex) { return vertex.position.allFinite(); }));

  auto singleton = std::make_shared<kpt::PointCloudIRGB>();
  singleton->push_back(maximum);
  ViewportModel singleton_model;
  singleton_model.setCloud(kpt::gui::makeViewportCloudSnapshot(singleton, 9));
  singleton_model.setStyle(guide_style);
  const auto singleton_frame = singleton_model.frame(kSquareExtent);
  REQUIRE(singleton_frame.view_projection.allFinite());
  REQUIRE(singleton_frame.world_origin.allFinite());
  REQUIRE(singleton_frame.world_scale > 0.0F);
  REQUIRE(std::all_of(
      singleton_frame.guides.begin(), singleton_frame.guides.end(),
      [](const auto &vertex) { return vertex.position.allFinite(); }));
}

TEST_CASE("newest cloud generation supersedes stale completions",
          "[viewport_model]") {
  ViewportModel model;
  const auto older =
      snapshot(4, Eigen::Vector3f::Zero(), Eigen::Vector3f::Ones());
  const auto newer = snapshot(9, Eigen::Vector3f::Constant(10.0F),
                              Eigen::Vector3f::Constant(12.0F));

  model.setCloud(newer);
  model.setCloud(older);
  REQUIRE(model.cloud() == newer);
  REQUIRE(model.cloudRevision() == 9);
  REQUIRE(model.bounds().center.isApprox(Eigen::Vector3f::Constant(11.0F)));

  model.setCloud(nullptr);
  REQUIRE(model.cloudRevision() == 0);
  REQUIRE_FALSE(model.cloud());

  // Clearing visibility does not lower the acceptance high-water mark.
  model.setCloud(older);
  REQUIRE(model.cloudRevision() == 0);

  const auto latest = snapshot(10, Eigen::Vector3f::Constant(20.0F),
                               Eigen::Vector3f::Constant(22.0F));
  model.setCloud(latest);
  REQUIRE(model.cloud() == latest);
  model.setCloud(snapshot(0, Eigen::Vector3f::Zero(), Eigen::Vector3f::Ones()));
  REQUIRE(model.cloudRevision() == 0);
  REQUIRE_FALSE(model.cloud());
}

TEST_CASE("camera and style mutations preserve cloud revision",
          "[viewport_model]") {
  ViewportModel model;
  model.setCloud(snapshot(11, {-2.0F, -3.0F, -4.0F}, {5.0F, 6.0F, 7.0F}));
  const auto fitted = model.frame(kSquareExtent);

  model.orbit(400.0F, 400.0F, 410.0F, 394.0F, kSquareExtent);
  const auto orbited = model.frame(kSquareExtent);
  REQUIRE(differs(fitted, orbited));

  model.pan(8.0F, 4.0F, kSquareExtent);
  const auto panned = model.frame(kSquareExtent);
  REQUIRE(differs(orbited, panned));

  model.zoom(30.0F);
  const auto zoomed = model.frame(kSquareExtent);
  REQUIRE(differs(panned, zoomed));

  kpt::gui::ViewportStyle style;
  style.color_by = kpt::ColorBy::Z;
  style.point_size = 6.5F;
  style.background = {0.1F, 0.2F, 0.3F};
  model.setStyle(style);
  const auto frame = model.frame(kSquareExtent);
  REQUIRE(frame.style.color_by == kpt::ColorBy::Z);
  REQUIRE(frame.style.point_size == 6.5F);
  REQUIRE(frame.style.background.isApprox(style.background));
  REQUIRE(frame.style.scalar_min == -4.0F);
  REQUIRE(frame.style.scalar_max == 7.0F);
  REQUIRE(model.cloudRevision() == 11);

  style.color_by = kpt::ColorBy::Intensity;
  model.setStyle(style);
  REQUIRE(model.frame(kSquareExtent).style.scalar_min == 2.0F);
  REQUIRE(model.frame(kSquareExtent).style.scalar_max == 8.0F);
  REQUIRE(model.cloudRevision() == 11);
}

TEST_CASE("intensity equalization selects scalar range and cdf payload",
          "[viewport_model][equalize]") {
  auto snap = std::make_shared<ViewportCloudSnapshot>();
  snap->revision = 1;
  snap->bounds.minimum = Eigen::Vector3f::Constant(-1.0F);
  snap->bounds.maximum = Eigen::Vector3f::Constant(1.0F);
  snap->bounds.center = Eigen::Vector3f::Zero();
  snap->bounds.radius = std::sqrt(3.0F);
  snap->bounds.intensity_min = 0.0F;
  snap->bounds.intensity_max = 100.0F;
  snap->bounds.intensity_p05 = 5.0F;
  snap->bounds.intensity_p90 = 90.0F;
  snap->bounds.intensity_cdf_valid = true;
  snap->bounds.finite_points = 1;
  for (std::size_t i = 0; i < snap->bounds.intensity_cdf.size(); ++i)
    snap->bounds.intensity_cdf[i] =
        static_cast<float>(i) /
        static_cast<float>(snap->bounds.intensity_cdf.size() - 1);
  snap->vertices.push_back(
      {Eigen::Vector3f::Zero(), Eigen::Vector3f::Ones(), 0.0F, 0.0F});

  ViewportModel model;
  model.setCloud(snap);

  kpt::gui::ViewportStyle style;
  style.color_by = kpt::ColorBy::Intensity;
  style.intensity_equalize = true;
  model.setStyle(style);
  const auto eq_on = model.frame(kSquareExtent);
  REQUIRE(eq_on.intensity_cdf_valid);
  REQUIRE(eq_on.style.scalar_min == 0.0F);
  REQUIRE(eq_on.style.scalar_max == 100.0F);

  style.intensity_equalize = false;
  model.setStyle(style);
  const auto eq_off = model.frame(kSquareExtent);
  REQUIRE_FALSE(eq_off.intensity_cdf_valid);
  REQUIRE(eq_off.style.scalar_min == 5.0F);
  REQUIRE(eq_off.style.scalar_max == 90.0F);
}

TEST_CASE("viewport model builds bounded depth-tested scene guides",
          "[viewport_model][guides]") {
  ViewportModel model;
  model.setCloud(snapshot(1, {-10.0F, -4.0F, -2.0F}, {10.0F, 6.0F, 3.0F}));

  kpt::gui::ViewportStyle style;
  model.setStyle(style);
  REQUIRE(model.frame(kSquareExtent).guides.empty());

  style.show_coordinate_axes = true;
  model.setStyle(style);
  const auto axes = model.frame(kSquareExtent);
  REQUIRE(axes.guides.size() == 30);
  REQUIRE(axes.grid_spacing == 0.0F);
  REQUIRE(axes.guides[1].color.x() > axes.guides[1].color.y());
  REQUIRE(axes.guides[11].color.y() > axes.guides[11].color.x());
  REQUIRE(axes.guides[21].color.z() > axes.guides[21].color.x());
  REQUIRE(axes.guides[3].position.x() < axes.guides[1].position.x());
  REQUIRE(std::abs(axes.guides[3].position.y()) > 0.0F);

  style.show_scale_grid = true;
  model.setStyle(style);
  const auto combined = model.frame(kSquareExtent);
  REQUIRE(combined.grid_spacing == 5.0F);
  REQUIRE(combined.guides.size() > axes.guides.size());
  REQUIRE(combined.guides.size() <= 516);
  REQUIRE(combined.guides.size() % 2 == 0);

  style.show_coordinate_axes = false;
  model.setStyle(style);
  const auto grid = model.frame(kSquareExtent);
  REQUIRE(grid.grid_spacing == combined.grid_spacing);
  REQUIRE(grid.guides.size() + 30 == combined.guides.size());

  const auto has_plane_line = [&grid](const int fixed_axis) {
    for (std::size_t index = 0; index + 1 < grid.guides.size(); index += 2) {
      const auto &start = grid.guides[index].position;
      const auto &end = grid.guides[index + 1].position;
      if (std::abs(start[fixed_axis]) >= 1.0e-6F ||
          std::abs(end[fixed_axis]) >= 1.0e-6F) {
        continue;
      }
      for (int moving_axis = 0; moving_axis < 3; ++moving_axis) {
        if (moving_axis == fixed_axis)
          continue;
        const int offset_axis = 3 - fixed_axis - moving_axis;
        if (std::abs(start[moving_axis] - end[moving_axis]) > 1.0e-6F &&
            std::abs(start[offset_axis] - end[offset_axis]) < 1.0e-6F &&
            std::abs(start[offset_axis]) > 1.0e-6F) {
          return true;
        }
      }
    }
    return false;
  };
  REQUIRE(has_plane_line(2)); // XY
  REQUIRE(has_plane_line(1)); // XZ
  REQUIRE(has_plane_line(0)); // YZ
}

TEST_CASE("scene guides respect sub-unit XY, dominant Z, and light backgrounds",
          "[viewport_model][guides]") {
  ViewportModel model;
  model.setCloud(snapshot(1, {0.0F, 0.0F, 0.0F}, {0.01F, 0.01F, 1000.0F}));
  kpt::gui::ViewportStyle style;
  style.show_coordinate_axes = true;
  style.show_scale_grid = true;
  style.background = Eigen::Vector3f::Ones();
  model.setStyle(style);

  const auto frame = model.frame(kSquareExtent);
  REQUIRE(frame.grid_spacing == Approx(200.0F));
  REQUIRE(std::any_of(frame.guides.begin(), frame.guides.end(),
                      [](const auto &vertex) {
                        return vertex.position.z() == Approx(1000.0F);
                      }));
  REQUIRE(frame.guides.front().color.maxCoeff() < 0.8F);
  REQUIRE(frame.guides[frame.guides.size() - 2].color.maxCoeff() < 0.8F);

  model.setCloud(snapshot(2, {0.0F, 0.0F, 0.0F}, {1.0e-6F, 1.0e-6F, 1.0e-6F}));
  const auto microscopic = model.frame(kSquareExtent);
  REQUIRE(microscopic.grid_spacing == Approx(2.0e-7F));
}

TEST_CASE("CloudCompare trackball and screen-plane pan are reversible",
          "[viewport_model][camera]") {
  ViewportModel model;
  model.setCloud(snapshot(1, {-2.0F, -2.0F, -2.0F}, {2.0F, 2.0F, 2.0F}));
  model.setView(kpt::gui::CameraPreset::Front);
  const auto initial = model.frame(kSquareExtent);

  model.orbit(400.0F, 400.0F, 560.0F, 320.0F, kSquareExtent);
  const auto rotated = model.frame(kSquareExtent);
  REQUIRE(differs(initial, rotated));
  model.orbit(560.0F, 320.0F, 400.0F, 400.0F, kSquareExtent);
  REQUIRE(model.frame(kSquareExtent)
              .view_projection.isApprox(initial.view_projection, 1.0e-4F));

  model.pan(25.0F, -14.0F, kSquareExtent);
  REQUIRE(differs(initial, model.frame(kSquareExtent)));
  model.pan(-25.0F, 14.0F, kSquareExtent);
  REQUIRE(model.frame(kSquareExtent)
              .view_projection.isApprox(initial.view_projection, 1.0e-4F));

  model.roll(100.0F, kSquareExtent);
  REQUIRE(differs(initial, model.frame(kSquareExtent)));
  model.roll(-100.0F, kSquareExtent);
  REQUIRE(model.frame(kSquareExtent)
              .view_projection.isApprox(initial.view_projection, 1.0e-4F));
}

TEST_CASE("middle-button picking changes the orbit center without jumping",
          "[viewport_model][camera]") {
  ViewportModel model;
  auto cloud = std::make_shared<ViewportCloudSnapshot>();
  cloud->revision = 1;
  cloud->bounds.minimum = {-2.0F, -2.0F, -2.0F};
  cloud->bounds.maximum = {2.0F, 2.0F, 2.0F};
  cloud->bounds.center = Eigen::Vector3f::Zero();
  cloud->bounds.radius = 4.0;
  cloud->bounds.finite_points = 2;
  const Eigen::Vector3f picked_point{1.0F, 0.0F, 0.5F};
  cloud->vertices.push_back(
      {Eigen::Vector3f::Zero(), Eigen::Vector3f::Ones(), 0.0F});
  cloud->vertices.push_back({picked_point, Eigen::Vector3f::Ones(), 0.0F});
  model.setCloud(std::move(cloud));
  model.setView(kpt::gui::CameraPreset::Front);

  const auto initial = model.frame(kSquareExtent);
  const Eigen::Vector2f cursor =
      screenPosition(initial, picked_point, kSquareExtent);
  const auto picked =
      model.pointFromScreen(cursor.x(), cursor.y(), kSquareExtent);
  REQUIRE(picked);
  REQUIRE(picked->isApprox(picked_point));
  REQUIRE(
      model.setRotationCenterFromScreen(cursor.x(), cursor.y(), kSquareExtent));
  REQUIRE(model.frame(kSquareExtent)
              .view_projection.isApprox(initial.view_projection));

  model.orbit(300.0F, 300.0F, 390.0F, 250.0F, kSquareExtent);
  const Eigen::Vector2f after_orbit =
      screenPosition(model.frame(kSquareExtent), picked_point, kSquareExtent);
  REQUIRE(after_orbit.x() == Approx(cursor.x()).margin(1.0e-3F));
  REQUIRE(after_orbit.y() == Approx(cursor.y()).margin(1.0e-3F));
  REQUIRE_FALSE(
      model.setRotationCenterFromScreen(-100.0F, -100.0F, kSquareExtent));
}

TEST_CASE("structured picking returns world attributes and preserves position API",
          "[viewport_model][camera]") {
  ViewportModel model;
  auto cloud = std::make_shared<ViewportCloudSnapshot>();
  cloud->revision = 1;
  cloud->bounds.radius = 2.0;
  cloud->bounds.finite_points = 1;
  const Eigen::Vector3f point{0.25F, -0.5F, 0.75F};
  cloud->vertices.push_back({point, Eigen::Vector3f::Ones(), 7.5F, 1.0F});
  model.setCloud(std::move(cloud));
  const auto cursor = screenPosition(model.frame(kSquareExtent), point,
                                     kSquareExtent);
  const auto picked =
      model.pickCloudFromScreen(cursor.x(), cursor.y(), kSquareExtent);
  REQUIRE(picked);
  REQUIRE(picked->cloud_position.isApprox(point));
  REQUIRE(picked->world_position.isApprox(point));
  REQUIRE(picked->intensity == 7.5F);
  REQUIRE(picked->noise == 1.0F);
  REQUIRE(model.pointFromScreen(cursor.x(), cursor.y(), kSquareExtent)
              ->isApprox(point));
}

TEST_CASE("camera snapshots validate atomically", "[viewport_model][camera]") {
  ViewportModel model;
  model.setCloud(snapshot(1, {-1.0F, -1.0F, -1.0F}, {1.0F, 1.0F, 1.0F}));
  static_cast<void>(model.frame(kSquareExtent));
  const CameraSnapshot initial = model.cameraSnapshot();
  CameraSnapshot replacement = initial;
  replacement.target = {3.0, 4.0, 5.0};
  replacement.rotation_center = {-2.0, 1.0, 6.0};
  replacement.distance = 12.0;
  replacement.fov_y_degrees = 72.0F;
  REQUIRE(model.setCameraSnapshot(replacement));
  const CameraSnapshot restored = model.cameraSnapshot();
  REQUIRE(restored.target.isApprox(replacement.target));
  REQUIRE(restored.rotation_center.isApprox(replacement.rotation_center));
  REQUIRE(restored.camera_to_world.isApprox(replacement.camera_to_world));
  REQUIRE(restored.distance == replacement.distance);
  REQUIRE(restored.fov_y_degrees == replacement.fov_y_degrees);

  CameraSnapshot invalid = replacement;
  invalid.distance = 0.0;
  REQUIRE_FALSE(model.setCameraSnapshot(invalid));
  invalid = replacement;
  invalid.fov_y_degrees = 180.0F;
  REQUIRE_FALSE(model.setCameraSnapshot(invalid));
  invalid = replacement;
  invalid.target.x() = std::numeric_limits<double>::quiet_NaN();
  REQUIRE_FALSE(model.setCameraSnapshot(invalid));
  invalid = replacement;
  invalid.camera_to_world(0, 0) = 2.0F;
  REQUIRE_FALSE(model.setCameraSnapshot(invalid));
  invalid = replacement;
  invalid.target.x() = std::numeric_limits<double>::max();
  REQUIRE_FALSE(model.setCameraSnapshot(invalid));
  invalid = replacement;
  invalid.distance = std::numeric_limits<double>::max();
  REQUIRE_FALSE(model.setCameraSnapshot(invalid));
  const CameraSnapshot after_invalid = model.cameraSnapshot();
  REQUIRE(after_invalid.target.isApprox(replacement.target));
  REQUIRE(after_invalid.rotation_center.isApprox(replacement.rotation_center));
  REQUIRE(after_invalid.camera_to_world.isApprox(replacement.camera_to_world));
  REQUIRE(after_invalid.distance == replacement.distance);
  REQUIRE(after_invalid.fov_y_degrees == replacement.fov_y_degrees);
}

TEST_CASE("snapshot bounds middle-button picking work",
          "[viewport_model][camera]") {
  auto cloud = std::make_shared<kpt::PointCloudIRGB>();
  cloud->points.resize(150'000U);
  for (std::size_t index = 0; index < cloud->size(); ++index) {
    cloud->points[index].x = static_cast<float>(index);
  }
  const auto snapshot = kpt::gui::makeViewportCloudSnapshot(cloud, 1);
  REQUIRE(snapshot->vertices.size() == 150'000U);
  REQUIRE(snapshot->picking_vertices.size() == 100'000U);
  REQUIRE(snapshot->picking_vertices.front().position.x() == 0.0F);
}

TEST_CASE("fit and every view preset produce finite camera matrices",
          "[viewport_model]") {
  ViewportModel model;
  model.setCloud(snapshot(1, {-5.0F, -2.0F, -1.0F}, {7.0F, 4.0F, 3.0F}));
  model.orbit(400.0F, 400.0F, 420.0F, 410.0F, kSquareExtent);
  model.fit();
  const auto fitted = model.frame(kSquareExtent).view_projection;
  REQUIRE(fitted.allFinite());

  const std::vector<kpt::gui::CameraPreset> views = {
      kpt::gui::CameraPreset::Top,   kpt::gui::CameraPreset::Bottom,
      kpt::gui::CameraPreset::Front, kpt::gui::CameraPreset::Back,
      kpt::gui::CameraPreset::Left,  kpt::gui::CameraPreset::Right,
      kpt::gui::CameraPreset::Iso1,  kpt::gui::CameraPreset::Iso2};
  std::vector<Eigen::Matrix4f> matrices;
  for (const auto view : views) {
    model.setView(view);
    const auto matrix = model.frame(kSquareExtent).view_projection;
    REQUIRE(matrix.allFinite());
    matrices.push_back(matrix);
  }
  REQUIRE(differs(matrices[0], matrices[1]));
  REQUIRE(differs(matrices[2], matrices[3]));
  REQUIRE(differs(matrices[4], matrices[5]));
  REQUIRE(screenAxis(matrices[0], 0).isApprox(Eigen::Vector3f::UnitX()));
  REQUIRE(screenAxis(matrices[0], 1).isApprox(Eigen::Vector3f::UnitY()));
  REQUIRE(screenAxis(matrices[1], 0).isApprox(-Eigen::Vector3f::UnitX()));
  REQUIRE(screenAxis(matrices[1], 1).isApprox(Eigen::Vector3f::UnitY()));
  REQUIRE(screenAxis(matrices[2], 0).isApprox(Eigen::Vector3f::UnitX()));
  REQUIRE(screenAxis(matrices[2], 1).isApprox(Eigen::Vector3f::UnitZ()));
  REQUIRE(screenAxis(matrices[3], 0).isApprox(-Eigen::Vector3f::UnitX()));
  REQUIRE(screenAxis(matrices[3], 1).isApprox(Eigen::Vector3f::UnitZ()));
  REQUIRE(screenAxis(matrices[4], 0).isApprox(-Eigen::Vector3f::UnitY()));
  REQUIRE(screenAxis(matrices[4], 1).isApprox(Eigen::Vector3f::UnitZ()));
  REQUIRE(screenAxis(matrices[5], 0).isApprox(Eigen::Vector3f::UnitY()));
  REQUIRE(screenAxis(matrices[5], 1).isApprox(Eigen::Vector3f::UnitZ()));
  REQUIRE(model.cloudRevision() == 1);
}

TEST_CASE("preserve camera keeps pose while accepting newer cloud",
          "[viewport_model]") {
  ViewportModel model;
  model.setCloud(snapshot(1, Eigen::Vector3f::Zero(), Eigen::Vector3f::Ones()));
  model.orbit(400.0F, 400.0F, 412.0F, 405.0F, kSquareExtent);
  model.pan(3.0F, -2.0F, kSquareExtent);
  const auto before = model.frame(kSquareExtent).view_projection;

  model.setCloud(snapshot(2, Eigen::Vector3f::Zero(), Eigen::Vector3f::Ones()),
                 CameraUpdate::Preserve);
  REQUIRE(model.frame(kSquareExtent).view_projection.isApprox(before));
  REQUIRE(model.cloudRevision() == 2);
}

TEST_CASE("projection uses physical pixel aspect and tolerates suspension",
          "[viewport_model]") {
  ViewportModel model;
  model.setCloud(snapshot(1, Eigen::Vector3f::Zero(), Eigen::Vector3f::Ones()));
  model.setView(kpt::gui::CameraPreset::Front);

  const auto square = model.frame({800, 800}).view_projection;
  const auto wide = model.frame({1600, 800}).view_projection;
  REQUIRE(wide.allFinite());
  REQUIRE(wide.row(0).norm() * 2.0F == Approx(square.row(0).norm()));
  REQUIRE(wide.row(1).isApprox(square.row(1)));

  const auto suspended = model.frame({0, 0}).view_projection;
  REQUIRE(suspended.allFinite());
  REQUIRE(suspended.isApprox(square));
}

TEST_CASE("renderer contract carries explicit frame context",
          "[viewport_model]") {
  FakeRenderer renderer;
  FakeFrameContext context;
  const std::vector<ViewportVertex> vertices(2);

  REQUIRE(renderer.upload(vertices, 12));
  REQUIRE(renderer.resize({640, 480}));
  REQUIRE(renderer.render({}, context));
  REQUIRE(renderer.uploaded.size() == 2);
  REQUIRE(renderer.uploaded_revision == 12);
  REQUIRE(renderer.extent() == kpt::gui::PixelExtent{640, 480});
  REQUIRE(renderer.rendered_backend == kpt::gui::BackendKind::OpenGL);
}
