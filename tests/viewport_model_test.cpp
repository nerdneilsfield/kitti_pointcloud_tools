#include "gui/viewport/model.hpp"
#include "gui/viewport/cloud_adapter.hpp"
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
  value->bounds.center = (minimum + maximum) * 0.5F;
  value->bounds.radius = std::max((maximum - minimum).norm() * 0.5F, 0.001F);
  value->bounds.z_min = minimum.z();
  value->bounds.z_max = maximum.z();
  value->bounds.intensity_min = 2.0F;
  value->bounds.intensity_max = 8.0F;
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
  REQUIRE(result->bounds.radius > 0.0F);
  REQUIRE(result->bounds.z_min == -1.0F);
  REQUIRE(result->bounds.z_max == 3.0F);
  REQUIRE(result->bounds.intensity_min == 2.0F);
  REQUIRE(result->bounds.intensity_max == 8.0F);
  REQUIRE(result->vertices.front().color.x() == 1.0F);
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
  const auto extreme_frame = model.frame(kSquareExtent);
  REQUIRE(extreme_frame.view_projection.allFinite());
  REQUIRE(extreme_frame.world_origin.allFinite());
  REQUIRE(extreme_frame.world_scale >= std::numeric_limits<float>::min());

  auto singleton = std::make_shared<kpt::PointCloudIRGB>();
  singleton->push_back(maximum);
  ViewportModel singleton_model;
  singleton_model.setCloud(kpt::gui::makeViewportCloudSnapshot(singleton, 9));
  const auto singleton_frame = singleton_model.frame(kSquareExtent);
  REQUIRE(singleton_frame.view_projection.allFinite());
  REQUIRE(singleton_frame.world_origin.allFinite());
  REQUIRE(singleton_frame.world_scale > 0.0F);
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
  model.setCloud(snapshot(0, Eigen::Vector3f::Zero(),
                          Eigen::Vector3f::Ones()));
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
  REQUIRE(axes.guides.size() == 6);
  REQUIRE(axes.grid_spacing == 0.0F);
  REQUIRE(axes.guides[1].color.x() > axes.guides[1].color.y());
  REQUIRE(axes.guides[3].color.y() > axes.guides[3].color.x());
  REQUIRE(axes.guides[5].color.z() > axes.guides[5].color.x());

  style.show_scale_grid = true;
  model.setStyle(style);
  const auto combined = model.frame(kSquareExtent);
  REQUIRE(combined.grid_spacing == 5.0F);
  REQUIRE(combined.guides.size() > axes.guides.size());
  REQUIRE(combined.guides.size() <= 170);
  REQUIRE(combined.guides.size() % 2 == 0);

  style.show_coordinate_axes = false;
  model.setStyle(style);
  const auto grid = model.frame(kSquareExtent);
  REQUIRE(grid.grid_spacing == combined.grid_spacing);
  REQUIRE(grid.guides.size() + 6 == combined.guides.size());
}

TEST_CASE("scene guides respect sub-unit XY, dominant Z, and light backgrounds",
          "[viewport_model][guides]") {
  ViewportModel model;
  model.setCloud(
      snapshot(1, {0.0F, 0.0F, 0.0F}, {0.01F, 0.01F, 1000.0F}));
  kpt::gui::ViewportStyle style;
  style.show_coordinate_axes = true;
  style.show_scale_grid = true;
  style.background = Eigen::Vector3f::Ones();
  model.setStyle(style);

  const auto frame = model.frame(kSquareExtent);
  REQUIRE(frame.grid_spacing == Approx(0.002F));
  REQUIRE(frame.guides.back().position.z() == Approx(1000.0F));
  REQUIRE(frame.guides.front().color.maxCoeff() < 0.8F);
  REQUIRE(frame.guides[frame.guides.size() - 2].color.maxCoeff() < 0.8F);

  model.setCloud(
      snapshot(2, {0.0F, 0.0F, 0.0F}, {1.0e-6F, 1.0e-6F, 1.0e-6F}));
  const auto microscopic = model.frame(kSquareExtent);
  REQUIRE(microscopic.grid_spacing == Approx(2.0e-7F));
}

TEST_CASE("CloudCompare trackball and screen-plane pan are reversible",
          "[viewport_model][camera]") {
  ViewportModel model;
  model.setCloud(snapshot(1, {-2.0F, -2.0F, -2.0F},
                          {2.0F, 2.0F, 2.0F}));
  model.setView(kpt::gui::CameraPreset::Front);
  const auto initial = model.frame(kSquareExtent);

  model.orbit(400.0F, 400.0F, 560.0F, 320.0F, kSquareExtent);
  const auto rotated = model.frame(kSquareExtent);
  REQUIRE(differs(initial, rotated));
  model.orbit(560.0F, 320.0F, 400.0F, 400.0F, kSquareExtent);
  REQUIRE(model.frame(kSquareExtent).view_projection.isApprox(
      initial.view_projection, 1.0e-4F));

  model.pan(25.0F, -14.0F, kSquareExtent);
  REQUIRE(differs(initial, model.frame(kSquareExtent)));
  model.pan(-25.0F, 14.0F, kSquareExtent);
  REQUIRE(model.frame(kSquareExtent).view_projection.isApprox(
      initial.view_projection, 1.0e-4F));

  model.roll(100.0F, kSquareExtent);
  REQUIRE(differs(initial, model.frame(kSquareExtent)));
  model.roll(-100.0F, kSquareExtent);
  REQUIRE(model.frame(kSquareExtent).view_projection.isApprox(
      initial.view_projection, 1.0e-4F));
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
