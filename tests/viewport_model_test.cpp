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
  const auto fitted = model.frame(kSquareExtent).view_projection;

  model.orbit(10.0F, -6.0F);
  const auto orbited = model.frame(kSquareExtent).view_projection;
  REQUIRE(differs(fitted, orbited));

  model.pan(8.0F, 4.0F);
  const auto panned = model.frame(kSquareExtent).view_projection;
  REQUIRE(differs(orbited, panned));

  model.zoom(2.0F);
  const auto zoomed = model.frame(kSquareExtent).view_projection;
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

TEST_CASE("fit and every view preset produce finite camera matrices",
          "[viewport_model]") {
  ViewportModel model;
  model.setCloud(snapshot(1, {-5.0F, -2.0F, -1.0F}, {7.0F, 4.0F, 3.0F}));
  model.orbit(20.0F, 10.0F);
  model.fit();
  const auto fitted = model.frame(kSquareExtent).view_projection;
  REQUIRE(fitted.allFinite());

  const std::vector<kpt::View> views = {
      kpt::View::Front,         kpt::View::Right,
      kpt::View::Back,          kpt::View::Left,
      kpt::View::Top,           kpt::View::Bottom,
      kpt::View::TopRightFront, kpt::View::TopLeftFront,
      kpt::View::BotRightFront, kpt::View::BotLeftFront};
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
  REQUIRE(model.cloudRevision() == 1);
}

TEST_CASE("preserve camera keeps pose while accepting newer cloud",
          "[viewport_model]") {
  ViewportModel model;
  model.setCloud(snapshot(1, Eigen::Vector3f::Zero(), Eigen::Vector3f::Ones()));
  model.orbit(12.0F, 5.0F);
  model.pan(3.0F, -2.0F);
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
  model.setView(kpt::View::Front);

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
