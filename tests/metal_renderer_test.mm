#include <catch2/catch.hpp>

#include "gui/backend/metal/test_support.hpp"
#include "gui/viewport/model.hpp"

#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <numeric>
#include <vector>

namespace {

kpt::gui::ViewportVertex vertex(float x, float y, float z, float red,
                                float green, float blue, float intensity,
                                float noise = 0.0F) {
  return {{x, y, z}, {red, green, blue}, intensity, noise};
}

kpt::gui::ViewportFrame
frame(kpt::ColorBy color_by,
      Eigen::Vector3f background = Eigen::Vector3f::Zero()) {
  kpt::gui::ViewportFrame result;
  result.style.color_by = color_by;
  result.style.point_size = 15.0F;
  result.style.background = background;
  result.style.scalar_min = 0.0F;
  result.style.scalar_max = 1.0F;
  return result;
}

kpt::gui::Rgba8Image renderRead(kpt::gui::MetalRendererTestFixture &fixture,
                                const kpt::gui::ViewportFrame &value) {
  auto context = kpt::gui::beginMetalFrameForTests(fixture);
  REQUIRE(context);
  REQUIRE(fixture.renderer.renderer->render(value, context.value().get()));
  auto result =
      fixture.renderer.readback->readColor(*fixture.renderer.renderer);
  REQUIRE(result);
  auto image = std::move(result).value();
  REQUIRE(image.bytes_per_row ==
          static_cast<std::size_t>(image.extent.width) * 4);
  REQUIRE(image.pixels.size() ==
          image.bytes_per_row * static_cast<std::size_t>(image.extent.height));
  return image;
}

kpt::gui::Rgba8Image
renderLayeredRead(kpt::gui::MetalRendererTestFixture &fixture,
                  const kpt::gui::ViewportFrame &value,
                  const kpt::gui::LayeredViewportFrame &layers) {
  auto context = kpt::gui::beginMetalFrameForTests(fixture);
  REQUIRE(context);
  REQUIRE(fixture.renderer.renderer->renderLayers(value, layers,
                                                  context.value().get()));
  auto result =
      fixture.renderer.readback->readColor(*fixture.renderer.renderer);
  REQUIRE(result);
  auto image = std::move(result).value();
  REQUIRE(image.bytes_per_row ==
          static_cast<std::size_t>(image.extent.width) * 4);
  REQUIRE(image.pixels.size() ==
          image.bytes_per_row * static_cast<std::size_t>(image.extent.height));
  return image;
}

bool differsFrom(const std::uint8_t *pixel,
                 const std::array<std::uint8_t, 3> &background) {
  constexpr int tolerance = 3;
  return std::abs(static_cast<int>(pixel[0]) - background[0]) > tolerance ||
         std::abs(static_cast<int>(pixel[1]) - background[1]) > tolerance ||
         std::abs(static_cast<int>(pixel[2]) - background[2]) > tolerance;
}

bool centerVisible(const kpt::gui::Rgba8Image &image,
                   std::array<std::uint8_t, 3> background) {
  const int center_x = image.extent.width / 2;
  const int center_y = image.extent.height / 2;
  for (int y = center_y - 2; y <= center_y + 2; ++y) {
    for (int x = center_x - 2; x <= center_x + 2; ++x) {
      const auto offset = (static_cast<std::size_t>(y) *
                               static_cast<std::size_t>(image.extent.width) +
                           static_cast<std::size_t>(x)) *
                          4;
      if (differsFrom(image.pixels.data() + offset, background))
        return true;
    }
  }
  return false;
}

std::uint64_t channelSum(const kpt::gui::Rgba8Image &image, int begin_x,
                         int end_x, int channel) {
  std::uint64_t sum = 0;
  for (int y = 0; y < image.extent.height; ++y) {
    for (int x = begin_x; x < end_x; ++x) {
      const auto offset = (static_cast<std::size_t>(y) *
                               static_cast<std::size_t>(image.extent.width) +
                           static_cast<std::size_t>(x)) *
                              4 +
                          static_cast<std::size_t>(channel);
      sum += image.pixels[offset];
    }
  }
  return sum;
}

} // namespace

TEST_CASE("Metal renderer satisfies viewport behavior contract",
          "[metal_renderer]") {
  auto fixture = kpt::gui::makeMetalRendererTestFixture();
  auto &renderer = *fixture.renderer.renderer;

  SECTION(
      "positive and suspended extents preserve native texture orientation") {
    REQUIRE(renderer.resize({73, 41}));
    REQUIRE(renderer.extent() == kpt::gui::PixelExtent{73, 41});
    REQUIRE(renderer.texture().ref.GetTexID() != ImTextureID_Invalid);
    REQUIRE(renderer.texture().uv0.x == 0.0F);
    REQUIRE(renderer.texture().uv0.y == 0.0F);
    REQUIRE(renderer.texture().uv1.x == 1.0F);
    REQUIRE(renderer.texture().uv1.y == 1.0F);
    const auto first_texture = renderer.texture().ref.GetTexID();

    REQUIRE(renderer.resize({19, 17}));
    REQUIRE(renderer.texture().ref.GetTexID() != first_texture);
    REQUIRE(renderer.resize({0, 17}));
    REQUIRE(renderer.extent() == kpt::gui::PixelExtent{0, 0});
    REQUIRE(renderer.texture().ref.GetTexID() == ImTextureID_Invalid);
    REQUIRE(renderer.resize({31, 29}));
    REQUIRE(renderer.texture().ref.GetTexID() != ImTextureID_Invalid);
  }

  SECTION("fit-like fixture reaches center and empty upload is background") {
    REQUIRE(renderer.resize({64, 64}));
    auto snapshot = std::make_shared<kpt::gui::ViewportCloudSnapshot>();
    snapshot->revision = 1;
    snapshot->vertices = {vertex(0.0F, 0.0F, 0.0F, 1.0F, 0.2F, 0.1F, 0.5F)};
    snapshot->bounds.finite_points = 1;
    snapshot->bounds.radius = 0.001F;
    kpt::gui::ViewportModel model;
    model.setCloud(snapshot, kpt::gui::CameraUpdate::Fit);
    auto fitted = model.frame(renderer.extent());
    fitted.style.color_by = kpt::ColorBy::RGB;
    fitted.style.point_size = 15.0F;
    REQUIRE(renderer.upload(snapshot->vertices, snapshot->revision));
    REQUIRE(centerVisible(renderRead(fixture, fitted), {0, 0, 0}));

    REQUIRE(renderer.upload({}, 2));
    REQUIRE_FALSE(centerVisible(renderRead(fixture, frame(kpt::ColorBy::RGB)),
                                {0, 0, 0}));
  }

  SECTION("layered renderer blends transparent pass against opaque depth") {
    REQUIRE(renderer.resize({64, 64}));
    const std::array opaque_points = {
        vertex(0.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F)};
    const std::array near_transparent_points = {
        vertex(0.0F, 0.0F, -0.25F, 0.0F, 0.0F, 1.0F, 0.0F)};
    const std::array uploads = {
        kpt::gui::ViewportLayerUpload{1, 1, opaque_points},
        kpt::gui::ViewportLayerUpload{2, 1, near_transparent_points},
    };
    REQUIRE(renderer.uploadLayers(uploads, 70));
    auto layer_style = frame(kpt::ColorBy::RGB).style;
    layer_style.point_size = 31.0F;
    kpt::gui::ViewportLayerDraw opaque{1, layer_style, {}, false, 1.0F};
    kpt::gui::ViewportLayerDraw transparent{2, layer_style, {}, false, 0.5F};
    const std::array opaque_draws = {opaque};
    const std::array transparent_draws = {transparent};
    const kpt::gui::LayeredViewportFrame layered{70, opaque_draws,
                                                 transparent_draws};
    const auto image =
        renderLayeredRead(fixture, frame(kpt::ColorBy::RGB), layered);
    const std::size_t center =
        (static_cast<std::size_t>(image.extent.height / 2) *
         image.bytes_per_row) +
        static_cast<std::size_t>(image.extent.width / 2) * 4U;
    REQUIRE(image.pixels[center] > 100U);
    REQUIRE(image.pixels[center + 1U] < 20U);
    REQUIRE(image.pixels[center + 2U] > 100U);

    const std::array far_transparent_points = {
        vertex(0.0F, 0.0F, 0.25F, 0.0F, 0.0F, 1.0F, 0.0F)};
    const std::array far_uploads = {
        kpt::gui::ViewportLayerUpload{1, 2, opaque_points},
        kpt::gui::ViewportLayerUpload{2, 2, far_transparent_points},
    };
    REQUIRE(renderer.uploadLayers(far_uploads, 71));
    const kpt::gui::LayeredViewportFrame far_layered{71, opaque_draws,
                                                     transparent_draws};
    const auto depth_tested =
        renderLayeredRead(fixture, frame(kpt::ColorBy::RGB), far_layered);
    REQUIRE(depth_tested.pixels[center] > 220U);
    REQUIRE(depth_tested.pixels[center + 2U] < 20U);
  }

  SECTION("layered interactive rendering keeps a bounded uniform LOD") {
    // Keep this above the production threshold by one point: the assertion
    // proves layered Metal buffers follow the same interaction policy as the
    // established single-cloud path without depending on pixel timing.
    std::vector<kpt::gui::ViewportVertex> points(
        500'001U, vertex(4.0F, 4.0F, 4.0F, 1.0F, 0.0F, 0.0F, 0.0F));
    const std::array uploads = {
        kpt::gui::ViewportLayerUpload{42, 1, points},
    };
    REQUIRE(renderer.uploadLayers(uploads, 72));
    REQUIRE(kpt::gui::metalLayeredLodPointCountForTests(renderer, 42) ==
            500'000U);
  }

  SECTION("scene guides render without point-cloud vertices") {
    REQUIRE(renderer.resize({64, 64}));
    REQUIRE(renderer.upload({}, 20));
    auto guide_frame = frame(kpt::ColorBy::RGB);
    guide_frame.guides = {
        {{-0.8F, 0.0F, 0.0F}, {0.9F, 0.2F, 0.2F}},
        {{0.8F, 0.0F, 0.0F}, {0.9F, 0.2F, 0.2F}},
    };
    REQUIRE(centerVisible(renderRead(fixture, guide_frame), {0, 0, 0}));
  }

  SECTION("unchanged viewport frames reuse the previous texture") {
    REQUIRE(renderer.resize({64, 64}));
    const std::array points = {
        vertex(0.0F, 0.0F, 0.0F, 0.8F, 0.4F, 0.2F, 0.5F)};
    REQUIRE(renderer.upload(points, 40));
    auto cached_frame = frame(kpt::ColorBy::RGB);

    (void)renderRead(fixture, cached_frame);
    REQUIRE(kpt::gui::metalEncodedFrameCountForTests(renderer) == 1);
    (void)renderRead(fixture, cached_frame);
    REQUIRE(kpt::gui::metalEncodedFrameCountForTests(renderer) == 1);

    cached_frame.style.point_size += 1.0F;
    (void)renderRead(fixture, cached_frame);
    REQUIRE(kpt::gui::metalEncodedFrameCountForTests(renderer) == 2);

    REQUIRE(renderer.resize({63, 64}));
    (void)renderRead(fixture, cached_frame);
    REQUIRE(kpt::gui::metalEncodedFrameCountForTests(renderer) == 3);

    REQUIRE(renderer.upload(points, 41));
    (void)renderRead(fixture, cached_frame);
    REQUIRE(kpt::gui::metalEncodedFrameCountForTests(renderer) == 4);
  }

  SECTION("large-offset singleton is rebased before scaling") {
    REQUIRE(renderer.resize({64, 64}));
    const float maximum = std::numeric_limits<float>::max();
    auto snapshot = std::make_shared<kpt::gui::ViewportCloudSnapshot>();
    snapshot->revision = 21;
    snapshot->vertices = {
        vertex(maximum, maximum, maximum, 0.7F, 0.8F, 1.0F, 0.5F)};
    snapshot->bounds.minimum = Eigen::Vector3f::Constant(maximum);
    snapshot->bounds.maximum = Eigen::Vector3f::Constant(maximum);
    snapshot->bounds.centroid = Eigen::Vector3f::Constant(maximum);
    snapshot->bounds.center = Eigen::Vector3f::Constant(maximum);
    snapshot->bounds.radius = 0.001;
    snapshot->bounds.finite_points = 1;
    kpt::gui::ViewportModel model;
    model.setCloud(snapshot);
    auto rebased = model.frame(renderer.extent());
    rebased.style.color_by = kpt::ColorBy::RGB;
    rebased.style.point_size = 15.0F;
    REQUIRE(renderer.upload(snapshot->vertices, snapshot->revision));
    REQUIRE(centerVisible(renderRead(fixture, rebased), {0, 0, 0}));

    snapshot->revision = 22;
    snapshot->vertices = {
        vertex(-maximum, -maximum, -maximum, 1.0F, 0.6F, 0.6F, 0.5F),
        vertex(maximum, maximum, maximum, 0.6F, 0.8F, 1.0F, 0.5F)};
    snapshot->bounds.minimum = Eigen::Vector3f::Constant(-maximum);
    snapshot->bounds.maximum = Eigen::Vector3f::Constant(maximum);
    snapshot->bounds.centroid = Eigen::Vector3f::Zero();
    snapshot->bounds.center = Eigen::Vector3f::Zero();
    snapshot->bounds.radius = std::sqrt(3.0) * static_cast<double>(maximum);
    kpt::gui::ViewportModel symmetric_model;
    symmetric_model.setCloud(snapshot);
    auto symmetric = symmetric_model.frame(renderer.extent());
    symmetric.style.color_by = kpt::ColorBy::RGB;
    symmetric.style.point_size = 15.0F;
    REQUIRE(symmetric.world_scale >= std::numeric_limits<float>::min());
    REQUIRE(renderer.upload(snapshot->vertices, snapshot->revision));
    (void)renderRead(fixture, symmetric);
  }

  SECTION("color modes differ by rendered image statistics") {
    REQUIRE(renderer.resize({80, 64}));
    const std::array points = {
        vertex(-0.45F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.05F),
        vertex(0.45F, 0.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.95F)};
    REQUIRE(renderer.upload(points, 3));
    const auto rgb = renderRead(fixture, frame(kpt::ColorBy::RGB));
    const auto intensity = renderRead(fixture, frame(kpt::ColorBy::Intensity));
    std::uint64_t distance = 0;
    for (std::size_t index = 0; index < rgb.pixels.size(); ++index)
      distance += static_cast<std::uint64_t>(
          std::abs(static_cast<int>(rgb.pixels[index]) -
                   static_cast<int>(intensity.pixels[index])));
    REQUIRE(distance > 1000);
    REQUIRE(channelSum(rgb, 0, rgb.extent.width / 2, 0) >
            channelSum(rgb, 0, rgb.extent.width / 2, 1));
    REQUIRE(channelSum(rgb, rgb.extent.width / 2, rgb.extent.width, 1) >
            channelSum(rgb, rgb.extent.width / 2, rgb.extent.width, 0));
  }

  SECTION("intensity colormaps produce selectable palettes") {
    REQUIRE(renderer.resize({80, 64}));
    const std::array points = {
        vertex(-0.45F, 0.0F, 0.0F, 1.0F, 1.0F, 1.0F, 0.05F),
        vertex(0.0F, 0.0F, 0.0F, 1.0F, 1.0F, 1.0F, 0.5F),
        vertex(0.45F, 0.0F, 0.0F, 1.0F, 1.0F, 1.0F, 0.95F)};
    REQUIRE(renderer.upload(points, 31));
    auto styled = frame(kpt::ColorBy::Intensity);
    std::vector<std::vector<std::uint8_t>> palettes;
    for (const auto color_map :
         {kpt::gui::ColorMap::Turbo, kpt::gui::ColorMap::Viridis,
          kpt::gui::ColorMap::Plasma, kpt::gui::ColorMap::Inferno,
          kpt::gui::ColorMap::Magma, kpt::gui::ColorMap::Grayscale}) {
      styled.style.color_map = color_map;
      palettes.push_back(renderRead(fixture, styled).pixels);
    }
    for (std::size_t left = 0; left < palettes.size(); ++left) {
      for (std::size_t right = left + 1; right < palettes.size(); ++right)
        REQUIRE(palettes[left] != palettes[right]);
    }
  }

  SECTION("noise color overrides every selectable base color") {
    REQUIRE(renderer.resize({80, 64}));
    const std::array points = {
        vertex(-0.45F, 0.0F, 0.0F, 0.1F, 0.2F, 1.0F, 0.05F, 0.0F),
        vertex(0.45F, 0.0F, 0.0F, 0.1F, 0.2F, 1.0F, 0.95F, 1.0F)};
    REQUIRE(renderer.upload(points, 30));
    auto styled = frame(kpt::ColorBy::None);
    styled.style.fixed_color = {0.0F, 1.0F, 0.0F};
    styled.style.noise_color = {1.0F, 0.0F, 0.0F};
    styled.style.highlight_noise = true;
    kpt::gui::Rgba8Image highlighted;
    for (const auto mode :
         {kpt::ColorBy::RGB, kpt::ColorBy::Intensity, kpt::ColorBy::Z,
          kpt::ColorBy::Label, kpt::ColorBy::None}) {
      styled.style.color_by = mode;
      highlighted = renderRead(fixture, styled);
      REQUIRE(channelSum(highlighted, highlighted.extent.width / 2,
                         highlighted.extent.width, 0) >
              channelSum(highlighted, highlighted.extent.width / 2,
                         highlighted.extent.width, 1));
    }
    REQUIRE(channelSum(highlighted, 0, highlighted.extent.width / 2, 1) >
            channelSum(highlighted, 0, highlighted.extent.width / 2, 0));

    styled.style.highlight_noise = false;
    const auto unhighlighted = renderRead(fixture, styled);
    REQUIRE(channelSum(unhighlighted, unhighlighted.extent.width / 2,
                       unhighlighted.extent.width, 1) >
            channelSum(unhighlighted, unhighlighted.extent.width / 2,
                       unhighlighted.extent.width, 0));
  }

  SECTION("non-finite input cannot poison a later upload") {
    REQUIRE(renderer.resize({64, 64}));
    const float nan = std::numeric_limits<float>::quiet_NaN();
    const std::array invalid = {
        vertex(nan, 0.0F, 0.0F, 1.0F, 1.0F, 1.0F, 1.0F)};
    REQUIRE(renderer.upload(invalid, 4));
    REQUIRE_FALSE(centerVisible(renderRead(fixture, frame(kpt::ColorBy::RGB)),
                                {0, 0, 0}));
    const std::array valid = {vertex(0.0F, 0.0F, 0.0F, 0.2F, 0.8F, 0.3F, 0.5F)};
    REQUIRE(renderer.upload(valid, 5));
    REQUIRE(centerVisible(renderRead(fixture, frame(kpt::ColorBy::RGB)),
                          {0, 0, 0}));
  }

  SECTION("inactive frame context fails without changing the last image") {
    REQUIRE(renderer.resize({32, 32}));
    const std::array points = {
        vertex(0.0F, 0.0F, 0.0F, 1.0F, 1.0F, 1.0F, 1.0F)};
    REQUIRE(renderer.upload(points, 6));
    const auto before = renderRead(fixture, frame(kpt::ColorBy::RGB));
    auto inactive = kpt::gui::makeInactiveMetalFrameContextForTests();
    const auto failed =
        renderer.render(frame(kpt::ColorBy::Intensity), *inactive);
    REQUIRE_FALSE(failed);
    REQUIRE(failed.error().code ==
            kpt::gui::RendererErrorCode::BackendMismatch);
    auto context = kpt::gui::beginMetalFrameForTests(fixture);
    REQUIRE(context);
    auto readback = fixture.renderer.readback->readColor(renderer);
    REQUIRE(readback);
    REQUIRE(readback.value().pixels == before.pixels);
  }
}

TEST_CASE("Metal renderer can be repeatedly created and destroyed",
          "[metal_renderer]") {
  for (int iteration = 0; iteration < 8; ++iteration) {
    auto fixture = kpt::gui::makeMetalRendererTestFixture();
    REQUIRE(fixture.renderer.renderer->resize({16, 16}));
    const std::array points = {
        vertex(0.0F, 0.0F, 0.0F, 1.0F, 1.0F, 1.0F, 1.0F)};
    REQUIRE(fixture.renderer.renderer->upload(
        points, static_cast<std::uint64_t>(iteration + 1)));
    REQUIRE(centerVisible(renderRead(fixture, frame(kpt::ColorBy::RGB)),
                          {0, 0, 0}));
  }
}
