#include <catch2/catch.hpp>

#include "gui/backend/opengl/test_support.hpp"
#include "gui/viewport/model.hpp"
#include "gui/viewport/test_access.hpp"

#include <glad/gl.h>

#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <numeric>
#include <stdexcept>
#include <vector>

namespace {

class HiddenOpenGLContext {
public:
  HiddenOpenGLContext() {
    if (glfwInit() == GLFW_FALSE)
      throw std::runtime_error("glfwInit failed");
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
    window_ = glfwCreateWindow(64, 64, "KPT renderer test", nullptr, nullptr);
    if (window_ == nullptr) {
      glfwTerminate();
      throw std::runtime_error("hidden GLFW context creation failed");
    }
    glfwMakeContextCurrent(window_);
  }

  ~HiddenOpenGLContext() {
    glfwMakeContextCurrent(nullptr);
    glfwDestroyWindow(window_);
    glfwTerminate();
  }

  HiddenOpenGLContext(const HiddenOpenGLContext &) = delete;
  HiddenOpenGLContext &operator=(const HiddenOpenGLContext &) = delete;

  [[nodiscard]] GLFWwindow *window() const { return window_; }

private:
  GLFWwindow *window_ = nullptr;
};

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

kpt::gui::Rgba8Image read(kpt::gui::RendererTestFixture &fixture) {
  auto result = fixture.readback->readColor(*fixture.renderer);
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

bool centerNeighborhoodVisible(const kpt::gui::Rgba8Image &image,
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

TEST_CASE("OpenGL renderer satisfies viewport behavior contract",
          "[opengl_renderer]") {
  HiddenOpenGLContext graphics;
  auto frame_context =
      kpt::gui::makeOpenGLFrameContextForTests(graphics.window());
  auto fixture = kpt::gui::makeOpenGLRendererTestFixture(graphics.window());
  auto &renderer = *fixture.renderer;

  SECTION("first offscreen allocation preserves default OpenGL bindings") {
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glBindVertexArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, 0);
    glBindRenderbuffer(GL_RENDERBUFFER, 0);

    REQUIRE(renderer.resize({73, 41}));

    int value = -1;
    glGetIntegerv(GL_DRAW_FRAMEBUFFER_BINDING, &value);
    REQUIRE(value == 0);
    glGetIntegerv(GL_READ_FRAMEBUFFER_BINDING, &value);
    REQUIRE(value == 0);
    glGetIntegerv(GL_VERTEX_ARRAY_BINDING, &value);
    REQUIRE(value == 0);
    glGetIntegerv(GL_ARRAY_BUFFER_BINDING, &value);
    REQUIRE(value == 0);
    glGetIntegerv(GL_TEXTURE_BINDING_2D, &value);
    REQUIRE(value == 0);
    glGetIntegerv(GL_RENDERBUFFER_BINDING, &value);
    REQUIRE(value == 0);
  }

  SECTION("positive and suspended extents preserve UI texture orientation") {
    REQUIRE(renderer.resize({73, 41}));
    REQUIRE(renderer.extent() == kpt::gui::PixelExtent{73, 41});
    REQUIRE(renderer.texture().ref.GetTexID() != ImTextureID_Invalid);
    REQUIRE(renderer.texture().uv0.x == 0.0F);
    REQUIRE(renderer.texture().uv0.y == 1.0F);
    REQUIRE(renderer.texture().uv1.x == 1.0F);
    REQUIRE(renderer.texture().uv1.y == 0.0F);
    const auto first_texture = renderer.texture().ref.GetTexID();

    REQUIRE(renderer.resize({19, 17}));
    REQUIRE(renderer.extent() == kpt::gui::PixelExtent{19, 17});
    REQUIRE(renderer.texture().ref.GetTexID() != first_texture);

    REQUIRE(renderer.resize({0, 17}));
    REQUIRE(renderer.extent() == kpt::gui::PixelExtent{0, 0});
    REQUIRE(renderer.render(frame(kpt::ColorBy::RGB), *frame_context));
    REQUIRE(renderer.texture().ref.GetTexID() == ImTextureID_Invalid);

    REQUIRE(renderer.resize({31, 29}));
    REQUIRE(renderer.extent() == kpt::gui::PixelExtent{31, 29});
    REQUIRE(renderer.texture().ref.GetTexID() != ImTextureID_Invalid);
  }

  SECTION("fit-like identity fixture reaches center and empty is background") {
    REQUIRE(renderer.resize({64, 64}));
    auto snapshot = std::make_shared<kpt::gui::ViewportCloudSnapshot>();
    snapshot->revision = 1;
    snapshot->vertices = {vertex(0.0F, 0.0F, 0.0F, 1.0F, 0.2F, 0.1F, 0.5F)};
    snapshot->bounds.finite_points = 1;
    snapshot->bounds.radius = 0.001F;
    kpt::gui::ViewportModel model;
    model.setCloud(snapshot, kpt::gui::CameraUpdate::Fit);
    auto fitted_frame = model.frame(renderer.extent());
    fitted_frame.style.color_by = kpt::ColorBy::RGB;
    fitted_frame.style.point_size = 15.0F;
    REQUIRE(renderer.upload(snapshot->vertices, snapshot->revision));
    REQUIRE(renderer.render(fitted_frame, *frame_context));
    REQUIRE(centerNeighborhoodVisible(read(fixture), {0, 0, 0}));

    REQUIRE(renderer.upload({}, 2));
    REQUIRE(renderer.render(frame(kpt::ColorBy::RGB), *frame_context));
    const auto empty = read(fixture);
    REQUIRE_FALSE(centerNeighborhoodVisible(empty, {0, 0, 0}));
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
    const kpt::gui::LayeredViewportFrame layered{
        70, opaque_draws, transparent_draws};
    REQUIRE(renderer.renderLayers(frame(kpt::ColorBy::RGB), layered,
                                  *frame_context));
    const auto image = read(fixture);
    const std::size_t center =
        (static_cast<std::size_t>(image.extent.height / 2) * image.bytes_per_row) +
        static_cast<std::size_t>(image.extent.width / 2) * 4U;
    REQUIRE(image.pixels[center] > 100U);
    REQUIRE(image.pixels[center + 1U] < 20U);
    REQUIRE(image.pixels[center + 2U] > 100U);

    // A transparent point behind the opaque layer must fail the same depth
    // test; alpha blending never turns it into a fake background colour.
    const std::array far_transparent_points = {
        vertex(0.0F, 0.0F, 0.25F, 0.0F, 0.0F, 1.0F, 0.0F)};
    const std::array far_uploads = {
        kpt::gui::ViewportLayerUpload{1, 2, opaque_points},
        kpt::gui::ViewportLayerUpload{2, 2, far_transparent_points},
    };
    REQUIRE(renderer.uploadLayers(far_uploads, 71));
    const kpt::gui::LayeredViewportFrame far_layered{
        71, opaque_draws, transparent_draws};
    REQUIRE(renderer.renderLayers(frame(kpt::ColorBy::RGB), far_layered,
                                  *frame_context));
    const auto depth_tested = read(fixture);
    REQUIRE(depth_tested.pixels[center] > 220U);
    REQUIRE(depth_tested.pixels[center + 2U] < 20U);
  }

  SECTION("scene guides render without point-cloud vertices") {
    REQUIRE(renderer.resize({64, 64}));
    REQUIRE(renderer.upload({}, 20));
    auto guide_frame = frame(kpt::ColorBy::RGB);
    guide_frame.guides = {
        {{-0.8F, 0.0F, 0.0F}, {0.9F, 0.2F, 0.2F}},
        {{0.8F, 0.0F, 0.0F}, {0.9F, 0.2F, 0.2F}},
    };
    REQUIRE(renderer.render(guide_frame, *frame_context));
    const auto expected = read(fixture);
    REQUIRE(centerNeighborhoodVisible(expected, {0, 0, 0}));
    const auto captured = renderer.captureRgba();
    REQUIRE(captured);
    REQUIRE(captured.value().pixels == expected.pixels);
  }

  SECTION("RGBA capture requires a completed frame for the current target") {
    REQUIRE(renderer.resize({41, 29}));
    REQUIRE_FALSE(renderer.captureRgba());

    const std::array points = {
        vertex(0.0F, 0.0F, 0.0F, 0.15F, 0.65F, 0.95F, 0.5F)};
    REQUIRE(renderer.upload(points, 21));
    REQUIRE(renderer.render(frame(kpt::ColorBy::RGB), *frame_context));
    REQUIRE(renderer.captureRgba());

    // A resize creates a different FBO/texture.  Its storage may contain
    // undefined data until the following render, including under WebGL2.
    REQUIRE(renderer.resize({42, 29}));
    const auto after_resize = renderer.captureRgba();
    REQUIRE_FALSE(after_resize);
    REQUIRE(after_resize.error().code ==
            kpt::gui::RendererErrorCode::EncodingFailed);
  }

  SECTION("unchanged viewport frames reuse the previous texture") {
    REQUIRE(renderer.resize({64, 64}));
    const std::array points = {
        vertex(0.0F, 0.0F, 0.0F, 0.8F, 0.4F, 0.2F, 0.5F)};
    REQUIRE(renderer.upload(points, 40));
    auto cached_frame = frame(kpt::ColorBy::RGB);

    REQUIRE(renderer.render(cached_frame, *frame_context));
    REQUIRE(kpt::gui::openGLEncodedFrameCountForTests(renderer) == 1);
    REQUIRE(renderer.render(cached_frame, *frame_context));
    REQUIRE(kpt::gui::openGLEncodedFrameCountForTests(renderer) == 1);

    cached_frame.style.point_size += 1.0F;
    REQUIRE(renderer.render(cached_frame, *frame_context));
    REQUIRE(kpt::gui::openGLEncodedFrameCountForTests(renderer) == 2);

    REQUIRE(renderer.resize({63, 64}));
    REQUIRE(renderer.render(cached_frame, *frame_context));
    REQUIRE(kpt::gui::openGLEncodedFrameCountForTests(renderer) == 3);

    REQUIRE(renderer.upload(points, 41));
    REQUIRE(renderer.render(cached_frame, *frame_context));
    REQUIRE(kpt::gui::openGLEncodedFrameCountForTests(renderer) == 4);
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
    REQUIRE(renderer.render(rebased, *frame_context));
    REQUIRE(centerNeighborhoodVisible(read(fixture), {0, 0, 0}));

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
    REQUIRE(renderer.render(symmetric, *frame_context));
  }

  SECTION("color modes differ by rendered image statistics") {
    REQUIRE(renderer.resize({80, 64}));
    const std::array points = {
        vertex(-0.45F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.05F),
        vertex(0.45F, 0.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.95F)};
    REQUIRE(renderer.upload(points, 3));
    REQUIRE(renderer.render(frame(kpt::ColorBy::RGB), *frame_context));
    const auto rgb = read(fixture);
    REQUIRE(renderer.render(frame(kpt::ColorBy::Intensity), *frame_context));
    const auto intensity = read(fixture);
    REQUIRE(rgb.pixels.size() == intensity.pixels.size());
    std::uint64_t distance = 0;
    for (std::size_t index = 0; index < rgb.pixels.size(); ++index) {
      distance += static_cast<std::uint64_t>(
          std::abs(static_cast<int>(rgb.pixels[index]) -
                   static_cast<int>(intensity.pixels[index])));
    }
    REQUIRE(distance > 1000);
    REQUIRE(channelSum(rgb, 0, rgb.extent.width / 2, 0) >
            channelSum(rgb, 0, rgb.extent.width / 2, 1));
    REQUIRE(channelSum(rgb, rgb.extent.width / 2, rgb.extent.width, 1) >
            channelSum(rgb, rgb.extent.width / 2, rgb.extent.width, 0));
    REQUIRE(channelSum(rgb, 0, rgb.extent.width / 2, 0) !=
            channelSum(intensity, 0, intensity.extent.width / 2, 0));
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
      REQUIRE(renderer.render(styled, *frame_context));
      palettes.push_back(read(fixture).pixels);
    }
    for (std::size_t left = 0; left < palettes.size(); ++left) {
      for (std::size_t right = left + 1; right < palettes.size(); ++right)
        REQUIRE(palettes[left] != palettes[right]);
    }
  }

  SECTION("intensity equalization brightens skewed low-intensity points") {
    REQUIRE(renderer.resize({80, 64}));
    // Three points share a low intensity value; a single high point sits at
    // the right edge. A linear colormap would paint all low points with the
    // same dark color, while equalization should map them across the ramp.
    const std::array points = {
        vertex(-0.45F, 0.0F, 0.0F, 1.0F, 1.0F, 1.0F, 0.05F),
        vertex(-0.15F, 0.0F, 0.0F, 1.0F, 1.0F, 1.0F, 0.10F),
        vertex(0.15F, 0.0F, 0.0F, 1.0F, 1.0F, 1.0F, 0.15F),
        vertex(0.45F, 0.0F, 0.0F, 1.0F, 1.0F, 1.0F, 0.95F)};
    REQUIRE(renderer.upload(points, 32));

    auto base = frame(kpt::ColorBy::Intensity);
    base.style.color_map = kpt::gui::ColorMap::Grayscale;
    base.style.scalar_min = 0.0F;
    base.style.scalar_max = 1.0F;

    auto linear = base;
    REQUIRE(renderer.render(linear, *frame_context));
    const auto linear_image = read(fixture);

    auto equalized = base;
    equalized.style.intensity_equalize = true;
    equalized.intensity_cdf_valid = true;
    // Heavily front-loaded CDF: 75% of the mass sits below intensity 0.2.
    for (std::size_t i = 0; i < equalized.intensity_cdf.size(); ++i) {
      const float x = static_cast<float>(i) /
                      static_cast<float>(equalized.intensity_cdf.size() - 1);
      equalized.intensity_cdf[i] = std::min(1.0F, x * 4.0F);
    }
    REQUIRE(renderer.render(equalized, *frame_context));
    const auto equalized_image = read(fixture);

    // The left-half (low-intensity points) must get brighter under equalization.
    const auto left_linear =
        channelSum(linear_image, 0, linear_image.extent.width / 2, 0);
    const auto left_equalized =
        channelSum(equalized_image, 0, equalized_image.extent.width / 2, 0);
    REQUIRE(left_equalized > left_linear);
  }

  SECTION("noise color overrides selectable base color") {
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
      REQUIRE(renderer.render(styled, *frame_context));
      highlighted = read(fixture);
      REQUIRE(channelSum(highlighted, highlighted.extent.width / 2,
                         highlighted.extent.width, 0) >
              channelSum(highlighted, highlighted.extent.width / 2,
                         highlighted.extent.width, 1));
    }
    REQUIRE(channelSum(highlighted, 0, highlighted.extent.width / 2, 1) >
            channelSum(highlighted, 0, highlighted.extent.width / 2, 0));

    styled.style.highlight_noise = false;
    REQUIRE(renderer.render(styled, *frame_context));
    const auto unhighlighted = read(fixture);
    REQUIRE(channelSum(unhighlighted, unhighlighted.extent.width / 2,
                       unhighlighted.extent.width, 1) >
            channelSum(unhighlighted, unhighlighted.extent.width / 2,
                       unhighlighted.extent.width, 0));
  }

  SECTION("non-finite input cannot poison later uploads") {
    REQUIRE(renderer.resize({64, 64}));
    const float nan = std::numeric_limits<float>::quiet_NaN();
    const std::array invalid = {
        vertex(nan, 0.0F, 0.0F, 1.0F, 1.0F, 1.0F, 1.0F)};
    REQUIRE(renderer.upload(invalid, 4));
    REQUIRE(renderer.render(frame(kpt::ColorBy::RGB), *frame_context));
    REQUIRE_FALSE(centerNeighborhoodVisible(read(fixture), {0, 0, 0}));

    const std::array valid = {vertex(0.0F, 0.0F, 0.0F, 0.2F, 0.8F, 0.3F, 0.5F)};
    REQUIRE(renderer.upload(valid, 5));
    REQUIRE(renderer.render(frame(kpt::ColorBy::RGB), *frame_context));
    REQUIRE(centerNeighborhoodVisible(read(fixture), {0, 0, 0}));
  }

  SECTION("inactive frame context fails without mutating the last image") {
    REQUIRE(renderer.resize({32, 32}));
    const std::array points = {
        vertex(0.0F, 0.0F, 0.0F, 1.0F, 1.0F, 1.0F, 1.0F)};
    REQUIRE(renderer.upload(points, 7));
    REQUIRE(renderer.render(frame(kpt::ColorBy::RGB), *frame_context));
    const auto before = read(fixture);
    auto inactive =
        kpt::gui::makeOpenGLFrameContextForTests(graphics.window(), false);
    const auto failed =
        renderer.render(frame(kpt::ColorBy::Intensity), *inactive);
    REQUIRE_FALSE(failed);
    REQUIRE(failed.error().code ==
            kpt::gui::RendererErrorCode::BackendMismatch);
    REQUIRE(read(fixture).pixels == before.pixels);
  }

  SECTION("render and readback restore caller OpenGL state") {
    REQUIRE(renderer.resize({32, 32}));
    const std::array points = {
        vertex(0.0F, 0.0F, 0.0F, 1.0F, 1.0F, 1.0F, 1.0F)};
    REQUIRE(renderer.upload(points, 6));

    unsigned caller_framebuffer = 0;
    unsigned caller_vertex_array = 0;
    unsigned caller_buffer = 0;
    unsigned caller_texture = 0;
    unsigned caller_renderbuffer = 0;
    glGenFramebuffers(1, &caller_framebuffer);
    glGenVertexArrays(1, &caller_vertex_array);
    glGenBuffers(1, &caller_buffer);
    glGenTextures(1, &caller_texture);
    glGenRenderbuffers(1, &caller_renderbuffer);
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, caller_framebuffer);
    glBindVertexArray(caller_vertex_array);
    glBindBuffer(GL_ARRAY_BUFFER, caller_buffer);
    glActiveTexture(GL_TEXTURE3);
    glBindTexture(GL_TEXTURE_2D, caller_texture);
    glBindRenderbuffer(GL_RENDERBUFFER, caller_renderbuffer);
    glViewport(3, 4, 21, 22);
    glClearColor(0.125F, 0.25F, 0.5F, 0.75F);
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_PROGRAM_POINT_SIZE);
    glEnable(GL_SCISSOR_TEST);
    glScissor(7, 8, 1, 1);
    glEnable(GL_BLEND);
    glBlendFuncSeparate(GL_ZERO, GL_ZERO, GL_ZERO, GL_ZERO);
    glBlendEquationSeparate(GL_FUNC_REVERSE_SUBTRACT, GL_FUNC_SUBTRACT);
    glDepthMask(GL_FALSE);
    glDepthFunc(GL_NEVER);
    glColorMask(GL_FALSE, GL_TRUE, GL_FALSE, GL_TRUE);
    glEnable(GL_RASTERIZER_DISCARD);
    REQUIRE(renderer.render(frame(kpt::ColorBy::RGB), *frame_context));
    REQUIRE(centerNeighborhoodVisible(read(fixture), {0, 0, 0}));

    std::array<int, 4> viewport{};
    std::array<float, 4> clear{};
    glGetIntegerv(GL_VIEWPORT, viewport.data());
    glGetFloatv(GL_COLOR_CLEAR_VALUE, clear.data());
    REQUIRE(viewport == std::array<int, 4>{3, 4, 21, 22});
    REQUIRE(clear[0] == Approx(0.125F));
    REQUIRE(clear[1] == Approx(0.25F));
    REQUIRE(clear[2] == Approx(0.5F));
    REQUIRE(clear[3] == Approx(0.75F));
    REQUIRE(glIsEnabled(GL_DEPTH_TEST) == GL_FALSE);
    REQUIRE(glIsEnabled(GL_PROGRAM_POINT_SIZE) == GL_FALSE);
    REQUIRE(glIsEnabled(GL_SCISSOR_TEST) == GL_TRUE);
    REQUIRE(glIsEnabled(GL_BLEND) == GL_TRUE);
    REQUIRE(glIsEnabled(GL_RASTERIZER_DISCARD) == GL_TRUE);
    std::array<int, 4> scissor{};
    int value = 0;
    glGetIntegerv(GL_SCISSOR_BOX, scissor.data());
    REQUIRE(scissor == std::array<int, 4>{7, 8, 1, 1});
    unsigned char depth_mask = GL_TRUE;
    glGetBooleanv(GL_DEPTH_WRITEMASK, &depth_mask);
    REQUIRE(depth_mask == GL_FALSE);
    glGetIntegerv(GL_DEPTH_FUNC, &value);
    REQUIRE(value == GL_NEVER);
    std::array<unsigned char, 4> color_mask{};
    glGetBooleanv(GL_COLOR_WRITEMASK, color_mask.data());
    REQUIRE(color_mask ==
            std::array<unsigned char, 4>{GL_FALSE, GL_TRUE, GL_FALSE, GL_TRUE});
    glGetIntegerv(GL_BLEND_SRC_RGB, &value);
    REQUIRE(value == GL_ZERO);
    glGetIntegerv(GL_BLEND_DST_RGB, &value);
    REQUIRE(value == GL_ZERO);
    glGetIntegerv(GL_BLEND_SRC_ALPHA, &value);
    REQUIRE(value == GL_ZERO);
    glGetIntegerv(GL_BLEND_DST_ALPHA, &value);
    REQUIRE(value == GL_ZERO);
    glGetIntegerv(GL_BLEND_EQUATION_RGB, &value);
    REQUIRE(value == GL_FUNC_REVERSE_SUBTRACT);
    glGetIntegerv(GL_BLEND_EQUATION_ALPHA, &value);
    REQUIRE(value == GL_FUNC_SUBTRACT);
    glGetIntegerv(GL_DRAW_FRAMEBUFFER_BINDING, &value);
    REQUIRE(value == static_cast<int>(caller_framebuffer));
    glGetIntegerv(GL_VERTEX_ARRAY_BINDING, &value);
    REQUIRE(value == static_cast<int>(caller_vertex_array));
    glGetIntegerv(GL_ARRAY_BUFFER_BINDING, &value);
    REQUIRE(value == static_cast<int>(caller_buffer));
    glGetIntegerv(GL_ACTIVE_TEXTURE, &value);
    REQUIRE(value == GL_TEXTURE3);
    glGetIntegerv(GL_TEXTURE_BINDING_2D, &value);
    REQUIRE(value == static_cast<int>(caller_texture));
    glGetIntegerv(GL_RENDERBUFFER_BINDING, &value);
    REQUIRE(value == static_cast<int>(caller_renderbuffer));

    unsigned caller_pack_buffer = 0;
    glGenBuffers(1, &caller_pack_buffer);
    glBindBuffer(GL_PIXEL_PACK_BUFFER, caller_pack_buffer);
    glBufferData(GL_PIXEL_PACK_BUFFER, 16, nullptr, GL_STREAM_READ);
    glPixelStorei(GL_PACK_ALIGNMENT, 8);
    glPixelStorei(GL_PACK_ROW_LENGTH, 37);
    glPixelStorei(GL_PACK_SKIP_ROWS, 2);
    glPixelStorei(GL_PACK_SKIP_PIXELS, 3);
    static_cast<void>(read(fixture));
    glGetIntegerv(GL_PACK_ALIGNMENT, &value);
    REQUIRE(value == 8);
    glGetIntegerv(GL_PACK_ROW_LENGTH, &value);
    REQUIRE(value == 37);
    glGetIntegerv(GL_PACK_SKIP_ROWS, &value);
    REQUIRE(value == 2);
    glGetIntegerv(GL_PACK_SKIP_PIXELS, &value);
    REQUIRE(value == 3);
    glGetIntegerv(GL_PIXEL_PACK_BUFFER_BINDING, &value);
    REQUIRE(value == static_cast<int>(caller_pack_buffer));

    glBindBuffer(GL_PIXEL_PACK_BUFFER, 0);
    glDeleteBuffers(1, &caller_pack_buffer);
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);
    glBindVertexArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glActiveTexture(GL_TEXTURE3);
    glBindTexture(GL_TEXTURE_2D, 0);
    glBindRenderbuffer(GL_RENDERBUFFER, 0);
    glDisable(GL_SCISSOR_TEST);
    glDisable(GL_BLEND);
    glDisable(GL_RASTERIZER_DISCARD);
    glDepthMask(GL_TRUE);
    glDepthFunc(GL_LESS);
    glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
    glDeleteRenderbuffers(1, &caller_renderbuffer);
    glDeleteTextures(1, &caller_texture);
    glDeleteBuffers(1, &caller_buffer);
    glDeleteVertexArrays(1, &caller_vertex_array);
    glDeleteFramebuffers(1, &caller_framebuffer);
  }
}

TEST_CASE("OpenGL renderer can be repeatedly created and destroyed",
          "[opengl_renderer]") {
  HiddenOpenGLContext graphics;
  for (int index = 0; index < 8; ++index) {
    auto fixture = kpt::gui::makeOpenGLRendererTestFixture(graphics.window());
    REQUIRE(fixture.renderer->resize({16 + index, 16 + index}));
  }
}
