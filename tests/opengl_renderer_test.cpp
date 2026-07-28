#include <catch2/catch.hpp>

#include "gui/backend/opengl/point_renderer.hpp"
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
                                float green, float blue, float intensity) {
  return {{x, y, z}, {red, green, blue}, intensity};
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

kpt::gui::RendererReadback read(kpt::gui::ViewportRenderer &renderer) {
  auto result = kpt::gui::RendererTestAccess::readColor(renderer);
  REQUIRE(result);
  return std::move(result).value();
}

bool differsFrom(const std::uint8_t *pixel,
                 const std::array<std::uint8_t, 3> &background) {
  constexpr int tolerance = 3;
  return std::abs(static_cast<int>(pixel[0]) - background[0]) > tolerance ||
         std::abs(static_cast<int>(pixel[1]) - background[1]) > tolerance ||
         std::abs(static_cast<int>(pixel[2]) - background[2]) > tolerance;
}

bool centerNeighborhoodVisible(const kpt::gui::RendererReadback &image,
                               std::array<std::uint8_t, 3> background) {
  const int center_x = image.extent.width / 2;
  const int center_y = image.extent.height / 2;
  for (int y = center_y - 2; y <= center_y + 2; ++y) {
    for (int x = center_x - 2; x <= center_x + 2; ++x) {
      const auto offset = (static_cast<std::size_t>(y) *
                               static_cast<std::size_t>(image.extent.width) +
                           static_cast<std::size_t>(x)) *
                          4;
      if (differsFrom(image.rgba.data() + offset, background))
        return true;
    }
  }
  return false;
}

std::uint64_t channelSum(const kpt::gui::RendererReadback &image, int begin_x,
                         int end_x, int channel) {
  std::uint64_t sum = 0;
  for (int y = 0; y < image.extent.height; ++y) {
    for (int x = begin_x; x < end_x; ++x) {
      const auto offset = (static_cast<std::size_t>(y) *
                               static_cast<std::size_t>(image.extent.width) +
                           static_cast<std::size_t>(x)) *
                              4 +
                          static_cast<std::size_t>(channel);
      sum += image.rgba[offset];
    }
  }
  return sum;
}

} // namespace

TEST_CASE("OpenGL renderer satisfies viewport behavior contract",
          "[opengl_renderer]") {
  HiddenOpenGLContext graphics;
  auto frame_context =
      kpt::gui::RendererTestAccess::makeOpenGLFrameContext(graphics.window());
  kpt::gui::OpenGLPointRenderer renderer(graphics.window());

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
    REQUIRE(centerNeighborhoodVisible(read(renderer), {0, 0, 0}));

    REQUIRE(renderer.upload({}, 2));
    REQUIRE(renderer.render(frame(kpt::ColorBy::RGB), *frame_context));
    const auto empty = read(renderer);
    REQUIRE_FALSE(centerNeighborhoodVisible(empty, {0, 0, 0}));
  }

  SECTION("color modes differ by rendered image statistics") {
    REQUIRE(renderer.resize({80, 64}));
    const std::array points = {
        vertex(-0.45F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.05F),
        vertex(0.45F, 0.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.95F)};
    REQUIRE(renderer.upload(points, 3));
    REQUIRE(renderer.render(frame(kpt::ColorBy::RGB), *frame_context));
    const auto rgb = read(renderer);
    REQUIRE(renderer.render(frame(kpt::ColorBy::Intensity), *frame_context));
    const auto intensity = read(renderer);
    REQUIRE(rgb.rgba.size() == intensity.rgba.size());
    std::uint64_t distance = 0;
    for (std::size_t index = 0; index < rgb.rgba.size(); ++index) {
      distance += static_cast<std::uint64_t>(
          std::abs(static_cast<int>(rgb.rgba[index]) -
                   static_cast<int>(intensity.rgba[index])));
    }
    REQUIRE(distance > 1000);
    REQUIRE(channelSum(rgb, 0, rgb.extent.width / 2, 0) >
            channelSum(rgb, 0, rgb.extent.width / 2, 1));
    REQUIRE(channelSum(rgb, rgb.extent.width / 2, rgb.extent.width, 1) >
            channelSum(rgb, rgb.extent.width / 2, rgb.extent.width, 0));
    REQUIRE(channelSum(rgb, 0, rgb.extent.width / 2, 0) !=
            channelSum(intensity, 0, intensity.extent.width / 2, 0));
  }

  SECTION("non-finite input cannot poison later uploads") {
    REQUIRE(renderer.resize({64, 64}));
    const float nan = std::numeric_limits<float>::quiet_NaN();
    const std::array invalid = {
        vertex(nan, 0.0F, 0.0F, 1.0F, 1.0F, 1.0F, 1.0F)};
    REQUIRE(renderer.upload(invalid, 4));
    REQUIRE(renderer.pointCount() == 0);
    REQUIRE(renderer.render(frame(kpt::ColorBy::RGB), *frame_context));
    REQUIRE_FALSE(centerNeighborhoodVisible(read(renderer), {0, 0, 0}));

    const std::array valid = {vertex(0.0F, 0.0F, 0.0F, 0.2F, 0.8F, 0.3F, 0.5F)};
    REQUIRE(renderer.upload(valid, 5));
    REQUIRE(renderer.pointCount() == 1);
    REQUIRE(renderer.render(frame(kpt::ColorBy::RGB), *frame_context));
    REQUIRE(centerNeighborhoodVisible(read(renderer), {0, 0, 0}));
  }

  SECTION("inactive frame context fails without mutating the last image") {
    REQUIRE(renderer.resize({32, 32}));
    const std::array points = {
        vertex(0.0F, 0.0F, 0.0F, 1.0F, 1.0F, 1.0F, 1.0F)};
    REQUIRE(renderer.upload(points, 7));
    REQUIRE(renderer.render(frame(kpt::ColorBy::RGB), *frame_context));
    const auto before = read(renderer);
    auto inactive =
        kpt::gui::RendererTestAccess::makeOpenGLFrameContext(graphics.window(),
                                                             false);
    const auto failed =
        renderer.render(frame(kpt::ColorBy::Intensity), *inactive);
    REQUIRE_FALSE(failed);
    REQUIRE(failed.error().code ==
            kpt::gui::RendererErrorCode::BackendMismatch);
    REQUIRE(read(renderer).rgba == before.rgba);
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
    REQUIRE(centerNeighborhoodVisible(read(renderer), {0, 0, 0}));

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
            std::array<unsigned char, 4>{GL_FALSE, GL_TRUE, GL_FALSE,
                                         GL_TRUE});
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
    static_cast<void>(read(renderer));
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
    kpt::gui::OpenGLPointRenderer renderer(graphics.window());
    REQUIRE(renderer.resize({16 + index, 16 + index}));
  }
}
