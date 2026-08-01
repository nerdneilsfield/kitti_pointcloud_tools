#pragma once

#include "gui/viewport/renderer.hpp"

#include <cstddef>
#include <cstdint>
#include <optional>

struct GLFWwindow;

namespace kpt::gui {

class OpenGLRendererTestAccess;
class GlfwOpenGLRuntime;
class GlfwWebGLRuntime;

class OpenGLFrameContext final : public FrameContext {
public:
  [[nodiscard]] BackendKind backendKind() const noexcept override {
#ifdef __EMSCRIPTEN__
    return BackendKind::WebGL;
#else
    return BackendKind::OpenGL;
#endif
  }

private:
  friend class GlfwOpenGLRuntime;
  friend class GlfwWebGLRuntime;
  friend class OpenGLPointRenderer;
  friend class OpenGLRendererTestAccess;

  explicit OpenGLFrameContext(GLFWwindow *expected_window,
                              bool active = true) noexcept;
  [[nodiscard]] GLFWwindow *expectedWindow() const noexcept {
    return expected_window_;
  }
  [[nodiscard]] bool isActive() const noexcept { return active_; }
  void activate() noexcept { active_ = true; }
  void invalidate() noexcept { active_ = false; }

  void bindWindow(GLFWwindow *expected_window) noexcept {
    expected_window_ = expected_window;
  }

  GLFWwindow *expected_window_ = nullptr;
  bool active_ = false;
};

class OpenGLPointRenderer final : public ViewportRenderer {
public:
  explicit OpenGLPointRenderer(GLFWwindow *expected_window);
  ~OpenGLPointRenderer() override;
  OpenGLPointRenderer(const OpenGLPointRenderer &) = delete;
  OpenGLPointRenderer &operator=(const OpenGLPointRenderer &) = delete;

  Result<void, RendererError> upload(std::span<const ViewportVertex> vertices,
                                     std::uint64_t revision) override;
  Result<void, RendererError> resize(PixelExtent physical_pixels) override;
  Result<void, RendererError> render(const ViewportFrame &frame,
                                     FrameContext &context) override;

  [[nodiscard]] ViewportTexture texture() const override;
  [[nodiscard]] PixelExtent extent() const override { return extent_; }
  [[nodiscard]] BackendKind backendKind() const noexcept override {
#ifdef __EMSCRIPTEN__
    return BackendKind::WebGL;
#else
    return BackendKind::OpenGL;
#endif
  }
  [[nodiscard]] std::size_t pointCount() const noexcept { return point_count_; }
  [[nodiscard]] std::uint64_t uploadedRevision() const noexcept {
    return uploaded_revision_;
  }

private:
  friend class OpenGLRendererTestAccess;

  [[nodiscard]] bool expectedContextIsCurrent() const noexcept;
  [[nodiscard]] std::uint64_t encodedFrameCountForTests() const noexcept {
    return encoded_frame_count_;
  }
  Result<void, RendererError> createStaticResources();
  void destroyStaticResources() noexcept;
  void destroyFramebuffer() noexcept;

  GLFWwindow *expected_window_ = nullptr;
  unsigned vertex_array_ = 0;
  unsigned vertex_buffer_ = 0;
#ifdef __EMSCRIPTEN__
  unsigned lod_index_buffer_ = 0;
  std::size_t lod_point_count_ = 0;
#endif
  unsigned guide_vertex_array_ = 0;
  unsigned guide_vertex_buffer_ = 0;
  unsigned program_ = 0;
  unsigned framebuffer_ = 0;
  unsigned color_texture_ = 0;
  unsigned depth_buffer_ = 0;
  int view_projection_location_ = -1;
  int point_size_location_ = -1;
  int world_origin_location_ = -1;
  int world_scale_location_ = -1;
  int color_mode_location_ = -1;
  int color_map_location_ = -1;
  int scalar_range_location_ = -1;
  int fixed_color_location_ = -1;
  int noise_color_location_ = -1;
  int highlight_noise_location_ = -1;
  int round_points_location_ = -1;
  PixelExtent extent_;
  std::size_t point_count_ = 0;
  std::uint64_t uploaded_revision_ = 0;
  std::optional<ViewportFrame> encoded_frame_;
  std::uint64_t encoded_revision_ = 0;
  std::uint64_t encoded_frame_count_ = 0;
};

} // namespace kpt::gui
