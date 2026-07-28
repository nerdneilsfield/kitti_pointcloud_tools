#pragma once

#include "gui/viewport/renderer.hpp"

#include <cstddef>
#include <cstdint>

struct GLFWwindow;

namespace kpt::gui {

class RendererTestAccess;
class GlfwOpenGLRuntime;

class OpenGLFrameContext final : public FrameContext {
public:
  [[nodiscard]] BackendKind backendKind() const noexcept override {
    return BackendKind::OpenGL;
  }

private:
  friend class GlfwOpenGLRuntime;
  friend class OpenGLPointRenderer;
  friend class RendererTestAccess;

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
    return BackendKind::OpenGL;
  }
  [[nodiscard]] std::size_t pointCount() const noexcept { return point_count_; }
  [[nodiscard]] std::uint64_t uploadedRevision() const noexcept {
    return uploaded_revision_;
  }

private:
  friend class RendererTestAccess;

  [[nodiscard]] bool expectedContextIsCurrent() const noexcept;
  Result<void, RendererError> createStaticResources();
  void destroyStaticResources() noexcept;
  void destroyFramebuffer() noexcept;

  GLFWwindow *expected_window_ = nullptr;
  unsigned vertex_array_ = 0;
  unsigned vertex_buffer_ = 0;
  unsigned program_ = 0;
  unsigned framebuffer_ = 0;
  unsigned color_texture_ = 0;
  unsigned depth_buffer_ = 0;
  int view_projection_location_ = -1;
  int point_size_location_ = -1;
  int color_mode_location_ = -1;
  int scalar_range_location_ = -1;
  PixelExtent extent_;
  std::size_t point_count_ = 0;
  std::uint64_t uploaded_revision_ = 0;
};

} // namespace kpt::gui
