#pragma once

#include "common/result.hpp"
#include "gui/viewport/renderer.hpp"

#include <cstdint>
#include <memory>
#include <vector>

struct GLFWwindow;

namespace kpt::gui {

struct RendererReadback {
  PixelExtent extent;
  // RGBA8 rows in top-left UI order.
  std::vector<std::uint8_t> rgba;
};

class RendererTestAccess {
public:
  static std::unique_ptr<FrameContext>
  makeOpenGLFrameContext(GLFWwindow *window, bool active = true);
  [[nodiscard]] static bool frameContextIsActive(const FrameContext &context);

  static Result<RendererReadback, RendererError>
  readColor(const ViewportRenderer &renderer);
};

} // namespace kpt::gui
