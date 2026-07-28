#pragma once

#include "gui/viewport/test_access.hpp"

#include <memory>

struct GLFWwindow;

namespace kpt::gui {

[[nodiscard]] RendererTestFixture
makeOpenGLRendererTestFixture(GLFWwindow *window);

[[nodiscard]] std::unique_ptr<FrameContext>
makeOpenGLFrameContextForTests(GLFWwindow *window, bool active = true);

[[nodiscard]] bool
openGLFrameContextIsActiveForTests(const FrameContext &context);

} // namespace kpt::gui
