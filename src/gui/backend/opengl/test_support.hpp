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

[[nodiscard]] std::uint64_t
openGLEncodedFrameCountForTests(const ViewportRenderer &renderer);

} // namespace kpt::gui
