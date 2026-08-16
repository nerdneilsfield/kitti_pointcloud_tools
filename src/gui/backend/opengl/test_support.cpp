#include "gui/backend/opengl/test_support.hpp"

#include "gui/backend/opengl/point_renderer.hpp"

#include <utility>

namespace kpt::gui {

class OpenGLRendererTestAccess final : public RendererTestAccess {
public:
  static std::unique_ptr<FrameContext> makeFrameContext(GLFWwindow *window,
                                                        bool active) {
    return std::unique_ptr<FrameContext>(
        new OpenGLFrameContext(window, active));
  }

  static bool frameContextIsActive(const FrameContext &context) {
    const auto *open_gl = dynamic_cast<const OpenGLFrameContext *>(&context);
    return open_gl != nullptr && open_gl->isActive();
  }

  static std::uint64_t encodedFrameCount(const OpenGLPointRenderer &renderer) {
    return renderer.encodedFrameCountForTests();
  }

  Result<Rgba8Image, RendererError>
  readColor(const ViewportRenderer &renderer) override {
    const auto *open_gl = dynamic_cast<const OpenGLPointRenderer *>(&renderer);
    if (open_gl == nullptr) {
      return RendererError{RendererErrorCode::BackendMismatch,
                           "OpenGL readback requires OpenGLPointRenderer"};
    }
    return open_gl->captureRgba();
  }
};

RendererTestFixture makeOpenGLRendererTestFixture(GLFWwindow *window) {
  RendererTestFixture fixture;
  fixture.renderer = std::make_unique<OpenGLPointRenderer>(window);
  fixture.readback = std::make_unique<OpenGLRendererTestAccess>();
  return fixture;
}

std::unique_ptr<FrameContext> makeOpenGLFrameContextForTests(GLFWwindow *window,
                                                             bool active) {
  return OpenGLRendererTestAccess::makeFrameContext(window, active);
}

bool openGLFrameContextIsActiveForTests(const FrameContext &context) {
  return OpenGLRendererTestAccess::frameContextIsActive(context);
}

std::uint64_t
openGLEncodedFrameCountForTests(const ViewportRenderer &renderer) {
  const auto *open_gl = dynamic_cast<const OpenGLPointRenderer *>(&renderer);
  return open_gl == nullptr
             ? 0
             : OpenGLRendererTestAccess::encodedFrameCount(*open_gl);
}

} // namespace kpt::gui
