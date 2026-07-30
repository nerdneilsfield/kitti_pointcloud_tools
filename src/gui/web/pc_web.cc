#include "gui/app.hpp"
#include "gui/runtime/factory.hpp"
#include "gui/web/bridge.hpp"
#include "platform/services.hpp"

#include <emscripten.h>
#include <imgui.h>

#include <iostream>
#include <memory>
#include <string>
#include <utility>

namespace {

EM_JS(void, showFatalError, (const char *message), {
  globalThis.KptWeb.fatal(UTF8ToString(message));
});

struct WebApplication {
  kpt::platform::Services services;
  std::unique_ptr<kpt::gui::GuiRuntime> runtime;
  std::unique_ptr<kpt::gui::App> app;
  bool failed = false;

  void fail(std::string message) {
    if (failed)
      return;
    failed = true;
    std::cerr << message << '\n';
    showFatalError(message.c_str());
    emscripten_cancel_main_loop();
  }
};

void frame(void *opaque) {
  auto &state = *static_cast<WebApplication *>(opaque);
  if (state.failed)
    return;
  state.runtime->pollEvents();
  auto begun = state.runtime->beginFrame();
  if (!begun) {
    state.fail("Web frame start failed: " + begun.error().message);
    return;
  }
  auto drawn =
      state.app->draw(begun.value().get(), state.runtime->framebufferMetrics());
  auto presented = state.runtime->renderAndPresent();
  if (!drawn) {
    state.fail("Web workbench draw failed: " + drawn.error().cause.message);
    return;
  }
  if (!presented) {
    state.fail("Web frame presentation failed: " + presented.error().message);
    return;
  }
}

} // namespace

extern "C" EMSCRIPTEN_KEEPALIVE int kpt_web_has_glyph(unsigned codepoint) {
  if (ImGui::GetCurrentContext() == nullptr)
    return 0;
  return ImGui::GetFont()
                 ->GetFontBaked(16.0F)
                 ->FindGlyphNoFallback(static_cast<ImWchar>(codepoint)) !=
             nullptr
             ? 1
             : 0;
}

int main() {
  auto services = kpt::platform::createServices();
  if (!services) {
    const std::string message =
        "Browser platform initialization failed: " + services.error().message;
    showFatalError(message.c_str());
    return 1;
  }

  auto state = std::make_unique<WebApplication>();
  state->services = std::move(services).value();
  state->runtime = kpt::gui::createGuiRuntime();
  kpt::gui::GuiRuntimeOptions options;
  options.width = 1440;
  options.height = 900;
  options.title = "KPT Web Workbench";
  options.fonts = state->services.fonts.get();
  options.settings = state->services.settings.get();
  auto initialized = state->runtime->initialize(options);
  if (!initialized) {
    const std::string message =
        "WebGL initialization failed: " + initialized.error().message;
    showFatalError(message.c_str());
    return 1;
  }
  auto main_renderer = state->runtime->createViewportRenderer();
  auto trajectory_renderer = state->runtime->createViewportRenderer();
  if (!main_renderer || !trajectory_renderer) {
    const std::string message =
        "WebGL renderer creation failed: " +
        (!main_renderer ? main_renderer.error().message
                        : trajectory_renderer.error().message);
    showFatalError(message.c_str());
    return 1;
  }
  state->app = std::make_unique<kpt::gui::App>(
      std::move(main_renderer).value(), std::move(trajectory_renderer).value(),
      4, kpt::gui::web::createAssetStager());
  WebApplication *lifetime = state.release();
  emscripten_set_main_loop_arg(frame, lifetime, 0, true);
  __builtin_unreachable();
}
