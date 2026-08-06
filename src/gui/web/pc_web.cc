#include "gui/app.hpp"
#include "gui/runtime/factory.hpp"
#include "gui/web/bridge.hpp"
#include "i18n/i18n.hpp"
#include "platform/services.hpp"

#include <emscripten.h>
#include <imgui.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <memory>
#include <string>
#include <utility>

namespace {

EM_JS(void, showFatalError, (const char *message),
      { globalThis.KptWeb.fatal(UTF8ToString(message)); });

// clang-format off
EM_JS(void, installWakeHandlers, (), {
  if (globalThis.__kptWakeHandlersInstalled)
    return;
  globalThis.__kptWakeHandlersInstalled = true;
  const wake = () => Module._kpt_web_wake_main_loop();
  for (const type of["pointerdown", "pointermove", "pointerup", "pointercancel",
                     "wheel", "keydown", "keyup", "input", "change",
  ]) {
    globalThis.addEventListener(type, wake, {capture : true, passive : true});
  }
  globalThis.addEventListener("resize", wake, {passive : true});
  document.addEventListener("visibilitychange", wake, {passive : true});
});
// clang-format on

constexpr int kQuietFramesBeforeThrottle = 12;
constexpr int kIdleFrameIntervalMilliseconds = 250;
constexpr int kActiveFrameRateLimit = 60;
constexpr double kActiveFrameIntervalMilliseconds =
    1000.0 / static_cast<double>(kActiveFrameRateLimit);

struct WebApplication {
  kpt::platform::Services services;
  std::unique_ptr<kpt::gui::GuiRuntime> runtime;
  std::unique_ptr<kpt::gui::App> app;
  bool failed = false;
  bool throttled = false;
  int quiet_frames = 0;
  double last_active_tick_milliseconds = 0.0;
  double active_frame_budget_milliseconds = 0.0;

  void fail(std::string message) {
    if (failed)
      return;
    failed = true;
    std::cerr << message << '\n';
    showFatalError(message.c_str());
    emscripten_cancel_main_loop();
  }
};

WebApplication *active_application = nullptr;

void setThrottled(WebApplication &state, bool throttled) {
  if (state.throttled == throttled)
    return;
  const int mode = throttled ? EM_TIMING_SETTIMEOUT : EM_TIMING_RAF;
  const int value = throttled ? kIdleFrameIntervalMilliseconds : 1;
  if (emscripten_set_main_loop_timing(mode, value) == 0) {
    state.throttled = throttled;
    if (!throttled) {
      state.last_active_tick_milliseconds = 0.0;
      state.active_frame_budget_milliseconds = 0.0;
    }
  }
}

bool activeFrameDue(WebApplication &state) {
  if (state.throttled)
    return true;
  const double now = emscripten_get_now();
  if (state.last_active_tick_milliseconds == 0.0) {
    state.last_active_tick_milliseconds = now;
    return true;
  }
  const double elapsed = std::clamp(now - state.last_active_tick_milliseconds,
                                    0.0, kIdleFrameIntervalMilliseconds * 1.0);
  state.last_active_tick_milliseconds = now;
  state.active_frame_budget_milliseconds += elapsed;
  constexpr double scheduling_tolerance_milliseconds = 0.5;
  if (state.active_frame_budget_milliseconds +
          scheduling_tolerance_milliseconds <
      kActiveFrameIntervalMilliseconds) {
    return false;
  }
  state.active_frame_budget_milliseconds =
      std::max(0.0, state.active_frame_budget_milliseconds -
                        kActiveFrameIntervalMilliseconds);
  if (state.active_frame_budget_milliseconds >=
      kActiveFrameIntervalMilliseconds) {
    state.active_frame_budget_milliseconds =
        std::fmod(state.active_frame_budget_milliseconds,
                  kActiveFrameIntervalMilliseconds);
  }
  return true;
}

bool hasInteractiveInput() {
  const ImGuiIO &io = ImGui::GetIO();
  if (ImGui::IsAnyItemActive() || io.MouseDelta.x != 0.0F ||
      io.MouseDelta.y != 0.0F || io.MouseWheel != 0.0F ||
      io.MouseWheelH != 0.0F) {
    return true;
  }
  for (const bool down : io.MouseDown) {
    if (down)
      return true;
  }
  return false;
}

void frame(void *opaque) {
  auto &state = *static_cast<WebApplication *>(opaque);
  if (state.failed)
    return;
  if (!activeFrameDue(state))
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
  if (state.app->needsContinuousRedraw() || hasInteractiveInput()) {
    state.quiet_frames = 0;
    setThrottled(state, false);
  } else if (!state.throttled &&
             ++state.quiet_frames >= kQuietFramesBeforeThrottle) {
    setThrottled(state, true);
  }
}

} // namespace

extern "C" EMSCRIPTEN_KEEPALIVE void kpt_web_wake_main_loop() {
  if (active_application == nullptr)
    return;
  active_application->quiet_frames = 0;
  setThrottled(*active_application, false);
}

extern "C" EMSCRIPTEN_KEEPALIVE int kpt_web_main_loop_throttled() {
  return active_application != nullptr && active_application->throttled ? 1 : 0;
}

extern "C" EMSCRIPTEN_KEEPALIVE int kpt_web_active_frame_rate_limit() {
  return kActiveFrameRateLimit;
}

extern "C" EMSCRIPTEN_KEEPALIVE int kpt_web_has_glyph(unsigned codepoint) {
  if (ImGui::GetCurrentContext() == nullptr)
    return 0;
  return ImGui::GetFont()->GetFontBaked(16.0F)->FindGlyphNoFallback(
             static_cast<ImWchar>(codepoint)) != nullptr
             ? 1
             : 0;
}

extern "C" EMSCRIPTEN_KEEPALIVE void kpt_web_set_language(const char *lang) {
  kpt::i18n::setLanguage(lang);
}

int main() {
  kpt::i18n::initialize();
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
  active_application = lifetime;
  installWakeHandlers();
  emscripten_set_main_loop_arg(frame, lifetime, 0, true);
  __builtin_unreachable();
}
