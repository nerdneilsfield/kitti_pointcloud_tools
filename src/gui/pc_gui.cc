#include "gui/app.hpp"
#include "gui/runtime/factory.hpp"
#include "platform/services.hpp"

#include <clocale>
#include <iostream>
#include <memory>
#include <string>
#include <string_view>
#include <utility>

namespace {

void logPlatformError(std::string_view operation,
                      const kpt::platform::PlatformError &error) {
  std::cerr << operation << ": " << error.message;
  if (error.system_error)
    std::cerr << " (" << error.system_error.message() << ')';
  std::cerr << '\n';
}

void logGuiError(std::string_view operation, const kpt::gui::GuiError &error) {
  std::cerr << operation << ": " << error.message;
  if (error.system_error)
    std::cerr << " (" << error.system_error.message() << ')';
  std::cerr << '\n';
}

void logAppError(const kpt::gui::AppError &error) {
  const char *role =
      error.role == kpt::gui::ViewportRole::Main ? "main" : "trajectory";
  const char *stage = "render";
  if (error.stage == kpt::gui::AppStage::Upload)
    stage = "upload";
  else if (error.stage == kpt::gui::AppStage::Resize)
    stage = "resize";
  std::cerr << role << " viewport " << stage << ": " << error.cause.message
            << '\n';
}

} // namespace

int main(int argc, char **argv) {
  const bool smoke_test = argc > 1 && std::string(argv[1]) == "--smoke-test";
  std::setlocale(LC_ALL, "");

  auto created_services = kpt::platform::createServices();
  if (!created_services) {
    logPlatformError("Platform initialization failed",
                     created_services.error());
    return 1;
  }
  auto services = std::move(created_services).value();
  auto runtime = kpt::gui::createGuiRuntime();
  kpt::gui::GuiRuntimeOptions options;
  options.visible = !smoke_test;
  options.persist_settings = !smoke_test;
  options.fonts = services.fonts.get();
  options.settings = services.settings.get();
  auto initialized = runtime->initialize(options);
  if (!initialized) {
    logGuiError("GUI initialization failed", initialized.error());
    runtime->shutdown();
    return 1;
  }

  int exit_code = 0;
  {
    auto main_renderer = runtime->createViewportRenderer();
    auto trajectory_renderer = runtime->createViewportRenderer();
    if (!main_renderer || !trajectory_renderer) {
      logGuiError("Renderer creation failed",
                  !main_renderer ? main_renderer.error()
                                 : trajectory_renderer.error());
      exit_code = 1;
    } else {
      kpt::gui::App app(std::move(main_renderer).value(),
                        std::move(trajectory_renderer).value());
      if (smoke_test)
        app.installSyntheticSmokeSnapshot();

      do {
        runtime->pollEvents();
        auto begun = runtime->beginFrame();
        if (!begun) {
          logGuiError("Frame start failed", begun.error());
          exit_code = 2;
          break;
        }

        auto drawn =
            app.draw(begun.value().get(), runtime->framebufferMetrics());
        // Presentation closes the frame on every path, including App errors.
        auto presented = runtime->renderAndPresent();
        if (!drawn) {
          logAppError(drawn.error());
          exit_code = 2;
          break;
        }
        if (!presented) {
          logGuiError("Frame presentation failed", presented.error());
          exit_code = 2;
          break;
        }
        if (smoke_test)
          break;
      } while (!runtime->shouldClose());
    }
  }

  runtime->shutdown();
  return exit_code;
}
