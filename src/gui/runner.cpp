#include "gui/runner.hpp"

#include "gui/app.hpp"
#include "gui/runtime/factory.hpp"
#include "platform/services.hpp"

#include <clocale>
#include <iostream>
#include <memory>
#include <spdlog/spdlog.h>
#include <string_view>
#include <utility>

namespace kpt::gui {
namespace {

void logPlatformError(std::string_view operation,
                      const platform::PlatformError &error) {
  std::cerr << operation << ": " << error.message;
  if (error.system_error)
    std::cerr << " (" << error.system_error.message() << ')';
  std::cerr << '\n';
}

void logGuiError(std::string_view operation, const GuiError &error) {
  std::cerr << operation << ": " << error.message;
  if (error.system_error)
    std::cerr << " (" << error.system_error.message() << ')';
  std::cerr << '\n';
}

void logAppError(const AppError &error) {
  const char *role = error.role == ViewportRole::Main ? "main" : "trajectory";
  const char *stage = "render";
  if (error.stage == AppStage::Upload)
    stage = "upload";
  else if (error.stage == AppStage::Resize)
    stage = "resize";
  std::cerr << role << " viewport " << stage << ": " << error.cause.message
            << '\n';
}

} // namespace

static int runWorkbenchImpl(WorkbenchLaunchRequest request) {
  const bool has_sequence =
      request.sequence.has_value() || request.sequence_source != nullptr;
  if (request.sequence && request.sequence_source) {
    std::cerr << "GUI launch accepts one sequence representation\n";
    return 1;
  }
  if (request.viewer_file && has_sequence) {
    std::cerr << "GUI launch accepts either a file or a sequence, not both\n";
    return 1;
  }
  if (request.smoke_test && (request.viewer_file || has_sequence)) {
    std::cerr << "Smoke test cannot launch a file or sequence\n";
    return 1;
  }
  if (request.sequence_fps && *request.sequence_fps <= 0) {
    std::cerr << "Sequence FPS must be positive\n";
    return 1;
  }
  if ((request.sequence_fps || request.sequence_autoplay) && !has_sequence) {
    std::cerr << "Sequence playback options require a sequence\n";
    return 1;
  }
  if (request.width <= 0 || request.height <= 0 || request.title.empty()) {
    std::cerr << "GUI launch requires positive dimensions and a title\n";
    return 1;
  }

  std::setlocale(LC_ALL, "");
  auto created_services = platform::createServices();
  if (!created_services) {
    logPlatformError("Platform initialization failed",
                     created_services.error());
    return 1;
  }
  auto services = std::move(created_services).value();
  auto runtime = createGuiRuntime();
  GuiRuntimeOptions runtime_options;
  runtime_options.width = request.width;
  runtime_options.height = request.height;
  runtime_options.title = request.title;
  runtime_options.visible = !request.smoke_test;
  runtime_options.persist_settings = !request.smoke_test;
  runtime_options.fonts = services.fonts.get();
  runtime_options.settings = services.settings.get();
  auto initialized = runtime->initialize(runtime_options);
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
      App app(std::move(main_renderer).value(),
              std::move(trajectory_renderer).value());
      if (request.style)
        app.setStartupStyle(*request.style);
      if (request.viewer_file)
        app.startViewer(*request.viewer_file);
      else if (request.sequence)
        app.startSequence(std::move(*request.sequence),
                          request.sequence_fps.value_or(10),
                          request.sequence_autoplay);
      else if (request.sequence_source)
        app.startSequence(std::move(request.sequence_source),
                          request.sequence_fps.value_or(10),
                          request.sequence_autoplay);
      if (request.smoke_test)
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
        for (auto &warning : app.takeLaunchWarnings())
          spdlog::warn("{}", warning);
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
        if (app.launchError()) {
          std::cerr << *app.launchError() << '\n';
          exit_code = 1;
          break;
        }
        if (app.launchCompletedEmpty())
          break;
        if (request.smoke_test)
          break;
      } while (!runtime->shouldClose());
    }
  }

  runtime->shutdown();
  return exit_code;
}

int runWorkbench(WorkbenchLaunchRequest request) {
  try {
    return runWorkbenchImpl(std::move(request));
  } catch (const std::exception &error) {
    std::cerr << "Workbench failed: " << error.what() << '\n';
    return 1;
  } catch (...) {
    std::cerr << "Workbench failed: unknown error\n";
    return 1;
  }
}

} // namespace kpt::gui
