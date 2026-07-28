#include "gui/app.hpp"
#include "gui/backend/opengl/point_renderer.hpp"
#include "platform/services.hpp"
#include "platform/utf8_path.hpp"

#include "backends/imgui_impl_glfw.h"
#include "backends/imgui_impl_opengl3.h"
#include "imgui.h"

#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>

#define GL_GLEXT_PROTOTYPES
#include <GL/glcorearb.h>

#include <clocale>
#include <iostream>
#include <memory>
#include <string>
#include <utility>

namespace {

void glfwError(int error, const char *description) {
  std::cerr << "GLFW " << error << ": " << description << '\n';
}

void logPlatformError(std::string_view operation,
                      const kpt::platform::PlatformError &error) {
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

void configureFonts(ImGuiIO &io, kpt::platform::Fonts &fonts) {
  ImFontConfig default_config;
  default_config.SizePixels = 16.0F;
  ImFont *default_font = io.Fonts->AddFontDefault(&default_config);

  const auto cjk_font = fonts.matchUiFont(U"中文路径文件选择点云轨迹标签");
  if (!cjk_font) {
    logPlatformError("CJK font lookup disabled", cjk_font.error());
    return;
  }
  if (!cjk_font.value())
    return;

  auto utf8_path = kpt::platform::pathToUtf8(cjk_font.value()->file);
  if (!utf8_path) {
    logPlatformError("CJK font path conversion failed", utf8_path.error());
    return;
  }
  ImFontConfig config;
  config.MergeMode = true;
  config.PixelSnapH = true;
  config.FontNo = static_cast<ImU32>(cjk_font.value()->face_index);
  config.DstFont = default_font;
  if (io.Fonts->AddFontFromFileTTF(utf8_path.value().c_str(), 16.0F, &config,
                                   io.Fonts->GetGlyphRangesChineseFull()) ==
      nullptr) {
    std::cerr << "CJK font load failed\n";
  }
}

bool loadSettings(kpt::platform::SettingsStore &settings) {
  auto loaded = settings.loadIni();
  if (!loaded) {
    logPlatformError("ImGui settings disabled", loaded.error());
    return false;
  }
  if (loaded.value()) {
    ImGui::LoadIniSettingsFromMemory(loaded.value()->data(),
                                     loaded.value()->size());
  }
  return true;
}

bool flushSettings(kpt::platform::SettingsStore &settings, ImGuiIO &io) {
  std::size_t size = 0;
  const char *contents = ImGui::SaveIniSettingsToMemory(&size);
  auto saved = settings.saveIniAtomically(std::string_view(contents, size));
  io.WantSaveIniSettings = false;
  if (!saved) {
    logPlatformError("ImGui settings save disabled", saved.error());
    return false;
  }
  return true;
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

  glfwSetErrorCallback(glfwError);
  if (glfwInit() == GLFW_FALSE)
    return 1;
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
  if (smoke_test)
    glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);

  GLFWwindow *window =
      glfwCreateWindow(1440, 900, "KPT Workbench", nullptr, nullptr);
  if (window == nullptr) {
    glfwTerminate();
    return 1;
  }
  glfwMakeContextCurrent(window);
  glfwSwapInterval(1);

  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImGuiIO &io = ImGui::GetIO();
  io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
  io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
  configureFonts(io, *services.fonts);
  io.IniFilename = nullptr;
  bool settings_enabled = !smoke_test && loadSettings(*services.settings);
  ImGui::StyleColorsDark();

  if (!ImGui_ImplGlfw_InitForOpenGL(window, true) ||
      !ImGui_ImplOpenGL3_Init("#version 330 core")) {
    ImGui::DestroyContext();
    glfwDestroyWindow(window);
    glfwTerminate();
    return 1;
  }

  int exit_code = 0;
  {
    kpt::gui::App app(
        std::make_unique<kpt::gui::OpenGLPointRenderer>(window),
        std::make_unique<kpt::gui::OpenGLPointRenderer>(window));
    if (smoke_test) {
      ImGui_ImplOpenGL3_NewFrame();
      ImGui_ImplGlfw_NewFrame();
      ImGui::NewFrame();
      app.installSyntheticSmokeSnapshot();
      kpt::gui::OpenGLFrameContext frame_context(window);
      auto drawn = app.draw(frame_context);
      frame_context.invalidate();
      ImGui::Render();
      if (!drawn) {
        logAppError(drawn.error());
        exit_code = 2;
      }
    } else {
      while (glfwWindowShouldClose(window) == GLFW_FALSE) {
        glfwPollEvents();
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();
        kpt::gui::OpenGLFrameContext frame_context(window);
        auto drawn = app.draw(frame_context);
        frame_context.invalidate();
        if (!drawn) {
          logAppError(drawn.error());
          exit_code = 2;
          break;
        }
        ImGui::Render();
        if (settings_enabled && io.WantSaveIniSettings)
          settings_enabled = flushSettings(*services.settings, io);

        int display_width = 0;
        int display_height = 0;
        glfwGetFramebufferSize(window, &display_width, &display_height);
        glViewport(0, 0, display_width, display_height);
        glClearColor(0.08F, 0.08F, 0.09F, 1.0F);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        glfwSwapBuffers(window);
      }
    }
  }

  if (settings_enabled)
    flushSettings(*services.settings, io);
  ImGui_ImplOpenGL3_Shutdown();
  ImGui_ImplGlfw_Shutdown();
  ImGui::DestroyContext();
  glfwDestroyWindow(window);
  glfwTerminate();
  return exit_code;
}
