#include "gui/app.hpp"

#include "backends/imgui_impl_glfw.h"
#include "backends/imgui_impl_opengl3.h"
#include "imgui.h"

#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>

#define GL_GLEXT_PROTOTYPES
#include <GL/glcorearb.h>

#include <clocale>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <optional>
#include <string>
#include <vector>

namespace {

void glfwError(int error, const char *description) {
  std::cerr << "GLFW " << error << ": " << description << '\n';
}

std::string iniPath() {
  std::filesystem::path base;
  if (const char *xdg = std::getenv("XDG_CONFIG_HOME"); xdg != nullptr) {
    base = xdg;
  } else if (const char *home = std::getenv("HOME"); home != nullptr) {
    base = std::filesystem::path(home) / ".config";
  } else {
    return {};
  }
  base /= "kpt";
  std::error_code ignored;
  std::filesystem::create_directories(base, ignored);
  return (base / "imgui.ini").string();
}

std::optional<std::filesystem::path> cjkFontPath() {
  std::vector<std::filesystem::path> candidates;
  if (const char *configured = std::getenv("KPT_CJK_FONT");
      configured != nullptr && *configured != '\0') {
    candidates.emplace_back(configured);
  }
  candidates.insert(
      candidates.end(),
      {
          "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
          "/usr/share/fonts/truetype/noto/NotoSansCJK-Regular.ttc",
          "/usr/share/fonts/truetype/droid/DroidSansFallbackFull.ttf",
          "/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc",
      });
  std::error_code ignored;
  for (const auto &candidate : candidates) {
    if (std::filesystem::is_regular_file(candidate, ignored))
      return candidate;
    ignored.clear();
  }
  return std::nullopt;
}

void configureFonts(ImGuiIO &io) {
  io.Fonts->AddFontDefault();
  const auto cjk_font = cjkFontPath();
  if (!cjk_font)
    return;

  ImFontConfig config;
  config.MergeMode = true;
  config.PixelSnapH = true;
  io.Fonts->AddFontFromFileTTF(cjk_font->string().c_str(), 16.0F, &config,
                               io.Fonts->GetGlyphRangesChineseFull());
}

} // namespace

int main(int argc, char **argv) {
  const bool smoke_test = argc > 1 && std::string(argv[1]) == "--smoke-test";
  std::setlocale(LC_ALL, "");

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
  configureFonts(io);
  const std::string ini_path = smoke_test ? std::string{} : iniPath();
  io.IniFilename = ini_path.empty() ? nullptr : ini_path.c_str();
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
    kpt::gui::App app;
    if (smoke_test) {
      ImGui_ImplOpenGL3_NewFrame();
      ImGui_ImplGlfw_NewFrame();
      ImGui::NewFrame();
      const bool passed = app.runSmokeTest();
      ImGui::Render();
      exit_code = passed ? 0 : 2;
    } else {
      while (glfwWindowShouldClose(window) == GLFW_FALSE) {
        glfwPollEvents();
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();
        app.draw();
        ImGui::Render();

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

  ImGui_ImplOpenGL3_Shutdown();
  ImGui_ImplGlfw_Shutdown();
  ImGui::DestroyContext();
  glfwDestroyWindow(window);
  glfwTerminate();
  return exit_code;
}
