#include "gui/app.hpp"

#include "backends/imgui_impl_glfw.h"
#include "backends/imgui_impl_opengl3.h"
#include "imgui.h"

#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>

#define GL_GLEXT_PROTOTYPES
#include <GL/glcorearb.h>

#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <string>

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

} // namespace

int main(int argc, char **argv) {
  const bool smoke_test = argc > 1 && std::string(argv[1]) == "--smoke-test";

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
