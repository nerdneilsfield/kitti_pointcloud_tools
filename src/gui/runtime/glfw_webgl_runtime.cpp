#include "gui/runtime/factory.hpp"

#include "gui/backend/opengl/point_renderer.hpp"
#include "platform/utf8_path.hpp"

#include "backends/imgui_impl_glfw.h"
#include "backends/imgui_impl_opengl3.h"
#include "imgui.h"

#include <GLES3/gl3.h>
#include <GLFW/glfw3.h>
#include <emscripten/html5.h>

#include <algorithm>
#include <exception>
#include <iostream>
#include <memory>
#include <string>
#include <string_view>
#include <utility>

namespace kpt::gui {
namespace {

GuiError error(GuiErrorCode code, std::string message) {
  return {code, std::move(message), {}};
}

void glfwError(int code, const char *description) {
  std::cerr << "GLFW " << code << ": "
            << (description == nullptr ? "unknown error" : description) << '\n';
}

void logPlatformError(std::string_view operation,
                      const platform::PlatformError &platform_error) {
  std::cerr << operation << ": " << platform_error.message << '\n';
}

} // namespace

class GlfwWebGLRuntime final : public GuiRuntime {
public:
  ~GlfwWebGLRuntime() override { shutdown(); }

  Result<void, GuiError> initialize(const GuiRuntimeOptions &options) override {
    if (state_ != State::Created)
      return error(GuiErrorCode::InvalidState,
                   "WebGL runtime can only be initialized once");
    if (options.width <= 0 || options.height <= 0 || options.title.empty())
      return error(GuiErrorCode::InvalidOptions,
                   "GUI width, height, and title must be non-empty");

    options_ = options;
    glfwSetErrorCallback(glfwError);
    if (glfwInit() == GLFW_FALSE)
      return error(GuiErrorCode::WindowSystemUnavailable,
                   "Emscripten GLFW initialization failed");
    glfw_initialized_ = true;

    glfwWindowHint(GLFW_CLIENT_API, GLFW_OPENGL_ES_API);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
    window_ = glfwCreateWindow(options.width, options.height,
                               options.title.c_str(), nullptr, nullptr);
    if (window_ == nullptr)
      return error(GuiErrorCode::GraphicsDeviceUnavailable,
                   "browser could not create a WebGL2 canvas");

    glfwMakeContextCurrent(window_);
    frame_context_.bindWindow(window_);

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    imgui_context_created_ = true;
    ImGuiIO &io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
    io.IniFilename = nullptr;
    configureFonts(io);
    ImGui::StyleColorsDark();

    settings_enabled_ = options.persist_settings && options.settings != nullptr;
    if (settings_enabled_)
      settings_enabled_ = loadSettings();

    if (!ImGui_ImplGlfw_InitForOpenGL(window_, true))
      return error(GuiErrorCode::GraphicsDeviceUnavailable,
                   "Dear ImGui GLFW Web backend initialization failed");
    imgui_glfw_initialized_ = true;
    ImGui_ImplGlfw_InstallEmscriptenCallbacks(window_, "#canvas");
    if (!ImGui_ImplOpenGL3_Init("#version 300 es"))
      return error(GuiErrorCode::GraphicsDeviceUnavailable,
                   "Dear ImGui WebGL2 backend initialization failed");
    imgui_opengl_initialized_ = true;

    glfwSetWindowUserPointer(window_, this);
    glfwSetWindowSizeCallback(window_, metricsCallback);
    glfwSetFramebufferSizeCallback(window_, metricsCallback);
    refreshMetrics();
    state_ = State::Initialized;
    return {};
  }

  bool shouldClose() const noexcept override {
    return state_ == State::Shutdown || window_ == nullptr ||
           glfwWindowShouldClose(window_) != GLFW_FALSE;
  }

  void pollEvents() noexcept override {
    if (state_ == State::Initialized)
      glfwPollEvents();
  }

  Result<std::reference_wrapper<FrameContext>, GuiError>
  beginFrame() override {
    if (state_ != State::Initialized)
      return error(GuiErrorCode::InvalidState,
                   "beginFrame requires an initialized WebGL runtime");
    if (glfwGetCurrentContext() != window_)
      glfwMakeContextCurrent(window_);
    refreshMetrics();
    ImGui::GetIO().DisplayFramebufferScale = metrics_.scale;
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();
    frame_context_.activate();
    state_ = State::FrameActive;
    return std::ref(static_cast<FrameContext &>(frame_context_));
  }

  Result<void, GuiError> renderAndPresent() override {
    if (state_ != State::FrameActive)
      return error(GuiErrorCode::InvalidState,
                   "renderAndPresent requires an active WebGL frame");
    const auto close_frame = [this] {
      frame_context_.invalidate();
      state_ = State::Initialized;
    };
    try {
      ImGui::Render();
      if (settings_enabled_ && ImGui::GetIO().WantSaveIniSettings)
        settings_enabled_ = flushSettings();
      refreshMetrics();
      glBindFramebuffer(GL_FRAMEBUFFER, 0);
      glViewport(0, 0, metrics_.framebuffer_size.width,
                 metrics_.framebuffer_size.height);
      glClearColor(0.08F, 0.08F, 0.09F, 1.0F);
      glClear(GL_COLOR_BUFFER_BIT);
      ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
      const unsigned gl_error = glGetError();
      if (gl_error != GL_NO_ERROR) {
        close_frame();
        return error(GuiErrorCode::PresentationFailed,
                     "WebGL presentation failed with error " +
                         std::to_string(gl_error));
      }
    } catch (const std::exception &exception) {
      close_frame();
      return error(GuiErrorCode::PresentationFailed,
                   std::string("WebGL presentation failed: ") +
                       exception.what());
    }
    close_frame();
    return {};
  }

  void shutdown() noexcept override {
    if (state_ == State::Shutdown)
      return;
    frame_context_.invalidate();
    if (state_ == State::FrameActive && imgui_context_created_)
      ImGui::EndFrame();
    state_ = State::Shutdown;
    if (settings_enabled_ && imgui_context_created_)
      static_cast<void>(flushSettings());
    if (imgui_opengl_initialized_)
      ImGui_ImplOpenGL3_Shutdown();
    if (imgui_glfw_initialized_)
      ImGui_ImplGlfw_Shutdown();
    if (imgui_context_created_)
      ImGui::DestroyContext();
    imgui_opengl_initialized_ = false;
    imgui_glfw_initialized_ = false;
    imgui_context_created_ = false;
    if (window_ != nullptr) {
      glfwDestroyWindow(window_);
      window_ = nullptr;
      frame_context_.bindWindow(nullptr);
    }
    if (glfw_initialized_)
      glfwTerminate();
    glfw_initialized_ = false;
  }

  FramebufferMetrics framebufferMetrics() const override { return metrics_; }

  Result<std::unique_ptr<ViewportRenderer>, GuiError>
  createViewportRenderer() override {
    if (state_ != State::Initialized)
      return error(GuiErrorCode::InvalidState,
                   "renderer creation requires an initialized WebGL runtime");
    try {
      return std::unique_ptr<ViewportRenderer>(
          std::make_unique<OpenGLPointRenderer>(window_));
    } catch (const std::exception &exception) {
      return error(GuiErrorCode::RendererCreationFailed,
                   std::string("WebGL renderer creation failed: ") +
                       exception.what());
    }
  }

private:
  enum class State { Created, Initialized, FrameActive, Shutdown };

  static void metricsCallback(GLFWwindow *window, int, int) {
    auto *runtime =
        static_cast<GlfwWebGLRuntime *>(glfwGetWindowUserPointer(window));
    if (runtime != nullptr)
      runtime->refreshMetrics();
  }

  void refreshMetrics() noexcept {
    if (window_ == nullptr)
      return;
    int logical_width = 0;
    int logical_height = 0;
    int framebuffer_width = 0;
    int framebuffer_height = 0;
    glfwGetWindowSize(window_, &logical_width, &logical_height);
    glfwGetFramebufferSize(window_, &framebuffer_width, &framebuffer_height);
    metrics_.logical_size = {static_cast<float>(logical_width),
                             static_cast<float>(logical_height)};
    metrics_.framebuffer_size = {std::max(0, framebuffer_width),
                                 std::max(0, framebuffer_height)};
    metrics_.scale = {
        logical_width > 0
            ? static_cast<float>(framebuffer_width) / logical_width
            : 1.0F,
        logical_height > 0
            ? static_cast<float>(framebuffer_height) / logical_height
            : 1.0F};
  }

  void configureFonts(ImGuiIO &io) {
    ImFontConfig default_config;
    default_config.SizePixels = 16.0F;
    ImFont *default_font = io.Fonts->AddFontDefault(&default_config);
    if (options_.fonts == nullptr)
      return;
    const auto font =
        options_.fonts->matchUiFont(U"中文路径文件选择点云轨迹标签");
    if (!font || !font.value()) {
      if (!font)
        logPlatformError("CJK font lookup disabled", font.error());
      return;
    }
    const auto path = platform::pathToUtf8(font.value()->file);
    if (!path)
      return;
    ImFontConfig config;
    config.MergeMode = true;
    config.PixelSnapH = true;
    config.OversampleH = 1;
    config.OversampleV = 1;
    config.FontNo = static_cast<ImU32>(font.value()->face_index);
    config.DstFont = default_font;
    if (io.Fonts->AddFontFromFileTTF(path.value().c_str(), 16.0F, &config,
                                     io.Fonts
                                         ->GetGlyphRangesChineseSimplifiedCommon()) ==
        nullptr) {
      std::cerr << "CJK font load failed: " << path.value() << '\n';
    }
  }

  bool loadSettings() {
    const auto loaded = options_.settings->loadIni();
    if (!loaded)
      return false;
    if (loaded.value())
      ImGui::LoadIniSettingsFromMemory(loaded.value()->data(),
                                       loaded.value()->size());
    return true;
  }

  bool flushSettings() {
    std::size_t size = 0;
    const char *contents = ImGui::SaveIniSettingsToMemory(&size);
    const auto saved =
        options_.settings->saveIniAtomically(std::string_view(contents, size));
    ImGui::GetIO().WantSaveIniSettings = false;
    return saved.hasValue();
  }

  State state_ = State::Created;
  bool glfw_initialized_ = false;
  bool imgui_context_created_ = false;
  bool imgui_glfw_initialized_ = false;
  bool imgui_opengl_initialized_ = false;
  bool settings_enabled_ = false;
  GLFWwindow *window_ = nullptr;
  OpenGLFrameContext frame_context_{nullptr, false};
  GuiRuntimeOptions options_;
  FramebufferMetrics metrics_;
};

std::unique_ptr<GuiRuntime> createGuiRuntime() {
  return std::make_unique<GlfwWebGLRuntime>();
}

} // namespace kpt::gui
