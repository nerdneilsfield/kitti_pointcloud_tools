#include "gui/runtime/factory.hpp"

#include "gui/backend/opengl/point_renderer.hpp"
#ifdef KPT_GUI_RUNTIME_TEST_SUPPORT
#include "gui/runtime/test_support.hpp"
#endif
#include "platform/utf8_path.hpp"

#include <glad/gl.h>

#include "backends/imgui_impl_glfw.h"
#include "backends/imgui_impl_opengl3.h"
#include "imgui.h"

#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>

#include <algorithm>
#include <exception>
#include <iostream>
#include <memory>
#include <spdlog/spdlog.h>
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
  std::cerr << operation << ": " << platform_error.message;
  if (platform_error.system_error)
    std::cerr << " (" << platform_error.system_error.message() << ')';
  std::cerr << '\n';
}

} // namespace

class GlfwOpenGLRuntime final : public GuiRuntime {
public:
  GlfwOpenGLRuntime() = default;
#ifdef KPT_GUI_RUNTIME_TEST_SUPPORT
  explicit GlfwOpenGLRuntime(detail::RuntimeTestHooks hooks) : hooks_(hooks) {}
#endif
  ~GlfwOpenGLRuntime() override { shutdown(); }

  Result<void, GuiError> initialize(const GuiRuntimeOptions &options) override {
    if (state_ != State::Created || initialization_attempted_)
      return error(GuiErrorCode::InvalidState,
                   "GUI runtime can only be initialized once");
    initialization_attempted_ = true;
    if (options.width <= 0 || options.height <= 0 || options.title.empty())
      return error(GuiErrorCode::InvalidOptions,
                   "GUI width, height, and title must be non-empty");

    options_ = options;
    glfwSetErrorCallback(glfwError);
    if (glfwInit() == GLFW_FALSE)
      return error(GuiErrorCode::WindowSystemUnavailable,
                   "GLFW initialization failed");
    glfw_initialized_ = true;
#ifdef KPT_GUI_RUNTIME_TEST_SUPPORT
    if (fault(detail::RuntimeFaultPoint::AfterWindowSystem))
      return error(GuiErrorCode::WindowSystemUnavailable,
                   "injected failure after GLFW initialization");
#endif

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_VISIBLE, options.visible ? GLFW_TRUE : GLFW_FALSE);
    window_ = glfwCreateWindow(options.width, options.height,
                               options.title.c_str(), nullptr, nullptr);
    if (window_ == nullptr)
      return error(GuiErrorCode::GraphicsDeviceUnavailable,
                   "GLFW could not create an OpenGL 3.3 window");
    frame_context_.bindWindow(window_);
#ifdef KPT_GUI_RUNTIME_TEST_SUPPORT
    if (fault(detail::RuntimeFaultPoint::AfterWindow))
      return error(GuiErrorCode::GraphicsDeviceUnavailable,
                   "injected failure after window creation");
#endif

    glfwMakeContextCurrent(window_);
    glfwSwapInterval(1);
    const int version =
        gladLoadGL(reinterpret_cast<GLADloadfunc>(glfwGetProcAddress));
    if (version == 0 || GLAD_VERSION_MAJOR(version) < 3 ||
        (GLAD_VERSION_MAJOR(version) == 3 && GLAD_VERSION_MINOR(version) < 3)) {
      return error(GuiErrorCode::GraphicsDeviceUnavailable,
                   "GLAD could not load an OpenGL 3.3 core context");
    }
    const auto gl_text = [](unsigned name) {
      const auto *value = glGetString(name);
      return value == nullptr
                 ? std::string("<unavailable>")
                 : std::string(reinterpret_cast<const char *>(value));
    };
    spdlog::info("OpenGL context: version='{}', renderer='{}', vendor='{}'",
                 gl_text(GL_VERSION), gl_text(GL_RENDERER), gl_text(GL_VENDOR));

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    imgui_context_created_ = true;
#ifdef KPT_GUI_RUNTIME_TEST_SUPPORT
    if (fault(detail::RuntimeFaultPoint::AfterImGuiContext))
      return error(GuiErrorCode::GraphicsDeviceUnavailable,
                   "injected failure after Dear ImGui context creation");
#endif

    ImGuiIO &io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
    io.IniFilename = nullptr;
    configureFonts(io);
    ImGui::StyleColorsDark();
    settings_enabled_ = options.persist_settings && options.settings != nullptr;
    if (settings_enabled_)
      settings_enabled_ = loadSettings();

    if (!ImGui_ImplGlfw_InitForOpenGL(window_, true)) {
      return error(GuiErrorCode::GraphicsDeviceUnavailable,
                   "Dear ImGui GLFW backend initialization failed");
    }
    imgui_glfw_initialized_ = true;
#ifdef KPT_GUI_RUNTIME_TEST_SUPPORT
    if (fault(detail::RuntimeFaultPoint::AfterPlatformBackend))
      return error(GuiErrorCode::GraphicsDeviceUnavailable,
                   "injected failure after Dear ImGui GLFW initialization");
#endif
    if (!ImGui_ImplOpenGL3_Init("#version 330 core")) {
      return error(GuiErrorCode::GraphicsDeviceUnavailable,
                   "Dear ImGui OpenGL backend initialization failed");
    }
    imgui_opengl_initialized_ = true;

    glfwSetWindowUserPointer(window_, this);
    glfwSetWindowSizeCallback(window_, metricsCallback);
    glfwSetFramebufferSizeCallback(window_, framebufferCallback);
    glfwSetWindowContentScaleCallback(window_, contentScaleCallback);
#ifdef KPT_GUI_RUNTIME_TEST_SUPPORT
    if (hooks_.window_ready != nullptr)
      hooks_.window_ready(window_);
#endif
    refreshMetrics();
    spdlog::debug("Initial framebuffer: logical={}x{}, physical={}x{}, "
                  "scale={:.2f}x{:.2f}",
                  metrics_.logical_size.x, metrics_.logical_size.y,
                  metrics_.framebuffer_size.width,
                  metrics_.framebuffer_size.height, metrics_.scale.x,
                  metrics_.scale.y);
    state_ = State::Initialized;
    return {};
  }

  [[nodiscard]] bool shouldClose() const noexcept override {
    return state_ == State::Shutdown || window_ == nullptr ||
           glfwWindowShouldClose(window_) != GLFW_FALSE;
  }

  void pollEvents() noexcept override {
    if (state_ == State::Initialized)
      glfwPollEvents();
  }

  Result<std::reference_wrapper<FrameContext>, GuiError> beginFrame() override {
    if (state_ != State::Initialized)
      return error(GuiErrorCode::InvalidState,
                   "beginFrame requires an initialized runtime with no active "
                   "frame");
    if (glfwGetCurrentContext() != window_)
      glfwMakeContextCurrent(window_);
    refreshMetrics();
    ImGuiIO &io = ImGui::GetIO();
    io.DisplayFramebufferScale = metrics_.scale;
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
                   "renderAndPresent requires one active frame");

    const auto close_frame = [this] {
      frame_context_.invalidate();
      state_ = State::Initialized;
    };
    if (glfwGetCurrentContext() != window_) {
      close_frame();
      return error(GuiErrorCode::PresentationFailed,
                   "OpenGL presentation used a different current context");
    }

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
#ifdef KPT_GUI_RUNTIME_TEST_SUPPORT
      if (hooks_.frame_rendered != nullptr)
        hooks_.frame_rendered(window_);
#endif
      if (!frame_diagnostics_logged_) {
        const ImDrawData *draw_data = ImGui::GetDrawData();
        if (draw_data != nullptr) {
          spdlog::debug(
              "Dear ImGui frame: command_lists={}, vertices={}, indices={}, "
              "display={}x{}",
              draw_data->CmdListsCount, draw_data->TotalVtxCount,
              draw_data->TotalIdxCount, draw_data->DisplaySize.x,
              draw_data->DisplaySize.y);
          if (options_.visible && draw_data->TotalVtxCount == 0)
            spdlog::warn("Dear ImGui produced an empty visible frame");
        }
        frame_diagnostics_logged_ = true;
      }
      const unsigned presentation_error = glGetError();
      if (presentation_error != GL_NO_ERROR) {
        spdlog::warn("OpenGL presentation reported error 0x{:x}",
                     presentation_error);
      }
      glfwSwapBuffers(window_);
      if (!first_frame_presented_logged_) {
        spdlog::debug("First OpenGL frame presented");
        first_frame_presented_logged_ = true;
      }
    } catch (const std::exception &exception) {
      close_frame();
      return error(GuiErrorCode::PresentationFailed,
                   std::string("frame presentation failed: ") +
                       exception.what());
    } catch (...) {
      close_frame();
      return error(GuiErrorCode::PresentationFailed,
                   "frame presentation failed with an unknown exception");
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
    settings_enabled_ = false;
    if (imgui_opengl_initialized_) {
      ImGui_ImplOpenGL3_Shutdown();
      imgui_opengl_initialized_ = false;
    }
    if (imgui_glfw_initialized_) {
      ImGui_ImplGlfw_Shutdown();
      imgui_glfw_initialized_ = false;
    }
    if (imgui_context_created_) {
      ImGui::DestroyContext();
      imgui_context_created_ = false;
    }
    if (window_ != nullptr) {
      glfwDestroyWindow(window_);
      window_ = nullptr;
      frame_context_.bindWindow(nullptr);
    }
    if (glfw_initialized_) {
      glfwTerminate();
      glfw_initialized_ = false;
    }
  }

  [[nodiscard]] FramebufferMetrics framebufferMetrics() const override {
    return metrics_;
  }

  Result<std::unique_ptr<ViewportRenderer>, GuiError>
  createViewportRenderer() override {
    if (state_ != State::Initialized)
      return error(GuiErrorCode::InvalidState,
                   "renderer creation requires an initialized runtime");
#ifdef KPT_GUI_RUNTIME_TEST_SUPPORT
    if (fault(detail::RuntimeFaultPoint::RendererCreation))
      return error(GuiErrorCode::RendererCreationFailed,
                   "injected renderer creation failure");
#endif
    if (glfwGetCurrentContext() != window_)
      glfwMakeContextCurrent(window_);
    try {
      return std::unique_ptr<ViewportRenderer>(
          std::make_unique<OpenGLPointRenderer>(window_));
    } catch (const std::exception &exception) {
      return error(GuiErrorCode::RendererCreationFailed,
                   std::string("OpenGL renderer creation failed: ") +
                       exception.what());
    } catch (...) {
      return error(GuiErrorCode::RendererCreationFailed,
                   "OpenGL renderer creation failed with an unknown exception");
    }
  }

private:
  enum class State { Created, Initialized, FrameActive, Shutdown };

#ifdef KPT_GUI_RUNTIME_TEST_SUPPORT
  [[nodiscard]] bool fault(detail::RuntimeFaultPoint point) const noexcept {
    return hooks_.fail_at == point;
  }
#endif

  static void metricsCallback(GLFWwindow *window, int, int) {
    auto *runtime =
        static_cast<GlfwOpenGLRuntime *>(glfwGetWindowUserPointer(window));
    if (runtime != nullptr)
      runtime->refreshMetrics();
  }

  static void framebufferCallback(GLFWwindow *window, int, int) {
    metricsCallback(window, 0, 0);
  }

  static void contentScaleCallback(GLFWwindow *window, float, float) {
    metricsCallback(window, 0, 0);
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
    float scale_x = 1.0F;
    float scale_y = 1.0F;
    glfwGetWindowContentScale(window_, &scale_x, &scale_y);
    if (logical_width > 0 && framebuffer_width > 0)
      scale_x = static_cast<float>(framebuffer_width) /
                static_cast<float>(logical_width);
    if (logical_height > 0 && framebuffer_height > 0)
      scale_y = static_cast<float>(framebuffer_height) /
                static_cast<float>(logical_height);
    metrics_.scale = {scale_x, scale_y};
  }

  void configureFonts(ImGuiIO &io) {
    ImFontConfig default_config;
    default_config.SizePixels = 16.0F;
    ImFont *default_font = io.Fonts->AddFontDefault(&default_config);
    if (options_.fonts == nullptr)
      return;

    const auto cjk_font =
        options_.fonts->matchUiFont(U"中文路径文件选择点云轨迹标签");
    if (!cjk_font) {
      logPlatformError("CJK font lookup disabled", cjk_font.error());
      return;
    }
    if (!cjk_font.value())
      return;
    auto utf8_path = platform::pathToUtf8(cjk_font.value()->file);
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

  [[nodiscard]] bool loadSettings() {
    auto loaded = options_.settings->loadIni();
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

  [[nodiscard]] bool flushSettings() {
    std::size_t size = 0;
    const char *contents = ImGui::SaveIniSettingsToMemory(&size);
    auto saved =
        options_.settings->saveIniAtomically(std::string_view(contents, size));
    ImGui::GetIO().WantSaveIniSettings = false;
    if (!saved) {
      logPlatformError("ImGui settings save disabled", saved.error());
      return false;
    }
    return true;
  }

#ifdef KPT_GUI_RUNTIME_TEST_SUPPORT
  detail::RuntimeTestHooks hooks_;
#endif
  State state_ = State::Created;
  bool initialization_attempted_ = false;
  bool glfw_initialized_ = false;
  bool imgui_context_created_ = false;
  bool imgui_glfw_initialized_ = false;
  bool imgui_opengl_initialized_ = false;
  bool settings_enabled_ = false;
  bool frame_diagnostics_logged_ = false;
  bool first_frame_presented_logged_ = false;
  GLFWwindow *window_ = nullptr;
  OpenGLFrameContext frame_context_{nullptr, false};
  GuiRuntimeOptions options_;
  FramebufferMetrics metrics_;
};

std::unique_ptr<GuiRuntime> createGuiRuntime() {
  return std::make_unique<GlfwOpenGLRuntime>();
}

#ifdef KPT_GUI_RUNTIME_TEST_SUPPORT
std::unique_ptr<GuiRuntime>
createGuiRuntimeForTests(detail::RuntimeTestHooks hooks) {
  return std::make_unique<GlfwOpenGLRuntime>(hooks);
}
#endif

} // namespace kpt::gui
