#include "gui/runtime/factory.hpp"

#include "gui/backend/metal/point_renderer.hpp"
#ifdef KPT_GUI_RUNTIME_TEST_SUPPORT
#include "gui/runtime/test_support.hpp"
#endif
#include "platform/utf8_path.hpp"

#include "backends/imgui_impl_glfw.h"
#include "backends/imgui_impl_metal.h"
#include "imgui.h"

#define GLFW_INCLUDE_NONE
#define GLFW_EXPOSE_NATIVE_COCOA
#include <GLFW/glfw3.h>
#include <GLFW/glfw3native.h>

#import <Metal/Metal.h>
#import <QuartzCore/CAMetalLayer.h>

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

void centerWindowOnPrimaryMonitor(GLFWwindow *window) {
  GLFWmonitor *monitor = glfwGetPrimaryMonitor();
  if (window == nullptr || monitor == nullptr)
    return;

  int work_x = 0;
  int work_y = 0;
  int work_width = 0;
  int work_height = 0;
  int window_width = 0;
  int window_height = 0;
  glfwGetMonitorWorkarea(monitor, &work_x, &work_y, &work_width, &work_height);
  glfwGetWindowSize(window, &window_width, &window_height);
  if (work_width <= 0 || work_height <= 0 || window_width <= 0 ||
      window_height <= 0)
    return;

  const int x = work_x + std::max(0, (work_width - window_width) / 2);
  const int y = work_y + std::max(0, (work_height - window_height) / 2);
  glfwSetWindowPos(window, x, y);
}

} // namespace

class GlfwMetalRuntime final : public GuiRuntime {
public:
  GlfwMetalRuntime() = default;
#ifdef KPT_GUI_RUNTIME_TEST_SUPPORT
  explicit GlfwMetalRuntime(detail::RuntimeTestHooks hooks) : hooks_(hooks) {}
#endif
  ~GlfwMetalRuntime() override { shutdown(); }

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

    glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);
    // Cocoa may otherwise place a visible unbundled executable on a secondary
    // display. Create it hidden so it can be positioned without a visible jump.
    glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
    window_ = glfwCreateWindow(options.width, options.height,
                               options.title.c_str(), nullptr, nullptr);
    if (window_ == nullptr)
      return error(GuiErrorCode::GraphicsDeviceUnavailable,
                   "GLFW could not create a Metal window");
#ifdef KPT_GUI_RUNTIME_TEST_SUPPORT
    if (fault(detail::RuntimeFaultPoint::AfterWindow))
      return error(GuiErrorCode::GraphicsDeviceUnavailable,
                   "injected failure after window creation");
#endif

    device_ = MTLCreateSystemDefaultDevice();
    command_queue_ = [device_ newCommandQueue];
    if (device_ == nil || command_queue_ == nil) {
      return error(GuiErrorCode::GraphicsDeviceUnavailable,
                   "Metal default device or command queue is unavailable");
    }
    NSWindow *native_window = glfwGetCocoaWindow(window_);
    layer_ = [CAMetalLayer layer];
    layer_.device = device_;
    layer_.pixelFormat = MTLPixelFormatBGRA8Unorm;
    layer_.framebufferOnly = YES;
    native_window.contentView.wantsLayer = YES;
    native_window.contentView.layer = layer_;
    spdlog::info("Metal device: '{}'", device_.name.UTF8String);

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

    if (!ImGui_ImplGlfw_InitForOther(window_, true)) {
      return error(GuiErrorCode::GraphicsDeviceUnavailable,
                   "Dear ImGui GLFW backend initialization failed");
    }
    imgui_glfw_initialized_ = true;
#ifdef KPT_GUI_RUNTIME_TEST_SUPPORT
    if (fault(detail::RuntimeFaultPoint::AfterPlatformBackend))
      return error(GuiErrorCode::GraphicsDeviceUnavailable,
                   "injected failure after Dear ImGui GLFW initialization");
#endif
    if (!ImGui_ImplMetal_Init(device_)) {
      return error(GuiErrorCode::GraphicsDeviceUnavailable,
                   "Dear ImGui Metal backend initialization failed");
    }
    imgui_metal_initialized_ = true;

    glfwSetWindowUserPointer(window_, this);
    glfwSetWindowSizeCallback(window_, metricsCallback);
    glfwSetFramebufferSizeCallback(window_, framebufferCallback);
    glfwSetWindowContentScaleCallback(window_, contentScaleCallback);
#ifdef KPT_GUI_RUNTIME_TEST_SUPPORT
    if (hooks_.window_ready != nullptr)
      hooks_.window_ready(window_);
#endif
    if (options.visible) {
      centerWindowOnPrimaryMonitor(window_);
      glfwShowWindow(window_);
    }
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
    refreshMetrics();
    drawable_ = [layer_ nextDrawable];
    command_buffer_ = [command_queue_ commandBuffer];
    if (drawable_ == nil || command_buffer_ == nil)
      return error(GuiErrorCode::PresentationFailed,
                   "Metal drawable or command buffer is unavailable");
    render_pass_ = [MTLRenderPassDescriptor renderPassDescriptor];
    render_pass_.colorAttachments[0].texture = drawable_.texture;
    render_pass_.colorAttachments[0].loadAction = MTLLoadActionClear;
    render_pass_.colorAttachments[0].storeAction = MTLStoreActionStore;
    render_pass_.colorAttachments[0].clearColor =
        MTLClearColorMake(0.08, 0.08, 0.09, 1.0);
    ImGuiIO &io = ImGui::GetIO();
    io.DisplayFramebufferScale = metrics_.scale;
    ImGui_ImplMetal_NewFrame(render_pass_);
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();
    frame_context_.activate((__bridge void *)device_,
                            (__bridge void *)command_queue_,
                            (__bridge void *)command_buffer_);
    state_ = State::FrameActive;
    return std::ref(static_cast<FrameContext &>(frame_context_));
  }

  Result<void, GuiError> renderAndPresent() override {
    if (state_ != State::FrameActive)
      return error(GuiErrorCode::InvalidState,
                   "renderAndPresent requires one active frame");

    const auto close_frame = [this] {
      frame_context_.invalidate();
      render_pass_ = nil;
      drawable_ = nil;
      command_buffer_ = nil;
      state_ = State::Initialized;
    };

    try {
      ImGui::Render();
      if (settings_enabled_ && ImGui::GetIO().WantSaveIniSettings)
        settings_enabled_ = flushSettings();
      refreshMetrics();
      id<MTLRenderCommandEncoder> encoder =
          [command_buffer_ renderCommandEncoderWithDescriptor:render_pass_];
      if (encoder == nil) {
        close_frame();
        return error(GuiErrorCode::PresentationFailed,
                     "Metal drawable encoder creation failed");
      }
      ImGui_ImplMetal_RenderDrawData(ImGui::GetDrawData(), command_buffer_,
                                    encoder);
      [encoder endEncoding];
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
      [command_buffer_ presentDrawable:drawable_];
      [command_buffer_ commit];
      if (!first_frame_presented_logged_) {
        spdlog::debug("First Metal frame presented");
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
    if (imgui_metal_initialized_) {
      ImGui_ImplMetal_Shutdown();
      imgui_metal_initialized_ = false;
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
      NSWindow *native_window = glfwGetCocoaWindow(window_);
      native_window.contentView.layer = nil;
      glfwDestroyWindow(window_);
      window_ = nullptr;
    }
    render_pass_ = nil;
    drawable_ = nil;
    command_buffer_ = nil;
    layer_ = nil;
    command_queue_ = nil;
    device_ = nil;
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
    try {
      return std::unique_ptr<ViewportRenderer>(
          std::make_unique<MetalPointRenderer>((__bridge void *)device_,
                                               (__bridge void *)command_queue_));
    } catch (const std::exception &exception) {
      return error(GuiErrorCode::RendererCreationFailed,
                   std::string("Metal renderer creation failed: ") +
                       exception.what());
    } catch (...) {
      return error(GuiErrorCode::RendererCreationFailed,
                   "Metal renderer creation failed with an unknown exception");
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
        static_cast<GlfwMetalRuntime *>(glfwGetWindowUserPointer(window));
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
    if (layer_ != nil) {
      layer_.contentsScale =
          static_cast<double>(std::max(scale_x, scale_y));
      layer_.drawableSize =
          CGSizeMake(metrics_.framebuffer_size.width,
                     metrics_.framebuffer_size.height);
    }
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
    if (!saved) {
      logPlatformError("ImGui settings save failed; will retry", saved.error());
      return false;
    }
    ImGui::GetIO().WantSaveIniSettings = false;
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
  bool imgui_metal_initialized_ = false;
  bool settings_enabled_ = false;
  bool frame_diagnostics_logged_ = false;
  bool first_frame_presented_logged_ = false;
  GLFWwindow *window_ = nullptr;
  id<MTLDevice> device_ = nil;
  id<MTLCommandQueue> command_queue_ = nil;
  CAMetalLayer *layer_ = nil;
  id<CAMetalDrawable> drawable_ = nil;
  id<MTLCommandBuffer> command_buffer_ = nil;
  MTLRenderPassDescriptor *render_pass_ = nil;
  MetalFrameContext frame_context_;
  GuiRuntimeOptions options_;
  FramebufferMetrics metrics_;
};

std::unique_ptr<GuiRuntime> createGuiRuntime() {
  return std::make_unique<GlfwMetalRuntime>();
}

#ifdef KPT_GUI_RUNTIME_TEST_SUPPORT
std::unique_ptr<GuiRuntime>
createGuiRuntimeForTests(detail::RuntimeTestHooks hooks) {
  return std::make_unique<GlfwMetalRuntime>(hooks);
}
#endif

} // namespace kpt::gui
