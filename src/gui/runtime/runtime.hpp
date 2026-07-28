#pragma once

#include "common/result.hpp"
#include "gui/viewport/renderer.hpp"
#include "platform/services.hpp"

#include <functional>
#include <memory>
#include <string>
#include <system_error>

namespace kpt::gui {

struct GuiRuntimeOptions {
  int width = 1440;
  int height = 900;
  std::string title = "KPT Workbench";
  bool visible = true;
  bool persist_settings = true;
  platform::Fonts *fonts = nullptr;
  platform::SettingsStore *settings = nullptr;
};

struct FramebufferMetrics {
  ImVec2 logical_size;
  PixelExtent framebuffer_size;
  ImVec2 scale{1.0F, 1.0F};
};

enum class GuiErrorCode {
  InvalidOptions,
  InvalidState,
  WindowSystemUnavailable,
  GraphicsDeviceUnavailable,
  RendererCreationFailed,
  PresentationFailed,
  UnsupportedBackend,
  ShaderCompilationFailed,
  BackendMismatch
};

struct GuiError {
  GuiErrorCode code = GuiErrorCode::InvalidState;
  std::string message;
  std::error_code system_error;
};

class GuiRuntime {
public:
  virtual ~GuiRuntime() = default;

  virtual Result<void, GuiError> initialize(const GuiRuntimeOptions &) = 0;
  [[nodiscard]] virtual bool shouldClose() const noexcept = 0;
  virtual void pollEvents() noexcept = 0;
  virtual Result<std::reference_wrapper<FrameContext>, GuiError>
  beginFrame() = 0;
  virtual Result<void, GuiError> renderAndPresent() = 0;
  virtual void shutdown() noexcept = 0;
  [[nodiscard]] virtual FramebufferMetrics framebufferMetrics() const = 0;

  [[nodiscard]] virtual Result<std::unique_ptr<ViewportRenderer>, GuiError>
  createViewportRenderer() = 0;
};

} // namespace kpt::gui
