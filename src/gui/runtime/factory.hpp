#pragma once

#include "gui/runtime/runtime.hpp"

#include <memory>

namespace kpt::gui {

namespace detail {

enum class RuntimeFaultPoint {
  None,
  AfterWindowSystem,
  AfterWindow,
  AfterImGuiContext,
  AfterPlatformBackend,
  RendererCreation
};

struct RuntimeTestHooks {
  RuntimeFaultPoint fail_at = RuntimeFaultPoint::None;
};

} // namespace detail

[[nodiscard]] std::unique_ptr<GuiRuntime> createGuiRuntime();

// Internal fault-injection seam. Production composition uses
// createGuiRuntime().
[[nodiscard]] std::unique_ptr<GuiRuntime>
createGuiRuntimeForTests(detail::RuntimeTestHooks hooks);

} // namespace kpt::gui
