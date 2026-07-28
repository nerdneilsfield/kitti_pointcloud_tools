#pragma once

#include "gui/runtime/factory.hpp"

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
  void (*window_ready)(void *) = nullptr;
};

} // namespace detail

// Available only from the testable runtime backend target.
[[nodiscard]] std::unique_ptr<GuiRuntime>
createGuiRuntimeForTests(detail::RuntimeTestHooks hooks);

} // namespace kpt::gui
