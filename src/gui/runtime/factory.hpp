#pragma once

#include "gui/runtime/runtime.hpp"

#include <memory>

namespace kpt::gui {

[[nodiscard]] std::unique_ptr<GuiRuntime> createGuiRuntime();

} // namespace kpt::gui
