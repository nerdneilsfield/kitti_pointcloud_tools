#pragma once

#include "gui/viewport/model.hpp"
#include "kpt/workflow/workflow.hpp"

#include <filesystem>
#include <optional>
#include <string>

namespace kpt::gui {

struct WorkbenchLaunchRequest {
  std::optional<std::filesystem::path> viewer_file;
  std::optional<workflow::SequenceOptions> sequence;
  std::optional<int> sequence_fps;
  bool sequence_autoplay = false;
  std::optional<ViewportStyle> style;
  std::string title = "KPT Workbench";
  int width = 1440;
  int height = 900;
  bool smoke_test = false;
};

// Owns platform services, GUI runtime, renderers and the single App frame loop.
// Every GUI executable delegates here; no entry point owns a graphics loop.
int runWorkbench(WorkbenchLaunchRequest request);

} // namespace kpt::gui
