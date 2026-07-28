#include "cli/legacy_gui_options.hpp"
#include "gui/runner.hpp"
#include "platform/utf8_path.hpp"

#include <Eigen/Core>
#include <spdlog/spdlog.h>

#include <iostream>
#include <span>
#include <string_view>
#include <utility>
#include <vector>

namespace {

void setLogLevel(int level) {
  constexpr spdlog::level::level_enum levels[] = {
      spdlog::level::err, spdlog::level::warn, spdlog::level::info,
      spdlog::level::debug};
  spdlog::set_level(levels[level]);
}

} // namespace

int main(int argc, char *argv[]) {
  std::vector<std::string_view> args;
  args.reserve(static_cast<std::size_t>(argc > 0 ? argc - 1 : 0));
  for (int index = 1; index < argc; ++index)
    args.emplace_back(argv[index]);

  auto parsed = kpt::cli::parseViewerArgs(args);
  if (!parsed) {
    std::cerr << parsed.error << '\n' << kpt::cli::viewerUsage();
    return 1;
  }
  if (parsed.value->help) {
    std::cout << kpt::cli::viewerUsage();
    return 0;
  }
  setLogLevel(parsed.value->log_level);

  auto input = kpt::platform::pathFromUtf8(parsed.value->input_file_utf8);
  if (!input) {
    std::cerr << "invalid UTF-8 input path: " << input.error().message << '\n';
    return 1;
  }

  kpt::gui::ViewportStyle style;
  style.color_by = parsed.value->style.color_by;
  style.point_size = parsed.value->style.point_size;
  style.background = Eigen::Vector3f(parsed.value->style.background[0],
                                     parsed.value->style.background[1],
                                     parsed.value->style.background[2]);

  kpt::gui::WorkbenchLaunchRequest request;
  request.viewer_file = std::move(input).value();
  request.style = style;
  request.title = parsed.value->input_file_utf8;
  return kpt::gui::runWorkbench(std::move(request));
}
