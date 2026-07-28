#include "cli/legacy_gui_options.hpp"
#include "cli/legacy_player_snapshot.hpp"
#include "platform/utf8_path.hpp"

#if KPT_HAS_GUI
#include "gui/runner.hpp"
#endif

#include <spdlog/spdlog.h>

#include <filesystem>
#include <iostream>
#include <optional>
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

std::optional<std::filesystem::path> nativePath(std::string_view value,
                                                std::string_view role) {
  auto converted = kpt::platform::pathFromUtf8(value);
  if (!converted) {
    std::cerr << role << ": " << converted.error().message << '\n';
    return std::nullopt;
  }
  return std::move(converted).value();
}

std::optional<kpt::workflow::SequenceOptions>
sequenceOptions(const kpt::cli::PlayerCliOptions &options) {
  kpt::workflow::SequenceOptions sequence;
  auto input = nativePath(options.input_dir_utf8, "invalid input directory");
  if (!input)
    return std::nullopt;
  sequence.input_dir = std::move(*input);
  sequence.glob = options.glob;

  const auto assign_optional =
      [](const std::optional<std::string> &source,
         std::optional<std::filesystem::path> &destination,
         std::string_view role) {
        if (!source)
          return true;
        auto converted = nativePath(*source, role);
        if (!converted)
          return false;
        destination = std::move(*converted);
        return true;
      };
  if (!assign_optional(options.label_dir_utf8, sequence.label_dir,
                       "invalid label directory") ||
      !assign_optional(options.poses_utf8, sequence.poses,
                       "invalid poses path") ||
      !assign_optional(options.poses2_utf8, sequence.poses2,
                       "invalid poses2 path")) {
    return std::nullopt;
  }
  return sequence;
}

} // namespace

int main(int argc, char *argv[]) {
  std::vector<std::string_view> args;
  args.reserve(static_cast<std::size_t>(argc > 0 ? argc - 1 : 0));
  for (int index = 1; index < argc; ++index)
    args.emplace_back(argv[index]);

  auto parsed = kpt::cli::parsePlayerArgs(args);
  if (!parsed) {
    std::cerr << parsed.error << '\n' << kpt::cli::playerUsage();
    return 1;
  }
  if (parsed.value->help) {
    std::cout << kpt::cli::playerUsage();
    return 0;
  }
  setLogLevel(parsed.value->log_level);

  auto sequence = sequenceOptions(*parsed.value);
  if (!sequence)
    return 1;

  if (parsed.value->snapshot) {
    auto prefix = nativePath(parsed.value->snapshot->output_prefix_utf8,
                             "invalid snapshot output prefix");
    if (!prefix)
      return 1;
    kpt::cli::PlayerSnapshotRequest request;
    request.sequence = std::move(*sequence);
    request.output_prefix = std::move(*prefix);
    request.width = parsed.value->snapshot->width;
    request.height = parsed.value->snapshot->height;
    request.fov = parsed.value->snapshot->fov;
    request.views = parsed.value->snapshot->views;
    try {
      const auto written = kpt::cli::runPlayerSnapshots(request);
      spdlog::info("wrote {} snapshots", written);
      return 0;
    } catch (const std::exception &error) {
      spdlog::error("{}", error.what());
      return 1;
    }
  }

#if KPT_HAS_GUI
  std::shared_ptr<kpt::workflow::SequenceSource> preflight;
  try {
    preflight = std::make_shared<kpt::workflow::SequenceSource>(*sequence);
    if (preflight->empty()) {
      spdlog::info("sequence contains no frames");
      return 0;
    }
  } catch (const std::exception &error) {
    spdlog::error("{}", error.what());
    return 1;
  }

  kpt::gui::ViewportStyle style;
  style.color_by = parsed.value->style.color_by;
  style.point_size = parsed.value->style.point_size;

  kpt::gui::WorkbenchLaunchRequest request;
  request.sequence_source = std::move(preflight);
  request.sequence_fps = parsed.value->fps;
  request.sequence_autoplay = true;
  request.style = style;
  request.title = "pc_player";
  return kpt::gui::runWorkbench(std::move(request));
#else
  std::cerr << "interactive pc_player requires KPT_BUILD_GUI=ON\n";
  return 1;
#endif
}
