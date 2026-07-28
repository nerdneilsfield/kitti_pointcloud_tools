#pragma once

#include "kpt/core_types.hpp"

#include <array>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace kpt::cli {

struct LegacyViewportStyle {
  ColorBy color_by = ColorBy::Intensity;
  float point_size = 3.0F;
  std::array<float, 3> background{0.0F, 0.0F, 0.0F};
};

struct ViewerCliOptions {
  bool help = false;
  int log_level = 2;
  std::string input_file_utf8;
  LegacyViewportStyle style;
};

struct PlayerSnapshotOptions {
  std::string output_prefix_utf8;
  int width = 640;
  int height = 480;
  float fov = 120.0F;
  std::vector<View> views;
  bool overwrite = true;
};

struct PlayerCliOptions {
  bool help = false;
  int log_level = 2;
  std::string input_dir_utf8;
  std::string glob = "*";
  std::optional<std::string> label_dir_utf8;
  std::optional<std::string> poses_utf8;
  std::optional<std::string> poses2_utf8;
  LegacyViewportStyle style;
  int fps = 10;
  std::optional<PlayerSnapshotOptions> snapshot;
};

template <typename T> struct CliParseResult {
  std::optional<T> value;
  std::string error;

  [[nodiscard]] explicit operator bool() const { return value.has_value(); }
};

CliParseResult<ViewerCliOptions>
parseViewerArgs(std::span<const std::string_view> args);

CliParseResult<PlayerCliOptions>
parsePlayerArgs(std::span<const std::string_view> args);

std::string_view viewerUsage();
std::string_view playerUsage();

} // namespace kpt::cli
