#include "cli/legacy_gui_options.hpp"

#include <charconv>
#include <cmath>
#include <cstddef>
#include <optional>
#include <string>
#include <system_error>
#include <utility>

namespace kpt::cli {
namespace {

constexpr std::array<View, 10> kAllViews = {
    View::Front,         View::Right,        View::Back,
    View::Left,          View::Top,          View::Bottom,
    View::TopRightFront, View::TopLeftFront, View::BotRightFront,
    View::BotLeftFront,
};

struct Token {
  std::string_view name;
  std::optional<std::string_view> inline_value;
};

Token splitOption(std::string_view arg) {
  const auto equal = arg.find('=');
  if (equal == std::string_view::npos)
    return {arg, std::nullopt};
  return {arg.substr(0, equal), arg.substr(equal + 1)};
}

std::string_view trim(std::string_view value) {
  const auto first = value.find_first_not_of(" \t\r\n");
  if (first == std::string_view::npos)
    return {};
  return value.substr(first, value.find_last_not_of(" \t\r\n") - first + 1);
}

template <typename T> std::optional<T> parseNumber(std::string_view value) {
  value = trim(value);
  T parsed{};
  const auto result =
      std::from_chars(value.data(), value.data() + value.size(), parsed);
  if (result.ec != std::errc{} || result.ptr != value.data() + value.size())
    return std::nullopt;
  return parsed;
}

template <typename Options>
CliParseResult<Options> failure(std::string message) {
  return {std::nullopt, std::move(message)};
}

std::optional<std::string_view>
optionValue(const Token &token, std::span<const std::string_view> args,
            std::size_t &index, std::string &error) {
  if (token.inline_value)
    return token.inline_value;
  if (index + 1 >= args.size()) {
    error = "missing value for " + std::string(token.name);
    return std::nullopt;
  }
  ++index;
  return args[index];
}

std::optional<ColorBy> parseColorBy(std::string_view value, bool allow_label) {
  if (value == "intensity")
    return ColorBy::Intensity;
  if (value == "rgb")
    return ColorBy::RGB;
  if (value == "z")
    return ColorBy::Z;
  if (allow_label && value == "label")
    return ColorBy::Label;
  if (value == "none")
    return ColorBy::None;
  return std::nullopt;
}

std::optional<View> parseView(std::string_view value) {
  if (value == "front")
    return View::Front;
  if (value == "right")
    return View::Right;
  if (value == "back")
    return View::Back;
  if (value == "left")
    return View::Left;
  if (value == "top")
    return View::Top;
  if (value == "bottom")
    return View::Bottom;
  if (value == "toprightfront")
    return View::TopRightFront;
  if (value == "topleftfront")
    return View::TopLeftFront;
  if (value == "botrightfront")
    return View::BotRightFront;
  if (value == "botleftfront")
    return View::BotLeftFront;
  return std::nullopt;
}

std::optional<std::array<float, 3>> parseBackground(std::string_view value) {
  std::array<float, 3> result{};
  for (std::size_t index = 0; index < result.size(); ++index) {
    const auto comma = value.find(',');
    const auto component =
        index + 1 == result.size() ? value : value.substr(0, comma);
    if (index + 1 != result.size() && comma == std::string_view::npos)
      return std::nullopt;
    const auto parsed = parseNumber<float>(component);
    if (!parsed || !std::isfinite(*parsed) || *parsed < 0.0F || *parsed > 1.0F)
      return std::nullopt;
    result[index] = *parsed;
    if (index + 1 != result.size())
      value.remove_prefix(comma + 1);
  }
  if (value.find(',') != std::string_view::npos)
    return std::nullopt;
  return result;
}

CliParseResult<std::vector<View>> parseViews(std::string_view value) {
  if (value == "all")
    return {std::vector<View>(kAllViews.begin(), kAllViews.end()), {}};
  std::vector<View> views;
  while (true) {
    const auto comma = value.find(',');
    const auto item = trim(value.substr(0, comma));
    const auto view = parseView(item);
    if (!view)
      return failure<std::vector<View>>("unknown snapshot view: " +
                                        std::string(item));
    views.push_back(*view);
    if (comma == std::string_view::npos)
      break;
    value.remove_prefix(comma + 1);
  }
  if (views.empty())
    return failure<std::vector<View>>(
        "--snapshot-views requires at least one view");
  return {std::move(views), {}};
}

bool isOption(std::string_view value, std::string_view short_name,
              std::string_view long_name) {
  return value == short_name || value == long_name;
}

} // namespace

CliParseResult<ViewerCliOptions>
parseViewerArgs(std::span<const std::string_view> args) {
  ViewerCliOptions options;
  bool positional_only = false;
  for (std::size_t index = 0; index < args.size(); ++index) {
    const auto arg = args[index];
    if (!positional_only && arg == "--") {
      positional_only = true;
      continue;
    }
    if (positional_only || !arg.starts_with('-') || arg == "-") {
      if (!options.input_file_utf8.empty())
        return failure<ViewerCliOptions>(
            "pc_viewer accepts exactly one input file");
      options.input_file_utf8 = std::string(arg);
      continue;
    }

    const Token token = splitOption(arg);
    if (isOption(token.name, "-h", "--help")) {
      if (token.inline_value)
        return failure<ViewerCliOptions>("--help does not accept a value");
      options.help = true;
      continue;
    }

    std::string error;
    const auto value = optionValue(token, args, index, error);
    if (!value)
      return failure<ViewerCliOptions>(std::move(error));
    if (isOption(token.name, "-l", "--log-level")) {
      const auto parsed = parseNumber<int>(*value);
      if (!parsed || *parsed < 0 || *parsed > 3)
        return failure<ViewerCliOptions>("--log-level must be 0, 1, 2, or 3");
      options.log_level = *parsed;
    } else if (isOption(token.name, "-c", "--colorby")) {
      const auto parsed = parseColorBy(*value, false);
      if (!parsed)
        return failure<ViewerCliOptions>(
            "--colorby must be intensity, rgb, z, or none");
      options.style.color_by = *parsed;
    } else if (isOption(token.name, "-s", "--point-size")) {
      const auto parsed = parseNumber<int>(*value);
      if (!parsed || *parsed <= 0)
        return failure<ViewerCliOptions>(
            "--point-size must be a positive integer");
      options.style.point_size = static_cast<float>(*parsed);
    } else if (isOption(token.name, "-b", "--bg")) {
      const auto parsed = parseBackground(*value);
      if (!parsed)
        return failure<ViewerCliOptions>(
            "--bg must contain three values in [0,1]: r,g,b");
      options.style.background = *parsed;
    } else {
      return failure<ViewerCliOptions>("unknown option: " +
                                       std::string(token.name));
    }
  }

  if (!options.help && options.input_file_utf8.empty())
    return failure<ViewerCliOptions>("pc_viewer requires one input file");
  return {std::move(options), {}};
}

CliParseResult<PlayerCliOptions>
parsePlayerArgs(std::span<const std::string_view> args) {
  PlayerCliOptions options;
  std::optional<std::string> snapshot_prefix;
  int snapshot_width = 640;
  int snapshot_height = 480;
  float snapshot_fov = 120.0F;
  std::vector<View> snapshot_views(kAllViews.begin(), kAllViews.end());

  for (std::size_t index = 0; index < args.size(); ++index) {
    const auto arg = args[index];
    if (!arg.starts_with('-') || arg == "-")
      return failure<PlayerCliOptions>("pc_player does not accept positional "
                                       "arguments: " +
                                       std::string(arg));
    const Token token = splitOption(arg);
    if (isOption(token.name, "-h", "--help")) {
      if (token.inline_value)
        return failure<PlayerCliOptions>("--help does not accept a value");
      options.help = true;
      continue;
    }

    std::string error;
    const auto value = optionValue(token, args, index, error);
    if (!value)
      return failure<PlayerCliOptions>(std::move(error));
    if (isOption(token.name, "-l", "--log-level")) {
      const auto parsed = parseNumber<int>(*value);
      if (!parsed || *parsed < 0 || *parsed > 3)
        return failure<PlayerCliOptions>("--log-level must be 0, 1, 2, or 3");
      options.log_level = *parsed;
    } else if (isOption(token.name, "-i", "--input-dir")) {
      options.input_dir_utf8 = std::string(*value);
    } else if (isOption(token.name, "-g", "--glob")) {
      options.glob = std::string(*value);
    } else if (token.name == "--label-dir") {
      if (!value->empty())
        options.label_dir_utf8 = std::string(*value);
    } else if (token.name == "--poses") {
      if (!value->empty())
        options.poses_utf8 = std::string(*value);
    } else if (token.name == "--poses2") {
      if (!value->empty())
        options.poses2_utf8 = std::string(*value);
    } else if (isOption(token.name, "-c", "--colorby")) {
      const auto parsed = parseColorBy(*value, true);
      if (!parsed)
        return failure<PlayerCliOptions>(
            "--colorby must be intensity, rgb, z, label, or none");
      options.style.color_by = *parsed;
    } else if (isOption(token.name, "-s", "--point-size")) {
      const auto parsed = parseNumber<int>(*value);
      if (!parsed || *parsed <= 0)
        return failure<PlayerCliOptions>(
            "--point-size must be a positive integer");
      options.style.point_size = static_cast<float>(*parsed);
    } else if (token.name == "--snapshot") {
      if (!value->empty())
        snapshot_prefix = std::string(*value);
    } else if (token.name == "--snapshot-w") {
      const auto parsed = parseNumber<int>(*value);
      if (!parsed || *parsed <= 0)
        return failure<PlayerCliOptions>("--snapshot-w must be positive");
      snapshot_width = *parsed;
    } else if (token.name == "--snapshot-h") {
      const auto parsed = parseNumber<int>(*value);
      if (!parsed || *parsed <= 0)
        return failure<PlayerCliOptions>("--snapshot-h must be positive");
      snapshot_height = *parsed;
    } else if (token.name == "--snapshot-fov") {
      const auto parsed = parseNumber<float>(*value);
      if (!parsed || !std::isfinite(*parsed) || *parsed <= 0.0F ||
          *parsed >= 180.0F)
        return failure<PlayerCliOptions>(
            "--snapshot-fov must be greater than 0 and less than 180");
      snapshot_fov = *parsed;
    } else if (token.name == "--snapshot-views") {
      auto parsed = parseViews(*value);
      if (!parsed)
        return failure<PlayerCliOptions>(std::move(parsed.error));
      snapshot_views = std::move(*parsed.value);
    } else if (isOption(token.name, "-f", "--fps")) {
      const auto parsed = parseNumber<int>(*value);
      if (!parsed || *parsed <= 0)
        return failure<PlayerCliOptions>("--fps must be positive");
      options.fps = *parsed;
    } else {
      return failure<PlayerCliOptions>("unknown option: " +
                                       std::string(token.name));
    }
  }

  if (!options.help && options.input_dir_utf8.empty())
    return failure<PlayerCliOptions>("pc_player requires --input-dir");
  if (snapshot_prefix) {
    options.snapshot = PlayerSnapshotOptions{std::move(*snapshot_prefix),
                                             snapshot_width,
                                             snapshot_height,
                                             snapshot_fov,
                                             std::move(snapshot_views),
                                             true};
  }
  return {std::move(options), {}};
}

std::string_view viewerUsage() {
  return "usage: pc_viewer <file> [options]\n"
         "  -h, --help\n"
         "  -l, --log-level <0|1|2|3>\n"
         "  -c, --colorby <intensity|rgb|z|none>\n"
         "  -s, --point-size <positive integer>\n"
         "  -b, --bg <r,g,b> (each component 0-1)\n";
}

std::string_view playerUsage() {
  return "usage: pc_player --input-dir <directory> [options]\n"
         "  -h, --help\n"
         "  -l, --log-level <0|1|2|3>\n"
         "  -i, --input-dir <directory>\n"
         "  -g, --glob <pattern>\n"
         "      --label-dir <directory>\n"
         "      --poses <csv>\n"
         "      --poses2 <csv>\n"
         "  -c, --colorby <intensity|rgb|z|label|none>\n"
         "  -s, --point-size <positive integer>\n"
         "  -f, --fps <positive integer>\n"
         "      --snapshot <output-prefix>\n"
         "      --snapshot-w <positive integer>\n"
         "      --snapshot-h <positive integer>\n"
         "      --snapshot-fov <0-180>\n"
         "      --snapshot-views <all|front,right,...>\n";
}

} // namespace kpt::cli
