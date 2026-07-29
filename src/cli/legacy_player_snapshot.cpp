#include "cli/legacy_player_snapshot.hpp"

#include "kpt/render/render.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <string>

namespace kpt::cli {
namespace {

bool isKnownViewName(std::string_view candidate) {
  constexpr std::array<View, 10> views = {
      View::Front,         View::Right,        View::Back,
      View::Left,          View::Top,          View::Bottom,
      View::TopRightFront, View::TopLeftFront, View::BotRightFront,
      View::BotLeftFront};
  return std::ranges::any_of(
      views, [candidate](View view) { return viewName(view) == candidate; });
}

} // namespace

std::filesystem::path
sequenceSnapshotOutputPath(const std::filesystem::path &prefix,
                           const std::filesystem::path &frame_path,
                           std::string_view view_name) {
  if (prefix.empty())
    throw std::invalid_argument("snapshot output prefix must not be empty");
  if (!isKnownViewName(view_name))
    throw std::invalid_argument("snapshot view name is not supported");

  auto output = prefix;
  output += "_";
  output += frame_path.stem().native();
  output += "_";
  output += std::filesystem::path(view_name).native();
  output += ".png";
  return output;
}

std::size_t runPlayerSnapshots(const PlayerSnapshotRequest &request) {
  if (request.output_prefix.empty())
    throw std::invalid_argument("snapshot output prefix must not be empty");
  if (request.width <= 0 || request.height <= 0)
    throw std::invalid_argument("snapshot dimensions must be positive");
  if (renderProjectionName(request.projection) == "unknown")
    throw std::invalid_argument("snapshot projection is not supported");
  if (request.projection == RenderProjection::Perspective &&
      !(request.fov > 0.0F && request.fov < 180.0F))
    throw std::invalid_argument(
        "snapshot FOV must be greater than 0 and less than 180");
  if (!std::isfinite(request.trim_percent) || request.trim_percent < 0.0F ||
      request.trim_percent >= 50.0F) {
    throw std::invalid_argument("snapshot trim percent must be in [0, 50)");
  }
  if (request.views.empty())
    throw std::invalid_argument("snapshot views must not be empty");
  for (const auto view : request.views) {
    if (!isKnownViewName(viewName(view)))
      throw std::invalid_argument("snapshot view is not supported");
  }

  workflow::SequenceSource source(request.sequence);
  RenderOpts render_options;
  render_options.width = request.width;
  render_options.height = request.height;
  render_options.fov = request.fov;
  render_options.projection = request.projection;
  render_options.trim_percent = request.trim_percent;
  render_options.views = request.views;

  std::size_t written = 0;
  for (std::size_t index = 0; index < source.size(); ++index) {
    const auto frame = source.load(index);
    const auto rendered = renderMultiView(frame.cloud, render_options);
    for (const auto &result : rendered) {
      const auto output = sequenceSnapshotOutputPath(
          request.output_prefix, frame.path, result.view_name);
      static_cast<void>(writeImageAtomic(output, result.image, true));
      ++written;
    }
  }
  return written;
}

} // namespace kpt::cli
