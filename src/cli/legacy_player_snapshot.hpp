#pragma once

#include "kpt/core_types.hpp"
#include "kpt/workflow/workflow.hpp"

#include <filesystem>
#include <string_view>
#include <vector>

namespace kpt::cli {

struct PlayerSnapshotRequest {
  workflow::SequenceOptions sequence;
  std::filesystem::path output_prefix;
  int width = 640;
  int height = 480;
  float fov = 120.0F;
  std::vector<View> views;
};

std::filesystem::path
sequenceSnapshotOutputPath(const std::filesystem::path &prefix,
                           const std::filesystem::path &frame_path,
                           std::string_view view_name);

// Renders every enumerated frame and returns the number of PNGs written.
// Existing files are overwritten to preserve pc_player's historical contract.
std::size_t runPlayerSnapshots(const PlayerSnapshotRequest &request);

} // namespace kpt::cli
