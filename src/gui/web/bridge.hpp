#pragma once

#include "gui/web/asset_stager.hpp"
#include "kpt/workflow/workflow.hpp"

#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <string_view>

namespace kpt::gui {
struct Rgba8Image;
}

namespace kpt::gui::web {

enum class PickerKind {
  Viewer = 0,
  Clouds = 1,
  Labels = 2,
  Poses = 3,
  Poses2 = 4
};

struct SelectionSnapshot {
  std::optional<std::filesystem::path> viewer;
  std::size_t cloud_count = 0;
  std::size_t label_count = 0;
  bool has_poses = false;
  bool has_poses2 = false;
  std::string error;
};

struct SequenceBuild {
  std::shared_ptr<workflow::SequenceSource> source;
  std::string error;
};

void openPicker(PickerKind kind);
SelectionSnapshot selectionSnapshot();
SequenceBuild buildSequence();
std::shared_ptr<AssetStager> createAssetStager();

// Copies a completed top-left RGBA viewport image into browser-owned memory,
// encodes it as PNG, and starts a user download.  Browser builds deliberately
// do not route this through the virtual filesystem or a background file job.
[[nodiscard]] bool downloadViewportPng(std::string_view filename,
                                       const Rgba8Image &image,
                                       std::string *error = nullptr);

} // namespace kpt::gui::web
