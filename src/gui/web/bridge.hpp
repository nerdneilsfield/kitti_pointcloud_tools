#pragma once

#include "gui/web/asset_stager.hpp"
#include "kpt/workflow/workflow.hpp"

#include <filesystem>
#include <memory>
#include <optional>
#include <string>

namespace kpt::gui::web {

enum class PickerKind { Viewer = 0, Clouds = 1, Labels = 2, Poses = 3, Poses2 = 4 };

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

} // namespace kpt::gui::web
