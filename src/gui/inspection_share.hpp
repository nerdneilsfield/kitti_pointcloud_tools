#pragma once

#include "gui/scene/scene.hpp"

#include <filesystem>
#include <optional>
#include <stop_token>
#include <string>
#include <vector>

namespace kpt::gui {

// A review share stores semantic state, never point data. Runtime LayerId is
// deliberately absent: import allocates fresh IDs and measurements refer to
// stable source_key values instead.
struct InspectionShareLayer {
  std::string source_key;
  // Relative to the share JSON file only. A missing path represents an opaque
  // source or a deliberately unresolved file layer; import has no fallback.
  std::optional<std::filesystem::path> relative_source_path;
  Eigen::Affine3d local_to_world = Eigen::Affine3d::Identity();
  LayerStyle style;
  bool visible = true;
};

struct InspectionShareMeasurement {
  std::string first_source_key;
  Eigen::Vector3d first_world = Eigen::Vector3d::Zero();
  std::optional<std::string> second_source_key;
  std::optional<Eigen::Vector3d> second_world;
};

struct InspectionShareDocument {
  std::vector<InspectionShareLayer> layers;
  std::optional<RoiBox> roi;
  std::vector<InspectionShareMeasurement> measurements;
  std::vector<CameraBookmark> bookmarks;
};

enum class InspectionShareSaveStatus { Written, Skipped, Cancelled, Failed };

struct InspectionShareSaveResult {
  InspectionShareSaveStatus status = InspectionShareSaveStatus::Failed;
  std::string message;

  [[nodiscard]] bool completed() const noexcept {
    return status == InspectionShareSaveStatus::Written ||
           status == InspectionShareSaveStatus::Skipped;
  }
};

// Versioned, atomically written portable review document. Existing files stay
// untouched when parsing/validation/writing fails. overwrite=false publishes
// with an atomic no-replace operation; it is not a preflight exists check.
class InspectionShareFile {
public:
  static constexpr int kSchemaVersion = 1;

  explicit InspectionShareFile(std::filesystem::path file);

  [[nodiscard]] const std::filesystem::path &path() const noexcept;
  [[nodiscard]] bool load(InspectionShareDocument &document,
                          std::string *error = nullptr) const;
  [[nodiscard]] InspectionShareSaveResult
  save(const InspectionShareDocument &document, bool overwrite,
       std::stop_token stop = {}) const;

  // Captures only review state. Path-source references become normalized
  // relative paths when that is representable; all other layers stay
  // unresolved on import rather than falling back to another directory.
  [[nodiscard]] static InspectionShareDocument
  capture(const Scene &scene, const InspectionSettings &settings,
          const std::filesystem::path &share_file);

  // Resolves only relative_source_path against this share file's parent. It
  // neither uses cwd/workspace state nor reconstructs a path from source_key.
  [[nodiscard]] static std::optional<std::filesystem::path>
  resolveSourcePath(const std::filesystem::path &share_file,
                    const InspectionShareLayer &layer);

private:
  std::filesystem::path file_;
};

} // namespace kpt::gui
