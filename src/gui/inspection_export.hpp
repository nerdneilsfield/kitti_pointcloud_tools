#pragma once

#include "kpt/io/format.hpp"
#include "kpt/types.hpp"

#include <filesystem>
#include <optional>
#include <span>
#include <stop_token>
#include <string>

namespace kpt::gui {

// The caller owns ROI filtering and local-to-world transformation. Each view
// therefore contains only points that are already in world coordinates.
struct WorldCloudView {
  const PointCloudIRGB &cloud;
};

// App-facing outcome for a non-throwing export job. `Failed` includes I/O and
// format errors reported by kpt::saveAtomic; `Cancelled` never publishes data.
enum class InspectionExportStatus { Written, Skipped, Cancelled, Failed };

struct InspectionExportResult {
  InspectionExportStatus status = InspectionExportStatus::Failed;
  std::string message;

  [[nodiscard]] bool completed() const noexcept {
    return status == InspectionExportStatus::Written ||
           status == InspectionExportStatus::Skipped;
  }
};

// Merges pre-filtered world-coordinate clouds. PointT is copied unchanged;
// output is always one unorganized row and has_noise is the logical OR.
[[nodiscard]] PointCloudIRGB
mergeWorldClouds(std::span<const WorldCloudView> clouds,
                 std::stop_token stop = {});

// Calls kpt::saveAtomic after merging. This service intentionally does no ROI
// or transform work, keeping export world-space semantics explicit at its
// callsite.
[[nodiscard]] InspectionExportResult exportWorldClouds(
    const std::filesystem::path &path, std::span<const WorldCloudView> clouds,
    bool overwrite, std::optional<Format> ascii_flavor = std::nullopt,
    std::stop_token stop = {});

} // namespace kpt::gui
