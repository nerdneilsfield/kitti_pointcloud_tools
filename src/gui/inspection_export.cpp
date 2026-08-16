#include "gui/inspection_export.hpp"

#include "kpt/cancellation.hpp"
#include "kpt/io/io.hpp"

#include <cstddef>
#include <exception>

namespace kpt::gui {

PointCloudIRGB mergeWorldClouds(std::span<const WorldCloudView> clouds,
                                std::stop_token stop) {
  std::size_t total_points = 0;
  bool has_noise = false;
  for (const WorldCloudView &view : clouds) {
    if (stop.stop_requested()) throw OperationCancelled();
    total_points += view.cloud.size();
    has_noise = has_noise || view.cloud.has_noise;
  }

  PointCloudIRGB merged;
  merged.reserve(total_points);
  merged.has_noise = has_noise;
  for (const WorldCloudView &view : clouds) {
    for (const PointT &point : view.cloud.points) {
      if (stop.stop_requested()) throw OperationCancelled();
      merged.points.push_back(point);
    }
  }
  merged.width = merged.points.size();
  merged.height = 1;
  return merged;
}

InspectionExportResult exportWorldClouds(
    const std::filesystem::path &path, std::span<const WorldCloudView> clouds,
    bool overwrite, std::optional<Format> ascii_flavor, std::stop_token stop) {
  try {
    PointCloudIRGB merged = mergeWorldClouds(clouds, stop);
    if (merged.empty()) {
      return {InspectionExportStatus::Empty,
              "no points remain after ROI filtering"};
    }
    const CloudWriteStatus result =
        saveAtomic(path, merged, overwrite, ascii_flavor, stop);
    if (result == CloudWriteStatus::Written) {
      return {InspectionExportStatus::Written, {}};
    }
    return {InspectionExportStatus::Skipped, "output already exists"};
  } catch (const OperationCancelled &exception) {
    return {InspectionExportStatus::Cancelled, exception.what()};
  } catch (const std::runtime_error &exception) {
    return {InspectionExportStatus::Failed, exception.what()};
  } catch (const std::exception &exception) {
    return {InspectionExportStatus::Failed, exception.what()};
  }
}

} // namespace kpt::gui
