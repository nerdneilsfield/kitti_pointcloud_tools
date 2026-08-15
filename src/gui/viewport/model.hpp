#pragma once

#include "gui/viewport/render_types.hpp"

#include <cstdint>
#include <memory>
#include <optional>

namespace kpt::gui {

struct CameraSnapshot {
  Eigen::Vector3d target = Eigen::Vector3d::Zero();
  Eigen::Vector3d rotation_center = Eigen::Vector3d::Zero();
  Eigen::Matrix3f camera_to_world = Eigen::Matrix3f::Identity();
  double distance = 10.0;
  float fov_y_degrees = 45.0F;
};

// A hit in the single ViewportCloudSnapshot owned by ViewportModel. Its
// position is in that cloud's input coordinate system; this model has no layer
// transform and therefore never performs a local-to-scene-world conversion.
// SceneController must apply its layer transform when it adopts this result.
struct PickResult {
  Eigen::Vector3f cloud_position = Eigen::Vector3f::Zero();
  float intensity = 0.0F;
  float noise = 0.0F;
  // Compatibility alias for pre-SceneController callers. It equals
  // cloud_position and must not be used as a transformed scene-world point.
  Eigen::Vector3f world_position = Eigen::Vector3f::Zero();
};

class ViewportModel {
public:
  ViewportModel();

  void setCloud(std::shared_ptr<const ViewportCloudSnapshot> snapshot,
                CameraUpdate camera_update = CameraUpdate::Fit);

  void fit();
  void orbit(float previous_x, float previous_y, float current_x,
             float current_y, PixelExtent viewport);
  void roll(float delta_x, PixelExtent viewport);
  void pan(float delta_x, float delta_y, PixelExtent viewport);
  void zoom(float wheel_delta_degrees);
  [[nodiscard]] std::optional<PickResult>
  pickCloudFromScreen(float x, float y, PixelExtent viewport);
  // Compatibility API. This model has exactly one cloud, so this forwards to
  // pickCloudFromScreen(); multi-layer selection belongs to SceneController.
  [[nodiscard]] std::optional<PickResult>
  pickFromScreen(float x, float y, PixelExtent viewport);
  [[nodiscard]] CameraSnapshot cameraSnapshot() const;
  // Reject invalid snapshots without changing the current camera state.
  [[nodiscard]] bool setCameraSnapshot(const CameraSnapshot &snapshot);
  // Compatibility API for callers that only need the picked position.
  [[nodiscard]] std::optional<Eigen::Vector3f>
  pointFromScreen(float x, float y, PixelExtent viewport);
  [[nodiscard]] bool setRotationCenterFromScreen(float x, float y,
                                                 PixelExtent viewport);
  void setView(CameraPreset view);
  void setStyle(ViewportStyle style);

  [[nodiscard]] std::shared_ptr<const ViewportCloudSnapshot> cloud() const;
  [[nodiscard]] ViewportFrame frame(PixelExtent physical_pixels);
  [[nodiscard]] const CloudBounds &bounds() const;
  [[nodiscard]] std::uint64_t cloudRevision() const;

private:
  std::shared_ptr<const ViewportCloudSnapshot> cloud_;
  // Clearing the visible cloud must not lower the accepted-revision high-water
  // mark, otherwise a late completion could resurrect an older cloud.
  std::uint64_t accepted_revision_ = 0;
  ViewportStyle style_;
  Eigen::Vector3d target_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d rotation_center_ = Eigen::Vector3d::Zero();
  Eigen::Matrix3f camera_to_world_ = Eigen::Matrix3f::Identity();
  double distance_ = 10.0;
  float fov_y_degrees_ = 45.0F;
  bool fit_pending_ = false;

  void applyFit(PixelExtent physical_pixels);
};

} // namespace kpt::gui
