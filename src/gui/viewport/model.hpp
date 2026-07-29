#pragma once

#include "gui/viewport/render_types.hpp"

#include <cstdint>
#include <memory>

namespace kpt::gui {

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
  void setView(View view);
  void setStyle(ViewportStyle style);

  [[nodiscard]] std::shared_ptr<const ViewportCloudSnapshot> cloud() const;
  [[nodiscard]] ViewportFrame frame(PixelExtent physical_pixels) const;
  [[nodiscard]] const CloudBounds &bounds() const;
  [[nodiscard]] std::uint64_t cloudRevision() const;

private:
  void setEyeDirection(const Eigen::Vector3f &direction,
                       const Eigen::Vector3f &up_hint =
                           Eigen::Vector3f::UnitZ());

  std::shared_ptr<const ViewportCloudSnapshot> cloud_;
  // Clearing the visible cloud must not lower the accepted-revision high-water
  // mark, otherwise a late completion could resurrect an older cloud.
  std::uint64_t accepted_revision_ = 0;
  ViewportStyle style_;
  Eigen::Vector3f target_ = Eigen::Vector3f::Zero();
  Eigen::Matrix3f camera_to_world_ = Eigen::Matrix3f::Identity();
  float distance_ = 10.0F;
};

} // namespace kpt::gui
