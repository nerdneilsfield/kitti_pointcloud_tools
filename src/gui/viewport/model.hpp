#pragma once

#include "gui/viewport/render_types.hpp"

#include <cstdint>
#include <memory>

namespace kpt::gui {

class ViewportModel {
public:
  void setCloud(std::shared_ptr<const ViewportCloudSnapshot> snapshot,
                CameraUpdate camera_update = CameraUpdate::Fit);

  void fit();
  void orbit(float delta_x, float delta_y);
  void pan(float delta_x, float delta_y);
  void zoom(float wheel_delta);
  void setView(View view);
  void setStyle(ViewportStyle style);

  [[nodiscard]] std::shared_ptr<const ViewportCloudSnapshot> cloud() const;
  [[nodiscard]] ViewportFrame frame(PixelExtent physical_pixels) const;
  [[nodiscard]] const CloudBounds &bounds() const;
  [[nodiscard]] std::uint64_t cloudRevision() const;

private:
  std::shared_ptr<const ViewportCloudSnapshot> cloud_;
  ViewportStyle style_;
  Eigen::Vector3f target_ = Eigen::Vector3f::Zero();
  float yaw_ = 0.75F;
  float pitch_ = 0.45F;
  float distance_ = 10.0F;
};

} // namespace kpt::gui
