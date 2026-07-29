#include "gui/viewport/model.hpp"

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <utility>

namespace kpt::gui {
namespace {

constexpr float kPi = 3.14159265358979323846F;

Eigen::Matrix4f perspective(float fov_y, float aspect, float near_plane,
                            float far_plane) {
  const float tangent = std::tan(fov_y * 0.5F);
  Eigen::Matrix4f result = Eigen::Matrix4f::Zero();
  result(0, 0) = 1.0F / (aspect * tangent);
  result(1, 1) = 1.0F / tangent;
  result(2, 2) = -(far_plane + near_plane) / (far_plane - near_plane);
  result(2, 3) = -(2.0F * far_plane * near_plane) / (far_plane - near_plane);
  result(3, 2) = -1.0F;
  return result;
}

Eigen::Matrix4f lookAt(const Eigen::Vector3f &eye,
                       const Eigen::Vector3f &target,
                       const Eigen::Vector3f &up) {
  const Eigen::Vector3f forward = (target - eye).normalized();
  const Eigen::Vector3f right = forward.cross(up).normalized();
  const Eigen::Vector3f corrected_up = right.cross(forward);

  Eigen::Matrix4f result = Eigen::Matrix4f::Identity();
  result.block<1, 3>(0, 0) = right.transpose();
  result.block<1, 3>(1, 0) = corrected_up.transpose();
  result.block<1, 3>(2, 0) = -forward.transpose();
  result(0, 3) = -right.dot(eye);
  result(1, 3) = -up.dot(eye);
  result(2, 3) = forward.dot(eye);
  return result;
}

Eigen::Vector3f trackballPoint(float x, float y, PixelExtent viewport) {
  const float half_width =
      std::max(1.0F, static_cast<float>(viewport.width) * 0.5F);
  const float half_height =
      std::max(1.0F, static_cast<float>(viewport.height) * 0.5F);
  Eigen::Vector3f point{
      std::clamp((x - half_width) / half_width, -1.0F, 1.0F),
      std::clamp((half_height - y) / half_height, -1.0F, 1.0F), 0.0F};
  const float radius_squared =
      point.x() * point.x() + point.y() * point.y();
  if (radius_squared > 1.0F) {
    point.head<2>().normalize();
  } else {
    point.z() = std::sqrt(1.0F - radius_squared);
  }
  return point;
}

} // namespace

ViewportModel::ViewportModel() {
  setEyeDirection({0.656F, 0.611F, 0.435F});
}

void ViewportModel::setCloud(
    std::shared_ptr<const ViewportCloudSnapshot> snapshot,
    CameraUpdate camera_update) {
  if (!snapshot || snapshot->revision == 0) {
    cloud_.reset();
    if (camera_update == CameraUpdate::Fit) {
      fit();
    }
    return;
  }
  if (snapshot->revision <= accepted_revision_) {
    return;
  }

  accepted_revision_ = snapshot->revision;
  cloud_ = std::move(snapshot);
  if (camera_update == CameraUpdate::Fit) {
    fit();
  }
}

void ViewportModel::fit() {
  target_ = bounds().center;
  distance_ = std::max(bounds().radius * 2.8F, 0.01F);
}

void ViewportModel::orbit(float previous_x, float previous_y, float current_x,
                          float current_y, PixelExtent viewport) {
  if (viewport.width <= 0 || viewport.height <= 0)
    return;
  const Eigen::Vector3f previous =
      trackballPoint(previous_x, previous_y, viewport);
  const Eigen::Vector3f current =
      trackballPoint(current_x, current_y, viewport);
  const Eigen::Quaternionf camera_rotation =
      Eigen::Quaternionf::FromTwoVectors(previous, current);
  camera_to_world_ =
      camera_to_world_ * camera_rotation.toRotationMatrix().transpose();
  camera_to_world_ =
      Eigen::Quaternionf(camera_to_world_).normalized().toRotationMatrix();
}

void ViewportModel::roll(float delta_x, PixelExtent viewport) {
  if (viewport.width <= 0)
    return;
  const float angle =
      2.0F * kPi * delta_x / static_cast<float>(viewport.width);
  const Eigen::AngleAxisf camera_rotation(angle, Eigen::Vector3f::UnitZ());
  camera_to_world_ =
      camera_to_world_ * camera_rotation.toRotationMatrix().transpose();
}

void ViewportModel::pan(float delta_x, float delta_y, PixelExtent viewport) {
  if (viewport.height <= 0)
    return;
  constexpr float fov_y = 45.0F * kPi / 180.0F;
  const float pixel_size =
      2.0F * distance_ * std::tan(fov_y * 0.5F) /
      static_cast<float>(viewport.height);
  target_ += camera_to_world_.col(0) * (-delta_x * pixel_size) +
             camera_to_world_.col(1) * (delta_y * pixel_size);
}

void ViewportModel::zoom(float wheel_delta_degrees) {
  const float scene_size = std::max(bounds().radius * 2.0F, 0.01F);
  const float default_increment = scene_size / 250.0F;
  const float near_plane = std::max(0.001F, distance_ * 0.001F);
  const float speed_ratio = 10.0F * near_plane / scene_size;
  const float speed = std::min(16.0F, std::exp(speed_ratio));
  distance_ -= wheel_delta_degrees * default_increment / 8.0F * speed;
  distance_ =
      std::clamp(distance_, bounds().radius * 0.01F, bounds().radius * 1000.0F);
}

void ViewportModel::setView(View view) {
  Eigen::Vector3f direction;
  Eigen::Vector3f up = Eigen::Vector3f::UnitZ();
  switch (view) {
  case View::Front:
    direction = Eigen::Vector3f::UnitX();
    break;
  case View::Right:
    direction = Eigen::Vector3f::UnitY();
    break;
  case View::Back:
    direction = -Eigen::Vector3f::UnitX();
    break;
  case View::Left:
    direction = -Eigen::Vector3f::UnitY();
    break;
  case View::Top:
    direction = Eigen::Vector3f::UnitZ();
    up = Eigen::Vector3f::UnitY();
    break;
  case View::Bottom:
    direction = -Eigen::Vector3f::UnitZ();
    up = Eigen::Vector3f::UnitY();
    break;
  case View::TopRightFront:
    direction = {1.0F, 1.0F, 1.0F};
    break;
  case View::TopLeftFront:
    direction = {1.0F, -1.0F, 1.0F};
    break;
  case View::BotRightFront:
    direction = {1.0F, 1.0F, -1.0F};
    break;
  case View::BotLeftFront:
    direction = {1.0F, -1.0F, -1.0F};
    break;
  }
  setEyeDirection(direction, up);
  fit();
}

void ViewportModel::setEyeDirection(const Eigen::Vector3f &direction,
                                    const Eigen::Vector3f &up_hint) {
  const Eigen::Vector3f back = direction.normalized();
  const Eigen::Vector3f forward = -back;
  const Eigen::Vector3f right = forward.cross(up_hint).normalized();
  const Eigen::Vector3f up = right.cross(forward).normalized();
  camera_to_world_.col(0) = right;
  camera_to_world_.col(1) = up;
  camera_to_world_.col(2) = back;
}

void ViewportModel::setStyle(ViewportStyle style) { style_ = std::move(style); }

std::shared_ptr<const ViewportCloudSnapshot> ViewportModel::cloud() const {
  return cloud_;
}

ViewportFrame ViewportModel::frame(PixelExtent physical_pixels) const {
  const Eigen::Vector3f eye =
      target_ + camera_to_world_.col(2) * distance_;
  const float near_plane = std::max(0.001F, distance_ * 0.001F);
  const float far_plane =
      std::max(near_plane + 1.0F, distance_ + bounds().radius * 8.0F);
  ViewportStyle frame_style = style_;
  if (frame_style.color_by == ColorBy::Intensity) {
    frame_style.scalar_min = bounds().intensity_min;
    frame_style.scalar_max = bounds().intensity_max;
  } else if (frame_style.color_by == ColorBy::Z) {
    frame_style.scalar_min = bounds().z_min;
    frame_style.scalar_max = bounds().z_max;
  }
  const float aspect = physical_pixels.width > 0 && physical_pixels.height > 0
                           ? static_cast<float>(physical_pixels.width) /
                                 static_cast<float>(physical_pixels.height)
                           : 1.0F;
  return {perspective(45.0F * kPi / 180.0F, aspect, near_plane, far_plane) *
              lookAt(eye, target_, camera_to_world_.col(1)),
          frame_style};
}

const CloudBounds &ViewportModel::bounds() const {
  static const CloudBounds empty_bounds;
  return cloud_ ? cloud_->bounds : empty_bounds;
}

std::uint64_t ViewportModel::cloudRevision() const {
  return cloud_ ? cloud_->revision : 0;
}

} // namespace kpt::gui
