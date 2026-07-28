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
                       const Eigen::Vector3f &target) {
  const Eigen::Vector3f forward = (target - eye).normalized();
  const Eigen::Vector3f right =
      forward.cross(Eigen::Vector3f::UnitZ()).normalized();
  const Eigen::Vector3f up = right.cross(forward);

  Eigen::Matrix4f result = Eigen::Matrix4f::Identity();
  result.block<1, 3>(0, 0) = right.transpose();
  result.block<1, 3>(1, 0) = up.transpose();
  result.block<1, 3>(2, 0) = -forward.transpose();
  result(0, 3) = -right.dot(eye);
  result(1, 3) = -up.dot(eye);
  result(2, 3) = forward.dot(eye);
  return result;
}

} // namespace

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

void ViewportModel::orbit(float delta_x, float delta_y) {
  yaw_ -= delta_x * 0.008F;
  pitch_ = std::clamp(pitch_ + delta_y * 0.008F, -1.553F, 1.553F);
}

void ViewportModel::pan(float delta_x, float delta_y) {
  const float cosine = std::cos(pitch_);
  const Eigen::Vector3f eye_direction(
      cosine * std::cos(yaw_), cosine * std::sin(yaw_), std::sin(pitch_));
  const Eigen::Vector3f right =
      (-eye_direction).cross(Eigen::Vector3f::UnitZ()).normalized();
  const Eigen::Vector3f up = right.cross(-eye_direction).normalized();
  const float scale = distance_ * 0.0015F;
  target_ += right * delta_x * scale + up * delta_y * scale;
}

void ViewportModel::zoom(float wheel_delta) {
  distance_ *= std::exp(-wheel_delta * 0.12F);
  distance_ =
      std::clamp(distance_, bounds().radius * 0.01F, bounds().radius * 1000.0F);
}

void ViewportModel::setView(View view) {
  struct Angles {
    float yaw;
    float pitch;
  };

  Angles angles{};
  switch (view) {
  case View::Front:
    angles = {0.0F, 0.0F};
    break;
  case View::Right:
    angles = {kPi * 0.5F, 0.0F};
    break;
  case View::Back:
    angles = {kPi, 0.0F};
    break;
  case View::Left:
    angles = {-kPi * 0.5F, 0.0F};
    break;
  case View::Top:
    angles = {0.0F, kPi * 0.5F - 0.001F};
    break;
  case View::Bottom:
    angles = {0.0F, -kPi * 0.5F + 0.001F};
    break;
  case View::TopRightFront:
    angles = {kPi * 0.25F, kPi * 0.25F};
    break;
  case View::TopLeftFront:
    angles = {-kPi * 0.25F, kPi * 0.25F};
    break;
  case View::BotRightFront:
    angles = {kPi * 0.25F, -kPi * 0.25F};
    break;
  case View::BotLeftFront:
    angles = {-kPi * 0.25F, -kPi * 0.25F};
    break;
  }
  yaw_ = angles.yaw;
  pitch_ = angles.pitch;
  fit();
}

void ViewportModel::setStyle(ViewportStyle style) { style_ = std::move(style); }

std::shared_ptr<const ViewportCloudSnapshot> ViewportModel::cloud() const {
  return cloud_;
}

ViewportFrame ViewportModel::frame(PixelExtent physical_pixels) const {
  const float cosine = std::cos(pitch_);
  const Eigen::Vector3f offset(distance_ * cosine * std::cos(yaw_),
                               distance_ * cosine * std::sin(yaw_),
                               distance_ * std::sin(pitch_));
  const Eigen::Vector3f eye = target_ + offset;
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
              lookAt(eye, target_),
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
