#include "gui/viewport/model.hpp"

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace kpt::gui {
namespace {

constexpr std::size_t kMaximumGridLines = 82;

float niceGridStep(float span) {
  if (!std::isfinite(span) || span <= 0.0F)
    return 1.0F;
  const float raw =
      std::max(span / 10.0F, std::numeric_limits<float>::min());
  const float magnitude = std::pow(10.0F, std::floor(std::log10(raw)));
  if (!std::isfinite(magnitude) || magnitude <= 0.0F)
    return 1.0F;
  const float normalized = raw / magnitude;
  const float multiple = normalized <= 1.0F   ? 1.0F
                         : normalized <= 2.0F ? 2.0F
                         : normalized <= 5.0F ? 5.0F
                                              : 10.0F;
  return multiple * magnitude;
}

void appendLine(std::vector<ViewportLineVertex> &vertices,
                const Eigen::Vector3f &from, const Eigen::Vector3f &to,
                const Eigen::Vector3f &color) {
  vertices.push_back({from, color});
  vertices.push_back({to, color});
}

std::vector<ViewportLineVertex> buildGuides(const CloudBounds &bounds,
                                            const ViewportStyle &style,
                                            float &grid_spacing) {
  std::vector<ViewportLineVertex> guides;
  grid_spacing = 0.0F;
  if (!style.show_coordinate_axes && !style.show_scale_grid)
    return guides;

  float horizontal_span = 1.0F;
  float axis_span = 1.0F;
  if (bounds.finite_points != 0 && bounds.minimum.allFinite() &&
      bounds.maximum.allFinite()) {
    const Eigen::Vector3f dimensions = bounds.maximum - bounds.minimum;
    if (dimensions.allFinite()) {
      horizontal_span =
          std::max({std::abs(bounds.minimum.x()), std::abs(bounds.maximum.x()),
                    std::abs(bounds.minimum.y()), std::abs(bounds.maximum.y()),
                    dimensions.x(), dimensions.y(),
                    std::numeric_limits<float>::min()});
      axis_span =
          std::max({horizontal_span, std::abs(bounds.minimum.z()),
                    std::abs(bounds.maximum.z()), dimensions.z(),
                    std::numeric_limits<float>::min()});
    }
  }
  const float step = niceGridStep(horizontal_span * 2.0F);
  const float unclamped_divisions = std::ceil(horizontal_span / step);
  const int divisions =
      std::clamp(static_cast<int>(std::min(
                     unclamped_divisions,
                     static_cast<float>((kMaximumGridLines - 2U) / 4U))),
                 1, 20);
  const float limit = static_cast<float>(divisions) * step;

  if (style.show_scale_grid && bounds.finite_points != 0) {
    const float luminance =
        style.background.dot(Eigen::Vector3f{0.2126F, 0.7152F, 0.0722F});
    const Eigen::Vector3f grid_color =
        luminance > 0.55F ? Eigen::Vector3f{0.28F, 0.32F, 0.38F}
                          : Eigen::Vector3f{0.72F, 0.78F, 0.86F};
    guides.reserve(static_cast<std::size_t>(divisions * 4 + 6));
    for (int index = -divisions; index <= divisions; ++index) {
      const float offset = static_cast<float>(index) * step;
      appendLine(guides, {-limit, offset, 0.0F}, {limit, offset, 0.0F},
                 grid_color);
      appendLine(guides, {offset, -limit, 0.0F}, {offset, limit, 0.0F},
                 grid_color);
    }
    grid_spacing = step;
  }

  if (style.show_coordinate_axes) {
    const float axis_length = std::max(axis_span, step);
    const float luminance =
        style.background.dot(Eigen::Vector3f{0.2126F, 0.7152F, 0.0722F});
    const bool light_background = luminance > 0.55F;
    appendLine(guides, Eigen::Vector3f::Zero(), {axis_length, 0.0F, 0.0F},
               light_background ? Eigen::Vector3f{0.68F, 0.22F, 0.22F}
                                : Eigen::Vector3f{1.0F, 0.61F, 0.61F});
    appendLine(guides, Eigen::Vector3f::Zero(), {0.0F, axis_length, 0.0F},
               light_background ? Eigen::Vector3f{0.16F, 0.55F, 0.29F}
                                : Eigen::Vector3f{0.61F, 0.94F, 0.73F});
    appendLine(guides, Eigen::Vector3f::Zero(), {0.0F, 0.0F, axis_length},
               light_background ? Eigen::Vector3f{0.20F, 0.38F, 0.70F}
                                : Eigen::Vector3f{0.61F, 0.76F, 1.0F});
  }
  return guides;
}

constexpr float kPi = 3.14159265358979323846F;

Eigen::Matrix4f perspective(float fov_y, float aspect, float near_plane,
                            float far_plane) {
  const float tangent = std::tan(fov_y * 0.5F);
  Eigen::Matrix4f result = Eigen::Matrix4f::Zero();
  result(0, 0) = 1.0F / (aspect * tangent);
  result(1, 1) = 1.0F / tangent;
  const double near_double = static_cast<double>(near_plane);
  const double far_double = static_cast<double>(far_plane);
  result(2, 2) = static_cast<float>(
      -(far_double + near_double) / (far_double - near_double));
  result(2, 3) = static_cast<float>(
      -(2.0 * far_double * near_double) / (far_double - near_double));
  result(3, 2) = -1.0F;
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

Eigen::Matrix3f cloudCompareView(float vertical_angle,
                                 float orthogonal_angle) {
  Eigen::Matrix3f base_view;
  base_view << 1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F, 0.0F, -1.0F, 0.0F;
  const Eigen::Matrix3f vertical =
      Eigen::AngleAxisf(vertical_angle, Eigen::Vector3f::UnitZ())
          .toRotationMatrix();
  const Eigen::Matrix3f orthogonal =
      Eigen::AngleAxisf(orthogonal_angle, Eigen::Vector3f::UnitX())
          .toRotationMatrix();
  return (orthogonal * base_view * vertical).transpose();
}

} // namespace

ViewportModel::ViewportModel() {
  camera_to_world_ = cloudCompareView(kPi * 0.25F, kPi * 0.25F);
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
  target_ = bounds().center.cast<double>();
  distance_ = std::max(bounds().radius * 2.8, 0.01);
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
  const double pixel_size =
      2.0 * distance_ * static_cast<double>(std::tan(fov_y * 0.5F)) /
      static_cast<double>(viewport.height);
  target_ +=
      camera_to_world_.col(0).cast<double>() *
          (-static_cast<double>(delta_x) * pixel_size) +
      camera_to_world_.col(1).cast<double>() *
          (static_cast<double>(delta_y) * pixel_size);
}

void ViewportModel::zoom(float wheel_delta_degrees) {
  const double scene_size = std::max(bounds().radius * 2.0, 0.01);
  const double default_increment = scene_size / 250.0;
  const double near_plane = std::max(0.001, distance_ * 0.001);
  const double speed_ratio = 10.0 * near_plane / scene_size;
  const double speed = std::min(16.0, std::exp(speed_ratio));
  distance_ -= static_cast<double>(wheel_delta_degrees) * default_increment /
               8.0 * speed;
  distance_ =
      std::clamp(distance_, bounds().radius * 0.01, bounds().radius * 1000.0);
}

void ViewportModel::setView(CameraPreset view) {
  float vertical_angle = 0.0F;
  float orthogonal_angle = 0.0F;
  switch (view) {
  case CameraPreset::Top:
    orthogonal_angle = kPi * 0.5F;
    break;
  case CameraPreset::Bottom:
    vertical_angle = -kPi;
    orthogonal_angle = -kPi * 0.5F;
    break;
  case CameraPreset::Front:
    break;
  case CameraPreset::Back:
    vertical_angle = -kPi;
    break;
  case CameraPreset::Left:
    vertical_angle = kPi * 0.5F;
    break;
  case CameraPreset::Right:
    vertical_angle = -kPi * 0.5F;
    break;
  case CameraPreset::Iso1:
    vertical_angle = kPi * 0.25F;
    orthogonal_angle = kPi * 0.25F;
    break;
  case CameraPreset::Iso2:
    vertical_angle = -kPi * 0.25F;
    orthogonal_angle = -kPi * 0.25F;
    break;
  }
  camera_to_world_ =
      cloudCompareView(vertical_angle, orthogonal_angle);
  fit();
}

void ViewportModel::setStyle(ViewportStyle style) { style_ = std::move(style); }

std::shared_ptr<const ViewportCloudSnapshot> ViewportModel::cloud() const {
  return cloud_;
}

ViewportFrame ViewportModel::frame(PixelExtent physical_pixels) const {
  const double normalization =
      std::max({std::abs(distance_), bounds().radius, 1.0e-300});
  const float world_scale = static_cast<float>(
      std::max(1.0 / normalization,
               static_cast<double>(std::numeric_limits<float>::min())));
  const double normalized_distance =
      std::max(distance_ * static_cast<double>(world_scale), 1.0e-18);
  const double normalized_radius =
      bounds().radius * static_cast<double>(world_scale);
  const float near_plane = static_cast<float>(
      std::max(normalized_distance * 0.001, 1.0e-21));
  const float far_plane = static_cast<float>(
      std::max({normalized_distance + normalized_radius * 8.0,
                normalized_distance * 2.0, 1.0e-17}));
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
  ViewportFrame result;
  const Eigen::Vector3d origin = bounds().center.cast<double>();
  const Eigen::Vector3f local_target =
      ((target_ - origin) * static_cast<double>(world_scale)).cast<float>();
  const Eigen::Vector3f local_eye =
      local_target +
      camera_to_world_.col(2) * static_cast<float>(normalized_distance);
  Eigen::Matrix4f view = Eigen::Matrix4f::Identity();
  view.block<3, 3>(0, 0) = camera_to_world_.transpose();
  view.block<3, 1>(0, 3) =
      -view.block<3, 3>(0, 0) * local_eye;
  result.view_projection =
      perspective(45.0F * kPi / 180.0F, aspect, near_plane, far_plane) *
      view;
  result.world_origin = bounds().center;
  result.world_scale = world_scale;
  result.style = frame_style;
  result.guides = buildGuides(bounds(), frame_style, result.grid_spacing);
  return result;
}

const CloudBounds &ViewportModel::bounds() const {
  static const CloudBounds empty_bounds;
  return cloud_ ? cloud_->bounds : empty_bounds;
}

std::uint64_t ViewportModel::cloudRevision() const {
  return cloud_ ? cloud_->revision : 0;
}

} // namespace kpt::gui
