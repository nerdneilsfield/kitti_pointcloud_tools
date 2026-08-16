#include "gui/viewport/model.hpp"

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <utility>
#include <vector>

namespace kpt::gui {
namespace {

constexpr std::size_t kMaximumGridLines = 243;

float niceGridStep(double span) {
  if (!std::isfinite(span) || span <= 0.0)
    return 1.0F;
  const double raw = std::max(
      span / 10.0, static_cast<double>(std::numeric_limits<float>::min()));
  const double magnitude = std::pow(10.0, std::floor(std::log10(raw)));
  if (!std::isfinite(magnitude) || magnitude <= 0.0)
    return 1.0F;
  const double normalized = raw / magnitude;
  const double multiple = normalized <= 1.0   ? 1.0
                          : normalized <= 2.0 ? 2.0
                          : normalized <= 5.0 ? 5.0
                                              : 10.0;
  return static_cast<float>(
      std::min(multiple * magnitude,
               static_cast<double>(std::numeric_limits<float>::max())));
}

void appendLine(std::vector<ViewportLineVertex> &vertices,
                const Eigen::Vector3f &from, const Eigen::Vector3f &to,
                const Eigen::Vector3f &color) {
  vertices.push_back({from, color});
  vertices.push_back({to, color});
}

void appendArrow(std::vector<ViewportLineVertex> &vertices,
                 const Eigen::Vector3f &direction,
                 const Eigen::Vector3f &perpendicular_a,
                 const Eigen::Vector3f &perpendicular_b, float length,
                 const Eigen::Vector3f &color) {
  const Eigen::Vector3f endpoint = direction * length;
  appendLine(vertices, Eigen::Vector3f::Zero(), endpoint, color);
  const float head_length = length * 0.12F;
  const float head_width = length * 0.055F;
  const Eigen::Vector3f head_base = endpoint - direction * head_length;
  appendLine(vertices, endpoint, head_base + perpendicular_a * head_width,
             color);
  appendLine(vertices, endpoint, head_base - perpendicular_a * head_width,
             color);
  appendLine(vertices, endpoint, head_base + perpendicular_b * head_width,
             color);
  appendLine(vertices, endpoint, head_base - perpendicular_b * head_width,
             color);
}

std::vector<ViewportLineVertex> buildGuides(const CloudBounds &bounds,
                                            const ViewportStyle &style,
                                            float &grid_spacing) {
  std::vector<ViewportLineVertex> guides;
  grid_spacing = 0.0F;
  if (!style.show_coordinate_axes && !style.show_scale_grid)
    return guides;

  float scene_span = 1.0F;
  float axis_span = 1.0F;
  if (bounds.finite_points != 0 && bounds.minimum.allFinite() &&
      bounds.maximum.allFinite()) {
    const Eigen::Vector3f dimensions = bounds.maximum - bounds.minimum;
    if (dimensions.allFinite()) {
      scene_span =
          std::max({std::abs(bounds.minimum.x()), std::abs(bounds.maximum.x()),
                    std::abs(bounds.minimum.y()), std::abs(bounds.maximum.y()),
                    std::abs(bounds.minimum.z()), std::abs(bounds.maximum.z()),
                    dimensions.x(), dimensions.y(), dimensions.z(),
                    std::numeric_limits<float>::min()});
      axis_span = scene_span;
    }
  }
  const float step = niceGridStep(static_cast<double>(scene_span) * 2.0);
  const double unclamped_divisions =
      std::ceil(static_cast<double>(scene_span) / static_cast<double>(step));
  const int maximum_safe_divisions = std::max(
      1, static_cast<int>(std::min(
             static_cast<double>(std::numeric_limits<int>::max()),
             std::floor(static_cast<double>(std::numeric_limits<float>::max()) /
                        static_cast<double>(step)))));
  const int divisions =
      std::clamp(static_cast<int>(std::min(
                     unclamped_divisions,
                     static_cast<double>((kMaximumGridLines - 3U) / 12U))),
                 1, std::min(20, maximum_safe_divisions));
  const float limit = static_cast<float>(divisions) * step;

  if (style.show_scale_grid && bounds.finite_points != 0) {
    const float luminance =
        style.background.dot(Eigen::Vector3f{0.2126F, 0.7152F, 0.0722F});
    const Eigen::Vector3f grid_color =
        luminance > 0.55F ? Eigen::Vector3f{0.28F, 0.32F, 0.38F}
                          : Eigen::Vector3f{0.72F, 0.78F, 0.86F};
    guides.reserve(static_cast<std::size_t>((divisions * 12 + 3) * 2 + 30));
    for (int index = -divisions; index <= divisions; ++index) {
      const float offset = static_cast<float>(index) * step;
      // XY plane.
      appendLine(guides, {-limit, offset, 0.0F}, {limit, offset, 0.0F},
                 grid_color);
      appendLine(guides, {offset, -limit, 0.0F}, {offset, limit, 0.0F},
                 grid_color);
      // XZ and YZ planes. Avoid drawing their shared center axes twice.
      if (index != 0) {
        appendLine(guides, {-limit, 0.0F, offset}, {limit, 0.0F, offset},
                   grid_color);
        appendLine(guides, {offset, 0.0F, -limit}, {offset, 0.0F, limit},
                   grid_color);
        appendLine(guides, {0.0F, -limit, offset}, {0.0F, limit, offset},
                   grid_color);
        appendLine(guides, {0.0F, offset, -limit}, {0.0F, offset, limit},
                   grid_color);
      } else {
        appendLine(guides, {0.0F, 0.0F, -limit}, {0.0F, 0.0F, limit},
                   grid_color);
      }
    }
    grid_spacing = step;
  }

  if (style.show_coordinate_axes) {
    const float axis_length = std::max(axis_span, step);
    const float luminance =
        style.background.dot(Eigen::Vector3f{0.2126F, 0.7152F, 0.0722F});
    const bool light_background = luminance > 0.55F;
    appendArrow(guides, Eigen::Vector3f::UnitX(), Eigen::Vector3f::UnitY(),
                Eigen::Vector3f::UnitZ(), axis_length,
                light_background ? Eigen::Vector3f{0.68F, 0.22F, 0.22F}
                                 : Eigen::Vector3f{1.0F, 0.61F, 0.61F});
    appendArrow(guides, Eigen::Vector3f::UnitY(), Eigen::Vector3f::UnitX(),
                Eigen::Vector3f::UnitZ(), axis_length,
                light_background ? Eigen::Vector3f{0.16F, 0.55F, 0.29F}
                                 : Eigen::Vector3f{0.61F, 0.94F, 0.73F});
    appendArrow(guides, Eigen::Vector3f::UnitZ(), Eigen::Vector3f::UnitX(),
                Eigen::Vector3f::UnitY(), axis_length,
                light_background ? Eigen::Vector3f{0.20F, 0.38F, 0.70F}
                                 : Eigen::Vector3f{0.61F, 0.76F, 1.0F});
  }
  return guides;
}

constexpr float kPi = 3.14159265358979323846F;
constexpr float kMinimumAutoFovDegrees = 35.0F;
constexpr float kMaximumAutoFovDegrees = 75.0F;
constexpr float kTargetFrameFill = 0.85F;
constexpr double kCameraDistanceMultiplier = 2.8;

Eigen::Matrix4f perspective(float fov_y, float aspect, float near_plane,
                            float far_plane) {
  const float tangent = std::tan(fov_y * 0.5F);
  Eigen::Matrix4f result = Eigen::Matrix4f::Zero();
  result(0, 0) = 1.0F / (aspect * tangent);
  result(1, 1) = 1.0F / tangent;
  const double near_double = static_cast<double>(near_plane);
  const double far_double = static_cast<double>(far_plane);
  result(2, 2) = static_cast<float>(-(far_double + near_double) /
                                    (far_double - near_double));
  result(2, 3) = static_cast<float>(-(2.0 * far_double * near_double) /
                                    (far_double - near_double));
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
  const float radius_squared = point.x() * point.x() + point.y() * point.y();
  if (radius_squared > 1.0F) {
    point.head<2>().normalize();
  } else {
    point.z() = std::sqrt(1.0F - radius_squared);
  }
  return point;
}

Eigen::Matrix3f cloudCompareView(float vertical_angle, float orthogonal_angle) {
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

bool hasPortableCameraSnapshotMagnitude(
    const CameraSnapshot &snapshot) noexcept {
  const auto within_limit = [](double value) noexcept {
    return std::isfinite(value) &&
           std::abs(value) <= kCameraSnapshotMaximumMagnitude;
  };
  for (Eigen::Index axis = 0; axis < snapshot.target.size(); ++axis) {
    if (!within_limit(snapshot.target[axis]) ||
        !within_limit(snapshot.rotation_center[axis])) {
      return false;
    }
  }
  for (Eigen::Index row = 0; row < snapshot.camera_to_world.rows(); ++row) {
    for (Eigen::Index column = 0; column < snapshot.camera_to_world.cols();
         ++column) {
      if (!within_limit(static_cast<double>(snapshot.camera_to_world(row, column))))
        return false;
    }
  }
  return within_limit(snapshot.distance) && snapshot.distance > 0.0;
}

bool hasPortableCameraSnapshotFov(float fov_y_degrees) noexcept {
  return std::isfinite(fov_y_degrees) &&
         fov_y_degrees >= kCameraSnapshotMinimumFovDegrees &&
         fov_y_degrees < kCameraSnapshotMaximumFovDegrees;
}

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
  } else {
    fit_pending_ = false;
  }
}

void ViewportModel::fit() {
  fit_pending_ = true;
}

void ViewportModel::applyFit(PixelExtent physical_pixels) {
  fit_pending_ = false;
  target_ = bounds().centroid.cast<double>();
  rotation_center_ = target_;
  if (!cloud_ || cloud_->vertices.empty()) {
    distance_ = 10.0;
    fov_y_degrees_ = 45.0F;
    return;
  }

  const auto &candidates = cloud_->picking_vertices.empty()
                               ? cloud_->vertices
                               : cloud_->picking_vertices;
  std::vector<double> radii;
  radii.reserve(candidates.size());
  for (const auto &vertex : candidates) {
    if (vertex.position.allFinite())
      radii.push_back((vertex.position.cast<double>() - target_).norm());
  }
  if (radii.empty()) {
    distance_ = 0.01;
    fov_y_degrees_ = 45.0F;
    return;
  }
  const std::size_t percentile_index =
      (radii.size() - 1U) * 95U / 100U;
  std::nth_element(radii.begin(), radii.begin() +
                                      static_cast<std::ptrdiff_t>(percentile_index),
                   radii.end());
  // Synthetic/manual snapshots may only carry a single representative vertex
  // while retaining wider bounds. In that degenerate case bounds.radius is the
  // only available scale; real snapshots use their sampled point distribution.
  const double robust_radius = radii.size() > 1U
                                   ? std::max(radii[percentile_index], 0.001)
                                   : std::max(bounds().radius, 0.001);
  distance_ = std::max(robust_radius * kCameraDistanceMultiplier, 0.01);

  const float aspect = physical_pixels.width > 0 && physical_pixels.height > 0
                           ? static_cast<float>(physical_pixels.width) /
                                 static_cast<float>(physical_pixels.height)
                           : 1.0F;
  std::vector<double> slopes;
  slopes.reserve(candidates.size());
  for (const auto &vertex : candidates) {
    if (!vertex.position.allFinite())
      continue;
    const Eigen::Vector3d offset = vertex.position.cast<double>() - target_;
    const double depth = distance_ -
                         camera_to_world_.col(2).cast<double>().dot(offset);
    if (!(depth > 1.0e-9))
      continue;
    const double x = std::abs(camera_to_world_.col(0).cast<double>().dot(offset));
    const double y = std::abs(camera_to_world_.col(1).cast<double>().dot(offset));
    slopes.push_back(std::max(x / (depth * static_cast<double>(aspect)),
                              y / depth));
  }
  if (slopes.empty()) {
    fov_y_degrees_ = 45.0F;
    return;
  }
  const std::size_t slope_index =
      (slopes.size() - 1U) * 95U / 100U;
  std::nth_element(slopes.begin(), slopes.begin() +
                                       static_cast<std::ptrdiff_t>(slope_index),
                   slopes.end());
  const double fov = 2.0 * std::atan(slopes[slope_index] /
                                      static_cast<double>(kTargetFrameFill)) *
                     180.0 / static_cast<double>(kPi);
  fov_y_degrees_ = static_cast<float>(std::clamp(
      std::isfinite(fov) ? fov : 45.0,
      static_cast<double>(kMinimumAutoFovDegrees),
      static_cast<double>(kMaximumAutoFovDegrees)));
}

void ViewportModel::orbit(float previous_x, float previous_y, float current_x,
                          float current_y, PixelExtent viewport) {
  fit_pending_ = false;
  if (viewport.width <= 0 || viewport.height <= 0)
    return;
  const Eigen::Vector3f previous =
      trackballPoint(previous_x, previous_y, viewport);
  const Eigen::Vector3f current =
      trackballPoint(current_x, current_y, viewport);
  const Eigen::Matrix3f previous_camera_to_world = camera_to_world_;
  const Eigen::Vector3d previous_eye =
      target_ + previous_camera_to_world.col(2).cast<double>() * distance_;
  const Eigen::Quaternionf camera_rotation =
      Eigen::Quaternionf::FromTwoVectors(previous, current);
  camera_to_world_ =
      camera_to_world_ * camera_rotation.toRotationMatrix().transpose();
  camera_to_world_ =
      Eigen::Quaternionf(camera_to_world_).normalized().toRotationMatrix();
  const Eigen::Matrix3d world_rotation =
      (camera_to_world_ * previous_camera_to_world.transpose()).cast<double>();
  const Eigen::Vector3d eye =
      rotation_center_ + world_rotation * (previous_eye - rotation_center_);
  target_ = eye - camera_to_world_.col(2).cast<double>() * distance_;
}

void ViewportModel::roll(float delta_x, PixelExtent viewport) {
  fit_pending_ = false;
  if (viewport.width <= 0)
    return;
  const Eigen::Matrix3f previous_camera_to_world = camera_to_world_;
  const Eigen::Vector3d previous_eye =
      target_ + previous_camera_to_world.col(2).cast<double>() * distance_;
  const float angle = 2.0F * kPi * delta_x / static_cast<float>(viewport.width);
  const Eigen::AngleAxisf camera_rotation(angle, Eigen::Vector3f::UnitZ());
  camera_to_world_ =
      camera_to_world_ * camera_rotation.toRotationMatrix().transpose();
  const Eigen::Matrix3d world_rotation =
      (camera_to_world_ * previous_camera_to_world.transpose()).cast<double>();
  const Eigen::Vector3d eye =
      rotation_center_ + world_rotation * (previous_eye - rotation_center_);
  target_ = eye - camera_to_world_.col(2).cast<double>() * distance_;
}

void ViewportModel::pan(float delta_x, float delta_y, PixelExtent viewport) {
  fit_pending_ = false;
  if (viewport.height <= 0)
    return;
  const float fov_y = fov_y_degrees_ * kPi / 180.0F;
  const double pixel_size = 2.0 * distance_ *
                            static_cast<double>(std::tan(fov_y * 0.5F)) /
                            static_cast<double>(viewport.height);
  target_ += camera_to_world_.col(0).cast<double>() *
                 (-static_cast<double>(delta_x) * pixel_size) +
             camera_to_world_.col(1).cast<double>() *
                 (static_cast<double>(delta_y) * pixel_size);
}

void ViewportModel::zoom(float wheel_delta_degrees) {
  fit_pending_ = false;
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

std::optional<PickResult>
ViewportModel::pickCloudFromScreen(float x, float y, PixelExtent viewport) {
  if (!cloud_ || cloud_->vertices.empty() || viewport.width <= 0 ||
      viewport.height <= 0 || !std::isfinite(x) || !std::isfinite(y)) {
    return std::nullopt;
  }

  const ViewportFrame current_frame = frame(viewport);
  constexpr float pick_radius_pixels = 8.0F;
  float best_distance_squared = pick_radius_pixels * pick_radius_pixels;
  float best_depth = std::numeric_limits<float>::infinity();
  const ViewportVertex *picked = nullptr;
  const auto &candidates = cloud_->picking_vertices.empty()
                               ? cloud_->vertices
                               : cloud_->picking_vertices;
  for (const auto &vertex : candidates) {
    if (!vertex.position.allFinite())
      continue;
    const Eigen::Vector3f local =
        (vertex.position - current_frame.world_origin) *
        current_frame.world_scale;
    const Eigen::Vector4f clip =
        current_frame.view_projection *
        Eigen::Vector4f(local.x(), local.y(), local.z(), 1.0F);
    if (!clip.allFinite() || clip.w() <= 0.0F)
      continue;
    const Eigen::Vector3f ndc = clip.head<3>() / clip.w();
    if (!ndc.allFinite() || ndc.z() < -1.0F || ndc.z() > 1.0F)
      continue;
    const float screen_x =
        (ndc.x() * 0.5F + 0.5F) * static_cast<float>(viewport.width);
    const float screen_y =
        (0.5F - ndc.y() * 0.5F) * static_cast<float>(viewport.height);
    const float dx = screen_x - x;
    const float dy = screen_y - y;
    const float distance_squared = dx * dx + dy * dy;
    if (distance_squared < best_distance_squared ||
        (distance_squared == best_distance_squared && ndc.z() < best_depth)) {
      best_distance_squared = distance_squared;
      best_depth = ndc.z();
      picked = &vertex;
    }
  }
  if (!picked)
    return std::nullopt;
  return PickResult{picked->position, picked->intensity, picked->noise,
                    picked->position};
}

std::optional<PickResult>
ViewportModel::pickFromScreen(float x, float y, PixelExtent viewport) {
  return pickCloudFromScreen(x, y, viewport);
}

CameraSnapshot ViewportModel::cameraSnapshot() const {
  return {target_, rotation_center_, camera_to_world_, distance_,
          fov_y_degrees_};
}

bool ViewportModel::setCameraSnapshot(const CameraSnapshot &snapshot) {
  constexpr float kOrthonormalTolerance = 1.0e-4F;
  const Eigen::Matrix3f identity = Eigen::Matrix3f::Identity();
  if (!hasPortableCameraSnapshotMagnitude(snapshot) ||
      !hasPortableCameraSnapshotFov(snapshot.fov_y_degrees) ||
      !(snapshot.camera_to_world.transpose() * snapshot.camera_to_world)
           .isApprox(identity, kOrthonormalTolerance) ||
      snapshot.camera_to_world.determinant() <= 0.0F) {
    return false;
  }

  // Finiteness of the fields alone is insufficient: values such as DBL_MAX
  // overflow when frame() normalizes the camera into renderer float space.
  // Validate a complete candidate first, so a rejected restore is atomic.
  ViewportModel candidate = *this;
  candidate.target_ = snapshot.target;
  candidate.rotation_center_ = snapshot.rotation_center;
  candidate.camera_to_world_ = snapshot.camera_to_world;
  candidate.distance_ = snapshot.distance;
  candidate.fov_y_degrees_ = snapshot.fov_y_degrees;
  candidate.fit_pending_ = false;
  const auto is_renderable = [](ViewportModel &model) {
    const CameraSnapshot state = model.cameraSnapshot();
    if (!state.target.allFinite() || !state.rotation_center.allFinite() ||
        !state.camera_to_world.allFinite() || !std::isfinite(state.distance) ||
        !std::isfinite(state.fov_y_degrees)) {
      return false;
    }
    const ViewportFrame frame = model.frame({1, 1});
    return frame.view_projection.allFinite() && frame.world_origin.allFinite() &&
           std::isfinite(frame.world_scale) && frame.world_scale > 0.0F &&
           std::isfinite(frame.fov_y_degrees) &&
           std::all_of(frame.guides.begin(), frame.guides.end(),
                       [](const ViewportLineVertex &vertex) {
                         return vertex.position.allFinite() &&
                                vertex.color.allFinite();
                       });
  };
  if (!is_renderable(candidate))
    return false;

  // rotation_center is not consumed by frame(), but is used in subsequent
  // orbit/roll operations. Probe both transformations before committing: a
  // finite giant pivot can otherwise overflow their rotate-around-pivot math.
  ViewportModel orbit_candidate = candidate;
  orbit_candidate.orbit(0.0F, 0.0F, 1.0F, 0.0F, {2, 2});
  ViewportModel roll_candidate = candidate;
  roll_candidate.roll(0.5F, {2, 2});
  if (!is_renderable(orbit_candidate) || !is_renderable(roll_candidate))
    return false;

  target_ = snapshot.target;
  rotation_center_ = snapshot.rotation_center;
  camera_to_world_ = snapshot.camera_to_world;
  distance_ = snapshot.distance;
  fov_y_degrees_ = snapshot.fov_y_degrees;
  fit_pending_ = false;
  return true;
}

std::optional<Eigen::Vector3f>
ViewportModel::pointFromScreen(float x, float y, PixelExtent viewport) {
  const auto picked = pickFromScreen(x, y, viewport);
  return picked ? std::optional<Eigen::Vector3f>{picked->cloud_position}
                : std::nullopt;
}

bool ViewportModel::setRotationCenterFromScreen(float x, float y,
                                                PixelExtent viewport) {
  const auto picked = pickFromScreen(x, y, viewport);
  if (!picked)
    return false;
  rotation_center_ = picked->cloud_position.cast<double>();
  return true;
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
  camera_to_world_ = cloudCompareView(vertical_angle, orthogonal_angle);
  fit();
}

void ViewportModel::setStyle(ViewportStyle style) { style_ = std::move(style); }

std::shared_ptr<const ViewportCloudSnapshot> ViewportModel::cloud() const {
  return cloud_;
}

ViewportFrame ViewportModel::frame(PixelExtent physical_pixels) {
  if (fit_pending_ && physical_pixels.width > 0 && physical_pixels.height > 0)
    applyFit(physical_pixels);
  const double normalization =
      std::max({std::abs(distance_), bounds().radius, 1.0e-300});
  const float world_scale = static_cast<float>(
      std::max(1.0 / normalization,
               static_cast<double>(std::numeric_limits<float>::min())));
  const double normalized_distance =
      std::max(distance_ * static_cast<double>(world_scale), 1.0e-18);
  const double normalized_radius =
      bounds().radius * static_cast<double>(world_scale);
  const float near_plane =
      static_cast<float>(std::max(normalized_distance * 0.001, 1.0e-21));
  const float far_plane = static_cast<float>(
      std::max({normalized_distance + normalized_radius * 8.0,
                normalized_distance * 2.0, 1.0e-17}));
  ViewportStyle frame_style = style_;
  bool equalize_active = false;
  if (frame_style.color_by == ColorBy::Intensity) {
    if (frame_style.intensity_equalize && bounds().intensity_cdf_valid) {
      equalize_active = true;
      frame_style.scalar_min = bounds().intensity_min;
      frame_style.scalar_max = bounds().intensity_max;
    } else {
      frame_style.scalar_min = bounds().intensity_p05;
      frame_style.scalar_max = bounds().intensity_p90;
    }
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
  view.block<3, 1>(0, 3) = -view.block<3, 3>(0, 0) * local_eye;
  result.view_projection =
      perspective(fov_y_degrees_ * kPi / 180.0F, aspect, near_plane, far_plane) * view;
  result.world_origin = bounds().center;
  result.world_scale = world_scale;
  result.fov_y_degrees = fov_y_degrees_;
  result.style = frame_style;
  result.intensity_cdf_valid = equalize_active;
  if (equalize_active)
    result.intensity_cdf = bounds().intensity_cdf;
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
