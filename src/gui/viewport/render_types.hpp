#pragma once

#include "kpt/core_types.hpp"

#include <Eigen/Core>
#include <imgui.h>

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace kpt::gui {

struct ViewportVertex {
  Eigen::Vector3f position = Eigen::Vector3f::Zero();
  Eigen::Vector3f color = Eigen::Vector3f::Zero();
  float intensity = 0.0F;
};

struct CloudBounds {
  Eigen::Vector3f minimum = Eigen::Vector3f::Zero();
  Eigen::Vector3f maximum = Eigen::Vector3f::Zero();
  Eigen::Vector3f center = Eigen::Vector3f::Zero();
  float radius = 1.0F;
  float intensity_min = 0.0F;
  float intensity_max = 1.0F;
  float z_min = 0.0F;
  float z_max = 1.0F;
  std::size_t finite_points = 0;
};

struct ViewportStyle {
  ColorBy color_by = ColorBy::Intensity;
  float point_size = 3.0F;
  Eigen::Vector3f background = Eigen::Vector3f::Zero();
  float scalar_min = 0.0F;
  float scalar_max = 1.0F;
};

struct ViewportFrame {
  Eigen::Matrix4f view_projection = Eigen::Matrix4f::Identity();
  ViewportStyle style;
};

struct ViewportCloudSnapshot {
  std::vector<ViewportVertex> vertices;
  CloudBounds bounds;
  std::uint64_t revision = 0;
};

enum class CameraUpdate { Fit, Preserve };

enum class CameraPreset {
  Top,
  Bottom,
  Front,
  Back,
  Left,
  Right,
  Iso1,
  Iso2
};

struct PixelExtent {
  int width = 0;
  int height = 0;

  friend bool operator==(const PixelExtent &, const PixelExtent &) = default;
};

struct FramebufferMetrics {
  ImVec2 logical_size;
  PixelExtent framebuffer_size;
  ImVec2 scale{1.0F, 1.0F};
};

struct ViewportTexture {
  ImTextureRef ref;
  ImVec2 uv0{0.0F, 0.0F};
  ImVec2 uv1{1.0F, 1.0F};
};

enum class BackendKind { OpenGL, Metal };

enum class RendererErrorCode {
  BackendMismatch,
  ResourceCreationFailed,
  EncodingFailed
};

struct RendererError {
  RendererErrorCode code = RendererErrorCode::EncodingFailed;
  std::string message;
};

} // namespace kpt::gui
