#pragma once

#include "kpt/core_types.hpp"

#include <Eigen/Core>
#include <imgui.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>
#include <string>
#include <vector>

namespace kpt::gui {

struct ViewportVertex {
  Eigen::Vector3f position = Eigen::Vector3f::Zero();
  Eigen::Vector3f color = Eigen::Vector3f::Zero();
  float intensity = 0.0F;
  float noise = 0.0F;
};

struct ViewportLineVertex {
  Eigen::Vector3f position = Eigen::Vector3f::Zero();
  Eigen::Vector3f color = Eigen::Vector3f::Ones();
};

struct CloudBounds {
  Eigen::Vector3f minimum = Eigen::Vector3f::Zero();
  Eigen::Vector3f maximum = Eigen::Vector3f::Zero();
  // Arithmetic mean of all finite point positions. Unlike center, this is not
  // derived from the AABB and is used as the interactive camera target.
  Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
  Eigen::Vector3f center = Eigen::Vector3f::Zero();
  double radius = 1.0;
  float intensity_min = 0.0F;
  float intensity_max = 1.0F;
  float intensity_p05 = 0.0F;
  float intensity_p90 = 1.0F;
  float z_min = 0.0F;
  float z_max = 1.0F;
  std::array<float, 256> intensity_cdf{};
  bool intensity_cdf_valid = false;
  std::size_t finite_points = 0;
  std::size_t noise_points = 0;
  bool has_noise = false;
};

enum class ColorMap {
  Turbo,
  Viridis,
  Plasma,
  Inferno,
  Magma,
  Grayscale,
  Hot,
  Jet,
  Spring,
  Autumn
};

struct ViewportStyle {
  ColorBy color_by = ColorBy::Intensity;
  ColorMap color_map = ColorMap::Turbo;
  float point_size = 3.0F;
  Eigen::Vector3f background = Eigen::Vector3f::Zero();
  float scalar_min = 0.0F;
  float scalar_max = 1.0F;
  Eigen::Vector3f fixed_color = Eigen::Vector3f::Ones();
  Eigen::Vector3f noise_color = Eigen::Vector3f{1.0F, 0.0F, 0.0F};
  bool highlight_noise = true;
  bool intensity_equalize = true;
  bool show_coordinate_axes = false;
  bool show_scale_grid = false;
};

struct ViewportFrame {
  Eigen::Matrix4f view_projection = Eigen::Matrix4f::Identity();
  Eigen::Vector3f world_origin = Eigen::Vector3f::Zero();
  float world_scale = 1.0F;
  float fov_y_degrees = 45.0F;
  ViewportStyle style;
  std::array<float, 256> intensity_cdf{};
  bool intensity_cdf_valid = false;
  std::vector<ViewportLineVertex> guides;
  float grid_spacing = 0.0F;
  bool interactive_lod = false;
};

struct ViewportCloudSnapshot {
  std::vector<ViewportVertex> vertices;
  // Bounded, evenly sampled picking candidates keep middle-click latency
  // independent of source cloud size.
  std::vector<ViewportVertex> picking_vertices;
  CloudBounds bounds;
  std::uint64_t revision = 0;
};

// One immutable draw/upload payload in a layered review viewport.  Positions
// are already in the scene's world coordinate system; the backend applies the
// common ViewportFrame rebasing transform.  Keeping alpha separate from
// vertex colour is deliberate: transparent layers must be blended against the
// framebuffer contents, never pre-composited against a guessed background.
struct ViewportLayerDraw {
  std::uint64_t layer_id = 0;
  ViewportStyle style;
  std::array<float, 256> intensity_cdf{};
  bool intensity_cdf_valid = false;
  float opacity = 1.0F;
};

struct ViewportLayerSnapshot {
  ViewportLayerDraw draw;
  std::vector<ViewportVertex> vertices;
  // Must change whenever vertices or draw state changes.  Backends use this
  // to retain layer-local GPU buffers without making source-layer IDs part of
  // a transient scene revision.
  std::uint64_t revision = 0;
};

// Owns a bounded real-point camera-fit sample plus draw payloads for one
// review Scene.  `camera_cloud` deliberately is not a flattened copy of layer
// vertices: it carries at most the compositor fit-sample cap and exact scene
// bounds. The opaque and transparent arrays are already ordered; renderer
// implementations must draw every opaque layer first, then transparent layers
// in this exact back-to-front order with depth writes disabled.
struct LayeredViewportSnapshot {
  std::shared_ptr<const ViewportCloudSnapshot> camera_cloud;
  std::vector<ViewportLayerSnapshot> opaque_layers;
  std::vector<ViewportLayerSnapshot> transparent_layers;
  std::uint64_t revision = 0;
};

// Non-owning frame arguments passed to a backend.  These make the persistent
// GPU layer cache explicit while retaining a simple single-cloud API for
// established callers.
struct ViewportLayerUpload {
  std::uint64_t layer_id = 0;
  std::uint64_t revision = 0;
  std::span<const ViewportVertex> vertices;
};

struct LayeredViewportFrame {
  std::uint64_t revision = 0;
  std::span<const ViewportLayerDraw> opaque_layers;
  std::span<const ViewportLayerDraw> transparent_layers;
};

enum class CameraUpdate { Fit, Preserve };

enum class CameraPreset { Top, Bottom, Front, Back, Left, Right, Iso1, Iso2 };

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

enum class BackendKind { OpenGL, Metal, WebGL };

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
