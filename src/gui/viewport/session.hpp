#pragma once

#include "common/result.hpp"
#include "gui/viewport/model.hpp"
#include "gui/viewport/renderer.hpp"

#include <cstdint>
#include <memory>
#include <optional>

namespace kpt::gui {

enum class ViewportRole { Main, Trajectory };
enum class AppStage { Upload, Resize, Render };

struct AppError {
  ViewportRole role = ViewportRole::Main;
  AppStage stage = AppStage::Render;
  RendererError cause;
};

class ViewportSession {
public:
  explicit ViewportSession(std::unique_ptr<ViewportRenderer> renderer);

  [[nodiscard]] std::uint64_t beginRequest();
  void cancelAndClear();
  [[nodiscard]] bool
  accept(std::shared_ptr<const ViewportCloudSnapshot> snapshot,
         CameraUpdate camera_update = CameraUpdate::Fit);
  // Accept a review-scene snapshot without flattening its layers.  Its
  // camera_cloud must carry the same monotonic revision as the wrapper so the
  // existing camera model continues to reject stale async results.
  [[nodiscard]] bool
  acceptLayered(std::shared_ptr<const LayeredViewportSnapshot> snapshot,
                CameraUpdate camera_update = CameraUpdate::Fit);
  [[nodiscard]] std::shared_ptr<const ViewportCloudSnapshot> cloud() const {
    return model_.cloud();
  }
  [[nodiscard]] std::uint64_t cloudRevision() const {
    return model_.cloudRevision();
  }
  [[nodiscard]] float gridSpacing() const { return grid_spacing_; }
  void setStyle(const ViewportStyle &style) { model_.setStyle(style); }
  void fit() { model_.fit(); }
  void setView(CameraPreset view) { model_.setView(view); }
  void orbit(float previous_x, float previous_y, float current_x,
             float current_y, PixelExtent viewport) {
    model_.orbit(previous_x, previous_y, current_x, current_y, viewport);
  }
  void roll(float delta_x, PixelExtent viewport) {
    model_.roll(delta_x, viewport);
  }
  void pan(float delta_x, float delta_y, PixelExtent viewport) {
    model_.pan(delta_x, delta_y, viewport);
  }
  void zoom(float wheel_delta_degrees) { model_.zoom(wheel_delta_degrees); }
  [[nodiscard]] std::optional<PickResult>
  pickCloudFromScreen(float x, float y, PixelExtent viewport) {
    return model_.pickCloudFromScreen(x, y, viewport);
  }
  // Compatibility API for this session's one viewport cloud. Scene-level
  // picking and local-to-world transforms remain outside ViewportSession.
  [[nodiscard]] std::optional<PickResult>
  pickFromScreen(float x, float y, PixelExtent viewport) {
    return model_.pickFromScreen(x, y, viewport);
  }
  [[nodiscard]] CameraSnapshot cameraSnapshot() const {
    return model_.cameraSnapshot();
  }
  [[nodiscard]] bool setCameraSnapshot(const CameraSnapshot &snapshot) {
    return model_.setCameraSnapshot(snapshot);
  }
  [[nodiscard]] ViewportFrame frameForPicking(PixelExtent viewport) {
    return model_.frame(viewport);
  }
  // Calculates a fit camera for an alternate immutable snapshot without
  // replacing this session's rendered cloud.  Review Scenes use it for
  // "Fit active" while continuing to draw every visible layer.
  [[nodiscard]] std::optional<CameraSnapshot>
  fitCameraFor(std::shared_ptr<const ViewportCloudSnapshot> snapshot,
               PixelExtent viewport) const;
  [[nodiscard]] std::optional<Eigen::Vector3f>
  pointFromScreen(float x, float y, PixelExtent viewport) {
    return model_.pointFromScreen(x, y, viewport);
  }
  [[nodiscard]] bool setRotationCenterFromScreen(float x, float y,
                                                 PixelExtent viewport) {
    return model_.setRotationCenterFromScreen(x, y, viewport);
  }
  Result<std::optional<ViewportTexture>, AppError>
  draw(PixelExtent physical_extent, FrameContext &frame_context,
       ViewportRole role, bool interactive_lod = false);

private:
  std::unique_ptr<ViewportRenderer> renderer_;
  ViewportModel model_;
  std::shared_ptr<const LayeredViewportSnapshot> layered_snapshot_;
  std::uint64_t uploaded_revision_ = 0;
  std::uint64_t uploaded_layered_revision_ = 0;
  bool rendered_layered_ = false;
  std::uint64_t latest_requested_revision_ = 0;
  float grid_spacing_ = 0.0F;
};

} // namespace kpt::gui
