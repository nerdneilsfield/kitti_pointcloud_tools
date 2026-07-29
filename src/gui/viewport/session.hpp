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
  [[nodiscard]] std::shared_ptr<const ViewportCloudSnapshot> cloud() const {
    return model_.cloud();
  }
  [[nodiscard]] std::uint64_t cloudRevision() const {
    return model_.cloudRevision();
  }
  void setStyle(const ViewportStyle &style) { model_.setStyle(style); }
  void fit() { model_.fit(); }
  void setView(View view) { model_.setView(view); }
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
  void zoom(float wheel_delta_degrees) {
    model_.zoom(wheel_delta_degrees);
  }
  Result<std::optional<ViewportTexture>, AppError>
  draw(PixelExtent physical_extent, FrameContext &frame_context,
       ViewportRole role);

private:
  std::unique_ptr<ViewportRenderer> renderer_;
  ViewportModel model_;
  std::uint64_t uploaded_revision_ = 0;
  std::uint64_t latest_requested_revision_ = 0;
};

} // namespace kpt::gui
