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
  Result<std::optional<ViewportTexture>, AppError>
  draw(PixelExtent physical_extent, FrameContext &frame_context,
       ViewportRole role);

  ViewportModel model;

private:
  std::unique_ptr<ViewportRenderer> renderer_;
  std::uint64_t uploaded_revision_ = 0;
  std::uint64_t latest_requested_revision_ = 0;
};

} // namespace kpt::gui
