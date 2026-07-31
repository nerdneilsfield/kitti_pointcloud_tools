#include "gui/viewport/session.hpp"

#include <algorithm>
#include <stdexcept>
#include <utility>

namespace kpt::gui {

ViewportSession::ViewportSession(
    std::unique_ptr<ViewportRenderer> viewport_renderer)
    : renderer_(std::move(viewport_renderer)) {
  if (!renderer_)
    throw std::invalid_argument("ViewportSession requires a renderer");
}

std::uint64_t ViewportSession::beginRequest() {
  return ++latest_requested_revision_;
}

void ViewportSession::cancelAndClear() {
  ++latest_requested_revision_;
  model_.setCloud(nullptr);
}

bool ViewportSession::accept(
    std::shared_ptr<const ViewportCloudSnapshot> snapshot,
    CameraUpdate camera_update) {
  if (!snapshot || snapshot->revision == 0 ||
      snapshot->revision != latest_requested_revision_) {
    return false;
  }
  model_.setCloud(std::move(snapshot), camera_update);
  return true;
}

Result<std::optional<ViewportTexture>, AppError>
ViewportSession::draw(PixelExtent physical_extent, FrameContext &frame_context,
                      ViewportRole role) {
  // ImGui may report negative content-region sizes when framing/decoration
  // exceeds available space. Non-positive dimensions suspend rendering.
  physical_extent.width = std::max(0, physical_extent.width);
  physical_extent.height = std::max(0, physical_extent.height);

  const auto snapshot = model_.cloud();
  if (snapshot && snapshot->revision != uploaded_revision_) {
    auto uploaded = renderer_->upload(snapshot->vertices, snapshot->revision);
    if (!uploaded)
      return AppError{role, AppStage::Upload, uploaded.error()};
    uploaded_revision_ = snapshot->revision;
  } else if (!snapshot && uploaded_revision_ != 0) {
    auto uploaded = renderer_->upload({}, 0);
    if (!uploaded)
      return AppError{role, AppStage::Upload, uploaded.error()};
    uploaded_revision_ = 0;
  }

  auto resized = renderer_->resize(physical_extent);
  if (!resized)
    return AppError{role, AppStage::Resize, resized.error()};
  if (physical_extent.width <= 0 || physical_extent.height <= 0)
    return std::optional<ViewportTexture>{};

  const auto viewport_frame = model_.frame(physical_extent);
  grid_spacing_ = viewport_frame.grid_spacing;
  auto rendered = renderer_->render(viewport_frame, frame_context);
  if (!rendered)
    return AppError{role, AppStage::Render, rendered.error()};
  return std::optional<ViewportTexture>{renderer_->texture()};
}

} // namespace kpt::gui
