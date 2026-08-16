#include "gui/viewport/session.hpp"

#include <algorithm>
#include <stdexcept>
#include <utility>
#include <vector>

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
  layered_snapshot_.reset();
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
  layered_snapshot_.reset();
  return true;
}

bool ViewportSession::acceptLayered(
    std::shared_ptr<const LayeredViewportSnapshot> snapshot,
    CameraUpdate camera_update) {
  if (!snapshot || snapshot->revision == 0 || !snapshot->camera_cloud ||
      snapshot->camera_cloud->revision != snapshot->revision ||
      snapshot->revision != latest_requested_revision_) {
    return false;
  }
  model_.setCloud(snapshot->camera_cloud, camera_update);
  if (model_.cloudRevision() != snapshot->revision) {
    return false;
  }
  layered_snapshot_ = std::move(snapshot);
  return true;
}

std::optional<CameraSnapshot> ViewportSession::fitCameraFor(
    std::shared_ptr<const ViewportCloudSnapshot> snapshot,
    PixelExtent viewport) const {
  if (!snapshot || snapshot->revision == 0)
    return std::nullopt;
  viewport.width = std::max(1, viewport.width);
  viewport.height = std::max(1, viewport.height);

  ViewportModel probe;
  probe.setCloud(std::move(snapshot), CameraUpdate::Preserve);
  if (!probe.setCameraSnapshot(model_.cameraSnapshot()))
    return std::nullopt;
  probe.fit();
  static_cast<void>(probe.frame(viewport));
  return probe.cameraSnapshot();
}

Result<std::optional<ViewportTexture>, AppError>
ViewportSession::draw(PixelExtent physical_extent, FrameContext &frame_context,
                      ViewportRole role, bool interactive_lod) {
  // ImGui may report negative content-region sizes when framing/decoration
  // exceeds available space. Non-positive dimensions suspend rendering.
  physical_extent.width = std::max(0, physical_extent.width);
  physical_extent.height = std::max(0, physical_extent.height);

  const auto snapshot = model_.cloud();
  const auto layered = layered_snapshot_;
  if (layered) {
    std::vector<ViewportLayerUpload> uploads;
    uploads.reserve(layered->opaque_layers.size() +
                    layered->transparent_layers.size());
    const auto append_uploads = [&uploads](
                                    const std::vector<ViewportLayerSnapshot>
                                        &layers) {
      for (const auto &layer : layers) {
        uploads.push_back({layer.draw.layer_id, layer.revision, layer.vertices});
      }
    };
    append_uploads(layered->opaque_layers);
    append_uploads(layered->transparent_layers);

    if (!rendered_layered_ ||
        layered->revision != uploaded_layered_revision_) {
      auto uploaded = renderer_->uploadLayers(uploads, layered->revision);
      if (!uploaded)
        return AppError{role, AppStage::Upload, uploaded.error()};
      uploaded_layered_revision_ = layered->revision;
      rendered_layered_ = true;
    }
  } else {
    if (rendered_layered_) {
      auto cleared = renderer_->uploadLayers({}, 0);
      if (!cleared)
        return AppError{role, AppStage::Upload, cleared.error()};
      uploaded_layered_revision_ = 0;
      rendered_layered_ = false;
    }
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
  }

  auto resized = renderer_->resize(physical_extent);
  if (!resized)
    return AppError{role, AppStage::Resize, resized.error()};
  if (physical_extent.width <= 0 || physical_extent.height <= 0)
    return std::optional<ViewportTexture>{};

  auto viewport_frame = model_.frame(physical_extent);
  viewport_frame.interactive_lod = interactive_lod;
  grid_spacing_ = viewport_frame.grid_spacing;
  Result<void, RendererError> rendered;
  if (layered) {
    std::vector<ViewportLayerDraw> opaque_draws;
    std::vector<ViewportLayerDraw> transparent_draws;
    opaque_draws.reserve(layered->opaque_layers.size());
    transparent_draws.reserve(layered->transparent_layers.size());
    for (const auto &layer : layered->opaque_layers)
      opaque_draws.push_back(layer.draw);
    for (const auto &layer : layered->transparent_layers)
      transparent_draws.push_back(layer.draw);
    rendered = renderer_->renderLayers(
        viewport_frame,
        {layered->revision, opaque_draws, transparent_draws}, frame_context);
  } else {
    rendered = renderer_->render(viewport_frame, frame_context);
  }
  if (!rendered)
    return AppError{role, AppStage::Render, rendered.error()};
  return std::optional<ViewportTexture>{renderer_->texture()};
}

} // namespace kpt::gui
