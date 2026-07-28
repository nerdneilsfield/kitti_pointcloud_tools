#include "gui/point_renderer.hpp"

#include "gui/viewport/pcl_adapter.hpp"

#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>

#include <stdexcept>
#include <utility>

namespace kpt::gui {
namespace {

void requireSuccess(Result<void, RendererError> result) {
  if (!result)
    throw std::runtime_error(result.error().message);
}

} // namespace

CloudBounds calculateBounds(const PointCloudIRGB &cloud) {
  auto owned = std::make_shared<PointCloudIRGB>(cloud);
  return makeViewportCloudSnapshot(owned, 1)->bounds;
}

PointRenderer::PointRenderer()
    : window_(glfwGetCurrentContext()), context_(window_), renderer_(window_) {}

PointRenderer::~PointRenderer() = default;

void PointRenderer::setCloud(const PointCloudIRGBConstPtr &cloud,
                             CameraUpdate camera_update) {
  auto snapshot = makeViewportCloudSnapshot(cloud, next_revision_++);
  model_.setCloud(snapshot, camera_update);
  requireSuccess(renderer_.upload(snapshot->vertices, snapshot->revision));
}

void PointRenderer::resize(int width, int height) {
  requireSuccess(renderer_.resize({width, height}));
}

void PointRenderer::render() {
  requireSuccess(renderer_.render(model_.frame(renderer_.extent()), context_));
}

void PointRenderer::setColorBy(ColorBy color_by) {
  style_.color_by = color_by;
  model_.setStyle(style_);
}

void PointRenderer::setPointSize(float size) {
  style_.point_size = size;
  model_.setStyle(style_);
}

void PointRenderer::setBackground(const Eigen::Vector3f &color) {
  style_.background = color;
  model_.setStyle(style_);
}

void PointRenderer::fit() { model_.fit(); }

void PointRenderer::orbit(float delta_x, float delta_y) {
  model_.orbit(delta_x, delta_y);
}

void PointRenderer::pan(float delta_x, float delta_y) {
  model_.pan(delta_x, delta_y);
}

void PointRenderer::zoom(float wheel_delta) { model_.zoom(wheel_delta); }

void PointRenderer::setView(View view) { model_.setView(view); }

} // namespace kpt::gui
