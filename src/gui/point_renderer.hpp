#pragma once

#include "gui/backend/opengl/point_renderer.hpp"
#include "gui/viewport/model.hpp"
#include "kpt/types.hpp"

namespace kpt::gui {

CloudBounds calculateBounds(const PointCloudIRGB &cloud);

// Migration-only facade. Task 11 removes it after App owns model/renderer
// sessions directly; new behavior belongs in those independent types.
class PointRenderer {
public:
  PointRenderer();
  ~PointRenderer();
  PointRenderer(const PointRenderer &) = delete;
  PointRenderer &operator=(const PointRenderer &) = delete;

  void setCloud(const PointCloudIRGBConstPtr &cloud,
                CameraUpdate camera_update = CameraUpdate::Fit);
  void resize(int width, int height);
  void render();

  [[nodiscard]] ViewportTexture texture() const { return renderer_.texture(); }
  [[nodiscard]] int width() const { return renderer_.extent().width; }
  [[nodiscard]] int height() const { return renderer_.extent().height; }
  [[nodiscard]] std::size_t pointCount() const {
    return renderer_.pointCount();
  }
  [[nodiscard]] const CloudBounds &bounds() const { return model_.bounds(); }

  void setColorBy(ColorBy color_by);
  void setPointSize(float size);
  void setBackground(const Eigen::Vector3f &color);

  void fit();
  void orbit(float delta_x, float delta_y);
  void pan(float delta_x, float delta_y);
  void zoom(float wheel_delta);
  void setView(View view);

private:
  GLFWwindow *window_ = nullptr;
  OpenGLFrameContext context_;
  OpenGLPointRenderer renderer_;
  ViewportModel model_;
  ViewportStyle style_;
  std::uint64_t next_revision_ = 1;
};

} // namespace kpt::gui
