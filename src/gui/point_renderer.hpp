#pragma once

#include "kpt/types.hpp"

#include <Eigen/Core>

#include <cstddef>

namespace kpt::gui {

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

CloudBounds calculateBounds(const PointCloudIRGB &cloud);

class PointRenderer {
public:
  PointRenderer();
  ~PointRenderer();
  PointRenderer(const PointRenderer &) = delete;
  PointRenderer &operator=(const PointRenderer &) = delete;

  void setCloud(const PointCloudIRGBConstPtr &cloud);
  void resize(int width, int height);
  void render();

  [[nodiscard]] unsigned texture() const { return color_texture_; }
  [[nodiscard]] int width() const { return width_; }
  [[nodiscard]] int height() const { return height_; }
  [[nodiscard]] std::size_t pointCount() const { return point_count_; }
  [[nodiscard]] const CloudBounds &bounds() const { return bounds_; }
  [[nodiscard]] bool centerPixelVisible() const;

  void setColorBy(ColorBy color_by) { color_by_ = color_by; }
  void setPointSize(float size) { point_size_ = size; }
  void setBackground(const Eigen::Vector3f &color) { background_ = color; }

  void fit();
  void orbit(float delta_x, float delta_y);
  void pan(float delta_x, float delta_y);
  void zoom(float wheel_delta);
  void setView(View view);

private:
  struct Vertex;
  void createResources();
  void destroyResources();
  void createFramebuffer();
  Eigen::Matrix4f viewProjection() const;

  unsigned vao_ = 0;
  unsigned vbo_ = 0;
  unsigned program_ = 0;
  unsigned framebuffer_ = 0;
  unsigned color_texture_ = 0;
  unsigned depth_buffer_ = 0;
  int width_ = 1;
  int height_ = 1;
  std::size_t point_count_ = 0;
  CloudBounds bounds_;

  Eigen::Vector3f target_ = Eigen::Vector3f::Zero();
  float yaw_ = 0.75F;
  float pitch_ = 0.45F;
  float distance_ = 10.0F;
  float point_size_ = 3.0F;
  Eigen::Vector3f background_ = Eigen::Vector3f::Zero();
  ColorBy color_by_ = ColorBy::Intensity;
};

} // namespace kpt::gui
