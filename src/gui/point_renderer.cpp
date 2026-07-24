#include "gui/point_renderer.hpp"

#define GL_GLEXT_PROTOTYPES
#include <GL/glcorearb.h>

#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace kpt::gui {
namespace {

constexpr float kPi = 3.14159265358979323846F;

unsigned compileShader(unsigned type, const char *source) {
  const unsigned shader = glCreateShader(type);
  glShaderSource(shader, 1, &source, nullptr);
  glCompileShader(shader);
  int success = 0;
  glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
  if (success != 0)
    return shader;

  std::array<char, 2048> log{};
  glGetShaderInfoLog(shader, static_cast<int>(log.size()), nullptr, log.data());
  glDeleteShader(shader);
  throw std::runtime_error("OpenGL shader compile failed: " +
                           std::string(log.data()));
}

unsigned createProgram() {
  static constexpr const char *vertex_source = R"glsl(
    #version 330 core
    layout(location = 0) in vec3 in_position;
    layout(location = 1) in vec3 in_color;
    layout(location = 2) in float in_intensity;
    uniform mat4 view_projection;
    uniform float point_size;
    out vec3 vertex_color;
    out float vertex_intensity;
    out float vertex_z;
    void main() {
      gl_Position = view_projection * vec4(in_position, 1.0);
      gl_PointSize = point_size;
      vertex_color = in_color;
      vertex_intensity = in_intensity;
      vertex_z = in_position.z;
    }
  )glsl";
  static constexpr const char *fragment_source = R"glsl(
    #version 330 core
    in vec3 vertex_color;
    in float vertex_intensity;
    in float vertex_z;
    uniform int color_mode;
    uniform vec2 scalar_range;
    out vec4 out_color;

    vec3 turbo(float x) {
      x = clamp(x, 0.0, 1.0);
      vec4 kRed = vec4(0.13572138, 4.61539260, -42.66032258, 132.13108234);
      vec4 kGreen = vec4(0.09140261, 2.19418839, 4.84296658, -14.18503333);
      vec4 kBlue = vec4(0.10667330, 12.64194608, -60.58204836, 110.36276771);
      vec2 kRed2 = vec2(-152.94239396, 59.28637943);
      vec2 kGreen2 = vec2(4.27729857, 2.82956604);
      vec2 kBlue2 = vec2(-89.90310912, 27.34824973);
      vec4 v4 = vec4(1.0, x, x * x, x * x * x);
      vec2 v2 = v4.zw * v4.z;
      return vec3(dot(v4, kRed) + dot(v2, kRed2),
                  dot(v4, kGreen) + dot(v2, kGreen2),
                  dot(v4, kBlue) + dot(v2, kBlue2));
    }

    void main() {
      if (length(gl_PointCoord - vec2(0.5)) > 0.5) discard;
      if (color_mode == 0) {
        out_color = vec4(vertex_color, 1.0);
      } else if (color_mode == 4) {
        out_color = vec4(1.0);
      } else {
        float value = color_mode == 1 ? vertex_intensity : vertex_z;
        float span = max(scalar_range.y - scalar_range.x, 1e-12);
        out_color = vec4(turbo((value - scalar_range.x) / span), 1.0);
      }
    }
  )glsl";

  const unsigned vertex = compileShader(GL_VERTEX_SHADER, vertex_source);
  const unsigned fragment = compileShader(GL_FRAGMENT_SHADER, fragment_source);
  const unsigned program = glCreateProgram();
  glAttachShader(program, vertex);
  glAttachShader(program, fragment);
  glLinkProgram(program);
  glDeleteShader(vertex);
  glDeleteShader(fragment);

  int success = 0;
  glGetProgramiv(program, GL_LINK_STATUS, &success);
  if (success != 0)
    return program;
  std::array<char, 2048> log{};
  glGetProgramInfoLog(program, static_cast<int>(log.size()), nullptr,
                      log.data());
  glDeleteProgram(program);
  throw std::runtime_error("OpenGL program link failed: " +
                           std::string(log.data()));
}

Eigen::Matrix4f perspective(float vertical_fov, float aspect, float near_plane,
                            float far_plane) {
  const float scale = 1.0F / std::tan(vertical_fov * 0.5F);
  Eigen::Matrix4f result = Eigen::Matrix4f::Zero();
  result(0, 0) = scale / aspect;
  result(1, 1) = scale;
  result(2, 2) = (far_plane + near_plane) / (near_plane - far_plane);
  result(2, 3) = (2.0F * far_plane * near_plane) / (near_plane - far_plane);
  result(3, 2) = -1.0F;
  return result;
}

Eigen::Matrix4f lookAt(const Eigen::Vector3f &eye,
                       const Eigen::Vector3f &target) {
  const Eigen::Vector3f forward = (target - eye).normalized();
  Eigen::Vector3f right = forward.cross(Eigen::Vector3f::UnitZ());
  if (right.squaredNorm() < 1.0e-8F)
    right = Eigen::Vector3f::UnitX();
  right.normalize();
  const Eigen::Vector3f up = right.cross(forward).normalized();

  Eigen::Matrix4f result = Eigen::Matrix4f::Identity();
  result.block<1, 3>(0, 0) = right.transpose();
  result.block<1, 3>(1, 0) = up.transpose();
  result.block<1, 3>(2, 0) = -forward.transpose();
  result(0, 3) = -right.dot(eye);
  result(1, 3) = -up.dot(eye);
  result(2, 3) = forward.dot(eye);
  return result;
}

} // namespace

struct PointRenderer::Vertex {
  float x;
  float y;
  float z;
  float r;
  float g;
  float b;
  float intensity;
};

CloudBounds calculateBounds(const PointCloudIRGB &cloud) {
  CloudBounds bounds;
  Eigen::Vector3f minimum =
      Eigen::Vector3f::Constant(std::numeric_limits<float>::max());
  Eigen::Vector3f maximum =
      Eigen::Vector3f::Constant(std::numeric_limits<float>::lowest());
  float intensity_min = std::numeric_limits<float>::max();
  float intensity_max = std::numeric_limits<float>::lowest();

  for (const auto &point : cloud) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
        !std::isfinite(point.z)) {
      continue;
    }
    const Eigen::Vector3f position(point.x, point.y, point.z);
    minimum = minimum.cwiseMin(position);
    maximum = maximum.cwiseMax(position);
    if (std::isfinite(point.intensity)) {
      intensity_min = std::min(intensity_min, point.intensity);
      intensity_max = std::max(intensity_max, point.intensity);
    }
    ++bounds.finite_points;
  }

  if (bounds.finite_points == 0)
    return bounds;
  bounds.minimum = minimum;
  bounds.maximum = maximum;
  bounds.center = (minimum + maximum) * 0.5F;
  bounds.radius = std::max((maximum - minimum).norm() * 0.5F, 0.001F);
  bounds.z_min = minimum.z();
  bounds.z_max = maximum.z();
  if (intensity_min <= intensity_max) {
    bounds.intensity_min = intensity_min;
    bounds.intensity_max = intensity_max;
  }
  return bounds;
}

PointRenderer::PointRenderer() { createResources(); }

PointRenderer::~PointRenderer() { destroyResources(); }

void PointRenderer::createResources() {
  program_ = createProgram();
  view_projection_location_ =
      glGetUniformLocation(program_, "view_projection");
  point_size_location_ = glGetUniformLocation(program_, "point_size");
  color_mode_location_ = glGetUniformLocation(program_, "color_mode");
  scalar_range_location_ = glGetUniformLocation(program_, "scalar_range");
  glGenVertexArrays(1, &vao_);
  glGenBuffers(1, &vbo_);
  glBindVertexArray(vao_);
  glBindBuffer(GL_ARRAY_BUFFER, vbo_);
  glEnableVertexAttribArray(0);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex),
                        reinterpret_cast<void *>(offsetof(Vertex, x)));
  glEnableVertexAttribArray(1);
  glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex),
                        reinterpret_cast<void *>(offsetof(Vertex, r)));
  glEnableVertexAttribArray(2);
  glVertexAttribPointer(2, 1, GL_FLOAT, GL_FALSE, sizeof(Vertex),
                        reinterpret_cast<void *>(offsetof(Vertex, intensity)));
  glBindVertexArray(0);
  createFramebuffer();
}

void PointRenderer::destroyResources() {
  if (depth_buffer_ != 0)
    glDeleteRenderbuffers(1, &depth_buffer_);
  if (color_texture_ != 0)
    glDeleteTextures(1, &color_texture_);
  if (framebuffer_ != 0)
    glDeleteFramebuffers(1, &framebuffer_);
  if (vbo_ != 0)
    glDeleteBuffers(1, &vbo_);
  if (vao_ != 0)
    glDeleteVertexArrays(1, &vao_);
  if (program_ != 0)
    glDeleteProgram(program_);
}

void PointRenderer::createFramebuffer() {
  if (framebuffer_ == 0)
    glGenFramebuffers(1, &framebuffer_);
  if (color_texture_ == 0)
    glGenTextures(1, &color_texture_);
  if (depth_buffer_ == 0)
    glGenRenderbuffers(1, &depth_buffer_);

  glBindTexture(GL_TEXTURE_2D, color_texture_);
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, width_, height_, 0, GL_RGBA,
               GL_UNSIGNED_BYTE, nullptr);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

  glBindRenderbuffer(GL_RENDERBUFFER, depth_buffer_);
  glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT24, width_, height_);

  glBindFramebuffer(GL_FRAMEBUFFER, framebuffer_);
  glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D,
                         color_texture_, 0);
  glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT,
                            GL_RENDERBUFFER, depth_buffer_);
  if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
    throw std::runtime_error("OpenGL framebuffer is incomplete");
  }
  glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void PointRenderer::setCloud(const PointCloudIRGBConstPtr &cloud,
                             CameraUpdate camera_update) {
  bounds_ = cloud ? calculateBounds(*cloud) : CloudBounds{};
  std::vector<Vertex> vertices;
  if (cloud)
    vertices.reserve(bounds_.finite_points);
  if (cloud) {
    for (const auto &point : *cloud) {
      if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
          !std::isfinite(point.z)) {
        continue;
      }
      vertices.push_back(
          {point.x, point.y, point.z, static_cast<float>(point.r) / 255.0F,
           static_cast<float>(point.g) / 255.0F,
           static_cast<float>(point.b) / 255.0F,
           std::isfinite(point.intensity) ? point.intensity : 0.0F});
    }
  }
  point_count_ = vertices.size();
  glBindBuffer(GL_ARRAY_BUFFER, vbo_);
  glBufferData(GL_ARRAY_BUFFER,
               static_cast<GLsizeiptr>(vertices.size() * sizeof(Vertex)),
               vertices.empty() ? nullptr : vertices.data(), GL_STATIC_DRAW);
  if (camera_update == CameraUpdate::Fit)
    fit();
}

void PointRenderer::resize(int width, int height) {
  width = std::max(width, 1);
  height = std::max(height, 1);
  if (width == width_ && height == height_)
    return;
  width_ = width;
  height_ = height;
  createFramebuffer();
}

Eigen::Matrix4f PointRenderer::viewProjection() const {
  const float cosine = std::cos(pitch_);
  const Eigen::Vector3f offset(distance_ * cosine * std::cos(yaw_),
                               distance_ * cosine * std::sin(yaw_),
                               distance_ * std::sin(pitch_));
  const Eigen::Vector3f eye = target_ + offset;
  const float near_plane = std::max(0.001F, distance_ * 0.001F);
  const float far_plane =
      std::max(near_plane + 1.0F, distance_ + bounds_.radius * 8.0F);
  return perspective(45.0F * kPi / 180.0F,
                     static_cast<float>(width_) / static_cast<float>(height_),
                     near_plane, far_plane) *
         lookAt(eye, target_);
}

void PointRenderer::render() {
  glBindFramebuffer(GL_FRAMEBUFFER, framebuffer_);
  glViewport(0, 0, width_, height_);
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_PROGRAM_POINT_SIZE);
  glClearColor(background_.x(), background_.y(), background_.z(), 1.0F);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  if (point_count_ != 0) {
    glUseProgram(program_);
    const Eigen::Matrix4f matrix = viewProjection();
    glUniformMatrix4fv(view_projection_location_, 1, GL_FALSE, matrix.data());
    glUniform1f(point_size_location_, point_size_);
    int color_mode = 4;
    if (color_by_ == ColorBy::RGB || color_by_ == ColorBy::Label) {
      color_mode = 0;
    } else if (color_by_ == ColorBy::Intensity) {
      color_mode = 1;
    } else if (color_by_ == ColorBy::Z) {
      color_mode = 2;
    }
    glUniform1i(color_mode_location_, color_mode);
    const bool intensity = color_by_ == ColorBy::Intensity;
    const float minimum = intensity ? bounds_.intensity_min : bounds_.z_min;
    const float maximum = intensity ? bounds_.intensity_max : bounds_.z_max;
    glUniform2f(scalar_range_location_, minimum, maximum);
    glBindVertexArray(vao_);
    glDrawArrays(GL_POINTS, 0, static_cast<int>(point_count_));
    glBindVertexArray(0);
  }
  glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

bool PointRenderer::centerPixelVisible() const {
  std::array<unsigned char, 4> pixel{};
  glBindFramebuffer(GL_FRAMEBUFFER, framebuffer_);
  glReadPixels(width_ / 2, height_ / 2, 1, 1, GL_RGBA, GL_UNSIGNED_BYTE,
               pixel.data());
  glBindFramebuffer(GL_FRAMEBUFFER, 0);
  const auto background_byte = [](float value) {
    return static_cast<unsigned char>(std::clamp(value, 0.0F, 1.0F) * 255.0F);
  };
  return pixel[0] != background_byte(background_.x()) ||
         pixel[1] != background_byte(background_.y()) ||
         pixel[2] != background_byte(background_.z());
}

void PointRenderer::fit() {
  target_ = bounds_.center;
  distance_ = std::max(bounds_.radius * 2.8F, 0.01F);
}

void PointRenderer::orbit(float delta_x, float delta_y) {
  yaw_ -= delta_x * 0.008F;
  pitch_ = std::clamp(pitch_ + delta_y * 0.008F, -1.553F, 1.553F);
}

void PointRenderer::pan(float delta_x, float delta_y) {
  const float cosine = std::cos(pitch_);
  const Eigen::Vector3f eye_direction(
      cosine * std::cos(yaw_), cosine * std::sin(yaw_), std::sin(pitch_));
  Eigen::Vector3f right =
      (-eye_direction).cross(Eigen::Vector3f::UnitZ()).normalized();
  const Eigen::Vector3f up = right.cross(-eye_direction).normalized();
  const float scale = distance_ * 0.0015F;
  target_ += right * delta_x * scale + up * delta_y * scale;
}

void PointRenderer::zoom(float wheel_delta) {
  distance_ *= std::exp(-wheel_delta * 0.12F);
  distance_ =
      std::clamp(distance_, bounds_.radius * 0.01F, bounds_.radius * 1000.0F);
}

void PointRenderer::setView(View view) {
  struct Angles {
    float yaw;
    float pitch;
  };
  Angles angles{};
  switch (view) {
  case View::Front:
    angles = {0.0F, 0.0F};
    break;
  case View::Right:
    angles = {kPi * 0.5F, 0.0F};
    break;
  case View::Back:
    angles = {kPi, 0.0F};
    break;
  case View::Left:
    angles = {-kPi * 0.5F, 0.0F};
    break;
  case View::Top:
    angles = {0.0F, kPi * 0.5F - 0.001F};
    break;
  case View::Bottom:
    angles = {0.0F, -kPi * 0.5F + 0.001F};
    break;
  case View::TopRightFront:
    angles = {kPi * 0.25F, kPi * 0.25F};
    break;
  case View::TopLeftFront:
    angles = {-kPi * 0.25F, kPi * 0.25F};
    break;
  case View::BotRightFront:
    angles = {kPi * 0.25F, -kPi * 0.25F};
    break;
  case View::BotLeftFront:
    angles = {-kPi * 0.25F, -kPi * 0.25F};
    break;
  }
  yaw_ = angles.yaw;
  pitch_ = angles.pitch;
  fit();
}

} // namespace kpt::gui
