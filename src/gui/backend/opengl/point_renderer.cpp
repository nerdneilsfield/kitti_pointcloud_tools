#include "gui/backend/opengl/point_renderer.hpp"

#include "gui/viewport/frame_cache.hpp"

#ifdef __EMSCRIPTEN__
#include <GLES3/gl3.h>
#else
#include <glad/gl.h>
#endif

#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>

#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <limits>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <vector>

namespace kpt::gui {
namespace {

struct GpuVertex {
  float position[3];
  float color[3];
  float intensity;
  float noise;
};

constexpr std::size_t kInteractivePointBudget = 500'000;

RendererError error(RendererErrorCode code, std::string message) {
  return {code, std::move(message)};
}

Result<void, RendererError> loadOpenGL(GLFWwindow *expected_window) {
  if (expected_window == nullptr ||
      glfwGetCurrentContext() != expected_window) {
    return error(RendererErrorCode::BackendMismatch,
                 "OpenGL loader requires the expected current GLFW context");
  }

#ifdef __EMSCRIPTEN__
  return {};
#else
  static std::mutex mutex;
  std::scoped_lock lock(mutex);

  // GLFW proc addresses are only valid for the lifetime of the context/driver
  // that supplied them. Reload GLAD when a renderer is attached to a newly
  // created context; Mesa may unload the old driver after glfwTerminate().
  const int version =
      gladLoadGL(reinterpret_cast<GLADloadfunc>(glfwGetProcAddress));
  if (version == 0 || GLAD_VERSION_MAJOR(version) < 3 ||
      (GLAD_VERSION_MAJOR(version) == 3 && GLAD_VERSION_MINOR(version) < 3)) {
    return error(RendererErrorCode::ResourceCreationFailed,
                 "GLAD could not load an OpenGL 3.3 core context");
  }
  return {};
#endif
}

unsigned compileShader(unsigned type, const char *source) {
  const unsigned shader = glCreateShader(type);
  if (shader == 0)
    throw std::runtime_error("glCreateShader returned zero");
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
#ifdef __EMSCRIPTEN__
  static constexpr const char *vertex_source = R"glsl(#version 300 es
    precision highp float;
    layout(location = 0) in vec3 in_position;
    layout(location = 1) in vec3 in_color;
    layout(location = 2) in float in_intensity;
    layout(location = 3) in float in_noise;
    uniform mat4 view_projection;
    uniform float point_size;
    uniform vec3 world_origin;
    uniform float world_scale;
    out vec3 vertex_color;
    out float vertex_intensity;
    out float vertex_z;
    out float vertex_noise;
    void main() {
      vec3 local_position = (in_position - world_origin) * world_scale;
      gl_Position = view_projection * vec4(local_position, 1.0);
      gl_PointSize = point_size;
      vertex_color = in_color;
      vertex_intensity = in_intensity;
      vertex_z = in_position.z;
      vertex_noise = in_noise;
    }
  )glsl";
  static constexpr const char *fragment_source = R"glsl(#version 300 es
    precision highp float;
    in vec3 vertex_color;
    in float vertex_intensity;
    in float vertex_z;
    in float vertex_noise;
    uniform int color_mode;
    uniform int color_map;
    uniform vec2 scalar_range;
    uniform vec3 fixed_color;
    uniform vec3 noise_color;
    uniform bool highlight_noise;
    uniform bool round_points;
    uniform sampler2D intensity_cdf_tex;
    uniform bool equalize_intensity;
    uniform float opacity;
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

    vec3 palette5(float value, vec3 c0, vec3 c1, vec3 c2, vec3 c3, vec3 c4) {
      float position = clamp(value, 0.0, 1.0) * 4.0;
      if (position < 1.0) return mix(c0, c1, position);
      if (position < 2.0) return mix(c1, c2, position - 1.0);
      if (position < 3.0) return mix(c2, c3, position - 2.0);
      return mix(c3, c4, position - 3.0);
    }

    vec3 scalarColor(float value) {
      if (color_map == 1)
        return palette5(value, vec3(0.267004, 0.004874, 0.329415),
                        vec3(0.229739, 0.322361, 0.545706),
                        vec3(0.127568, 0.566949, 0.550556),
                        vec3(0.369214, 0.788888, 0.382914),
                        vec3(0.993248, 0.906157, 0.143936));
      if (color_map == 2)
        return palette5(value, vec3(0.050383, 0.029803, 0.527975),
                        vec3(0.494877, 0.011990, 0.657865),
                        vec3(0.798216, 0.280197, 0.469538),
                        vec3(0.973416, 0.585761, 0.251540),
                        vec3(0.940015, 0.975158, 0.131326));
      if (color_map == 3)
        return palette5(value, vec3(0.001462, 0.000466, 0.013866),
                        vec3(0.341500, 0.062300, 0.429400),
                        vec3(0.735700, 0.215900, 0.330200),
                        vec3(0.978400, 0.557900, 0.034900),
                        vec3(0.988362, 0.998364, 0.644924));
      if (color_map == 4)
        return palette5(value, vec3(0.001462, 0.000466, 0.013866),
                        vec3(0.316654, 0.071690, 0.485380),
                        vec3(0.716387, 0.214982, 0.475290),
                        vec3(0.986700, 0.535582, 0.382210),
                        vec3(0.987053, 0.991438, 0.749504));
      if (color_map == 5) return vec3(clamp(value, 0.0, 1.0));
      if (color_map == 6)
        return palette5(value, vec3(0.0, 0.0, 0.0),
                        vec3(0.5, 0.0, 0.0),
                        vec3(1.0, 0.5, 0.0),
                        vec3(1.0, 1.0, 0.5),
                        vec3(1.0, 1.0, 1.0));
      if (color_map == 7)
        return palette5(value, vec3(0.0, 0.0, 0.5),
                        vec3(0.0, 0.0, 1.0),
                        vec3(0.0, 1.0, 1.0),
                        vec3(1.0, 1.0, 0.0),
                        vec3(1.0, 0.0, 0.0));
      if (color_map == 8)
        return mix(vec3(1.0, 0.0, 1.0), vec3(1.0, 1.0, 0.0),
                   clamp(value, 0.0, 1.0));
      if (color_map == 9)
        return mix(vec3(1.0, 0.0, 0.0), vec3(1.0, 1.0, 0.0),
                   clamp(value, 0.0, 1.0));
      return turbo(value);
    }

    void main() {
      if (round_points && length(gl_PointCoord - vec2(0.5)) > 0.5) discard;
      vec3 base_color;
      if (color_mode == 0) {
        base_color = vertex_color;
      } else if (color_mode == 4) {
        base_color = fixed_color;
      } else {
        float value = color_mode == 1 ? vertex_intensity : vertex_z;
        float span = max(scalar_range.y - scalar_range.x, 1e-12);
        float normalized = (value - scalar_range.x) / span;
        if (color_mode == 1 && equalize_intensity) {
          float t = texture(intensity_cdf_tex,
                            vec2(clamp(normalized, 0.0, 1.0), 0.5)).r;
          base_color = scalarColor(t);
        } else {
          base_color = color_mode == 1 ? scalarColor(normalized)
                                       : turbo(normalized);
        }
      }
      if (highlight_noise && vertex_noise > 0.5)
        base_color = noise_color;
      out_color = vec4(base_color, opacity);
    }
  )glsl";
#else
  static constexpr const char *vertex_source = R"glsl(
    #version 330 core
    layout(location = 0) in vec3 in_position;
    layout(location = 1) in vec3 in_color;
    layout(location = 2) in float in_intensity;
    layout(location = 3) in float in_noise;
    uniform mat4 view_projection;
    uniform float point_size;
    uniform vec3 world_origin;
    uniform float world_scale;
    out vec3 vertex_color;
    out float vertex_intensity;
    out float vertex_z;
    out float vertex_noise;
    void main() {
      vec3 local_position = (in_position - world_origin) * world_scale;
      gl_Position = view_projection * vec4(local_position, 1.0);
      gl_PointSize = point_size;
      vertex_color = in_color;
      vertex_intensity = in_intensity;
      vertex_z = in_position.z;
      vertex_noise = in_noise;
    }
  )glsl";
  static constexpr const char *fragment_source = R"glsl(
    #version 330 core
    in vec3 vertex_color;
    in float vertex_intensity;
    in float vertex_z;
    in float vertex_noise;
    uniform int color_mode;
    uniform int color_map;
    uniform vec2 scalar_range;
    uniform vec3 fixed_color;
    uniform vec3 noise_color;
    uniform bool highlight_noise;
    uniform bool round_points;
    uniform sampler2D intensity_cdf_tex;
    uniform bool equalize_intensity;
    uniform float opacity;
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

    vec3 palette5(float value, vec3 c0, vec3 c1, vec3 c2, vec3 c3, vec3 c4) {
      float position = clamp(value, 0.0, 1.0) * 4.0;
      if (position < 1.0) return mix(c0, c1, position);
      if (position < 2.0) return mix(c1, c2, position - 1.0);
      if (position < 3.0) return mix(c2, c3, position - 2.0);
      return mix(c3, c4, position - 3.0);
    }

    vec3 scalarColor(float value) {
      if (color_map == 1)
        return palette5(value, vec3(0.267004, 0.004874, 0.329415),
                        vec3(0.229739, 0.322361, 0.545706),
                        vec3(0.127568, 0.566949, 0.550556),
                        vec3(0.369214, 0.788888, 0.382914),
                        vec3(0.993248, 0.906157, 0.143936));
      if (color_map == 2)
        return palette5(value, vec3(0.050383, 0.029803, 0.527975),
                        vec3(0.494877, 0.011990, 0.657865),
                        vec3(0.798216, 0.280197, 0.469538),
                        vec3(0.973416, 0.585761, 0.251540),
                        vec3(0.940015, 0.975158, 0.131326));
      if (color_map == 3)
        return palette5(value, vec3(0.001462, 0.000466, 0.013866),
                        vec3(0.341500, 0.062300, 0.429400),
                        vec3(0.735700, 0.215900, 0.330200),
                        vec3(0.978400, 0.557900, 0.034900),
                        vec3(0.988362, 0.998364, 0.644924));
      if (color_map == 4)
        return palette5(value, vec3(0.001462, 0.000466, 0.013866),
                        vec3(0.316654, 0.071690, 0.485380),
                        vec3(0.716387, 0.214982, 0.475290),
                        vec3(0.986700, 0.535582, 0.382210),
                        vec3(0.987053, 0.991438, 0.749504));
      if (color_map == 5) return vec3(clamp(value, 0.0, 1.0));
      if (color_map == 6)
        return palette5(value, vec3(0.0, 0.0, 0.0),
                        vec3(0.5, 0.0, 0.0),
                        vec3(1.0, 0.5, 0.0),
                        vec3(1.0, 1.0, 0.5),
                        vec3(1.0, 1.0, 1.0));
      if (color_map == 7)
        return palette5(value, vec3(0.0, 0.0, 0.5),
                        vec3(0.0, 0.0, 1.0),
                        vec3(0.0, 1.0, 1.0),
                        vec3(1.0, 1.0, 0.0),
                        vec3(1.0, 0.0, 0.0));
      if (color_map == 8)
        return mix(vec3(1.0, 0.0, 1.0), vec3(1.0, 1.0, 0.0),
                   clamp(value, 0.0, 1.0));
      if (color_map == 9)
        return mix(vec3(1.0, 0.0, 0.0), vec3(1.0, 1.0, 0.0),
                   clamp(value, 0.0, 1.0));
      return turbo(value);
    }

    void main() {
      if (round_points && length(gl_PointCoord - vec2(0.5)) > 0.5) discard;
      vec3 base_color;
      if (color_mode == 0) {
        base_color = vertex_color;
      } else if (color_mode == 4) {
        base_color = fixed_color;
      } else {
        float value = color_mode == 1 ? vertex_intensity : vertex_z;
        float span = max(scalar_range.y - scalar_range.x, 1e-12);
        float normalized = (value - scalar_range.x) / span;
        if (color_mode == 1 && equalize_intensity) {
          float t = texture(intensity_cdf_tex,
                            vec2(clamp(normalized, 0.0, 1.0), 0.5)).r;
          base_color = scalarColor(t);
        } else {
          base_color = color_mode == 1 ? scalarColor(normalized)
                                       : turbo(normalized);
        }
      }
      if (highlight_noise && vertex_noise > 0.5)
        base_color = noise_color;
      out_color = vec4(base_color, opacity);
    }
  )glsl";
#endif

  const unsigned vertex = compileShader(GL_VERTEX_SHADER, vertex_source);
  unsigned fragment = 0;
  unsigned program = 0;
  try {
    fragment = compileShader(GL_FRAGMENT_SHADER, fragment_source);
    program = glCreateProgram();
    if (program == 0)
      throw std::runtime_error("glCreateProgram returned zero");
    glAttachShader(program, vertex);
    glAttachShader(program, fragment);
    glLinkProgram(program);
    int success = 0;
    glGetProgramiv(program, GL_LINK_STATUS, &success);
    if (success == 0) {
      std::array<char, 2048> log{};
      glGetProgramInfoLog(program, static_cast<int>(log.size()), nullptr,
                          log.data());
      throw std::runtime_error("OpenGL program link failed: " +
                               std::string(log.data()));
    }
  } catch (...) {
    if (program != 0)
      glDeleteProgram(program);
    if (fragment != 0)
      glDeleteShader(fragment);
    glDeleteShader(vertex);
    throw;
  }
  glDeleteShader(vertex);
  glDeleteShader(fragment);
  return program;
}

struct RenderState {
  enum class Scope { Full, Upload, Resize, Render };

  int draw_framebuffer = 0;
  int read_framebuffer = 0;
  std::array<int, 4> viewport{};
  int program = 0;
  int vertex_array = 0;
  int array_buffer = 0;
  int active_texture = 0;
  int texture_2d = 0;
  int renderbuffer = 0;
  std::array<float, 4> clear_color{};
  std::array<int, 4> scissor_box{};
  std::array<unsigned char, 4> color_mask{};
  bool depth_test = false;
  bool scissor_test = false;
  bool blend = false;
  bool rasterizer_discard = false;
  bool program_point_size = false;
  unsigned char depth_write_mask = GL_TRUE;
  int depth_func = GL_LESS;
  int blend_src_rgb = GL_ONE;
  int blend_dst_rgb = GL_ZERO;
  int blend_src_alpha = GL_ONE;
  int blend_dst_alpha = GL_ZERO;
  int blend_equation_rgb = GL_FUNC_ADD;
  int blend_equation_alpha = GL_FUNC_ADD;
  Scope scope = Scope::Full;

  explicit RenderState(Scope requested = Scope::Full) : scope(requested) {
    const bool full = scope == Scope::Full;
    const bool resize = full || scope == Scope::Resize;
    const bool upload = full || scope == Scope::Upload || scope == Scope::Render;
    const bool render = full || scope == Scope::Render;
    if (resize) {
      glGetIntegerv(GL_DRAW_FRAMEBUFFER_BINDING, &draw_framebuffer);
      glGetIntegerv(GL_READ_FRAMEBUFFER_BINDING, &read_framebuffer);
      glGetIntegerv(GL_ACTIVE_TEXTURE, &active_texture);
      glGetIntegerv(GL_TEXTURE_BINDING_2D, &texture_2d);
      glGetIntegerv(GL_RENDERBUFFER_BINDING, &renderbuffer);
    } else if (render) {
      glGetIntegerv(GL_DRAW_FRAMEBUFFER_BINDING, &draw_framebuffer);
    }
    if (full || render)
      glGetIntegerv(GL_VIEWPORT, viewport.data());
    if (full || render)
      glGetIntegerv(GL_CURRENT_PROGRAM, &program);
    if (upload) {
      glGetIntegerv(GL_VERTEX_ARRAY_BINDING, &vertex_array);
      glGetIntegerv(GL_ARRAY_BUFFER_BINDING, &array_buffer);
    }
    if (full || render) {
      glGetFloatv(GL_COLOR_CLEAR_VALUE, clear_color.data());
      glGetBooleanv(GL_COLOR_WRITEMASK, color_mask.data());
      glGetBooleanv(GL_DEPTH_WRITEMASK, &depth_write_mask);
      glGetIntegerv(GL_DEPTH_FUNC, &depth_func);
      depth_test = glIsEnabled(GL_DEPTH_TEST) == GL_TRUE;
      scissor_test = glIsEnabled(GL_SCISSOR_TEST) == GL_TRUE;
      blend = glIsEnabled(GL_BLEND) == GL_TRUE;
      rasterizer_discard = glIsEnabled(GL_RASTERIZER_DISCARD) == GL_TRUE;
#ifndef __EMSCRIPTEN__
      program_point_size = glIsEnabled(GL_PROGRAM_POINT_SIZE) == GL_TRUE;
#endif
    }
    if (full || render) {
      glGetIntegerv(GL_SCISSOR_BOX, scissor_box.data());
      glGetIntegerv(GL_BLEND_SRC_RGB, &blend_src_rgb);
      glGetIntegerv(GL_BLEND_DST_RGB, &blend_dst_rgb);
      glGetIntegerv(GL_BLEND_SRC_ALPHA, &blend_src_alpha);
      glGetIntegerv(GL_BLEND_DST_ALPHA, &blend_dst_alpha);
      glGetIntegerv(GL_BLEND_EQUATION_RGB, &blend_equation_rgb);
      glGetIntegerv(GL_BLEND_EQUATION_ALPHA, &blend_equation_alpha);
    }
  }

  ~RenderState() {
    const bool full = scope == Scope::Full;
    const bool resize = full || scope == Scope::Resize;
    const bool upload = full || scope == Scope::Upload || scope == Scope::Render;
    const bool render = full || scope == Scope::Render;
    if (resize) {
      glBindFramebuffer(GL_DRAW_FRAMEBUFFER,
                        static_cast<unsigned>(draw_framebuffer));
      glBindFramebuffer(GL_READ_FRAMEBUFFER,
                        static_cast<unsigned>(read_framebuffer));
      glActiveTexture(static_cast<unsigned>(active_texture));
      glBindTexture(GL_TEXTURE_2D, static_cast<unsigned>(texture_2d));
      glBindRenderbuffer(GL_RENDERBUFFER, static_cast<unsigned>(renderbuffer));
    } else if (render) {
      glBindFramebuffer(GL_DRAW_FRAMEBUFFER,
                        static_cast<unsigned>(draw_framebuffer));
    }
    if (full || render)
      glViewport(viewport[0], viewport[1], viewport[2], viewport[3]);
    if (full || render)
      glUseProgram(static_cast<unsigned>(program));
    if (upload) {
      glBindVertexArray(static_cast<unsigned>(vertex_array));
      glBindBuffer(GL_ARRAY_BUFFER, static_cast<unsigned>(array_buffer));
    }
    if (full || render) {
      glScissor(scissor_box[0], scissor_box[1], scissor_box[2], scissor_box[3]);
      glBlendFuncSeparate(static_cast<unsigned>(blend_src_rgb),
                          static_cast<unsigned>(blend_dst_rgb),
                          static_cast<unsigned>(blend_src_alpha),
                          static_cast<unsigned>(blend_dst_alpha));
      glBlendEquationSeparate(static_cast<unsigned>(blend_equation_rgb),
                              static_cast<unsigned>(blend_equation_alpha));
    }
    if (full || render) {
      glColorMask(color_mask[0], color_mask[1], color_mask[2], color_mask[3]);
      glDepthMask(depth_write_mask);
      glDepthFunc(static_cast<unsigned>(depth_func));
      if (depth_test)
        glEnable(GL_DEPTH_TEST);
      else
        glDisable(GL_DEPTH_TEST);
      if (scissor_test)
        glEnable(GL_SCISSOR_TEST);
      else
        glDisable(GL_SCISSOR_TEST);
      if (blend)
        glEnable(GL_BLEND);
      else
        glDisable(GL_BLEND);
      if (rasterizer_discard)
        glEnable(GL_RASTERIZER_DISCARD);
      else
        glDisable(GL_RASTERIZER_DISCARD);
#ifndef __EMSCRIPTEN__
      if (program_point_size)
        glEnable(GL_PROGRAM_POINT_SIZE);
      else
        glDisable(GL_PROGRAM_POINT_SIZE);
#endif
      glClearColor(clear_color[0], clear_color[1], clear_color[2],
                   clear_color[3]);
    }
  }

  void replaceFramebufferObjects(unsigned old_framebuffer,
                                 unsigned new_framebuffer, unsigned old_texture,
                                 unsigned new_texture,
                                 unsigned old_renderbuffer,
                                 unsigned new_renderbuffer) {
    if (old_framebuffer != 0 &&
        draw_framebuffer == static_cast<int>(old_framebuffer))
      draw_framebuffer = static_cast<int>(new_framebuffer);
    if (old_framebuffer != 0 &&
        read_framebuffer == static_cast<int>(old_framebuffer))
      read_framebuffer = static_cast<int>(new_framebuffer);
    if (old_texture != 0 && texture_2d == static_cast<int>(old_texture))
      texture_2d = static_cast<int>(new_texture);
    if (old_renderbuffer != 0 &&
        renderbuffer == static_cast<int>(old_renderbuffer))
      renderbuffer = static_cast<int>(new_renderbuffer);
  }

  void replaceVertexObjects(unsigned old_vertex_array,
                            unsigned new_vertex_array,
                            unsigned old_vertex_buffer,
                            unsigned new_vertex_buffer) {
    if (old_vertex_array != 0 &&
        vertex_array == static_cast<int>(old_vertex_array))
      vertex_array = static_cast<int>(new_vertex_array);
    if (old_vertex_buffer != 0 &&
        array_buffer == static_cast<int>(old_vertex_buffer))
      array_buffer = static_cast<int>(new_vertex_buffer);
  }
};

bool finite(const ViewportVertex &vertex) {
  return vertex.position.allFinite() && vertex.color.allFinite() &&
         std::isfinite(vertex.intensity) && std::isfinite(vertex.noise);
}

bool finite(const ViewportLineVertex &vertex) {
  return vertex.position.allFinite() && vertex.color.allFinite();
}

void clearOpenGLErrors() {
  while (glGetError() != GL_NO_ERROR) {
  }
}

// GL_OUT_OF_MEMORY is not a malformed draw/upload request.  It means the
// driver could not create or grow a GPU resource, which lets the review
// admission path retry with a smaller LOD instead of surfacing a terminal
// encoding error.  Keep every other GL error as EncodingFailed: those are
// programming/state failures for which reducing the point count is useless.
[[nodiscard]] RendererErrorCode
classifyOpenGLError(unsigned gl_error) noexcept {
  return gl_error == GL_OUT_OF_MEMORY
             ? RendererErrorCode::ResourceCreationFailed
             : RendererErrorCode::EncodingFailed;
}

[[nodiscard]] RendererError openGLError(std::string operation,
                                        unsigned gl_error) {
  return error(classifyOpenGLError(gl_error),
               std::move(operation) + " with error " +
                   std::to_string(gl_error));
}

} // namespace

OpenGLFrameContext::OpenGLFrameContext(GLFWwindow *expected_window,
                                       bool active) noexcept
    : expected_window_(expected_window), active_(active) {}

OpenGLPointRenderer::OpenGLPointRenderer(GLFWwindow *expected_window)
    : expected_window_(expected_window) {
  auto loaded = loadOpenGL(expected_window_);
  if (!loaded)
    throw std::runtime_error(loaded.error().message);
  auto created = createStaticResources();
  if (!created)
    throw std::runtime_error(created.error().message);
}

OpenGLPointRenderer::~OpenGLPointRenderer() {
  assert(expectedContextIsCurrent());
  if (!expectedContextIsCurrent())
    return;
  destroyFramebuffer();
  destroyStaticResources();
}

bool OpenGLPointRenderer::expectedContextIsCurrent() const noexcept {
  return expected_window_ != nullptr &&
         glfwGetCurrentContext() == expected_window_;
}

Result<void, RendererError> OpenGLPointRenderer::createStaticResources() {
  if (!expectedContextIsCurrent()) {
    return error(RendererErrorCode::BackendMismatch,
                 "OpenGL renderer resource creation used the wrong context");
  }
  RenderState saved;
  try {
    program_ = createProgram();
    view_projection_location_ =
        glGetUniformLocation(program_, "view_projection");
    point_size_location_ = glGetUniformLocation(program_, "point_size");
    world_origin_location_ = glGetUniformLocation(program_, "world_origin");
    world_scale_location_ = glGetUniformLocation(program_, "world_scale");
    color_mode_location_ = glGetUniformLocation(program_, "color_mode");
    color_map_location_ = glGetUniformLocation(program_, "color_map");
    scalar_range_location_ = glGetUniformLocation(program_, "scalar_range");
    fixed_color_location_ = glGetUniformLocation(program_, "fixed_color");
    noise_color_location_ = glGetUniformLocation(program_, "noise_color");
    highlight_noise_location_ =
        glGetUniformLocation(program_, "highlight_noise");
    round_points_location_ = glGetUniformLocation(program_, "round_points");
    cdf_tex_location_ = glGetUniformLocation(program_, "intensity_cdf_tex");
    equalize_location_ = glGetUniformLocation(program_, "equalize_intensity");
    opacity_location_ = glGetUniformLocation(program_, "opacity");
    glGenVertexArrays(1, &vertex_array_);
    glGenBuffers(1, &vertex_buffer_);
    glGenVertexArrays(1, &guide_vertex_array_);
    glGenBuffers(1, &guide_vertex_buffer_);
    glGenTextures(1, &cdf_texture_);
    if (vertex_array_ == 0 || vertex_buffer_ == 0 || guide_vertex_array_ == 0 ||
        guide_vertex_buffer_ == 0 || cdf_texture_ == 0)
      throw std::runtime_error("OpenGL vertex resource creation returned zero");
    glBindTexture(GL_TEXTURE_2D, cdf_texture_);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_R8, 256, 1, 0, GL_RED, GL_UNSIGNED_BYTE,
                 nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glBindVertexArray(vertex_array_);
    glBindBuffer(GL_ARRAY_BUFFER, vertex_buffer_);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(
        0, 3, GL_FLOAT, GL_FALSE, sizeof(GpuVertex),
        reinterpret_cast<void *>(offsetof(GpuVertex, position)));
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(GpuVertex),
                          reinterpret_cast<void *>(offsetof(GpuVertex, color)));
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(
        2, 1, GL_FLOAT, GL_FALSE, sizeof(GpuVertex),
        reinterpret_cast<void *>(offsetof(GpuVertex, intensity)));
    glEnableVertexAttribArray(3);
    glVertexAttribPointer(3, 1, GL_FLOAT, GL_FALSE, sizeof(GpuVertex),
                          reinterpret_cast<void *>(offsetof(GpuVertex, noise)));
    glBindVertexArray(guide_vertex_array_);
    glBindBuffer(GL_ARRAY_BUFFER, guide_vertex_buffer_);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(
        0, 3, GL_FLOAT, GL_FALSE, sizeof(GpuVertex),
        reinterpret_cast<void *>(offsetof(GpuVertex, position)));
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(GpuVertex),
                          reinterpret_cast<void *>(offsetof(GpuVertex, color)));
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(
        2, 1, GL_FLOAT, GL_FALSE, sizeof(GpuVertex),
        reinterpret_cast<void *>(offsetof(GpuVertex, intensity)));
    glEnableVertexAttribArray(3);
    glVertexAttribPointer(3, 1, GL_FLOAT, GL_FALSE, sizeof(GpuVertex),
                          reinterpret_cast<void *>(offsetof(GpuVertex, noise)));
  } catch (const std::exception &exception) {
    destroyStaticResources();
    return error(RendererErrorCode::ResourceCreationFailed, exception.what());
  }
  return {};
}

void OpenGLPointRenderer::destroyStaticResources() noexcept {
  for (auto &[layer_id, buffer] : layer_buffers_) {
    static_cast<void>(layer_id);
    destroyLayerBuffer(buffer);
  }
  layer_buffers_.clear();
  uploaded_layered_revision_ = 0;
  if (lod_index_buffer_ != 0)
    glDeleteBuffers(1, &lod_index_buffer_);
  lod_index_buffer_ = 0;
  lod_index_capacity_ = 0;
  lod_point_count_ = 0;
  if (guide_vertex_buffer_ != 0)
    glDeleteBuffers(1, &guide_vertex_buffer_);
  if (guide_vertex_array_ != 0)
    glDeleteVertexArrays(1, &guide_vertex_array_);
  if (vertex_buffer_ != 0)
    glDeleteBuffers(1, &vertex_buffer_);
  if (vertex_array_ != 0)
    glDeleteVertexArrays(1, &vertex_array_);
  if (cdf_texture_ != 0)
    glDeleteTextures(1, &cdf_texture_);
  if (program_ != 0)
    glDeleteProgram(program_);
  vertex_buffer_ = 0;
  vertex_array_ = 0;
  guide_vertex_buffer_ = 0;
  guide_vertex_array_ = 0;
  vertex_buffer_capacity_ = 0;
  guide_buffer_capacity_ = 0;
  uploaded_guides_.clear();
  guide_point_count_ = 0;
  cdf_texture_ = 0;
  cdf_uploaded_ = false;
  program_ = 0;
}

void OpenGLPointRenderer::destroyFramebuffer() noexcept {
  if (depth_buffer_ != 0)
    glDeleteRenderbuffers(1, &depth_buffer_);
  if (color_texture_ != 0)
    glDeleteTextures(1, &color_texture_);
  if (framebuffer_ != 0)
    glDeleteFramebuffers(1, &framebuffer_);
  depth_buffer_ = 0;
  color_texture_ = 0;
  framebuffer_ = 0;
  encoded_frame_.reset();
}

Result<void, RendererError>
OpenGLPointRenderer::upload(std::span<const ViewportVertex> vertices,
                            std::uint64_t revision) {
  if (!expectedContextIsCurrent()) {
    return error(RendererErrorCode::BackendMismatch,
                 "OpenGL upload used a non-current expected context");
  }
  if (revision == uploaded_revision_)
    return {};

  std::vector<GpuVertex> copied;
  copied.reserve(vertices.size());
  for (const auto &vertex : vertices) {
    if (!finite(vertex))
      continue;
    copied.push_back(
        {{vertex.position.x(), vertex.position.y(), vertex.position.z()},
         {vertex.color.x(), vertex.color.y(), vertex.color.z()},
         vertex.intensity,
         vertex.noise});
  }

  if (copied.size() >
      (std::numeric_limits<std::size_t>::max)() / sizeof(GpuVertex)) {
    return error(RendererErrorCode::EncodingFailed,
                 "OpenGL vertex upload size overflows");
  }
  const auto vertex_bytes = copied.size() * sizeof(GpuVertex);
  if (vertex_bytes > static_cast<std::size_t>(
                        (std::numeric_limits<GLsizeiptr>::max)())) {
    return error(RendererErrorCode::EncodingFailed,
                 "OpenGL vertex upload exceeds GLsizeiptr");
  }

  RenderState saved(RenderState::Scope::Upload);
  clearOpenGLErrors();
  glBindVertexArray(vertex_array_);
  glBindBuffer(GL_ARRAY_BUFFER, vertex_buffer_);
  const bool vertex_buffer_grew = vertex_bytes > vertex_buffer_capacity_;
  if (vertex_buffer_grew) {
    glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(vertex_bytes),
                 copied.empty() ? nullptr : copied.data(), GL_STREAM_DRAW);
  } else if (vertex_bytes != 0) {
    glBufferSubData(GL_ARRAY_BUFFER, 0, static_cast<GLsizeiptr>(vertex_bytes),
                    copied.data());
  }
  std::size_t next_lod_point_count = 0;
  bool lod_index_buffer_grew = false;
  std::size_t next_lod_index_capacity = lod_index_capacity_;
  if (copied.size() > kInteractivePointBudget) {
    if (copied.size() > (std::numeric_limits<std::uint32_t>::max)()) {
      return error(RendererErrorCode::EncodingFailed,
                   "WebGL point index exceeds uint32_t");
    }
    std::vector<std::uint32_t> lod_indices(kInteractivePointBudget);
    for (std::size_t index = 0; index < lod_indices.size(); ++index) {
      const auto source =
          (static_cast<std::uint64_t>(index) * copied.size()) /
          lod_indices.size();
      lod_indices[index] = static_cast<std::uint32_t>(source);
    }
    const auto lod_bytes = lod_indices.size() * sizeof(std::uint32_t);
    if (lod_index_buffer_ == 0)
      glGenBuffers(1, &lod_index_buffer_);
    if (lod_index_buffer_ == 0) {
      return error(RendererErrorCode::ResourceCreationFailed,
                   "WebGL LOD index buffer creation returned zero");
    }
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, lod_index_buffer_);
    lod_index_buffer_grew = lod_bytes > lod_index_capacity_;
    if (lod_index_buffer_grew) {
      glBufferData(GL_ELEMENT_ARRAY_BUFFER,
                   static_cast<GLsizeiptr>(lod_bytes), lod_indices.data(),
                   GL_STATIC_DRAW);
      next_lod_index_capacity = lod_bytes;
    } else {
      glBufferSubData(GL_ELEMENT_ARRAY_BUFFER, 0,
                      static_cast<GLsizeiptr>(lod_bytes), lod_indices.data());
    }
    next_lod_point_count = lod_indices.size();
  }
  const unsigned gl_error = glGetError();
  if (gl_error != GL_NO_ERROR) {
    return openGLError("OpenGL vertex upload failed", gl_error);
  }
  // Publish capacity only after every GPU command succeeded.  In particular,
  // an OOM from glBufferData must leave the old capacity intact so the review
  // LOD retry allocates rather than attempting a corrupt glBufferSubData.
  if (vertex_buffer_grew)
    vertex_buffer_capacity_ = vertex_bytes;
  if (lod_index_buffer_grew)
    lod_index_capacity_ = next_lod_index_capacity;
  lod_point_count_ = next_lod_point_count;
  point_count_ = copied.size();
  uploaded_revision_ = revision;
  return {};
}

void OpenGLPointRenderer::destroyLayerBuffer(LayerBuffer &buffer) noexcept {
  if (buffer.lod_index_buffer != 0)
    glDeleteBuffers(1, &buffer.lod_index_buffer);
  if (buffer.vertex_buffer != 0)
    glDeleteBuffers(1, &buffer.vertex_buffer);
  if (buffer.vertex_array != 0)
    glDeleteVertexArrays(1, &buffer.vertex_array);
  buffer = {};
}

Result<void, RendererError> OpenGLPointRenderer::uploadLayerBuffer(
    LayerBuffer &buffer, std::span<const ViewportVertex> vertices,
    std::uint64_t revision) {
  if (buffer.revision == revision)
    return {};

  std::vector<GpuVertex> copied;
  copied.reserve(vertices.size());
  for (const auto &vertex : vertices) {
    if (!finite(vertex))
      continue;
    copied.push_back(
        {{vertex.position.x(), vertex.position.y(), vertex.position.z()},
         {vertex.color.x(), vertex.color.y(), vertex.color.z()},
         vertex.intensity,
         vertex.noise});
  }
  if (copied.size() >
      (std::numeric_limits<std::size_t>::max)() / sizeof(GpuVertex)) {
    return error(RendererErrorCode::EncodingFailed,
                 "OpenGL layered vertex upload size overflows");
  }
  const auto vertex_bytes = copied.size() * sizeof(GpuVertex);
  if (vertex_bytes > static_cast<std::size_t>(
                         (std::numeric_limits<GLsizeiptr>::max)())) {
    return error(RendererErrorCode::EncodingFailed,
                 "OpenGL layered vertex upload exceeds GLsizeiptr");
  }

  if (buffer.vertex_array == 0)
    glGenVertexArrays(1, &buffer.vertex_array);
  if (buffer.vertex_buffer == 0)
    glGenBuffers(1, &buffer.vertex_buffer);
  if (buffer.vertex_array == 0 || buffer.vertex_buffer == 0) {
    return error(RendererErrorCode::ResourceCreationFailed,
                 "OpenGL layered vertex resource creation returned zero");
  }

  glBindVertexArray(buffer.vertex_array);
  glBindBuffer(GL_ARRAY_BUFFER, buffer.vertex_buffer);
  glEnableVertexAttribArray(0);
  glVertexAttribPointer(
      0, 3, GL_FLOAT, GL_FALSE, sizeof(GpuVertex),
      reinterpret_cast<void *>(offsetof(GpuVertex, position)));
  glEnableVertexAttribArray(1);
  glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(GpuVertex),
                        reinterpret_cast<void *>(offsetof(GpuVertex, color)));
  glEnableVertexAttribArray(2);
  glVertexAttribPointer(
      2, 1, GL_FLOAT, GL_FALSE, sizeof(GpuVertex),
      reinterpret_cast<void *>(offsetof(GpuVertex, intensity)));
  glEnableVertexAttribArray(3);
  glVertexAttribPointer(3, 1, GL_FLOAT, GL_FALSE, sizeof(GpuVertex),
                        reinterpret_cast<void *>(offsetof(GpuVertex, noise)));
  const bool vertex_buffer_grew =
      vertex_bytes > buffer.vertex_buffer_capacity;
  if (vertex_buffer_grew) {
    glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(vertex_bytes),
                 copied.empty() ? nullptr : copied.data(), GL_STREAM_DRAW);
  } else if (vertex_bytes != 0) {
    glBufferSubData(GL_ARRAY_BUFFER, 0, static_cast<GLsizeiptr>(vertex_bytes),
                    copied.data());
  }
  std::size_t next_lod_point_count = 0;
  bool lod_index_buffer_grew = false;
  std::size_t next_lod_index_capacity = buffer.lod_index_capacity;
  if (copied.size() > kInteractivePointBudget) {
    if (copied.size() > (std::numeric_limits<std::uint32_t>::max)()) {
      return error(RendererErrorCode::EncodingFailed,
                   "WebGL layered point index exceeds uint32_t");
    }
    std::vector<std::uint32_t> lod_indices(kInteractivePointBudget);
    for (std::size_t index = 0; index < lod_indices.size(); ++index) {
      const auto source =
          (static_cast<std::uint64_t>(index) * copied.size()) /
          lod_indices.size();
      lod_indices[index] = static_cast<std::uint32_t>(source);
    }
    const auto lod_bytes = lod_indices.size() * sizeof(std::uint32_t);
    if (buffer.lod_index_buffer == 0)
      glGenBuffers(1, &buffer.lod_index_buffer);
    if (buffer.lod_index_buffer == 0) {
      return error(RendererErrorCode::ResourceCreationFailed,
                   "WebGL layered LOD index buffer creation returned zero");
    }
    // The element binding belongs to this layer's VAO, preserving independent
    // opaque/transparent layer state across the later draw pass.
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, buffer.lod_index_buffer);
    lod_index_buffer_grew = lod_bytes > buffer.lod_index_capacity;
    if (lod_index_buffer_grew) {
      glBufferData(GL_ELEMENT_ARRAY_BUFFER, static_cast<GLsizeiptr>(lod_bytes),
                   lod_indices.data(), GL_STATIC_DRAW);
      next_lod_index_capacity = lod_bytes;
    } else {
      glBufferSubData(GL_ELEMENT_ARRAY_BUFFER, 0,
                      static_cast<GLsizeiptr>(lod_bytes), lod_indices.data());
    }
    next_lod_point_count = lod_indices.size();
  }
  const unsigned gl_error = glGetError();
  if (gl_error != GL_NO_ERROR) {
    return openGLError("OpenGL layered vertex upload failed", gl_error);
  }
  if (vertex_buffer_grew)
    buffer.vertex_buffer_capacity = vertex_bytes;
  if (lod_index_buffer_grew)
    buffer.lod_index_capacity = next_lod_index_capacity;
  buffer.lod_point_count = next_lod_point_count;
  buffer.point_count = copied.size();
  buffer.revision = revision;
  return {};
}

Result<void, RendererError> OpenGLPointRenderer::uploadLayers(
    std::span<const ViewportLayerUpload> layers,
    std::uint64_t scene_revision) {
  if (!expectedContextIsCurrent()) {
    return error(RendererErrorCode::BackendMismatch,
                 "OpenGL layered upload used a non-current expected context");
  }
  if (scene_revision == 0) {
    for (auto &[layer_id, buffer] : layer_buffers_) {
      static_cast<void>(layer_id);
      destroyLayerBuffer(buffer);
    }
    layer_buffers_.clear();
    uploaded_layered_revision_ = 0;
    return {};
  }
  if (scene_revision == uploaded_layered_revision_)
    return {};

  std::unordered_set<std::uint64_t> seen;
  seen.reserve(layers.size());
  for (const auto &layer : layers) {
    if (layer.layer_id == 0 || layer.revision == 0 ||
        !seen.insert(layer.layer_id).second) {
      return error(RendererErrorCode::EncodingFailed,
                   "OpenGL layered upload requires unique non-zero layer IDs");
    }
  }

  RenderState saved(RenderState::Scope::Upload);
  clearOpenGLErrors();
  try {
    for (const auto &layer : layers) {
      const auto [iterator, inserted] =
          layer_buffers_.try_emplace(layer.layer_id);
      auto uploaded =
          uploadLayerBuffer(iterator->second, layer.vertices, layer.revision);
      if (!uploaded) {
        if (inserted) {
          destroyLayerBuffer(iterator->second);
          layer_buffers_.erase(iterator);
        }
        return uploaded.error();
      }
    }
  } catch (const std::exception &exception) {
    return error(RendererErrorCode::ResourceCreationFailed,
                 "OpenGL layered upload failed: " +
                     std::string(exception.what()));
  }

  for (auto iterator = layer_buffers_.begin(); iterator != layer_buffers_.end();) {
    if (seen.contains(iterator->first)) {
      ++iterator;
      continue;
    }
    destroyLayerBuffer(iterator->second);
    iterator = layer_buffers_.erase(iterator);
  }
  uploaded_layered_revision_ = scene_revision;
  return {};
}

Result<void, RendererError>
OpenGLPointRenderer::resize(PixelExtent physical_pixels) {
  if (!expectedContextIsCurrent()) {
    return error(RendererErrorCode::BackendMismatch,
                 "OpenGL resize used a non-current expected context");
  }
  if (physical_pixels.width < 0 || physical_pixels.height < 0) {
    return error(RendererErrorCode::ResourceCreationFailed,
                 "OpenGL viewport extent cannot be negative");
  }
  if (physical_pixels == extent_)
    return {};
  if (physical_pixels.width == 0 || physical_pixels.height == 0) {
    RenderState saved(RenderState::Scope::Resize);
    saved.replaceFramebufferObjects(framebuffer_, 0, color_texture_, 0,
                                    depth_buffer_, 0);
    destroyFramebuffer();
    extent_ = {};
    return {};
  }

  RenderState saved(RenderState::Scope::Resize);
  clearOpenGLErrors();
  unsigned new_framebuffer = 0;
  unsigned new_texture = 0;
  unsigned new_depth_buffer = 0;
  glGenFramebuffers(1, &new_framebuffer);
  glGenTextures(1, &new_texture);
  glGenRenderbuffers(1, &new_depth_buffer);
  if (new_framebuffer == 0 || new_texture == 0 || new_depth_buffer == 0) {
    if (new_depth_buffer != 0)
      glDeleteRenderbuffers(1, &new_depth_buffer);
    if (new_texture != 0)
      glDeleteTextures(1, &new_texture);
    if (new_framebuffer != 0)
      glDeleteFramebuffers(1, &new_framebuffer);
    return error(RendererErrorCode::ResourceCreationFailed,
                 "OpenGL framebuffer object creation returned zero");
  }

  glBindTexture(GL_TEXTURE_2D, new_texture);
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, physical_pixels.width,
               physical_pixels.height, 0, GL_RGBA, GL_UNSIGNED_BYTE, nullptr);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
  glBindRenderbuffer(GL_RENDERBUFFER, new_depth_buffer);
  glRenderbufferStorage(GL_RENDERBUFFER,
#ifdef __EMSCRIPTEN__
                        GL_DEPTH_COMPONENT16,
#else
                        GL_DEPTH_COMPONENT24,
#endif
                        physical_pixels.width, physical_pixels.height);
  glBindFramebuffer(GL_FRAMEBUFFER, new_framebuffer);
  glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D,
                         new_texture, 0);
#ifdef __EMSCRIPTEN__
  constexpr unsigned draw_buffer = GL_COLOR_ATTACHMENT0;
  glDrawBuffers(1, &draw_buffer);
  glReadBuffer(GL_COLOR_ATTACHMENT0);
#endif
  const unsigned color_only_status = glCheckFramebufferStatus(GL_FRAMEBUFFER);
  glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT,
                            GL_RENDERBUFFER, new_depth_buffer);
  const unsigned framebuffer_status = glCheckFramebufferStatus(GL_FRAMEBUFFER);
  const bool complete = framebuffer_status == GL_FRAMEBUFFER_COMPLETE;
  const unsigned gl_error = glGetError();
  if (!complete || gl_error != GL_NO_ERROR) {
    glDeleteRenderbuffers(1, &new_depth_buffer);
    glDeleteTextures(1, &new_texture);
    glDeleteFramebuffers(1, &new_framebuffer);
    return error(gl_error == GL_NO_ERROR
                     ? RendererErrorCode::ResourceCreationFailed
                     : classifyOpenGLError(gl_error),
                 "OpenGL framebuffer is incomplete (status " +
                     std::to_string(framebuffer_status) + ", error " +
                     std::to_string(gl_error) + ", extent " +
                     std::to_string(physical_pixels.width) + "x" +
                     std::to_string(physical_pixels.height) +
                     ", color-only status " +
                     std::to_string(color_only_status) + ")");
  }

  const unsigned old_framebuffer = framebuffer_;
  const unsigned old_texture = color_texture_;
  const unsigned old_depth_buffer = depth_buffer_;
  framebuffer_ = new_framebuffer;
  color_texture_ = new_texture;
  depth_buffer_ = new_depth_buffer;
  extent_ = physical_pixels;
  encoded_frame_.reset();
  saved.replaceFramebufferObjects(old_framebuffer, new_framebuffer, old_texture,
                                  new_texture, old_depth_buffer,
                                  new_depth_buffer);
  if (old_depth_buffer != 0)
    glDeleteRenderbuffers(1, &old_depth_buffer);
  if (old_texture != 0)
    glDeleteTextures(1, &old_texture);
  if (old_framebuffer != 0)
    glDeleteFramebuffers(1, &old_framebuffer);
  return {};
}

Result<void, RendererError>
OpenGLPointRenderer::render(const ViewportFrame &frame, FrameContext &context) {
  if (context.backendKind() !=
#ifdef __EMSCRIPTEN__
      BackendKind::WebGL
#else
      BackendKind::OpenGL
#endif
  ) {
    assert(false && "OpenGL renderer received a non-OpenGL frame context");
    return error(RendererErrorCode::BackendMismatch,
                 "OpenGL renderer received a non-OpenGL frame context");
  }
  const auto *open_gl_context = dynamic_cast<OpenGLFrameContext *>(&context);
  if (open_gl_context == nullptr || !open_gl_context->isActive() ||
      open_gl_context->expectedWindow() != expected_window_ ||
      !expectedContextIsCurrent()) {
    return error(RendererErrorCode::BackendMismatch,
                 "OpenGL frame context is inactive or belongs to another "
                 "GLFW context");
  }
  if (extent_.width == 0 || extent_.height == 0)
    return {};
  if (encoded_frame_ && encoded_revision_ == uploaded_revision_ &&
      detail::framesRenderEqual(*encoded_frame_, frame)) {
    return {};
  }

  RenderState saved(RenderState::Scope::Render);
  clearOpenGLErrors();
  glBindFramebuffer(GL_DRAW_FRAMEBUFFER, framebuffer_);
  glViewport(0, 0, extent_.width, extent_.height);
  glDisable(GL_SCISSOR_TEST);
  glDisable(GL_BLEND);
  glDisable(GL_RASTERIZER_DISCARD);
  glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
  glDepthMask(GL_TRUE);
  glDepthFunc(GL_LESS);
  glEnable(GL_DEPTH_TEST);
#ifndef __EMSCRIPTEN__
  glEnable(GL_PROGRAM_POINT_SIZE);
#endif
  glClearColor(frame.style.background.x(), frame.style.background.y(),
               frame.style.background.z(), 1.0F);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  if (point_count_ != 0 && frame.style.point_size > 0.0F) {
    if (point_count_ >
        static_cast<std::size_t>((std::numeric_limits<GLsizei>::max)())) {
      return error(RendererErrorCode::EncodingFailed,
                   "OpenGL point count exceeds GLsizei");
    }
    glUseProgram(program_);
    glUniformMatrix4fv(view_projection_location_, 1, GL_FALSE,
                       frame.view_projection.data());
    glUniform3f(world_origin_location_, frame.world_origin.x(),
                frame.world_origin.y(), frame.world_origin.z());
    glUniform1f(world_scale_location_, frame.world_scale);
    glUniform1f(point_size_location_,
                std::clamp(frame.style.point_size, 0.0F, 5.0F));
    int color_mode = 4;
    if (frame.style.color_by == ColorBy::RGB ||
        frame.style.color_by == ColorBy::Label) {
      color_mode = 0;
    } else if (frame.style.color_by == ColorBy::Intensity) {
      color_mode = 1;
    } else if (frame.style.color_by == ColorBy::Z) {
      color_mode = 2;
    }
    glUniform1i(color_mode_location_, color_mode);
    glUniform1i(color_map_location_, static_cast<int>(frame.style.color_map));
    glUniform2f(scalar_range_location_, frame.style.scalar_min,
                frame.style.scalar_max);
    glUniform3f(fixed_color_location_, frame.style.fixed_color.x(),
                frame.style.fixed_color.y(), frame.style.fixed_color.z());
    glUniform3f(noise_color_location_, frame.style.noise_color.x(),
                frame.style.noise_color.y(), frame.style.noise_color.z());
    glUniform1i(highlight_noise_location_, frame.style.highlight_noise);
    glUniform1i(round_points_location_, GL_TRUE);

    // Update and bind the intensity CDF lookup texture. The texture lives on a
    // dedicated unit so it never interferes with the (currently unused) point
    // sprite path that may rely on the default unit.
    const bool equalize_active =
        frame.intensity_cdf_valid && frame.style.intensity_equalize &&
        frame.style.color_by == ColorBy::Intensity;
    if (equalize_active) {
      std::array<unsigned char, 256> cdf_bytes{};
      for (std::size_t i = 0; i < cdf_bytes.size(); ++i) {
        const float sample =
            std::clamp(frame.intensity_cdf[i], 0.0F, 1.0F);
        cdf_bytes[i] = static_cast<unsigned char>(
            std::lround(sample * 255.0F));
      }
      glActiveTexture(GL_TEXTURE1);
      glBindTexture(GL_TEXTURE_2D, cdf_texture_);
      glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, 256, 1, GL_RED, GL_UNSIGNED_BYTE,
                      cdf_bytes.data());
      cdf_uploaded_ = true;
    } else if (cdf_uploaded_) {
      glActiveTexture(GL_TEXTURE1);
      glBindTexture(GL_TEXTURE_2D, cdf_texture_);
    }
    if (cdf_tex_location_ >= 0)
      glUniform1i(cdf_tex_location_, 1);
    if (equalize_location_ >= 0)
      glUniform1i(equalize_location_, equalize_active ? GL_TRUE : GL_FALSE);
    if (opacity_location_ >= 0)
      glUniform1f(opacity_location_, 1.0F);

    glBindVertexArray(vertex_array_);
    if (frame.interactive_lod && lod_point_count_ != 0) {
      glDrawElements(GL_POINTS, static_cast<GLsizei>(lod_point_count_),
                     GL_UNSIGNED_INT, nullptr);
    } else
    {
      glDrawArrays(GL_POINTS, 0, static_cast<GLsizei>(point_count_));
    }
  }
  const bool guides_changed = frame.guides.size() != uploaded_guides_.size() ||
                              !std::equal(
                                  frame.guides.begin(), frame.guides.end(),
                                  uploaded_guides_.begin(),
                                  [](const auto &left, const auto &right) {
                                    return left.position == right.position &&
                                           left.color == right.color;
                                  });
  if (guides_changed) {
    uploaded_guides_ = frame.guides;
    std::vector<GpuVertex> guides;
    guides.reserve(uploaded_guides_.size());
    for (const auto &vertex : uploaded_guides_) {
      if (finite(vertex))
        guides.push_back(
            {{vertex.position.x(), vertex.position.y(), vertex.position.z()},
             {vertex.color.x(), vertex.color.y(), vertex.color.z()}, 0.0F,
             0.0F});
    }
    guide_point_count_ = guides.size();
    if (guide_point_count_ != 0) {
      if (guides.size() >
          (std::numeric_limits<std::size_t>::max)() / sizeof(GpuVertex)) {
        return error(RendererErrorCode::EncodingFailed,
                     "OpenGL guide upload size overflows");
      }
      const auto guide_bytes = guides.size() * sizeof(GpuVertex);
      glUseProgram(program_);
      glUniformMatrix4fv(view_projection_location_, 1, GL_FALSE,
                         frame.view_projection.data());
      glUniform3f(world_origin_location_, frame.world_origin.x(),
                  frame.world_origin.y(), frame.world_origin.z());
      glUniform1f(world_scale_location_, frame.world_scale);
      glUniform1i(color_mode_location_, 0);
      glUniform1i(highlight_noise_location_, GL_FALSE);
      glUniform1i(round_points_location_, GL_FALSE);
      if (opacity_location_ >= 0)
        glUniform1f(opacity_location_, 1.0F);
      glBindVertexArray(guide_vertex_array_);
      glBindBuffer(GL_ARRAY_BUFFER, guide_vertex_buffer_);
      if (guide_bytes > guide_buffer_capacity_) {
        glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(guide_bytes),
                     guides.data(), GL_DYNAMIC_DRAW);
        guide_buffer_capacity_ = guide_bytes;
      } else {
        glBufferSubData(GL_ARRAY_BUFFER, 0,
                        static_cast<GLsizeiptr>(guide_bytes), guides.data());
      }
    }
  }
  if (guide_point_count_ != 0) {
    if (guide_point_count_ >
        static_cast<std::size_t>((std::numeric_limits<GLsizei>::max)())) {
      return error(RendererErrorCode::EncodingFailed,
                   "OpenGL guide count exceeds GLsizei");
    }
    glUseProgram(program_);
    glUniformMatrix4fv(view_projection_location_, 1, GL_FALSE,
                       frame.view_projection.data());
    glUniform3f(world_origin_location_, frame.world_origin.x(),
                frame.world_origin.y(), frame.world_origin.z());
    glUniform1f(world_scale_location_, frame.world_scale);
    glUniform1i(color_mode_location_, 0);
    glUniform1i(highlight_noise_location_, GL_FALSE);
    glUniform1i(round_points_location_, GL_FALSE);
    if (opacity_location_ >= 0)
      glUniform1f(opacity_location_, 1.0F);
    // Guides annotate geometry but must not change later depth outcomes. The
    // shared OpenGL/WebGL pass still depth-tests them against the cloud.
    glEnable(GL_DEPTH_TEST);
    glDepthMask(GL_FALSE);
    glBindVertexArray(guide_vertex_array_);
    glDrawArrays(GL_LINES, 0, static_cast<GLsizei>(guide_point_count_));
    glDepthMask(GL_TRUE);
  }
  const unsigned gl_error = glGetError();
  if (gl_error != GL_NO_ERROR) {
    return openGLError("OpenGL render failed", gl_error);
  }
  encoded_frame_ = frame;
  encoded_revision_ = uploaded_revision_;
  ++encoded_frame_count_;
  return {};
}

Result<void, RendererError>
OpenGLPointRenderer::renderLayers(const ViewportFrame &frame,
                                  const LayeredViewportFrame &layers,
                                  FrameContext &context) {
  if (context.backendKind() !=
#ifdef __EMSCRIPTEN__
      BackendKind::WebGL
#else
      BackendKind::OpenGL
#endif
  ) {
    return error(RendererErrorCode::BackendMismatch,
                 "OpenGL layered renderer received a non-OpenGL frame context");
  }
  const auto *open_gl_context = dynamic_cast<OpenGLFrameContext *>(&context);
  if (open_gl_context == nullptr || !open_gl_context->isActive() ||
      open_gl_context->expectedWindow() != expected_window_ ||
      !expectedContextIsCurrent()) {
    return error(RendererErrorCode::BackendMismatch,
                 "OpenGL layered frame context is inactive or belongs to another "
                 "GLFW context");
  }
  if (extent_.width == 0 || extent_.height == 0)
    return {};
  if (layers.revision == 0 || layers.revision != uploaded_layered_revision_) {
    return error(RendererErrorCode::EncodingFailed,
                 "OpenGL layered frame does not match uploaded scene revision");
  }

  RenderState saved(RenderState::Scope::Render);
  clearOpenGLErrors();
  glBindFramebuffer(GL_DRAW_FRAMEBUFFER, framebuffer_);
  glViewport(0, 0, extent_.width, extent_.height);
  glDisable(GL_SCISSOR_TEST);
  glDisable(GL_BLEND);
  glDisable(GL_RASTERIZER_DISCARD);
  glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
  glDepthMask(GL_TRUE);
  glDepthFunc(GL_LESS);
  glEnable(GL_DEPTH_TEST);
#ifndef __EMSCRIPTEN__
  glEnable(GL_PROGRAM_POINT_SIZE);
#endif
  glClearColor(frame.style.background.x(), frame.style.background.y(),
               frame.style.background.z(), 1.0F);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  const auto draw_layer = [this, &frame](const ViewportLayerDraw &draw)
      -> Result<void, RendererError> {
    const auto iterator = layer_buffers_.find(draw.layer_id);
    if (iterator == layer_buffers_.end()) {
      return error(RendererErrorCode::EncodingFailed,
                   "OpenGL layered draw references an unavailable layer buffer");
    }
    const LayerBuffer &buffer = iterator->second;
    if (buffer.point_count == 0 || draw.style.point_size <= 0.0F ||
        draw.opacity <= 0.0F) {
      return {};
    }
    if (buffer.point_count >
        static_cast<std::size_t>((std::numeric_limits<GLsizei>::max)())) {
      return error(RendererErrorCode::EncodingFailed,
                   "OpenGL layered point count exceeds GLsizei");
    }
    if (!std::isfinite(draw.opacity) || !draw.style.fixed_color.allFinite() ||
        !draw.style.noise_color.allFinite() ||
        !std::isfinite(draw.style.scalar_min) ||
        !std::isfinite(draw.style.scalar_max)) {
      return error(RendererErrorCode::EncodingFailed,
                   "OpenGL layered draw style is not finite");
    }

    glUseProgram(program_);
    glUniformMatrix4fv(view_projection_location_, 1, GL_FALSE,
                       frame.view_projection.data());
    glUniform3f(world_origin_location_, frame.world_origin.x(),
                frame.world_origin.y(), frame.world_origin.z());
    glUniform1f(world_scale_location_, frame.world_scale);
    glUniform1f(point_size_location_,
                std::clamp(draw.style.point_size, 0.0F, 5.0F));
    int color_mode = 4;
    if (draw.style.color_by == ColorBy::RGB ||
        draw.style.color_by == ColorBy::Label) {
      color_mode = 0;
    } else if (draw.style.color_by == ColorBy::Intensity) {
      color_mode = 1;
    } else if (draw.style.color_by == ColorBy::Z) {
      color_mode = 2;
    }
    glUniform1i(color_mode_location_, color_mode);
    glUniform1i(color_map_location_, static_cast<int>(draw.style.color_map));
    glUniform2f(scalar_range_location_, draw.style.scalar_min,
                draw.style.scalar_max);
    glUniform3f(fixed_color_location_, draw.style.fixed_color.x(),
                draw.style.fixed_color.y(), draw.style.fixed_color.z());
    glUniform3f(noise_color_location_, draw.style.noise_color.x(),
                draw.style.noise_color.y(), draw.style.noise_color.z());
    glUniform1i(highlight_noise_location_, draw.style.highlight_noise);
    glUniform1i(round_points_location_, GL_TRUE);
    if (opacity_location_ >= 0)
      glUniform1f(opacity_location_, std::clamp(draw.opacity, 0.0F, 1.0F));

    const bool equalize_active =
        draw.intensity_cdf_valid && draw.style.intensity_equalize &&
        draw.style.color_by == ColorBy::Intensity;
    if (equalize_active) {
      std::array<unsigned char, 256> cdf_bytes{};
      for (std::size_t index = 0; index < cdf_bytes.size(); ++index) {
        const float sample = std::clamp(draw.intensity_cdf[index], 0.0F, 1.0F);
        cdf_bytes[index] =
            static_cast<unsigned char>(std::lround(sample * 255.0F));
      }
      glActiveTexture(GL_TEXTURE1);
      glBindTexture(GL_TEXTURE_2D, cdf_texture_);
      glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, 256, 1, GL_RED,
                      GL_UNSIGNED_BYTE, cdf_bytes.data());
      cdf_uploaded_ = true;
    } else if (cdf_uploaded_) {
      glActiveTexture(GL_TEXTURE1);
      glBindTexture(GL_TEXTURE_2D, cdf_texture_);
    }
    if (cdf_tex_location_ >= 0)
      glUniform1i(cdf_tex_location_, 1);
    if (equalize_location_ >= 0)
      glUniform1i(equalize_location_, equalize_active ? GL_TRUE : GL_FALSE);
    glBindVertexArray(buffer.vertex_array);
    if (frame.interactive_lod && buffer.lod_point_count != 0) {
      if (buffer.lod_point_count >
          static_cast<std::size_t>((std::numeric_limits<GLsizei>::max)())) {
        return error(RendererErrorCode::EncodingFailed,
                     "OpenGL layered interactive LOD exceeds GLsizei");
      }
      glDrawElements(GL_POINTS, static_cast<GLsizei>(buffer.lod_point_count),
                     GL_UNSIGNED_INT, nullptr);
    } else {
      glDrawArrays(GL_POINTS, 0, static_cast<GLsizei>(buffer.point_count));
    }
    return {};
  };

  // Opaque layers establish the depth buffer. Transparent layers follow the
  // adapter's back-to-front order, depth-test against opaque geometry, and
  // never write depth themselves so later transparent layers remain visible.
  glDisable(GL_BLEND);
  glDepthMask(GL_TRUE);
  for (const auto &draw : layers.opaque_layers) {
    auto drawn = draw_layer(draw);
    if (!drawn)
      return drawn.error();
  }
  if (!layers.transparent_layers.empty()) {
    glEnable(GL_BLEND);
    glBlendEquationSeparate(GL_FUNC_ADD, GL_FUNC_ADD);
    glBlendFuncSeparate(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA, GL_ONE,
                        GL_ONE_MINUS_SRC_ALPHA);
    glDepthMask(GL_FALSE);
    for (const auto &draw : layers.transparent_layers) {
      auto drawn = draw_layer(draw);
      if (!drawn)
        return drawn.error();
    }
    glDepthMask(GL_TRUE);
    glDisable(GL_BLEND);
  }

  const bool guides_changed = frame.guides.size() != uploaded_guides_.size() ||
                              !std::equal(
                                  frame.guides.begin(), frame.guides.end(),
                                  uploaded_guides_.begin(),
                                  [](const auto &left, const auto &right) {
                                    return left.position == right.position &&
                                           left.color == right.color;
                                  });
  if (guides_changed) {
    uploaded_guides_ = frame.guides;
    std::vector<GpuVertex> guides;
    guides.reserve(uploaded_guides_.size());
    for (const auto &vertex : uploaded_guides_) {
      if (finite(vertex)) {
        guides.push_back(
            {{vertex.position.x(), vertex.position.y(), vertex.position.z()},
             {vertex.color.x(), vertex.color.y(), vertex.color.z()}, 0.0F,
             0.0F});
      }
    }
    guide_point_count_ = guides.size();
    if (guide_point_count_ != 0) {
      if (guides.size() >
          (std::numeric_limits<std::size_t>::max)() / sizeof(GpuVertex)) {
        return error(RendererErrorCode::EncodingFailed,
                     "OpenGL layered guide upload size overflows");
      }
      const auto guide_bytes = guides.size() * sizeof(GpuVertex);
      glBindVertexArray(guide_vertex_array_);
      glBindBuffer(GL_ARRAY_BUFFER, guide_vertex_buffer_);
      if (guide_bytes > guide_buffer_capacity_) {
        glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(guide_bytes),
                     guides.data(), GL_DYNAMIC_DRAW);
        guide_buffer_capacity_ = guide_bytes;
      } else {
        glBufferSubData(GL_ARRAY_BUFFER, 0,
                        static_cast<GLsizeiptr>(guide_bytes), guides.data());
      }
    }
  }
  if (guide_point_count_ != 0) {
    if (guide_point_count_ >
        static_cast<std::size_t>((std::numeric_limits<GLsizei>::max)())) {
      return error(RendererErrorCode::EncodingFailed,
                   "OpenGL layered guide count exceeds GLsizei");
    }
    glUseProgram(program_);
    glUniformMatrix4fv(view_projection_location_, 1, GL_FALSE,
                       frame.view_projection.data());
    glUniform3f(world_origin_location_, frame.world_origin.x(),
                frame.world_origin.y(), frame.world_origin.z());
    glUniform1f(world_scale_location_, frame.world_scale);
    glUniform1i(color_mode_location_, 0);
    glUniform1i(highlight_noise_location_, GL_FALSE);
    glUniform1i(round_points_location_, GL_FALSE);
    if (opacity_location_ >= 0)
      glUniform1f(opacity_location_, 1.0F);
    // Keep depth tests for occlusion while preventing the final guide pass
    // from writing depth; this matches the transparent-layer contract.
    glEnable(GL_DEPTH_TEST);
    glDepthMask(GL_FALSE);
    glBindVertexArray(guide_vertex_array_);
    glDrawArrays(GL_LINES, 0, static_cast<GLsizei>(guide_point_count_));
    glDepthMask(GL_TRUE);
  }

  const unsigned gl_error = glGetError();
  if (gl_error != GL_NO_ERROR) {
    return openGLError("OpenGL layered render failed", gl_error);
  }
  ++encoded_frame_count_;
  return {};
}

Result<Rgba8Image, RendererError> OpenGLPointRenderer::captureRgba() const {
  if (!expectedContextIsCurrent()) {
    return error(RendererErrorCode::BackendMismatch,
                 "OpenGL capture used a non-current expected context");
  }
  if (framebuffer_ == 0 || extent_.width <= 0 || extent_.height <= 0) {
    return error(RendererErrorCode::EncodingFailed,
                 "OpenGL capture requires a rendered non-empty viewport");
  }
  if (static_cast<std::size_t>(extent_.width) >
      (std::numeric_limits<std::size_t>::max)() / std::size_t{4}) {
    return error(RendererErrorCode::EncodingFailed,
                 "OpenGL capture row size overflows");
  }
  const std::size_t bytes_per_row =
      static_cast<std::size_t>(extent_.width) * std::size_t{4};
  if (static_cast<std::size_t>(extent_.height) >
      (std::numeric_limits<std::size_t>::max)() / bytes_per_row) {
    return error(RendererErrorCode::EncodingFailed,
                 "OpenGL capture image size overflows");
  }

  // Screenshot callers must not inherit a changed framebuffer, pack buffer,
  // or row layout.  WebGL2 exposes this same ES3 state, so this is one
  // implementation for desktop OpenGL and the browser backend.
  RenderState saved(RenderState::Scope::Full);
  int pixel_pack_buffer = 0;
  int pack_alignment = 0;
  int pack_row_length = 0;
  int pack_skip_rows = 0;
  int pack_skip_pixels = 0;
  glGetIntegerv(GL_PIXEL_PACK_BUFFER_BINDING, &pixel_pack_buffer);
  glGetIntegerv(GL_PACK_ALIGNMENT, &pack_alignment);
  glGetIntegerv(GL_PACK_ROW_LENGTH, &pack_row_length);
  glGetIntegerv(GL_PACK_SKIP_ROWS, &pack_skip_rows);
  glGetIntegerv(GL_PACK_SKIP_PIXELS, &pack_skip_pixels);

  clearOpenGLErrors();
  glBindFramebuffer(GL_READ_FRAMEBUFFER, framebuffer_);
  glReadBuffer(GL_COLOR_ATTACHMENT0);
  glBindBuffer(GL_PIXEL_PACK_BUFFER, 0);
  glPixelStorei(GL_PACK_ALIGNMENT, 1);
  glPixelStorei(GL_PACK_ROW_LENGTH, 0);
  glPixelStorei(GL_PACK_SKIP_ROWS, 0);
  glPixelStorei(GL_PACK_SKIP_PIXELS, 0);

  Rgba8Image result;
  result.extent = extent_;
  result.bytes_per_row = bytes_per_row;
  result.pixels.resize(bytes_per_row * static_cast<std::size_t>(extent_.height));
  glReadPixels(0, 0, extent_.width, extent_.height, GL_RGBA, GL_UNSIGNED_BYTE,
               result.pixels.data());
  const unsigned read_error = glGetError();

  glPixelStorei(GL_PACK_SKIP_PIXELS, pack_skip_pixels);
  glPixelStorei(GL_PACK_SKIP_ROWS, pack_skip_rows);
  glPixelStorei(GL_PACK_ROW_LENGTH, pack_row_length);
  glPixelStorei(GL_PACK_ALIGNMENT, pack_alignment);
  glBindBuffer(GL_PIXEL_PACK_BUFFER,
               static_cast<unsigned>(pixel_pack_buffer));

  if (read_error != GL_NO_ERROR) {
    return openGLError("OpenGL capture failed", read_error);
  }

  // GL and WebGL return their first row at the framebuffer's lower edge;
  // public capture data always uses top-left UI order.
  for (int top = 0, bottom = extent_.height - 1; top < bottom;
       ++top, --bottom) {
    const auto top_begin = result.pixels.begin() +
                           static_cast<std::ptrdiff_t>(top) *
                               static_cast<std::ptrdiff_t>(bytes_per_row);
    const auto bottom_begin = result.pixels.begin() +
                              static_cast<std::ptrdiff_t>(bottom) *
                                  static_cast<std::ptrdiff_t>(bytes_per_row);
    std::swap_ranges(top_begin,
                     top_begin + static_cast<std::ptrdiff_t>(bytes_per_row),
                     bottom_begin);
  }
  return result;
}

ViewportTexture OpenGLPointRenderer::texture() const {
  return {ImTextureRef{static_cast<ImTextureID>(color_texture_)},
          ImVec2{0.0F, 1.0F}, ImVec2{1.0F, 0.0F}};
}

} // namespace kpt::gui
