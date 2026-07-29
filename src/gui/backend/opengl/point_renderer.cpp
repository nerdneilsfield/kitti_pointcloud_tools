#include "gui/backend/opengl/point_renderer.hpp"

#include <glad/gl.h>

#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>

#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

namespace kpt::gui {
namespace {

struct GpuVertex {
  float position[3];
  float color[3];
  float intensity;
};

RendererError error(RendererErrorCode code, std::string message) {
  return {code, std::move(message)};
}

Result<void, RendererError> loadOpenGL(GLFWwindow *expected_window) {
  if (expected_window == nullptr ||
      glfwGetCurrentContext() != expected_window) {
    return error(RendererErrorCode::BackendMismatch,
                 "OpenGL loader requires the expected current GLFW context");
  }

  static std::mutex mutex;
  static bool loaded = false;
  std::scoped_lock lock(mutex);
  if (loaded)
    return {};

  const int version =
      gladLoadGL(reinterpret_cast<GLADloadfunc>(glfwGetProcAddress));
  if (version == 0 || GLAD_VERSION_MAJOR(version) < 3 ||
      (GLAD_VERSION_MAJOR(version) == 3 && GLAD_VERSION_MINOR(version) < 3)) {
    return error(RendererErrorCode::ResourceCreationFailed,
                 "GLAD could not load an OpenGL 3.3 core context");
  }
  loaded = true;
  return {};
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

  RenderState() {
    glGetIntegerv(GL_DRAW_FRAMEBUFFER_BINDING, &draw_framebuffer);
    glGetIntegerv(GL_READ_FRAMEBUFFER_BINDING, &read_framebuffer);
    glGetIntegerv(GL_VIEWPORT, viewport.data());
    glGetIntegerv(GL_CURRENT_PROGRAM, &program);
    glGetIntegerv(GL_VERTEX_ARRAY_BINDING, &vertex_array);
    glGetIntegerv(GL_ARRAY_BUFFER_BINDING, &array_buffer);
    glGetIntegerv(GL_ACTIVE_TEXTURE, &active_texture);
    glGetIntegerv(GL_TEXTURE_BINDING_2D, &texture_2d);
    glGetIntegerv(GL_RENDERBUFFER_BINDING, &renderbuffer);
    glGetFloatv(GL_COLOR_CLEAR_VALUE, clear_color.data());
    glGetIntegerv(GL_SCISSOR_BOX, scissor_box.data());
    glGetBooleanv(GL_COLOR_WRITEMASK, color_mask.data());
    glGetBooleanv(GL_DEPTH_WRITEMASK, &depth_write_mask);
    glGetIntegerv(GL_DEPTH_FUNC, &depth_func);
    glGetIntegerv(GL_BLEND_SRC_RGB, &blend_src_rgb);
    glGetIntegerv(GL_BLEND_DST_RGB, &blend_dst_rgb);
    glGetIntegerv(GL_BLEND_SRC_ALPHA, &blend_src_alpha);
    glGetIntegerv(GL_BLEND_DST_ALPHA, &blend_dst_alpha);
    glGetIntegerv(GL_BLEND_EQUATION_RGB, &blend_equation_rgb);
    glGetIntegerv(GL_BLEND_EQUATION_ALPHA, &blend_equation_alpha);
    depth_test = glIsEnabled(GL_DEPTH_TEST) == GL_TRUE;
    scissor_test = glIsEnabled(GL_SCISSOR_TEST) == GL_TRUE;
    blend = glIsEnabled(GL_BLEND) == GL_TRUE;
    rasterizer_discard = glIsEnabled(GL_RASTERIZER_DISCARD) == GL_TRUE;
    program_point_size = glIsEnabled(GL_PROGRAM_POINT_SIZE) == GL_TRUE;
  }

  ~RenderState() {
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER,
                      static_cast<unsigned>(draw_framebuffer));
    glBindFramebuffer(GL_READ_FRAMEBUFFER,
                      static_cast<unsigned>(read_framebuffer));
    glViewport(viewport[0], viewport[1], viewport[2], viewport[3]);
    glUseProgram(static_cast<unsigned>(program));
    glBindVertexArray(static_cast<unsigned>(vertex_array));
    glBindBuffer(GL_ARRAY_BUFFER, static_cast<unsigned>(array_buffer));
    glActiveTexture(static_cast<unsigned>(active_texture));
    glBindTexture(GL_TEXTURE_2D, static_cast<unsigned>(texture_2d));
    glBindRenderbuffer(GL_RENDERBUFFER, static_cast<unsigned>(renderbuffer));
    glScissor(scissor_box[0], scissor_box[1], scissor_box[2],
              scissor_box[3]);
    glColorMask(color_mask[0], color_mask[1], color_mask[2], color_mask[3]);
    glDepthMask(depth_write_mask);
    glDepthFunc(static_cast<unsigned>(depth_func));
    glBlendFuncSeparate(static_cast<unsigned>(blend_src_rgb),
                        static_cast<unsigned>(blend_dst_rgb),
                        static_cast<unsigned>(blend_src_alpha),
                        static_cast<unsigned>(blend_dst_alpha));
    glBlendEquationSeparate(static_cast<unsigned>(blend_equation_rgb),
                            static_cast<unsigned>(blend_equation_alpha));
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
    if (program_point_size)
      glEnable(GL_PROGRAM_POINT_SIZE);
    else
      glDisable(GL_PROGRAM_POINT_SIZE);
    glClearColor(clear_color[0], clear_color[1], clear_color[2],
                 clear_color[3]);
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
         std::isfinite(vertex.intensity);
}

void clearOpenGLErrors() {
  while (glGetError() != GL_NO_ERROR) {
  }
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
    color_mode_location_ = glGetUniformLocation(program_, "color_mode");
    scalar_range_location_ = glGetUniformLocation(program_, "scalar_range");
    glGenVertexArrays(1, &vertex_array_);
    glGenBuffers(1, &vertex_buffer_);
    if (vertex_array_ == 0 || vertex_buffer_ == 0)
      throw std::runtime_error("OpenGL vertex resource creation returned zero");
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
  } catch (const std::exception &exception) {
    destroyStaticResources();
    return error(RendererErrorCode::ResourceCreationFailed, exception.what());
  }
  return {};
}

void OpenGLPointRenderer::destroyStaticResources() noexcept {
  if (vertex_buffer_ != 0)
    glDeleteBuffers(1, &vertex_buffer_);
  if (vertex_array_ != 0)
    glDeleteVertexArrays(1, &vertex_array_);
  if (program_ != 0)
    glDeleteProgram(program_);
  vertex_buffer_ = 0;
  vertex_array_ = 0;
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
         vertex.intensity});
  }

  RenderState saved;
  unsigned new_vertex_array = 0;
  unsigned new_vertex_buffer = 0;
  glGenVertexArrays(1, &new_vertex_array);
  glGenBuffers(1, &new_vertex_buffer);
  if (new_vertex_array == 0 || new_vertex_buffer == 0) {
    if (new_vertex_buffer != 0)
      glDeleteBuffers(1, &new_vertex_buffer);
    if (new_vertex_array != 0)
      glDeleteVertexArrays(1, &new_vertex_array);
    return error(RendererErrorCode::ResourceCreationFailed,
                 "OpenGL upload resource creation returned zero");
  }
  clearOpenGLErrors();
  glBindVertexArray(new_vertex_array);
  glBindBuffer(GL_ARRAY_BUFFER, new_vertex_buffer);
  glBufferData(GL_ARRAY_BUFFER,
               static_cast<GLsizeiptr>(copied.size() * sizeof(GpuVertex)),
               copied.empty() ? nullptr : copied.data(), GL_STATIC_DRAW);
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
  const unsigned gl_error = glGetError();
  if (gl_error != GL_NO_ERROR) {
    glDeleteBuffers(1, &new_vertex_buffer);
    glDeleteVertexArrays(1, &new_vertex_array);
    return error(RendererErrorCode::EncodingFailed,
                 "OpenGL vertex upload failed with error " +
                     std::to_string(gl_error));
  }

  const unsigned old_vertex_array = vertex_array_;
  const unsigned old_vertex_buffer = vertex_buffer_;
  vertex_array_ = new_vertex_array;
  vertex_buffer_ = new_vertex_buffer;
  saved.replaceVertexObjects(old_vertex_array, new_vertex_array,
                             old_vertex_buffer, new_vertex_buffer);
  if (old_vertex_buffer != 0)
    glDeleteBuffers(1, &old_vertex_buffer);
  if (old_vertex_array != 0)
    glDeleteVertexArrays(1, &old_vertex_array);
  point_count_ = copied.size();
  uploaded_revision_ = revision;
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
    RenderState saved;
    saved.replaceFramebufferObjects(framebuffer_, 0, color_texture_, 0,
                                    depth_buffer_, 0);
    destroyFramebuffer();
    extent_ = {};
    return {};
  }

  RenderState saved;
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
  glBindRenderbuffer(GL_RENDERBUFFER, new_depth_buffer);
  glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT24,
                        physical_pixels.width, physical_pixels.height);
  glBindFramebuffer(GL_FRAMEBUFFER, new_framebuffer);
  glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D,
                         new_texture, 0);
  glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT,
                            GL_RENDERBUFFER, new_depth_buffer);
  const bool complete =
      glCheckFramebufferStatus(GL_FRAMEBUFFER) == GL_FRAMEBUFFER_COMPLETE;
  const unsigned gl_error = glGetError();
  if (!complete || gl_error != GL_NO_ERROR) {
    glDeleteRenderbuffers(1, &new_depth_buffer);
    glDeleteTextures(1, &new_texture);
    glDeleteFramebuffers(1, &new_framebuffer);
    return error(RendererErrorCode::ResourceCreationFailed,
                 "OpenGL framebuffer is incomplete");
  }

  const unsigned old_framebuffer = framebuffer_;
  const unsigned old_texture = color_texture_;
  const unsigned old_depth_buffer = depth_buffer_;
  framebuffer_ = new_framebuffer;
  color_texture_ = new_texture;
  depth_buffer_ = new_depth_buffer;
  extent_ = physical_pixels;
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
  if (context.backendKind() != BackendKind::OpenGL) {
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

  RenderState saved;
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
  glEnable(GL_PROGRAM_POINT_SIZE);
  glClearColor(frame.style.background.x(), frame.style.background.y(),
               frame.style.background.z(), 1.0F);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  if (point_count_ != 0) {
    glUseProgram(program_);
    glUniformMatrix4fv(view_projection_location_, 1, GL_FALSE,
                       frame.view_projection.data());
    glUniform1f(point_size_location_,
                std::clamp(frame.style.point_size, 1.0F, 64.0F));
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
    glUniform2f(scalar_range_location_, frame.style.scalar_min,
                frame.style.scalar_max);
    glBindVertexArray(vertex_array_);
    glDrawArrays(GL_POINTS, 0, static_cast<int>(point_count_));
  }
  const unsigned gl_error = glGetError();
  if (gl_error != GL_NO_ERROR) {
    return error(RendererErrorCode::EncodingFailed,
                 "OpenGL render failed with error " + std::to_string(gl_error));
  }
  return {};
}

ViewportTexture OpenGLPointRenderer::texture() const {
  return {ImTextureRef{static_cast<ImTextureID>(color_texture_)},
          ImVec2{0.0F, 1.0F}, ImVec2{1.0F, 0.0F}};
}

} // namespace kpt::gui
