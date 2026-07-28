#include "gui/viewport/test_access.hpp"

#include "gui/backend/opengl/point_renderer.hpp"

#include <glad/gl.h>

#include <algorithm>
#include <cstddef>
#include <utility>

namespace kpt::gui {

Result<RendererReadback, RendererError>
RendererTestAccess::readColor(const ViewportRenderer &renderer) {
  const auto *open_gl = dynamic_cast<const OpenGLPointRenderer *>(&renderer);
  if (open_gl == nullptr) {
    return RendererError{RendererErrorCode::BackendMismatch,
                         "OpenGL readback requires OpenGLPointRenderer"};
  }
  if (!open_gl->expectedContextIsCurrent()) {
    return RendererError{RendererErrorCode::BackendMismatch,
                         "OpenGL readback used the wrong current context"};
  }

  RendererReadback result;
  result.extent = open_gl->extent_;
  if (result.extent.width == 0 || result.extent.height == 0)
    return result;

  int read_framebuffer = 0;
  int read_buffer = 0;
  int pixel_pack_buffer = 0;
  int pack_alignment = 0;
  int pack_row_length = 0;
  int pack_skip_rows = 0;
  int pack_skip_pixels = 0;
  glGetIntegerv(GL_READ_FRAMEBUFFER_BINDING, &read_framebuffer);
  glGetIntegerv(GL_READ_BUFFER, &read_buffer);
  glGetIntegerv(GL_PIXEL_PACK_BUFFER_BINDING, &pixel_pack_buffer);
  glGetIntegerv(GL_PACK_ALIGNMENT, &pack_alignment);
  glGetIntegerv(GL_PACK_ROW_LENGTH, &pack_row_length);
  glGetIntegerv(GL_PACK_SKIP_ROWS, &pack_skip_rows);
  glGetIntegerv(GL_PACK_SKIP_PIXELS, &pack_skip_pixels);

  glBindFramebuffer(GL_READ_FRAMEBUFFER, open_gl->framebuffer_);
  glReadBuffer(GL_COLOR_ATTACHMENT0);
  glBindBuffer(GL_PIXEL_PACK_BUFFER, 0);
  glPixelStorei(GL_PACK_ALIGNMENT, 1);
  glPixelStorei(GL_PACK_ROW_LENGTH, 0);
  glPixelStorei(GL_PACK_SKIP_ROWS, 0);
  glPixelStorei(GL_PACK_SKIP_PIXELS, 0);
  while (glGetError() != GL_NO_ERROR) {
  }

  const std::size_t row_bytes =
      static_cast<std::size_t>(result.extent.width) * 4;
  result.rgba.resize(row_bytes *
                     static_cast<std::size_t>(result.extent.height));
  glReadPixels(0, 0, result.extent.width, result.extent.height, GL_RGBA,
               GL_UNSIGNED_BYTE, result.rgba.data());
  const unsigned read_error = glGetError();

  glPixelStorei(GL_PACK_SKIP_PIXELS, pack_skip_pixels);
  glPixelStorei(GL_PACK_SKIP_ROWS, pack_skip_rows);
  glPixelStorei(GL_PACK_ROW_LENGTH, pack_row_length);
  glPixelStorei(GL_PACK_ALIGNMENT, pack_alignment);
  glBindBuffer(GL_PIXEL_PACK_BUFFER, static_cast<unsigned>(pixel_pack_buffer));
  glBindFramebuffer(GL_READ_FRAMEBUFFER,
                    static_cast<unsigned>(read_framebuffer));
  glReadBuffer(static_cast<unsigned>(read_buffer));

  if (read_error != GL_NO_ERROR) {
    return RendererError{RendererErrorCode::EncodingFailed,
                         "OpenGL readback failed with error " +
                             std::to_string(read_error)};
  }

  for (int top = 0, bottom = result.extent.height - 1; top < bottom;
       ++top, --bottom) {
    const auto top_begin =
        result.rgba.begin() + static_cast<std::ptrdiff_t>(top) *
                                  static_cast<std::ptrdiff_t>(row_bytes);
    const auto bottom_begin =
        result.rgba.begin() + static_cast<std::ptrdiff_t>(bottom) *
                                  static_cast<std::ptrdiff_t>(row_bytes);
    std::swap_ranges(top_begin,
                     top_begin + static_cast<std::ptrdiff_t>(row_bytes),
                     bottom_begin);
  }
  return result;
}

} // namespace kpt::gui
