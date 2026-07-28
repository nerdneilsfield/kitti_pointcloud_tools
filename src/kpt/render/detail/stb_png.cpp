#include "kpt/render/detail/stb_png.hpp"

#define STB_IMAGE_WRITE_STATIC
#define STBI_WRITE_NO_STDIO
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include <stb_image_write.h>

namespace kpt::render_detail {

int writePng(PngWriteCallback callback, void *context, int width, int height,
             int components, const void *pixels, int stride_bytes) {
  return stbi_write_png_to_func(callback, context, width, height, components,
                                pixels, stride_bytes);
}

} // namespace kpt::render_detail
