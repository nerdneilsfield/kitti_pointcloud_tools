#include "kpt/render/detail/stb_jpg.hpp"

// stb_png.cpp owns the stb_image_write implementation but compiles it with
// STB_IMAGE_WRITE_STATIC, which gives every stb function internal linkage.
// A translation unit that only includes the header therefore cannot link
// against stb_png.cpp's copy. This file carries its own private
// implementation; both copies have internal linkage, so no symbol is
// exported twice and the two never interact.
#define STB_IMAGE_WRITE_STATIC
#define STBI_WRITE_NO_STDIO
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include <stb_image_write.h>

#include <stdexcept>

namespace kpt::render_detail {
namespace {

void appendJpegBytes(void *context, void *data, int size) {
  auto &out = *static_cast<std::vector<std::uint8_t> *>(context);
  const auto *bytes = static_cast<const std::uint8_t *>(data);
  out.insert(out.end(), bytes, bytes + static_cast<std::size_t>(size));
}

} // namespace

std::vector<std::uint8_t> writeJpegToMemory(kpt::ImageView view, int quality) {
  if (view.width <= 0 || view.height <= 0)
    throw std::invalid_argument("JPEG image dimensions must be positive");
  if (view.stride_bytes != view.width * 3)
    throw std::invalid_argument("JPEG input must be tightly packed RGB");
  if (view.pixels.size() <
      static_cast<std::size_t>(view.stride_bytes) *
          static_cast<std::size_t>(view.height))
    throw std::invalid_argument("JPEG pixel buffer is too small");
  std::vector<std::uint8_t> out;
  const int encoded =
      stbi_write_jpg_to_func(appendJpegBytes, &out, view.width, view.height, 3,
                             view.pixels.data(), quality);
  if (encoded == 0)
    throw std::runtime_error("failed to encode JPEG image");
  return out;
}

} // namespace kpt::render_detail
