#pragma once

namespace kpt::render_detail {

using PngWriteCallback = void (*)(void *context, void *data, int size);

int writePng(PngWriteCallback callback, void *context, int width, int height,
             int components, const void *pixels, int stride_bytes);

} // namespace kpt::render_detail
