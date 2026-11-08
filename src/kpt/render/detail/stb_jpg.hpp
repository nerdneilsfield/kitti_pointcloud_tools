#pragma once

#include "kpt/render/image.hpp"

#include <cstdint>
#include <vector>

namespace kpt::render_detail {

// Encodes a tightly packed RGB image as a JPEG in memory. `view.stride_bytes`
// must equal `view.width * 3` because stb's JPEG entry point takes no stride.
// `quality` above 90 keeps stb on 4:4:4 chroma; at 90 or below stb enables
// 4:2:0 subsampling and edge-pads odd dimensions into 16x16 MCUs.
// Throws std::invalid_argument for a non-tight or undersized view and
// std::runtime_error when the encoder fails.
[[nodiscard]] std::vector<std::uint8_t> writeJpegToMemory(kpt::ImageView view,
                                                          int quality);

} // namespace kpt::render_detail
