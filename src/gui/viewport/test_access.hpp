#pragma once

#include "common/result.hpp"
#include "gui/viewport/renderer.hpp"

#include <cstdint>
#include <vector>

namespace kpt::gui {

struct RendererReadback {
  PixelExtent extent;
  // RGBA8 rows in top-left UI order.
  std::vector<std::uint8_t> rgba;
};

class RendererTestAccess {
public:
  static Result<RendererReadback, RendererError>
  readColor(const ViewportRenderer &renderer);
};

} // namespace kpt::gui
