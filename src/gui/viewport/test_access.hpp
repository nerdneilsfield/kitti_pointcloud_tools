#pragma once

#include "common/result.hpp"
#include "gui/viewport/renderer.hpp"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

namespace kpt::gui {

struct Rgba8Image {
  PixelExtent extent;
  // RGBA8 rows in top-left UI order.
  std::vector<std::uint8_t> pixels;
  std::size_t bytes_per_row = 0;
};

class RendererTestAccess {
public:
  virtual ~RendererTestAccess() = default;

  virtual Result<Rgba8Image, RendererError>
  readColor(const ViewportRenderer &renderer) = 0;
};

struct RendererTestFixture {
  std::unique_ptr<ViewportRenderer> renderer;
  std::unique_ptr<RendererTestAccess> readback;
};

} // namespace kpt::gui
