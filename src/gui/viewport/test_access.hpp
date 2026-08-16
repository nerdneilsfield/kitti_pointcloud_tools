#pragma once

#include "common/result.hpp"
#include "gui/viewport/renderer.hpp"

#include <memory>

namespace kpt::gui {

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
