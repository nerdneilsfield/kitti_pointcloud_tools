#pragma once

#include "common/result.hpp"
#include "gui/viewport/render_types.hpp"

#include <cstdint>
#include <span>

namespace kpt::gui {

class FrameContext {
public:
  virtual ~FrameContext() = default;

  [[nodiscard]] virtual BackendKind backendKind() const noexcept = 0;
};

class ViewportRenderer {
public:
  virtual ~ViewportRenderer() = default;

  virtual Result<void, RendererError>
  upload(std::span<const ViewportVertex> vertices, std::uint64_t revision) = 0;
  virtual Result<void, RendererError> resize(PixelExtent physical_pixels) = 0;
  virtual Result<void, RendererError> render(const ViewportFrame &frame,
                                             FrameContext &context) = 0;

  [[nodiscard]] virtual ViewportTexture texture() const = 0;
  [[nodiscard]] virtual PixelExtent extent() const = 0;
  [[nodiscard]] virtual BackendKind backendKind() const noexcept = 0;
};

} // namespace kpt::gui
