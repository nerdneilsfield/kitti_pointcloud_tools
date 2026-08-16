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
  // Layered review rendering is opt-in so lightweight test/fallback renderers
  // retain the established single-cloud contract.  Concrete native backends
  // override both methods and keep one GPU buffer per stable layer ID.
  virtual Result<void, RendererError>
  uploadLayers(std::span<const ViewportLayerUpload> layers,
               std::uint64_t scene_revision) {
    static_cast<void>(layers);
    static_cast<void>(scene_revision);
    return RendererError{RendererErrorCode::EncodingFailed,
                         "renderer does not support layered viewport uploads"};
  }
  virtual Result<void, RendererError> resize(PixelExtent physical_pixels) = 0;
  virtual Result<void, RendererError> render(const ViewportFrame &frame,
                                             FrameContext &context) = 0;
  virtual Result<void, RendererError>
  renderLayers(const ViewportFrame &frame, const LayeredViewportFrame &layers,
               FrameContext &context) {
    static_cast<void>(frame);
    static_cast<void>(layers);
    static_cast<void>(context);
    return RendererError{RendererErrorCode::EncodingFailed,
                         "renderer does not support layered viewport rendering"};
  }

  [[nodiscard]] virtual ViewportTexture texture() const = 0;
  [[nodiscard]] virtual PixelExtent extent() const = 0;
  [[nodiscard]] virtual BackendKind backendKind() const noexcept = 0;
};

} // namespace kpt::gui
