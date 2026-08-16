#pragma once

#include "gui/viewport/renderer.hpp"

#include <cstddef>
#include <cstdint>
#include <memory>

namespace kpt::gui {

class MetalRendererTestAccess;

class MetalFrameContext final : public FrameContext {
public:
  [[nodiscard]] BackendKind backendKind() const noexcept override {
    return BackendKind::Metal;
  }

private:
  friend class GlfwMetalRuntime;
  friend class MetalPointRenderer;
  friend class MetalRendererTestAccess;

  explicit MetalFrameContext(void *device = nullptr,
                             void *command_queue = nullptr,
                             void *command_buffer = nullptr,
                             bool active = false) noexcept
      : device_(device), command_queue_(command_queue),
        command_buffer_(command_buffer), active_(active) {}

  void activate(void *device, void *command_queue,
                void *command_buffer) noexcept {
    device_ = device;
    command_queue_ = command_queue;
    command_buffer_ = command_buffer;
    active_ = true;
  }
  void invalidate() noexcept {
    command_buffer_ = nullptr;
    active_ = false;
  }
  [[nodiscard]] void *device() const noexcept { return device_; }
  [[nodiscard]] void *commandQueue() const noexcept { return command_queue_; }
  [[nodiscard]] void *commandBuffer() const noexcept { return command_buffer_; }
  [[nodiscard]] bool isActive() const noexcept { return active_; }

  void *device_ = nullptr;
  void *command_queue_ = nullptr;
  void *command_buffer_ = nullptr;
  bool active_ = false;
};

class MetalPointRenderer final : public ViewportRenderer {
public:
  MetalPointRenderer(void *device, void *command_queue);
  ~MetalPointRenderer() override;
  MetalPointRenderer(const MetalPointRenderer &) = delete;
  MetalPointRenderer &operator=(const MetalPointRenderer &) = delete;

  Result<void, RendererError> upload(std::span<const ViewportVertex> vertices,
                                     std::uint64_t revision) override;
  Result<void, RendererError>
  uploadLayers(std::span<const ViewportLayerUpload> layers,
               std::uint64_t scene_revision) override;
  Result<void, RendererError> resize(PixelExtent physical_pixels) override;
  Result<void, RendererError> render(const ViewportFrame &frame,
                                     FrameContext &context) override;
  Result<void, RendererError> renderLayers(const ViewportFrame &frame,
                                           const LayeredViewportFrame &layers,
                                           FrameContext &context) override;

  [[nodiscard]] ViewportTexture texture() const override;
  [[nodiscard]] PixelExtent extent() const override;
  [[nodiscard]] BackendKind backendKind() const noexcept override {
    return BackendKind::Metal;
  }
  [[nodiscard]] std::size_t pointCount() const noexcept;
  [[nodiscard]] std::uint64_t uploadedRevision() const noexcept;

private:
  friend class MetalRendererTestAccess;
  [[nodiscard]] void *colorTextureForTests() const noexcept;
  [[nodiscard]] std::uint64_t encodedFrameCountForTests() const noexcept;
  [[nodiscard]] std::size_t
  layeredLodPointCountForTests(std::uint64_t layer_id) const noexcept;
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

} // namespace kpt::gui
