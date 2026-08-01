#include "gui/backend/metal/test_support.hpp"

#include "gui/backend/metal/point_renderer.hpp"

#include <algorithm>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <utility>

#import <Metal/Metal.h>

namespace kpt::gui {
namespace {

RendererError error(RendererErrorCode code, std::string message) {
  return {code, std::move(message)};
}

} // namespace

class MetalRendererTestAccess final : public RendererTestAccess {
public:
  static std::unique_ptr<MetalFrameContext>
  makeContext(id<MTLDevice> device, id<MTLCommandQueue> queue, bool active) {
    return std::unique_ptr<MetalFrameContext>(new MetalFrameContext(
        (__bridge void *)device, (__bridge void *)queue, nullptr, active));
  }

  static Result<std::reference_wrapper<FrameContext>, RendererError>
  begin(MetalFrameContext &context, std::shared_ptr<void> &retained_command) {
    id<MTLDevice> device = (__bridge id<MTLDevice>)context.device();
    id<MTLCommandQueue> queue =
        (__bridge id<MTLCommandQueue>)context.commandQueue();
    id<MTLCommandBuffer> command = [queue commandBuffer];
    if (command == nil)
      return error(RendererErrorCode::ResourceCreationFailed,
                   "Metal test command buffer creation failed");
    retained_command.reset((__bridge_retained void *)command, [](void *value) {
      if (value != nullptr)
        static_cast<void>(CFBridgingRelease(value));
    });
    context.activate((__bridge void *)device, (__bridge void *)queue,
                     retained_command.get());
    return std::ref(static_cast<FrameContext &>(context));
  }

  static std::uint64_t encodedFrameCount(const MetalPointRenderer &renderer) {
    return renderer.encodedFrameCountForTests();
  }

  MetalRendererTestAccess(id<MTLDevice> device, MetalFrameContext *context)
      : device_(device), context_(context) {}

  Result<Rgba8Image, RendererError>
  readColor(const ViewportRenderer &renderer) override {
    const auto *metal_renderer =
        dynamic_cast<const MetalPointRenderer *>(&renderer);
    if (metal_renderer == nullptr || context_ == nullptr ||
        !context_->isActive() || context_->commandBuffer() == nullptr)
      return error(RendererErrorCode::BackendMismatch,
                   "Metal readback requires an active matching frame");
    id<MTLTexture> texture =
        (__bridge id<MTLTexture>)metal_renderer->colorTextureForTests();
    if (texture == nil)
      return Rgba8Image{};

    const NSUInteger packed_row = texture.width * 4;
    const NSUInteger aligned_row = (packed_row + 255) & ~NSUInteger(255);
    const NSUInteger size = aligned_row * texture.height;
    id<MTLBuffer> buffer =
        [device_ newBufferWithLength:size options:MTLResourceStorageModeShared];
    id<MTLCommandBuffer> command =
        (__bridge id<MTLCommandBuffer>)context_->commandBuffer();
    id<MTLBlitCommandEncoder> blit = [command blitCommandEncoder];
    if (buffer == nil || blit == nil)
      return error(RendererErrorCode::ResourceCreationFailed,
                   "Metal readback resource creation failed");
    [blit copyFromTexture:texture
             sourceSlice:0
             sourceLevel:0
            sourceOrigin:MTLOriginMake(0, 0, 0)
              sourceSize:MTLSizeMake(texture.width, texture.height, 1)
                toBuffer:buffer
       destinationOffset:0
  destinationBytesPerRow:aligned_row
destinationBytesPerImage:size];
    [blit endEncoding];
    [command commit];
    [command waitUntilCompleted];
    context_->invalidate();
    if (command.status == MTLCommandBufferStatusError) {
      const char *message = command.error.localizedDescription.UTF8String;
      return error(RendererErrorCode::EncodingFailed,
                   std::string("Metal readback failed: ") +
                       (message == nullptr ? "unknown error" : message));
    }

    Rgba8Image image;
    image.extent = {static_cast<int>(texture.width),
                    static_cast<int>(texture.height)};
    image.bytes_per_row = packed_row;
    image.pixels.resize(packed_row * texture.height);
    const auto *source = static_cast<const std::uint8_t *>(buffer.contents);
    for (NSUInteger y = 0; y < texture.height; ++y) {
      const auto *row = source + y * aligned_row;
      auto *destination = image.pixels.data() + y * packed_row;
      for (NSUInteger x = 0; x < texture.width; ++x) {
        destination[x * 4 + 0] = row[x * 4 + 2];
        destination[x * 4 + 1] = row[x * 4 + 1];
        destination[x * 4 + 2] = row[x * 4 + 0];
        destination[x * 4 + 3] = row[x * 4 + 3];
      }
    }
    return image;
  }

private:
  id<MTLDevice> device_;
  MetalFrameContext *context_ = nullptr;
};

MetalRendererTestFixture makeMetalRendererTestFixture() {
  id<MTLDevice> device = MTLCreateSystemDefaultDevice();
  id<MTLCommandQueue> queue = [device newCommandQueue];
  if (device == nil || queue == nil)
    throw std::runtime_error("Metal test device or command queue unavailable");

  MetalRendererTestFixture fixture;
  auto context = MetalRendererTestAccess::makeContext(device, queue, false);
  auto *context_pointer = context.get();
  fixture.renderer.renderer = std::make_unique<MetalPointRenderer>(
      (__bridge void *)device, (__bridge void *)queue);
  fixture.renderer.readback = std::make_unique<MetalRendererTestAccess>(
      device, context_pointer);
  fixture.frame_context = std::move(context);
  return fixture;
}

Result<std::reference_wrapper<FrameContext>, RendererError>
beginMetalFrameForTests(MetalRendererTestFixture &fixture) {
  auto *context =
      dynamic_cast<MetalFrameContext *>(fixture.frame_context.get());
  auto *renderer =
      dynamic_cast<MetalPointRenderer *>(fixture.renderer.renderer.get());
  if (context == nullptr || renderer == nullptr)
    return error(RendererErrorCode::BackendMismatch,
                 "Metal test fixture has incompatible objects");
  return MetalRendererTestAccess::begin(*context, fixture.command_buffer);
}

std::unique_ptr<FrameContext> makeInactiveMetalFrameContextForTests() {
  id<MTLDevice> device = MTLCreateSystemDefaultDevice();
  return MetalRendererTestAccess::makeContext(device, nil, false);
}

std::uint64_t
metalEncodedFrameCountForTests(const ViewportRenderer &renderer) {
  const auto *metal_renderer =
      dynamic_cast<const MetalPointRenderer *>(&renderer);
  return metal_renderer == nullptr
             ? 0
             : MetalRendererTestAccess::encodedFrameCount(*metal_renderer);
}

} // namespace kpt::gui
