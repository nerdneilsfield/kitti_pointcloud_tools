#include "gui/backend/metal/point_renderer.hpp"

#include "gui/viewport/frame_cache.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#import <Foundation/Foundation.h>
#import <Metal/Metal.h>
#import <mach-o/dyld.h>
#import <simd/simd.h>

namespace kpt::gui {
namespace {

struct GpuVertex {
  float position[3];
  float color[3];
  float intensity;
  float noise;
};
static_assert(sizeof(GpuVertex) == 32);
static_assert(offsetof(GpuVertex, position) == 0);
static_assert(offsetof(GpuVertex, color) == 12);
static_assert(offsetof(GpuVertex, intensity) == 24);
static_assert(offsetof(GpuVertex, noise) == 28);

constexpr std::size_t kInteractivePointBudget = 500'000;

struct alignas(16) Uniforms {
  simd_float4x4 view_projection;
  simd_float4 background;
  simd_float4 parameters;
  simd_float4 transform;
  simd_float4 fixed_color;
  simd_float4 noise_color;
};
static_assert(sizeof(Uniforms) == 144);

RendererError error(RendererErrorCode code, std::string message) {
  return {code, std::move(message)};
}

bool finite(const ViewportVertex &vertex) {
  return vertex.position.allFinite() && vertex.color.allFinite() &&
         std::isfinite(vertex.intensity) && std::isfinite(vertex.noise);
}

std::string executableDirectory() {
  std::vector<char> bytes(1024);
  std::uint32_t size = static_cast<std::uint32_t>(bytes.size());
  if (_NSGetExecutablePath(bytes.data(), &size) != 0) {
    bytes.resize(size);
    if (_NSGetExecutablePath(bytes.data(), &size) != 0)
      return {};
  }
  NSString *path =
      [[NSString stringWithUTF8String:bytes.data()] stringByStandardizingPath];
  return [[[path stringByDeletingLastPathComponent] stringByStandardizingPath]
      UTF8String];
}

NSString *shaderLibraryPath() {
  if (NSString *bundled =
          [[NSBundle mainBundle] pathForResource:@"kpt_point_shaders"
                                         ofType:@"metallib"])
    return bundled;
  const std::string directory = executableDirectory();
  if (directory.empty())
    return nil;
  NSString *executable_directory =
      [NSString stringWithUTF8String:directory.c_str()];
  NSString *bundle_resource = [[executable_directory
      stringByAppendingPathComponent:
          @"../Resources/kpt_point_shaders.metallib"]
      stringByStandardizingPath];
  if ([[NSFileManager defaultManager] fileExistsAtPath:bundle_resource])
    return bundle_resource;
  return [executable_directory
      stringByAppendingPathComponent:@"kpt_point_shaders.metallib"];
}

int colorMode(ColorBy color_by) {
  if (color_by == ColorBy::RGB || color_by == ColorBy::Label)
    return 0;
  if (color_by == ColorBy::Intensity)
    return 1;
  if (color_by == ColorBy::Z)
    return 2;
  return 4;
}

} // namespace

struct MetalPointRenderer::Impl {
  struct VertexSlot {
    id<MTLBuffer> buffer = nil;
    NSUInteger capacity = 0;
    id<MTLBuffer> lod_buffer = nil;
    NSUInteger lod_capacity = 0;
    std::size_t lod_count = 0;
    id<MTLCommandBuffer> last_use = nil;
  };

  static constexpr std::size_t vertex_slot_count = 3;
  id<MTLDevice> device = nil;
  id<MTLCommandQueue> command_queue = nil;
  id<MTLRenderPipelineState> pipeline = nil;
  id<MTLRenderPipelineState> guide_pipeline = nil;
  id<MTLDepthStencilState> depth_state = nil;
  std::array<VertexSlot, vertex_slot_count> vertex_slots{};
  std::size_t active_vertex_slot = 0;
  id<MTLBuffer> guide_buffer = nil;
  id<MTLTexture> color_texture = nil;
  id<MTLTexture> depth_texture = nil;
  PixelExtent extent;
  std::size_t point_count = 0;
  std::uint64_t uploaded_revision = 0;
  std::optional<ViewportFrame> encoded_frame;
  std::uint64_t encoded_revision = 0;
  std::uint64_t encoded_frame_count = 0;
};

MetalPointRenderer::MetalPointRenderer(void *device, void *command_queue)
    : impl_(std::make_unique<Impl>()) {
  impl_->device = (__bridge id<MTLDevice>)device;
  impl_->command_queue = (__bridge id<MTLCommandQueue>)command_queue;
  if (impl_->device == nil || impl_->command_queue == nil)
    throw std::runtime_error("Metal renderer requires a device and queue");

  NSError *library_error = nil;
  NSString *path = shaderLibraryPath();
  id<MTLLibrary> library =
      path == nil
          ? nil
          : [impl_->device
                newLibraryWithURL:[NSURL fileURLWithPath:path]
                            error:&library_error];
  if (library == nil) {
    const char *message =
        library_error == nil ? "shader library not found"
                             : library_error.localizedDescription.UTF8String;
    throw std::runtime_error(std::string("Metal shader library load failed: ") +
                             (message == nullptr ? "unknown error" : message));
  }

  id<MTLFunction> vertex = [library newFunctionWithName:@"point_vertex"];
  id<MTLFunction> fragment = [library newFunctionWithName:@"point_fragment"];
  id<MTLFunction> guide_fragment =
      [library newFunctionWithName:@"guide_fragment"];
  if (vertex == nil || fragment == nil || guide_fragment == nil)
    throw std::runtime_error("Metal shader entry point is missing");

  MTLRenderPipelineDescriptor *pipeline = [MTLRenderPipelineDescriptor new];
  pipeline.label = @"KPT point pipeline";
  pipeline.vertexFunction = vertex;
  pipeline.fragmentFunction = fragment;
  pipeline.colorAttachments[0].pixelFormat = MTLPixelFormatBGRA8Unorm;
  pipeline.depthAttachmentPixelFormat = MTLPixelFormatDepth32Float;
  NSError *pipeline_error = nil;
  impl_->pipeline =
      [impl_->device newRenderPipelineStateWithDescriptor:pipeline
                                                    error:&pipeline_error];
  if (impl_->pipeline == nil) {
    const char *message =
        pipeline_error == nil ? "unknown error"
                              : pipeline_error.localizedDescription.UTF8String;
    throw std::runtime_error(std::string("Metal pipeline creation failed: ") +
                             (message == nullptr ? "unknown error" : message));
  }

  MTLRenderPipelineDescriptor *guide_pipeline =
      [MTLRenderPipelineDescriptor new];
  guide_pipeline.label = @"KPT guide pipeline";
  guide_pipeline.vertexFunction = vertex;
  guide_pipeline.fragmentFunction = guide_fragment;
  guide_pipeline.colorAttachments[0].pixelFormat = MTLPixelFormatBGRA8Unorm;
  guide_pipeline.depthAttachmentPixelFormat = MTLPixelFormatDepth32Float;
  NSError *guide_pipeline_error = nil;
  impl_->guide_pipeline = [impl_->device
      newRenderPipelineStateWithDescriptor:guide_pipeline
                                     error:&guide_pipeline_error];
  if (impl_->guide_pipeline == nil) {
    const char *message =
        guide_pipeline_error == nil
            ? "unknown error"
            : guide_pipeline_error.localizedDescription.UTF8String;
    throw std::runtime_error(
        std::string("Metal guide pipeline creation failed: ") +
        (message == nullptr ? "unknown error" : message));
  }

  MTLDepthStencilDescriptor *depth = [MTLDepthStencilDescriptor new];
  depth.depthCompareFunction = MTLCompareFunctionLess;
  depth.depthWriteEnabled = YES;
  impl_->depth_state = [impl_->device newDepthStencilStateWithDescriptor:depth];
  if (impl_->depth_state == nil)
    throw std::runtime_error("Metal depth state creation failed");
}

MetalPointRenderer::~MetalPointRenderer() = default;

Result<void, RendererError>
MetalPointRenderer::upload(std::span<const ViewportVertex> vertices,
                           std::uint64_t revision) {
  if (revision == impl_->uploaded_revision)
    return {};

  std::vector<GpuVertex> copied;
  copied.reserve(vertices.size());
  for (const auto &vertex : vertices) {
    if (!finite(vertex))
      continue;
    copied.push_back(
        {{vertex.position.x(), vertex.position.y(), vertex.position.z()},
         {vertex.color.x(), vertex.color.y(), vertex.color.z()},
         vertex.intensity,
         vertex.noise});
  }

  if (!copied.empty()) {
    if (copied.size() >
        (std::numeric_limits<NSUInteger>::max)() / sizeof(GpuVertex)) {
      return error(RendererErrorCode::EncodingFailed,
                   "Metal vertex upload size overflows");
    }
    const NSUInteger size = copied.size() * sizeof(GpuVertex);
    std::optional<std::size_t> selected;
    for (std::size_t offset = 1; offset <= Impl::vertex_slot_count; ++offset) {
      const auto candidate =
          (impl_->active_vertex_slot + offset) % Impl::vertex_slot_count;
      auto &slot = impl_->vertex_slots[candidate];
      if (slot.last_use == nil ||
          slot.last_use.status == MTLCommandBufferStatusCompleted ||
          slot.last_use.status == MTLCommandBufferStatusError) {
        selected = candidate;
        break;
      }
    }
    if (!selected) {
      return error(RendererErrorCode::ResourceCreationFailed,
                   "Metal vertex upload has no completed buffer slot");
    }
    auto &slot = impl_->vertex_slots[*selected];
    if (slot.buffer == nil || slot.capacity < size) {
      slot.buffer = [impl_->device
          newBufferWithLength:size
                       options:MTLResourceStorageModeShared];
      slot.capacity = slot.buffer == nil ? 0 : size;
    }
    if (slot.buffer == nil)
      return error(RendererErrorCode::ResourceCreationFailed,
                   "Metal shared vertex buffer creation failed");
    std::memcpy(slot.buffer.contents, copied.data(), size);
    if (copied.size() > kInteractivePointBudget) {
      const NSUInteger lod_size =
          static_cast<NSUInteger>(kInteractivePointBudget * sizeof(GpuVertex));
      if (slot.lod_buffer == nil || slot.lod_capacity < lod_size) {
        slot.lod_buffer = [impl_->device
            newBufferWithLength:lod_size
                         options:MTLResourceStorageModeShared];
        slot.lod_capacity = slot.lod_buffer == nil ? 0 : lod_size;
      }
      if (slot.lod_buffer == nil)
        return error(RendererErrorCode::ResourceCreationFailed,
                     "Metal shared LOD buffer creation failed");
      auto *lod = static_cast<GpuVertex *>(slot.lod_buffer.contents);
      for (std::size_t index = 0; index < kInteractivePointBudget; ++index) {
        const auto source =
            (static_cast<std::uint64_t>(index) * copied.size()) /
            static_cast<std::uint64_t>(kInteractivePointBudget);
        lod[index] = copied[static_cast<std::size_t>(source)];
      }
      slot.lod_count = kInteractivePointBudget;
    } else {
      slot.lod_count = 0;
    }
    impl_->active_vertex_slot = *selected;
  }
  impl_->point_count = copied.size();
  impl_->uploaded_revision = revision;
  impl_->encoded_frame.reset();
  return {};
}

Result<void, RendererError>
MetalPointRenderer::resize(PixelExtent physical_pixels) {
  if (physical_pixels.width < 0 || physical_pixels.height < 0)
    return error(RendererErrorCode::ResourceCreationFailed,
                 "Metal viewport extent cannot be negative");
  if (physical_pixels == impl_->extent)
    return {};
  if (physical_pixels.width == 0 || physical_pixels.height == 0) {
    impl_->color_texture = nil;
    impl_->depth_texture = nil;
    impl_->extent = {};
    impl_->encoded_frame.reset();
    return {};
  }

  MTLTextureDescriptor *color = [MTLTextureDescriptor
      texture2DDescriptorWithPixelFormat:MTLPixelFormatBGRA8Unorm
                                   width:static_cast<NSUInteger>(
                                             physical_pixels.width)
                                  height:static_cast<NSUInteger>(
                                             physical_pixels.height)
                               mipmapped:NO];
  color.storageMode = MTLStorageModePrivate;
  color.usage = MTLTextureUsageRenderTarget | MTLTextureUsageShaderRead;
  id<MTLTexture> new_color =
      [impl_->device newTextureWithDescriptor:color];

  MTLTextureDescriptor *depth = [MTLTextureDescriptor
      texture2DDescriptorWithPixelFormat:MTLPixelFormatDepth32Float
                                   width:static_cast<NSUInteger>(
                                             physical_pixels.width)
                                  height:static_cast<NSUInteger>(
                                             physical_pixels.height)
                               mipmapped:NO];
  depth.storageMode = MTLStorageModePrivate;
  depth.usage = MTLTextureUsageRenderTarget;
  id<MTLTexture> new_depth =
      [impl_->device newTextureWithDescriptor:depth];
  if (new_color == nil || new_depth == nil)
    return error(RendererErrorCode::ResourceCreationFailed,
                 "Metal offscreen texture creation failed");

  new_color.label = @"KPT viewport color";
  new_depth.label = @"KPT viewport depth";
  impl_->color_texture = new_color;
  impl_->depth_texture = new_depth;
  impl_->extent = physical_pixels;
  impl_->encoded_frame.reset();
  return {};
}

Result<void, RendererError>
MetalPointRenderer::render(const ViewportFrame &frame, FrameContext &context) {
  if (context.backendKind() != BackendKind::Metal)
    return error(RendererErrorCode::BackendMismatch,
                 "Metal renderer received a non-Metal frame context");
  auto *metal_context = dynamic_cast<MetalFrameContext *>(&context);
  if (metal_context == nullptr || !metal_context->isActive() ||
      metal_context->device() != (__bridge void *)impl_->device ||
      metal_context->commandQueue() != (__bridge void *)impl_->command_queue ||
      metal_context->commandBuffer() == nullptr) {
    return error(RendererErrorCode::BackendMismatch,
                 "Metal frame context is inactive or belongs to another device or command queue");
  }
  if (impl_->extent.width == 0 || impl_->extent.height == 0)
    return {};
  if (impl_->encoded_frame &&
      impl_->encoded_revision == impl_->uploaded_revision &&
      detail::framesRenderEqual(*impl_->encoded_frame, frame)) {
    return {};
  }

  id<MTLCommandBuffer> command =
      (__bridge id<MTLCommandBuffer>)metal_context->commandBuffer();
  MTLRenderPassDescriptor *pass = [MTLRenderPassDescriptor renderPassDescriptor];
  pass.colorAttachments[0].texture = impl_->color_texture;
  pass.colorAttachments[0].loadAction = MTLLoadActionClear;
  pass.colorAttachments[0].storeAction = MTLStoreActionStore;
  pass.colorAttachments[0].clearColor =
      MTLClearColorMake(static_cast<double>(frame.style.background.x()),
                        static_cast<double>(frame.style.background.y()),
                        static_cast<double>(frame.style.background.z()), 1.0);
  pass.depthAttachment.texture = impl_->depth_texture;
  pass.depthAttachment.loadAction = MTLLoadActionClear;
  pass.depthAttachment.storeAction = MTLStoreActionDontCare;
  pass.depthAttachment.clearDepth = 1.0;

  id<MTLRenderCommandEncoder> encoder =
      [command renderCommandEncoderWithDescriptor:pass];
  if (encoder == nil)
    return error(RendererErrorCode::EncodingFailed,
                 "Metal offscreen command encoder creation failed");
  encoder.label = @"KPT point pass";
  [encoder setRenderPipelineState:impl_->pipeline];
  [encoder setDepthStencilState:impl_->depth_state];
  [encoder setViewport:MTLViewport{0.0, 0.0,
                                   static_cast<double>(impl_->extent.width),
                                   static_cast<double>(impl_->extent.height),
                                   0.0, 1.0}];

  Uniforms uniforms{};
  std::memcpy(&uniforms.view_projection, frame.view_projection.data(),
              sizeof(uniforms.view_projection));
  uniforms.background =
      simd_make_float4(frame.style.background.x(), frame.style.background.y(),
                       frame.style.background.z(), 1.0F);
  uniforms.parameters =
      simd_make_float4(std::clamp(frame.style.point_size, 1.0F, 64.0F),
                       static_cast<float>(colorMode(frame.style.color_by)),
                       frame.style.scalar_min, frame.style.scalar_max);
  uniforms.transform =
      simd_make_float4(frame.world_origin.x(), frame.world_origin.y(),
                       frame.world_origin.z(), frame.world_scale);
  uniforms.fixed_color =
      simd_make_float4(frame.style.fixed_color.x(), frame.style.fixed_color.y(),
                       frame.style.fixed_color.z(), 0.0F);
  uniforms.noise_color = simd_make_float4(
      frame.style.noise_color.x(), frame.style.noise_color.y(),
      frame.style.noise_color.z(), frame.style.highlight_noise ? 1.0F : 0.0F);
  const bool equalize_active =
      frame.intensity_cdf_valid && frame.style.intensity_equalize &&
      frame.style.color_by == kpt::ColorBy::Intensity;
  uniforms.extras =
      simd_make_float4(equalize_active ? 1.0F : 0.0F,
                       static_cast<float>(frame.style.color_map), 0.0F, 0.0F);
  if (impl_->point_count != 0) {
    const auto &slot = impl_->vertex_slots[impl_->active_vertex_slot];
    if (slot.buffer == nil) {
      [encoder endEncoding];
      return error(RendererErrorCode::EncodingFailed,
                   "Metal point buffer is unavailable");
    }
    id<MTLBuffer> vertex_buffer = slot.buffer;
    std::size_t vertex_count = impl_->point_count;
    if (frame.interactive_lod && slot.lod_count != 0) {
      vertex_buffer = slot.lod_buffer;
      vertex_count = slot.lod_count;
    }
    [encoder setVertexBuffer:vertex_buffer offset:0 atIndex:0];
    [encoder setVertexBytes:&uniforms length:sizeof(uniforms) atIndex:1];
    [encoder setFragmentBytes:&uniforms length:sizeof(uniforms) atIndex:1];
    [encoder setFragmentBytes:frame.intensity_cdf.data()
                        length:frame.intensity_cdf.size() * sizeof(float)
                     atIndex:2];
    [encoder drawPrimitives:MTLPrimitiveTypePoint
                vertexStart:0
                vertexCount:vertex_count];
  }
  if (!frame.guides.empty()) {
    std::vector<GpuVertex> guides;
    guides.reserve(frame.guides.size());
    for (const auto &vertex : frame.guides) {
      if (!vertex.position.allFinite() || !vertex.color.allFinite())
        continue;
      guides.push_back(
          {{vertex.position.x(), vertex.position.y(), vertex.position.z()},
           {vertex.color.x(), vertex.color.y(), vertex.color.z()}, 0.0F,
           0.0F});
    }
    if (!guides.empty()) {
      impl_->guide_buffer =
          [impl_->device newBufferWithBytes:guides.data()
                                     length:guides.size() * sizeof(GpuVertex)
                                    options:MTLResourceStorageModeShared];
      if (impl_->guide_buffer == nil) {
        [encoder endEncoding];
        return error(RendererErrorCode::ResourceCreationFailed,
                     "Metal guide buffer creation failed");
      }
      [encoder setRenderPipelineState:impl_->guide_pipeline];
      [encoder setVertexBuffer:impl_->guide_buffer offset:0 atIndex:0];
      [encoder setVertexBytes:&uniforms length:sizeof(uniforms) atIndex:1];
      [encoder drawPrimitives:MTLPrimitiveTypeLine
                  vertexStart:0
                  vertexCount:guides.size()];
    }
  } else {
    impl_->guide_buffer = nil;
  }
  [encoder endEncoding];
  if (impl_->point_count != 0)
    impl_->vertex_slots[impl_->active_vertex_slot].last_use = command;
  impl_->encoded_frame = frame;
  impl_->encoded_revision = impl_->uploaded_revision;
  ++impl_->encoded_frame_count;
  return {};
}

ViewportTexture MetalPointRenderer::texture() const {
  if (impl_->color_texture == nil)
    return {};
  return {ImTextureRef{(__bridge void *)impl_->color_texture},
          ImVec2{0.0F, 0.0F}, ImVec2{1.0F, 1.0F}};
}

PixelExtent MetalPointRenderer::extent() const { return impl_->extent; }
std::size_t MetalPointRenderer::pointCount() const noexcept {
  return impl_->point_count;
}
std::uint64_t MetalPointRenderer::uploadedRevision() const noexcept {
  return impl_->uploaded_revision;
}
void *MetalPointRenderer::colorTextureForTests() const noexcept {
  return (__bridge void *)impl_->color_texture;
}
std::uint64_t MetalPointRenderer::encodedFrameCountForTests() const noexcept {
  return impl_->encoded_frame_count;
}

} // namespace kpt::gui
