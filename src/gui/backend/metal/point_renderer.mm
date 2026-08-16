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
#include <unordered_map>
#include <unordered_set>
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
  simd_float4 extras;
};
static_assert(alignof(Uniforms) == 16);
static_assert(offsetof(Uniforms, view_projection) == 0);
static_assert(offsetof(Uniforms, background) == 64);
static_assert(offsetof(Uniforms, parameters) == 80);
static_assert(offsetof(Uniforms, transform) == 96);
static_assert(offsetof(Uniforms, fixed_color) == 112);
static_assert(offsetof(Uniforms, noise_color) == 128);
static_assert(offsetof(Uniforms, extras) == 144);
static_assert(sizeof(Uniforms) == 160);

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
      stringByAppendingPathComponent:@"../Resources/kpt_point_shaders.metallib"]
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
  struct GuideSlot {
    id<MTLBuffer> buffer = nil;
    NSUInteger capacity = 0;
    id<MTLCommandBuffer> last_use = nil;
  };
  struct LayerBuffer {
    id<MTLBuffer> buffer = nil;
    NSUInteger capacity = 0;
    // Unlike the single-cloud ring slots, layered review buffers persist by
    // stable layer ID.  Keep a compact, uniformly sampled sibling so camera
    // interaction never redraws every point in each visible layer.
    id<MTLBuffer> lod_buffer = nil;
    NSUInteger lod_capacity = 0;
    std::size_t point_count = 0;
    std::size_t lod_count = 0;
    std::uint64_t revision = 0;
  };

  static constexpr std::size_t vertex_slot_count = 3;
  id<MTLDevice> device = nil;
  id<MTLCommandQueue> command_queue = nil;
  id<MTLRenderPipelineState> pipeline = nil;
  id<MTLRenderPipelineState> transparent_pipeline = nil;
  id<MTLRenderPipelineState> guide_pipeline = nil;
  id<MTLDepthStencilState> depth_state = nil;
  id<MTLDepthStencilState> transparent_depth_state = nil;
  std::array<VertexSlot, vertex_slot_count> vertex_slots{};
  std::size_t active_vertex_slot = 0;
  std::array<GuideSlot, vertex_slot_count> guide_slots{};
  std::size_t active_guide_slot = 0;
  id<MTLTexture> color_texture = nil;
  id<MTLTexture> depth_texture = nil;
  PixelExtent extent;
  std::size_t point_count = 0;
  std::uint64_t uploaded_revision = 0;
  std::optional<ViewportFrame> encoded_frame;
  std::uint64_t encoded_revision = 0;
  std::uint64_t encoded_frame_count = 0;
  // Kept only long enough to establish capture ordering. The runtime owns and
  // commits this buffer; screenshot readback must never race an uncommitted
  // viewport pass or silently copy the previous texture contents.
  id<MTLCommandBuffer> last_viewport_render = nil;
  std::unordered_map<std::uint64_t, LayerBuffer> layer_buffers;
  std::uint64_t uploaded_layered_revision = 0;
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
          : [impl_->device newLibraryWithURL:[NSURL fileURLWithPath:path]
                                       error:&library_error];
  if (library == nil) {
    const char *message = library_error == nil
                              ? "shader library not found"
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
    const char *message = pipeline_error == nil
                              ? "unknown error"
                              : pipeline_error.localizedDescription.UTF8String;
    throw std::runtime_error(std::string("Metal pipeline creation failed: ") +
                             (message == nullptr ? "unknown error" : message));
  }

  MTLRenderPipelineDescriptor *transparent_pipeline = [pipeline copy];
  transparent_pipeline.label = @"KPT transparent point pipeline";
  MTLRenderPipelineColorAttachmentDescriptor *transparent_color =
      transparent_pipeline.colorAttachments[0];
  transparent_color.blendingEnabled = YES;
  transparent_color.rgbBlendOperation = MTLBlendOperationAdd;
  transparent_color.alphaBlendOperation = MTLBlendOperationAdd;
  transparent_color.sourceRGBBlendFactor = MTLBlendFactorSourceAlpha;
  transparent_color.destinationRGBBlendFactor =
      MTLBlendFactorOneMinusSourceAlpha;
  transparent_color.sourceAlphaBlendFactor = MTLBlendFactorOne;
  transparent_color.destinationAlphaBlendFactor =
      MTLBlendFactorOneMinusSourceAlpha;
  NSError *transparent_pipeline_error = nil;
  impl_->transparent_pipeline = [impl_->device
      newRenderPipelineStateWithDescriptor:transparent_pipeline
                                     error:&transparent_pipeline_error];
  if (impl_->transparent_pipeline == nil) {
    const char *message =
        transparent_pipeline_error == nil
            ? "unknown error"
            : transparent_pipeline_error.localizedDescription.UTF8String;
    throw std::runtime_error(
        std::string("Metal transparent pipeline creation failed: ") +
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
  MTLDepthStencilDescriptor *transparent_depth = [depth copy];
  transparent_depth.depthWriteEnabled = NO;
  impl_->transparent_depth_state =
      [impl_->device newDepthStencilStateWithDescriptor:transparent_depth];
  if (impl_->transparent_depth_state == nil)
    throw std::runtime_error("Metal transparent depth state creation failed");
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
      slot.buffer =
          [impl_->device newBufferWithLength:size
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
        slot.lod_buffer =
            [impl_->device newBufferWithLength:lod_size
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
MetalPointRenderer::uploadLayers(std::span<const ViewportLayerUpload> layers,
                                 std::uint64_t scene_revision) {
  if (scene_revision == 0) {
    impl_->layer_buffers.clear();
    impl_->uploaded_layered_revision = 0;
    return {};
  }
  if (scene_revision == impl_->uploaded_layered_revision)
    return {};

  std::unordered_set<std::uint64_t> seen;
  seen.reserve(layers.size());
  for (const auto &layer : layers) {
    if (layer.layer_id == 0 || layer.revision == 0 ||
        !seen.insert(layer.layer_id).second) {
      return error(RendererErrorCode::EncodingFailed,
                   "Metal layered upload requires unique non-zero layer IDs");
    }
  }

  try {
    for (const auto &layer : layers) {
      const auto existing = impl_->layer_buffers.find(layer.layer_id);
      if (existing != impl_->layer_buffers.end() &&
          existing->second.revision == layer.revision) {
        continue;
      }
      std::vector<GpuVertex> copied;
      copied.reserve(layer.vertices.size());
      for (const auto &vertex : layer.vertices) {
        if (!finite(vertex))
          continue;
        copied.push_back(
            {{vertex.position.x(), vertex.position.y(), vertex.position.z()},
             {vertex.color.x(), vertex.color.y(), vertex.color.z()},
             vertex.intensity,
             vertex.noise});
      }
      if (copied.size() >
          (std::numeric_limits<NSUInteger>::max)() / sizeof(GpuVertex)) {
        return error(RendererErrorCode::EncodingFailed,
                     "Metal layered vertex upload size overflows");
      }

      Impl::LayerBuffer next;
      next.revision = layer.revision;
      next.point_count = copied.size();
      if (!copied.empty()) {
        const NSUInteger size = copied.size() * sizeof(GpuVertex);
        next.buffer =
            [impl_->device newBufferWithLength:size
                                       options:MTLResourceStorageModeShared];
        next.capacity = next.buffer == nil ? 0 : size;
        if (next.buffer == nil) {
          return error(RendererErrorCode::ResourceCreationFailed,
                       "Metal layered shared vertex buffer creation failed");
        }
        std::memcpy(next.buffer.contents, copied.data(), size);
        if (copied.size() > kInteractivePointBudget) {
          const NSUInteger lod_size = static_cast<NSUInteger>(
              kInteractivePointBudget * sizeof(GpuVertex));
          next.lod_buffer =
              [impl_->device newBufferWithLength:lod_size
                                         options:MTLResourceStorageModeShared];
          next.lod_capacity = next.lod_buffer == nil ? 0 : lod_size;
          if (next.lod_buffer == nil) {
            return error(RendererErrorCode::ResourceCreationFailed,
                         "Metal layered shared LOD buffer creation failed");
          }
          auto *lod = static_cast<GpuVertex *>(next.lod_buffer.contents);
          for (std::size_t index = 0; index < kInteractivePointBudget;
               ++index) {
            const auto source =
                (static_cast<std::uint64_t>(index) * copied.size()) /
                static_cast<std::uint64_t>(kInteractivePointBudget);
            lod[index] = copied[static_cast<std::size_t>(source)];
          }
          next.lod_count = kInteractivePointBudget;
        }
      }
      impl_->layer_buffers.insert_or_assign(layer.layer_id, std::move(next));
    }
  } catch (const std::exception &exception) {
    return error(RendererErrorCode::ResourceCreationFailed,
                 "Metal layered upload failed: " +
                     std::string(exception.what()));
  }

  for (auto iterator = impl_->layer_buffers.begin();
       iterator != impl_->layer_buffers.end();) {
    if (seen.contains(iterator->first)) {
      ++iterator;
    } else {
      iterator = impl_->layer_buffers.erase(iterator);
    }
  }
  impl_->uploaded_layered_revision = scene_revision;
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
    impl_->last_viewport_render = nil;
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
  id<MTLTexture> new_color = [impl_->device newTextureWithDescriptor:color];

  MTLTextureDescriptor *depth = [MTLTextureDescriptor
      texture2DDescriptorWithPixelFormat:MTLPixelFormatDepth32Float
                                   width:static_cast<NSUInteger>(
                                             physical_pixels.width)
                                  height:static_cast<NSUInteger>(
                                             physical_pixels.height)
                               mipmapped:NO];
  depth.storageMode = MTLStorageModePrivate;
  depth.usage = MTLTextureUsageRenderTarget;
  id<MTLTexture> new_depth = [impl_->device newTextureWithDescriptor:depth];
  if (new_color == nil || new_depth == nil)
    return error(RendererErrorCode::ResourceCreationFailed,
                 "Metal offscreen texture creation failed");

  new_color.label = @"KPT viewport color";
  new_depth.label = @"KPT viewport depth";
  impl_->color_texture = new_color;
  impl_->depth_texture = new_depth;
  impl_->last_viewport_render = nil;
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
                 "Metal frame context is inactive or belongs to another device "
                 "or command queue");
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
  MTLRenderPassDescriptor *pass =
      [MTLRenderPassDescriptor renderPassDescriptor];
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
      simd_make_float4(std::clamp(frame.style.point_size, 0.0F, 5.0F),
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
  const bool equalize_active = frame.intensity_cdf_valid &&
                               frame.style.intensity_equalize &&
                               frame.style.color_by == kpt::ColorBy::Intensity;
  uniforms.extras =
      simd_make_float4(equalize_active ? 1.0F : 0.0F,
                       static_cast<float>(frame.style.color_map), 1.0F, 0.0F);
  if (impl_->point_count != 0 && frame.style.point_size > 0.0F) {
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
           {vertex.color.x(), vertex.color.y(), vertex.color.z()},
           0.0F,
           0.0F});
    }
    if (!guides.empty()) {
      const NSUInteger guide_size = guides.size() * sizeof(GpuVertex);
      std::optional<std::size_t> selected;
      for (std::size_t offset = 1; offset <= Impl::vertex_slot_count;
           ++offset) {
        const auto candidate =
            (impl_->active_guide_slot + offset) % Impl::vertex_slot_count;
        const auto &slot = impl_->guide_slots[candidate];
        if (slot.last_use == nil ||
            slot.last_use.status == MTLCommandBufferStatusCompleted ||
            slot.last_use.status == MTLCommandBufferStatusError) {
          selected = candidate;
          break;
        }
      }
      if (!selected) {
        [encoder endEncoding];
        return error(RendererErrorCode::ResourceCreationFailed,
                     "Metal guide upload has no completed buffer slot");
      }
      auto &guide_slot = impl_->guide_slots[*selected];
      if (guide_slot.buffer == nil || guide_slot.capacity < guide_size) {
        guide_slot.buffer =
            [impl_->device newBufferWithLength:guide_size
                                       options:MTLResourceStorageModeShared];
        guide_slot.capacity = guide_slot.buffer == nil ? 0 : guide_size;
      }
      if (guide_slot.buffer == nil) {
        [encoder endEncoding];
        return error(RendererErrorCode::ResourceCreationFailed,
                     "Metal guide buffer creation failed");
      }
      std::memcpy(guide_slot.buffer.contents, guides.data(), guide_size);
      impl_->active_guide_slot = *selected;
      [encoder setRenderPipelineState:impl_->guide_pipeline];
      [encoder setVertexBuffer:guide_slot.buffer offset:0 atIndex:0];
      [encoder setVertexBytes:&uniforms length:sizeof(uniforms) atIndex:1];
      [encoder drawPrimitives:MTLPrimitiveTypeLine
                  vertexStart:0
                  vertexCount:guides.size()];
    }
  }
  [encoder endEncoding];
  if (impl_->point_count != 0)
    impl_->vertex_slots[impl_->active_vertex_slot].last_use = command;
  if (!frame.guides.empty())
    impl_->guide_slots[impl_->active_guide_slot].last_use = command;
  impl_->last_viewport_render = command;
  impl_->encoded_frame = frame;
  impl_->encoded_revision = impl_->uploaded_revision;
  ++impl_->encoded_frame_count;
  return {};
}

Result<void, RendererError>
MetalPointRenderer::renderLayers(const ViewportFrame &frame,
                                 const LayeredViewportFrame &layers,
                                 FrameContext &context) {
  if (context.backendKind() != BackendKind::Metal)
    return error(RendererErrorCode::BackendMismatch,
                 "Metal layered renderer received a non-Metal frame context");
  auto *metal_context = dynamic_cast<MetalFrameContext *>(&context);
  if (metal_context == nullptr || !metal_context->isActive() ||
      metal_context->device() != (__bridge void *)impl_->device ||
      metal_context->commandQueue() != (__bridge void *)impl_->command_queue ||
      metal_context->commandBuffer() == nullptr) {
    return error(
        RendererErrorCode::BackendMismatch,
        "Metal layered frame context is inactive or belongs to another "
        "device or command queue");
  }
  if (impl_->extent.width == 0 || impl_->extent.height == 0)
    return {};
  if (layers.revision == 0 ||
      layers.revision != impl_->uploaded_layered_revision) {
    return error(RendererErrorCode::EncodingFailed,
                 "Metal layered frame does not match uploaded scene revision");
  }

  id<MTLCommandBuffer> command =
      (__bridge id<MTLCommandBuffer>)metal_context->commandBuffer();
  MTLRenderPassDescriptor *pass =
      [MTLRenderPassDescriptor renderPassDescriptor];
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
                 "Metal layered offscreen command encoder creation failed");
  encoder.label = @"KPT layered point pass";
  [encoder setViewport:MTLViewport{0.0, 0.0,
                                   static_cast<double>(impl_->extent.width),
                                   static_cast<double>(impl_->extent.height),
                                   0.0, 1.0}];

  const auto draw_layer =
      [this, &frame, encoder](const ViewportLayerDraw &draw,
                              bool transparent) -> Result<void, RendererError> {
    const auto iterator = impl_->layer_buffers.find(draw.layer_id);
    if (iterator == impl_->layer_buffers.end()) {
      return error(RendererErrorCode::EncodingFailed,
                   "Metal layered draw references an unavailable layer buffer");
    }
    const Impl::LayerBuffer &buffer = iterator->second;
    if (buffer.point_count == 0 || draw.style.point_size <= 0.0F ||
        draw.opacity <= 0.0F) {
      return {};
    }
    if (buffer.buffer == nil || !std::isfinite(draw.opacity) ||
        !std::isfinite(draw.style.point_size) ||
        !std::isfinite(draw.style.scalar_min) ||
        !std::isfinite(draw.style.scalar_max) ||
        !draw.style.fixed_color.allFinite() ||
        !draw.style.noise_color.allFinite()) {
      return error(RendererErrorCode::EncodingFailed,
                   "Metal layered draw style or buffer is invalid");
    }
    [encoder setRenderPipelineState:transparent ? impl_->transparent_pipeline
                                                : impl_->pipeline];
    [encoder setDepthStencilState:transparent ? impl_->transparent_depth_state
                                              : impl_->depth_state];

    Uniforms uniforms{};
    std::memcpy(&uniforms.view_projection, frame.view_projection.data(),
                sizeof(uniforms.view_projection));
    uniforms.background =
        simd_make_float4(frame.style.background.x(), frame.style.background.y(),
                         frame.style.background.z(), 1.0F);
    uniforms.parameters =
        simd_make_float4(std::clamp(draw.style.point_size, 0.0F, 5.0F),
                         static_cast<float>(colorMode(draw.style.color_by)),
                         draw.style.scalar_min, draw.style.scalar_max);
    uniforms.transform =
        simd_make_float4(frame.world_origin.x(), frame.world_origin.y(),
                         frame.world_origin.z(), frame.world_scale);
    uniforms.fixed_color =
        simd_make_float4(draw.style.fixed_color.x(), draw.style.fixed_color.y(),
                         draw.style.fixed_color.z(), 0.0F);
    uniforms.noise_color = simd_make_float4(
        draw.style.noise_color.x(), draw.style.noise_color.y(),
        draw.style.noise_color.z(), draw.style.highlight_noise ? 1.0F : 0.0F);
    const bool equalize_active = draw.intensity_cdf_valid &&
                                 draw.style.intensity_equalize &&
                                 draw.style.color_by == kpt::ColorBy::Intensity;
    uniforms.extras = simd_make_float4(
        equalize_active ? 1.0F : 0.0F, static_cast<float>(draw.style.color_map),
        std::clamp(draw.opacity, 0.0F, 1.0F), 0.0F);
    id<MTLBuffer> vertex_buffer = buffer.buffer;
    std::size_t vertex_count = buffer.point_count;
    if (frame.interactive_lod && buffer.lod_count != 0) {
      vertex_buffer = buffer.lod_buffer;
      vertex_count = buffer.lod_count;
    }
    if (vertex_buffer == nil) {
      return error(RendererErrorCode::EncodingFailed,
                   "Metal layered interactive LOD buffer is unavailable");
    }
    [encoder setVertexBuffer:vertex_buffer offset:0 atIndex:0];
    [encoder setVertexBytes:&uniforms length:sizeof(uniforms) atIndex:1];
    [encoder setFragmentBytes:&uniforms length:sizeof(uniforms) atIndex:1];
    [encoder setFragmentBytes:draw.intensity_cdf.data()
                       length:draw.intensity_cdf.size() * sizeof(float)
                      atIndex:2];
    [encoder drawPrimitives:MTLPrimitiveTypePoint
                vertexStart:0
                vertexCount:vertex_count];
    return {};
  };

  // Opaque layers populate depth first. Transparent layers preserve the
  // SceneRenderAdapter's back-to-front order, keep depth testing enabled, and
  // use a separate state with depth writes disabled.
  for (const auto &draw : layers.opaque_layers) {
    auto drawn = draw_layer(draw, false);
    if (!drawn) {
      [encoder endEncoding];
      return drawn.error();
    }
  }
  for (const auto &draw : layers.transparent_layers) {
    auto drawn = draw_layer(draw, true);
    if (!drawn) {
      [encoder endEncoding];
      return drawn.error();
    }
  }

  if (!frame.guides.empty()) {
    std::vector<GpuVertex> guides;
    guides.reserve(frame.guides.size());
    for (const auto &vertex : frame.guides) {
      if (!vertex.position.allFinite() || !vertex.color.allFinite())
        continue;
      guides.push_back(
          {{vertex.position.x(), vertex.position.y(), vertex.position.z()},
           {vertex.color.x(), vertex.color.y(), vertex.color.z()},
           0.0F,
           0.0F});
    }
    if (!guides.empty()) {
      const NSUInteger guide_size = guides.size() * sizeof(GpuVertex);
      std::optional<std::size_t> selected;
      for (std::size_t offset = 1; offset <= Impl::vertex_slot_count;
           ++offset) {
        const auto candidate =
            (impl_->active_guide_slot + offset) % Impl::vertex_slot_count;
        const auto &slot = impl_->guide_slots[candidate];
        if (slot.last_use == nil ||
            slot.last_use.status == MTLCommandBufferStatusCompleted ||
            slot.last_use.status == MTLCommandBufferStatusError) {
          selected = candidate;
          break;
        }
      }
      if (!selected) {
        [encoder endEncoding];
        return error(RendererErrorCode::ResourceCreationFailed,
                     "Metal layered guide upload has no completed buffer slot");
      }
      auto &guide_slot = impl_->guide_slots[*selected];
      if (guide_slot.buffer == nil || guide_slot.capacity < guide_size) {
        guide_slot.buffer =
            [impl_->device newBufferWithLength:guide_size
                                       options:MTLResourceStorageModeShared];
        guide_slot.capacity = guide_slot.buffer == nil ? 0 : guide_size;
      }
      if (guide_slot.buffer == nil) {
        [encoder endEncoding];
        return error(RendererErrorCode::ResourceCreationFailed,
                     "Metal layered guide buffer creation failed");
      }
      std::memcpy(guide_slot.buffer.contents, guides.data(), guide_size);
      impl_->active_guide_slot = *selected;
      Uniforms uniforms{};
      std::memcpy(&uniforms.view_projection, frame.view_projection.data(),
                  sizeof(uniforms.view_projection));
      uniforms.transform =
          simd_make_float4(frame.world_origin.x(), frame.world_origin.y(),
                           frame.world_origin.z(), frame.world_scale);
      uniforms.extras = simd_make_float4(0.0F, 0.0F, 1.0F, 0.0F);
      [encoder setRenderPipelineState:impl_->guide_pipeline];
      [encoder setDepthStencilState:impl_->depth_state];
      [encoder setVertexBuffer:guide_slot.buffer offset:0 atIndex:0];
      [encoder setVertexBytes:&uniforms length:sizeof(uniforms) atIndex:1];
      [encoder drawPrimitives:MTLPrimitiveTypeLine
                  vertexStart:0
                  vertexCount:guides.size()];
      guide_slot.last_use = command;
    }
  }
  [encoder endEncoding];
  impl_->last_viewport_render = command;
  ++impl_->encoded_frame_count;
  return {};
}

Result<Rgba8Image, RendererError> MetalPointRenderer::captureRgba() const {
  if (impl_->color_texture == nil || impl_->extent.width <= 0 ||
      impl_->extent.height <= 0) {
    return error(RendererErrorCode::EncodingFailed,
                 "Metal capture requires a rendered non-empty viewport");
  }

  id<MTLCommandBuffer> rendered = impl_->last_viewport_render;
  if (rendered == nil || rendered.status == MTLCommandBufferStatusNotEnqueued) {
    return error(RendererErrorCode::EncodingFailed,
                 "Metal capture requires a completed viewport render");
  }
  if (rendered.status != MTLCommandBufferStatusCompleted) {
    [rendered waitUntilCompleted];
  }
  if (rendered.status == MTLCommandBufferStatusError) {
    const char *message = rendered.error.localizedDescription.UTF8String;
    return error(RendererErrorCode::EncodingFailed,
                 std::string("Metal viewport render failed before capture: ") +
                     (message == nullptr ? "unknown error" : message));
  }

  id<MTLTexture> texture = impl_->color_texture;
  const NSUInteger width = texture.width;
  const NSUInteger height = texture.height;
  if (width == 0 || height == 0 ||
      width > static_cast<NSUInteger>((std::numeric_limits<int>::max)()) ||
      height > static_cast<NSUInteger>((std::numeric_limits<int>::max)()) ||
      width > (std::numeric_limits<NSUInteger>::max)() / NSUInteger{4}) {
    return error(RendererErrorCode::EncodingFailed,
                 "Metal capture texture extent is invalid");
  }
  const NSUInteger packed_row = width * NSUInteger{4};
  constexpr NSUInteger alignment = 256;
  if (packed_row >
      (std::numeric_limits<NSUInteger>::max)() - (alignment - NSUInteger{1})) {
    return error(RendererErrorCode::EncodingFailed,
                 "Metal capture row alignment overflows");
  }
  const NSUInteger aligned_row =
      (packed_row + (alignment - NSUInteger{1})) & ~(alignment - NSUInteger{1});
  if (height > (std::numeric_limits<NSUInteger>::max)() / aligned_row ||
      packed_row > (std::numeric_limits<std::size_t>::max)() ||
      height > (std::numeric_limits<std::size_t>::max)() /
                   static_cast<std::size_t>(packed_row)) {
    return error(RendererErrorCode::EncodingFailed,
                 "Metal capture image size overflows");
  }
  const NSUInteger buffer_size = aligned_row * height;

  // Inspector actions run before this frame's viewport draw, so this separate
  // command observes the most recently completed offscreen texture without
  // committing the runtime-owned presentation command buffer.
  id<MTLCommandBuffer> command = [impl_->command_queue commandBuffer];
  id<MTLBuffer> buffer =
      [impl_->device newBufferWithLength:buffer_size
                                 options:MTLResourceStorageModeShared];
  if (command == nil || buffer == nil) {
    return error(RendererErrorCode::ResourceCreationFailed,
                 "Metal capture command or readback buffer creation failed");
  }
  id<MTLBlitCommandEncoder> blit = [command blitCommandEncoder];
  if (blit == nil) {
    return error(RendererErrorCode::ResourceCreationFailed,
                 "Metal capture blit encoder creation failed");
  }
  [blit copyFromTexture:texture
                   sourceSlice:0
                   sourceLevel:0
                  sourceOrigin:MTLOriginMake(0, 0, 0)
                    sourceSize:MTLSizeMake(width, height, 1)
                      toBuffer:buffer
             destinationOffset:0
        destinationBytesPerRow:aligned_row
      destinationBytesPerImage:buffer_size];
  [blit endEncoding];
  [command commit];
  [command waitUntilCompleted];
  if (command.status == MTLCommandBufferStatusError) {
    const char *message = command.error.localizedDescription.UTF8String;
    return error(RendererErrorCode::EncodingFailed,
                 std::string("Metal capture failed: ") +
                     (message == nullptr ? "unknown error" : message));
  }

  Rgba8Image result;
  result.extent = {static_cast<int>(width), static_cast<int>(height)};
  result.bytes_per_row = static_cast<std::size_t>(packed_row);
  result.pixels.resize(result.bytes_per_row * static_cast<std::size_t>(height));
  const auto *source = static_cast<const std::uint8_t *>(buffer.contents);
  // Metal's BGRA8 texture rows are already top-left ordered.  Convert only
  // channel order; public Rgba8Image is RGBA8 for all three backends.
  for (NSUInteger row = 0; row < height; ++row) {
    const auto *input = source + row * aligned_row;
    auto *output = result.pixels.data() +
                   static_cast<std::size_t>(row) * result.bytes_per_row;
    for (NSUInteger column = 0; column < width; ++column) {
      const auto *pixel = input + column * NSUInteger{4};
      auto *rgba = output + column * NSUInteger{4};
      rgba[0] = pixel[2];
      rgba[1] = pixel[1];
      rgba[2] = pixel[0];
      rgba[3] = pixel[3];
    }
  }
  return result;
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
std::size_t MetalPointRenderer::layeredLodPointCountForTests(
    std::uint64_t layer_id) const noexcept {
  const auto iterator = impl_->layer_buffers.find(layer_id);
  return iterator == impl_->layer_buffers.end() ? 0
                                                : iterator->second.lod_count;
}

} // namespace kpt::gui
