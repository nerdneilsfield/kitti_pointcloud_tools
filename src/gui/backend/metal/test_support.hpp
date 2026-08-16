#pragma once

#include "gui/viewport/test_access.hpp"

#include <memory>

namespace kpt::gui {

struct MetalRendererTestFixture {
  RendererTestFixture renderer;
  std::unique_ptr<FrameContext> frame_context;
  std::shared_ptr<void> command_buffer;
};

[[nodiscard]] MetalRendererTestFixture makeMetalRendererTestFixture();
[[nodiscard]] Result<std::reference_wrapper<FrameContext>, RendererError>
beginMetalFrameForTests(MetalRendererTestFixture &fixture);
[[nodiscard]] std::unique_ptr<FrameContext>
makeInactiveMetalFrameContextForTests();
[[nodiscard]] std::uint64_t
metalEncodedFrameCountForTests(const ViewportRenderer &renderer);
[[nodiscard]] std::size_t
metalLayeredLodPointCountForTests(const ViewportRenderer &renderer,
                                  std::uint64_t layer_id);

} // namespace kpt::gui
