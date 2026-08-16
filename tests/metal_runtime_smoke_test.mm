#include <catch2/catch.hpp>

#include "gui/runtime/test_support.hpp"

#include <imgui.h>

#include <array>

namespace {

kpt::gui::GuiRuntimeOptions hiddenOptions() {
  kpt::gui::GuiRuntimeOptions options;
  options.width = 96;
  options.height = 72;
  options.title = "KPT Metal runtime test";
  options.visible = false;
  options.persist_settings = false;
  return options;
}

} // namespace

TEST_CASE("Metal runtime rejects invalid state transitions",
          "[metal_runtime]") {
  auto runtime = kpt::gui::createGuiRuntimeForTests({});
  auto invalid = hiddenOptions();
  invalid.width = 0;
  REQUIRE_FALSE(runtime->initialize(invalid));
  REQUIRE_FALSE(runtime->initialize(hiddenOptions()));
  REQUIRE_FALSE(runtime->beginFrame());
  REQUIRE_FALSE(runtime->renderAndPresent());
  runtime->shutdown();
  runtime->shutdown();
}

TEST_CASE("Metal runtime cleans partial initialization idempotently",
          "[metal_runtime]") {
  constexpr std::array fault_points = {
      kpt::gui::detail::RuntimeFaultPoint::AfterWindowSystem,
      kpt::gui::detail::RuntimeFaultPoint::AfterWindow,
      kpt::gui::detail::RuntimeFaultPoint::AfterImGuiContext,
      kpt::gui::detail::RuntimeFaultPoint::AfterPlatformBackend};
  for (const auto fault_point : fault_points) {
    DYNAMIC_SECTION("fault point " << static_cast<int>(fault_point)) {
      kpt::gui::detail::RuntimeTestHooks hooks;
      hooks.fail_at = fault_point;
      auto runtime = kpt::gui::createGuiRuntimeForTests(hooks);
      REQUIRE_FALSE(runtime->initialize(hiddenOptions()));
      runtime->shutdown();
      runtime->shutdown();
    }
  }
}

TEST_CASE("Metal runtime owns one frame and presents Dear ImGui",
          "[metal_runtime]") {
  auto runtime = kpt::gui::createGuiRuntimeForTests({});
  REQUIRE(runtime->initialize(hiddenOptions()));
  const auto metrics = runtime->framebufferMetrics();
  REQUIRE(metrics.logical_size.x > 0.0F);
  REQUIRE(metrics.logical_size.y > 0.0F);
  REQUIRE(metrics.framebuffer_size.width > 0);
  REQUIRE(metrics.framebuffer_size.height > 0);
  REQUIRE(metrics.scale.x > 0.0F);
  REQUIRE(metrics.scale.y > 0.0F);

  auto context = runtime->beginFrame();
  REQUIRE(context);
  REQUIRE(context.value().get().backendKind() == kpt::gui::BackendKind::Metal);
  REQUIRE_FALSE(runtime->beginFrame());
  ImGui::Begin("Metal smoke");
  ImGui::TextUnformatted("Metal frame");
  ImGui::End();
  REQUIRE(runtime->renderAndPresent());
  REQUIRE_FALSE(runtime->renderAndPresent());
  runtime->shutdown();
}

TEST_CASE("Metal runtime creates compatible renderers and reports failures",
          "[metal_runtime]") {
  SECTION("compatible renderer") {
    auto runtime = kpt::gui::createGuiRuntimeForTests({});
    REQUIRE(runtime->initialize(hiddenOptions()));
    auto renderer = runtime->createViewportRenderer();
    REQUIRE(renderer);
    REQUIRE(renderer.value()->backendKind() == kpt::gui::BackendKind::Metal);
    runtime->shutdown();
  }

  SECTION("structured creation failure") {
    kpt::gui::detail::RuntimeTestHooks hooks;
    hooks.fail_at = kpt::gui::detail::RuntimeFaultPoint::RendererCreation;
    auto runtime = kpt::gui::createGuiRuntimeForTests(hooks);
    REQUIRE(runtime->initialize(hiddenOptions()));
    const auto renderer = runtime->createViewportRenderer();
    REQUIRE_FALSE(renderer);
    REQUIRE(renderer.error().code ==
            kpt::gui::GuiErrorCode::RendererCreationFailed);
    runtime->shutdown();
  }
}

TEST_CASE("Metal runtime captures the prior committed viewport frame",
          "[metal_runtime][capture]") {
  // This intentionally exercises GlfwMetalRuntime::renderAndPresent(), not
  // MetalRendererTestAccess. The first viewport pass is committed by the
  // runtime; capture happens at the beginning of the next active UI frame,
  // matching App's one-frame-deferred screenshot path.
  auto runtime = kpt::gui::createGuiRuntimeForTests({});
  REQUIRE(runtime->initialize(hiddenOptions()));
  auto created = runtime->createViewportRenderer();
  REQUIRE(created);
  auto renderer = std::move(created).value();
  REQUIRE(renderer->resize({64, 48}));
  const std::array vertices = {
      kpt::gui::ViewportVertex{{0.0F, 0.0F, 0.0F},
                               {0.2F, 0.8F, 0.4F}, 0.5F, 0.0F},
  };
  REQUIRE(renderer->upload(vertices, 1));
  kpt::gui::ViewportFrame frame;
  frame.style.color_by = kpt::ColorBy::RGB;
  frame.style.point_size = 15.0F;

  auto first = runtime->beginFrame();
  REQUIRE(first);
  REQUIRE(renderer->render(frame, first.value().get()));
  REQUIRE(runtime->renderAndPresent());

  auto second = runtime->beginFrame();
  REQUIRE(second);
  // This must not see the new, uncommitted second-frame command buffer.
  const auto captured = renderer->captureRgba();
  REQUIRE(captured);
  REQUIRE(captured.value().extent == kpt::gui::PixelExtent{64, 48});
  REQUIRE(captured.value().pixels.size() == 64U * 48U * 4U);
  REQUIRE(renderer->render(frame, second.value().get()));
  REQUIRE(runtime->renderAndPresent());
  runtime->shutdown();
}

TEST_CASE("Metal runtime supports repeated startup and shutdown",
          "[metal_runtime]") {
  for (int iteration = 0; iteration < 4; ++iteration) {
    auto runtime = kpt::gui::createGuiRuntimeForTests({});
    REQUIRE(runtime->initialize(hiddenOptions()));
    REQUIRE(runtime->beginFrame());
    ImGui::NewLine();
    REQUIRE(runtime->renderAndPresent());
    runtime->shutdown();
  }
}
