#include <catch2/catch.hpp>

#include "gui/backend/opengl/point_renderer.hpp"
#include "gui/runtime/factory.hpp"

#include <array>
#include <memory>

namespace {

kpt::gui::GuiRuntimeOptions hiddenOptions() {
  kpt::gui::GuiRuntimeOptions options;
  options.width = 96;
  options.height = 64;
  options.title = "KPT runtime test";
  options.visible = false;
  options.persist_settings = false;
  return options;
}

} // namespace

TEST_CASE("runtime rejects invalid options and invalid state transitions",
          "[gui][runtime]") {
  auto runtime = kpt::gui::createGuiRuntime();
  auto before_initialize = runtime->beginFrame();
  REQUIRE_FALSE(before_initialize);
  REQUIRE(before_initialize.error().code ==
          kpt::gui::GuiErrorCode::InvalidState);
  REQUIRE_FALSE(runtime->renderAndPresent());

  auto options = hiddenOptions();
  options.width = 0;
  auto initialized = runtime->initialize(options);
  REQUIRE_FALSE(initialized);
  REQUIRE(initialized.error().code == kpt::gui::GuiErrorCode::InvalidOptions);
  runtime->shutdown();
  runtime->shutdown();
  REQUIRE(runtime->shouldClose());
}

TEST_CASE("partial runtime initialization has idempotent cleanup",
          "[gui][runtime]") {
  constexpr std::array fault_points{
      kpt::gui::detail::RuntimeFaultPoint::AfterWindowSystem,
      kpt::gui::detail::RuntimeFaultPoint::AfterWindow,
      kpt::gui::detail::RuntimeFaultPoint::AfterImGuiContext,
      kpt::gui::detail::RuntimeFaultPoint::AfterPlatformBackend};

  for (const auto fault_point : fault_points) {
    DYNAMIC_SECTION("fault point " << static_cast<int>(fault_point)) {
      auto runtime = kpt::gui::createGuiRuntimeForTests({fault_point});
      REQUIRE_FALSE(runtime->initialize(hiddenOptions()));
      runtime->shutdown();
      runtime->shutdown();
      REQUIRE(runtime->shouldClose());
    }
  }
}

TEST_CASE("runtime owns exactly one active frame and refreshes metrics",
          "[gui][runtime]") {
  auto runtime = kpt::gui::createGuiRuntime();
  REQUIRE(runtime->initialize(hiddenOptions()));
  const auto metrics = runtime->framebufferMetrics();
  REQUIRE(metrics.logical_size.x > 0.0F);
  REQUIRE(metrics.logical_size.y > 0.0F);
  REQUIRE(metrics.framebuffer_size.width > 0);
  REQUIRE(metrics.framebuffer_size.height > 0);
  REQUIRE(metrics.scale.x > 0.0F);
  REQUIRE(metrics.scale.y > 0.0F);

  auto begun = runtime->beginFrame();
  REQUIRE(begun);
  auto *context =
      dynamic_cast<kpt::gui::OpenGLFrameContext *>(&begun.value().get());
  REQUIRE(context != nullptr);
  REQUIRE(context->isActive());
  auto nested = runtime->beginFrame();
  REQUIRE_FALSE(nested);
  REQUIRE(nested.error().code == kpt::gui::GuiErrorCode::InvalidState);
  REQUIRE(runtime->renderAndPresent());
  REQUIRE_FALSE(context->isActive());
  REQUIRE_FALSE(runtime->renderAndPresent());
  runtime->shutdown();
}

TEST_CASE("runtime creates compatible renderers and reports creation failure",
          "[gui][runtime]") {
  SECTION("compatible renderer") {
    auto runtime = kpt::gui::createGuiRuntime();
    REQUIRE(runtime->initialize(hiddenOptions()));
    {
      auto renderer = runtime->createViewportRenderer();
      REQUIRE(renderer);
      REQUIRE(renderer.value()->backendKind() == kpt::gui::BackendKind::OpenGL);
      REQUIRE(renderer.value()->resize({16, 16}));
      auto begun = runtime->beginFrame();
      REQUIRE(begun);
      kpt::gui::ViewportFrame frame;
      REQUIRE(renderer.value()->render(frame, begun.value().get()));
      REQUIRE(runtime->renderAndPresent());
      auto outside_frame = renderer.value()->render(frame, begun.value().get());
      REQUIRE_FALSE(outside_frame);
      REQUIRE(outside_frame.error().code ==
              kpt::gui::RendererErrorCode::BackendMismatch);
    }
    runtime->shutdown();
  }

  SECTION("structured failure") {
    auto runtime = kpt::gui::createGuiRuntimeForTests(
        {kpt::gui::detail::RuntimeFaultPoint::RendererCreation});
    REQUIRE(runtime->initialize(hiddenOptions()));
    auto renderer = runtime->createViewportRenderer();
    REQUIRE_FALSE(renderer);
    REQUIRE(renderer.error().code ==
            kpt::gui::GuiErrorCode::RendererCreationFailed);
    runtime->shutdown();
  }
}
