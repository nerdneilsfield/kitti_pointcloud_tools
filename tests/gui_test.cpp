#include <catch2/catch.hpp>

#include "gui/app.hpp"
#include "gui/jobs/job_system.hpp"
#include "gui/viewport/pcl_adapter.hpp"

#include <atomic>
#include <chrono>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <vector>

using namespace std::chrono_literals;

namespace {

class FakeFrameContext final : public kpt::gui::FrameContext {
public:
  [[nodiscard]] kpt::gui::BackendKind backendKind() const noexcept override {
    return kpt::gui::BackendKind::OpenGL;
  }
};

class FakeRenderer final : public kpt::gui::ViewportRenderer {
public:
  kpt::Result<void, kpt::gui::RendererError>
  upload(std::span<const kpt::gui::ViewportVertex>,
         std::uint64_t revision) override {
    calls.push_back("upload:" + std::to_string(revision));
    if (fail_stage == kpt::gui::AppStage::Upload)
      return error();
    return {};
  }

  kpt::Result<void, kpt::gui::RendererError>
  resize(kpt::gui::PixelExtent physical_pixels) override {
    extent_ = physical_pixels;
    calls.push_back("resize:" + std::to_string(physical_pixels.width) + "x" +
                    std::to_string(physical_pixels.height));
    if (fail_stage == kpt::gui::AppStage::Resize)
      return error();
    return {};
  }

  kpt::Result<void, kpt::gui::RendererError>
  render(const kpt::gui::ViewportFrame &,
         kpt::gui::FrameContext &context) override {
    seen_context = &context;
    calls.push_back("render");
    if (fail_stage == kpt::gui::AppStage::Render)
      return error();
    return {};
  }

  [[nodiscard]] kpt::gui::ViewportTexture texture() const override {
    calls.push_back("texture");
    return {};
  }
  [[nodiscard]] kpt::gui::PixelExtent extent() const override {
    return extent_;
  }
  [[nodiscard]] kpt::gui::BackendKind backendKind() const noexcept override {
    return kpt::gui::BackendKind::OpenGL;
  }

  [[nodiscard]] static kpt::gui::RendererError error() {
    return {kpt::gui::RendererErrorCode::EncodingFailed, "fake failure"};
  }

  mutable std::vector<std::string> calls;
  kpt::gui::FrameContext *seen_context = nullptr;
  std::optional<kpt::gui::AppStage> fail_stage;
  kpt::gui::PixelExtent extent_;
};

std::shared_ptr<const kpt::gui::ViewportCloudSnapshot>
snapshot(std::uint64_t revision) {
  auto value = std::make_shared<kpt::gui::ViewportCloudSnapshot>();
  value->revision = revision;
  value->vertices.push_back({});
  return value;
}

} // namespace

TEST_CASE("viewport session orders GPU work and only uploads cloud revisions",
          "[gui]") {
  auto renderer = std::make_unique<FakeRenderer>();
  auto *fake = renderer.get();
  kpt::gui::ViewportSession session(std::move(renderer));
  const auto generation = session.beginRequest();
  REQUIRE(session.accept(snapshot(generation)));
  FakeFrameContext context;

  auto first =
      session.draw({640, 480}, context, kpt::gui::ViewportRole::Main);
  REQUIRE(first);
  REQUIRE(first.value().has_value());
  REQUIRE(fake->calls ==
          std::vector<std::string>{"upload:1", "resize:640x480", "render",
                                   "texture"});
  REQUIRE(fake->seen_context == &context);

  fake->calls.clear();
  session.model.orbit(1.0F, 2.0F);
  kpt::gui::ViewportStyle style;
  style.point_size = 8.0F;
  session.model.setStyle(style);
  REQUIRE(session.draw({640, 480}, context, kpt::gui::ViewportRole::Main));
  REQUIRE(fake->calls ==
          std::vector<std::string>{"resize:640x480", "render", "texture"});
}

TEST_CASE("viewport sessions share context and reject stale completions",
          "[gui]") {
  auto first_renderer = std::make_unique<FakeRenderer>();
  auto second_renderer = std::make_unique<FakeRenderer>();
  auto *first = first_renderer.get();
  auto *second = second_renderer.get();
  kpt::gui::ViewportSession main(std::move(first_renderer));
  kpt::gui::ViewportSession trajectory(std::move(second_renderer));
  const auto old_generation = main.beginRequest();
  const auto newest_generation = main.beginRequest();
  REQUIRE_FALSE(main.accept(snapshot(old_generation)));
  REQUIRE(main.accept(snapshot(newest_generation)));
  const auto trajectory_generation = trajectory.beginRequest();
  REQUIRE(trajectory.accept(snapshot(trajectory_generation)));

  FakeFrameContext context;
  REQUIRE(main.draw({320, 240}, context, kpt::gui::ViewportRole::Main));
  REQUIRE(trajectory.draw({160, 120}, context,
                          kpt::gui::ViewportRole::Trajectory));
  REQUIRE(first->seen_context == &context);
  REQUIRE(second->seen_context == &context);
  REQUIRE(main.model.cloudRevision() == newest_generation);
}

TEST_CASE("viewport session skips zero-sized rendering and reports stage",
          "[gui]") {
  auto renderer = std::make_unique<FakeRenderer>();
  auto *fake = renderer.get();
  kpt::gui::ViewportSession session(std::move(renderer));
  REQUIRE(session.accept(snapshot(session.beginRequest())));
  FakeFrameContext context;

  auto zero = session.draw({0, 12}, context, kpt::gui::ViewportRole::Main);
  REQUIRE(zero);
  REQUIRE_FALSE(zero.value().has_value());
  REQUIRE(fake->calls ==
          std::vector<std::string>{"upload:1", "resize:0x12"});

  for (const auto stage : {kpt::gui::AppStage::Upload,
                           kpt::gui::AppStage::Resize,
                           kpt::gui::AppStage::Render}) {
    auto failing_renderer = std::make_unique<FakeRenderer>();
    failing_renderer->fail_stage = stage;
    kpt::gui::ViewportSession failing(std::move(failing_renderer));
    REQUIRE(failing.accept(snapshot(failing.beginRequest())));
    auto result =
        failing.draw({10, 10}, context, kpt::gui::ViewportRole::Trajectory);
    REQUIRE_FALSE(result);
    REQUIRE(result.error().role == kpt::gui::ViewportRole::Trajectory);
    REQUIRE(result.error().stage == stage);
    REQUIRE(result.error().cause.message == "fake failure");
  }
}

TEST_CASE("GUI bounds ignore non-finite points and track scalar ranges",
          "[gui]") {
  kpt::PointCloudIRGB cloud;
  kpt::PointT first{};
  first.x = -2.0F;
  first.y = 1.0F;
  first.z = 3.0F;
  first.intensity = 0.25F;
  cloud.push_back(first);

  kpt::PointT second{};
  second.x = 4.0F;
  second.y = 5.0F;
  second.z = -1.0F;
  second.intensity = 2.0F;
  cloud.push_back(second);

  kpt::PointT invalid{};
  invalid.x = std::numeric_limits<float>::quiet_NaN();
  cloud.push_back(invalid);

  const auto bounds = kpt::gui::calculateBounds(cloud);
  REQUIRE(bounds.finite_points == 2);
  REQUIRE(bounds.minimum.x() == -2.0F);
  REQUIRE(bounds.maximum.y() == 5.0F);
  REQUIRE(bounds.z_min == -1.0F);
  REQUIRE(bounds.z_max == 3.0F);
  REQUIRE(bounds.intensity_min == 0.25F);
  REQUIRE(bounds.intensity_max == 2.0F);
  REQUIRE(bounds.radius > 0.0F);
}

TEST_CASE("GUI bounds are benign for empty cloud", "[gui]") {
  const auto bounds = kpt::gui::calculateBounds({});
  REQUIRE(bounds.finite_points == 0);
  REQUIRE(bounds.center.isZero());
  REQUIRE(bounds.radius == 1.0F);
}

TEST_CASE("job system reports completion and cancellation", "[gui]") {
  kpt::gui::JobSystem jobs;
  jobs.setWorkerLimit(1);
  std::atomic<bool> release{false};
  const auto blocker =
      jobs.submit("blocker", kpt::gui::JobPriority::High,
                  [&release](std::stop_token stop,
                             const kpt::gui::JobSystem::Reporter &report) {
                    while (!release.load() && !stop.stop_requested()) {
                      std::this_thread::sleep_for(1ms);
                    }
                    report(1.0F, "released");
                  });
  const auto cancelled = jobs.submit(
      "cancelled", kpt::gui::JobPriority::Low,
      [](std::stop_token, const kpt::gui::JobSystem::Reporter &) {});
  jobs.cancel(cancelled);
  release.store(true);

  const auto deadline = std::chrono::steady_clock::now() + 2s;
  bool blocker_done = false;
  bool queued_cancelled = false;
  while (std::chrono::steady_clock::now() < deadline) {
    for (const auto &snapshot : jobs.snapshots()) {
      if (snapshot.id == blocker &&
          snapshot.state == kpt::gui::JobState::Succeeded) {
        blocker_done = true;
      }
      if (snapshot.id == cancelled &&
          snapshot.state == kpt::gui::JobState::Cancelled) {
        queued_cancelled = true;
      }
    }
    if (blocker_done && queued_cancelled)
      break;
    std::this_thread::sleep_for(2ms);
  }
  REQUIRE(blocker_done);
  REQUIRE(queued_cancelled);
}

TEST_CASE("job system reserves one active-player worker for high priority",
          "[gui]") {
  kpt::gui::JobSystem jobs;
  if (jobs.maxWorkers() < 2) {
    SUCCEED("single-core host has no reservable worker");
    return;
  }

  jobs.setWorkerLimit(2);
  jobs.setPlayerActive(true);
  std::atomic<bool> normal_started{false};
  std::atomic<bool> release_normal{false};
  std::atomic<bool> low_started{false};
  std::atomic<bool> high_started{false};

  jobs.submit("normal blocker", kpt::gui::JobPriority::Normal,
              [&](std::stop_token stop, const kpt::gui::JobSystem::Reporter &) {
                normal_started.store(true);
                while (!release_normal.load() && !stop.stop_requested())
                  std::this_thread::sleep_for(1ms);
              });

  const auto wait_for = [](const std::atomic<bool> &flag) {
    const auto deadline = std::chrono::steady_clock::now() + 2s;
    while (!flag.load() && std::chrono::steady_clock::now() < deadline)
      std::this_thread::sleep_for(1ms);
    return flag.load();
  };
  REQUIRE(wait_for(normal_started));

  jobs.submit("low", kpt::gui::JobPriority::Low,
              [&](std::stop_token, const kpt::gui::JobSystem::Reporter &) {
                low_started.store(true);
              });
  std::this_thread::sleep_for(30ms);
  REQUIRE_FALSE(low_started.load());

  jobs.submit("high", kpt::gui::JobPriority::High,
              [&](std::stop_token, const kpt::gui::JobSystem::Reporter &) {
                high_started.store(true);
              });
  REQUIRE(wait_for(high_started));
  REQUIRE_FALSE(low_started.load());

  release_normal.store(true);
  REQUIRE(wait_for(low_started));
}
