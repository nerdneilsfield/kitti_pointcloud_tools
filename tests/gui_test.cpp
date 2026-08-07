#include <catch2/catch.hpp>

#include "gui/app.hpp"
#include "gui/jobs/job_system.hpp"
#include "gui/viewport/cloud_adapter.hpp"
#include "gui/viewport/session.hpp"
#include "platform/utf8_path.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <filesystem>
#include <limits>
#include <memory>
#include <string>
#include <string_view>
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
  explicit FakeRenderer(std::string renderer_label = {},
                        std::vector<std::string> *trace_sink = nullptr)
      : label(std::move(renderer_label)), shared_trace(trace_sink) {}

  kpt::Result<void, kpt::gui::RendererError>
  upload(std::span<const kpt::gui::ViewportVertex> vertices,
         std::uint64_t revision) override {
    calls.push_back("upload:" + std::to_string(revision));
    uploaded_sizes.push_back(vertices.size());
    trace("upload:" + std::to_string(revision));
    if (fail_stage == kpt::gui::AppStage::Upload)
      return error();
    return {};
  }

  kpt::Result<void, kpt::gui::RendererError>
  resize(kpt::gui::PixelExtent physical_pixels) override {
    extent_ = physical_pixels;
    calls.push_back("resize:" + std::to_string(physical_pixels.width) + "x" +
                    std::to_string(physical_pixels.height));
    trace("resize:" + std::to_string(physical_pixels.width) + "x" +
          std::to_string(physical_pixels.height));
    if (fail_stage == kpt::gui::AppStage::Resize)
      return error();
    return {};
  }

  kpt::Result<void, kpt::gui::RendererError>
  render(const kpt::gui::ViewportFrame &frame,
         kpt::gui::FrameContext &context) override {
    seen_context = &context;
    last_interactive_lod = frame.interactive_lod;
    calls.push_back("render");
    trace("render");
    if (fail_stage == kpt::gui::AppStage::Render)
      return error();
    return {};
  }

  [[nodiscard]] kpt::gui::ViewportTexture texture() const override {
    calls.push_back("texture");
    trace("texture");
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
  std::vector<std::size_t> uploaded_sizes;
  kpt::gui::FrameContext *seen_context = nullptr;
  bool last_interactive_lod = false;
  std::optional<kpt::gui::AppStage> fail_stage;
  kpt::gui::PixelExtent extent_;

private:
  void trace(const std::string &event) const {
    if (shared_trace != nullptr)
      shared_trace->push_back(label + "-" + event);
  }

  std::string label;
  std::vector<std::string> *shared_trace = nullptr;
};

class FakeAssetStager final : public kpt::gui::web::AssetStager {
public:
  void stage(std::vector<std::filesystem::path> paths,
             Completion next) override {
    staged = std::move(paths);
    completion = std::move(next);
  }

  void release(const std::vector<std::filesystem::path> &) override {
    ++release_count;
  }

  std::vector<std::filesystem::path> staged;
  Completion completion;
  unsigned release_count = 0;
};

std::shared_ptr<const kpt::gui::ViewportCloudSnapshot>
snapshot(std::uint64_t revision) {
  auto value = std::make_shared<kpt::gui::ViewportCloudSnapshot>();
  value->revision = revision;
  value->vertices.push_back({});
  return value;
}

} // namespace

namespace kpt::gui {

class AppTestAccess {
public:
  static std::uint64_t advanceSequence(App &app) {
    return ++app.sequence_generation_;
  }

  static std::uint64_t beginMainRequest(App &app) {
    return app.main_viewport_.beginRequest();
  }

  static void
  postMainCompletion(App &app, std::uint64_t sequence_generation,
                     std::shared_ptr<const ViewportCloudSnapshot> completed,
                     std::vector<std::string> &trace, std::string marker) {
    app.ui_.post([&app, sequence_generation, completed = std::move(completed),
                  &trace, marker = std::move(marker)] {
      trace.push_back(marker);
      if (sequence_generation != app.sequence_generation_)
        return;
      static_cast<void>(app.main_viewport_.accept(completed));
    });
  }

  static bool
  setTrajectory(App &app,
                std::shared_ptr<const ViewportCloudSnapshot> completed) {
    static_cast<void>(app.trajectory_viewport_.beginRequest());
    return app.trajectory_viewport_.accept(std::move(completed));
  }

  static std::uint64_t mainRevision(const App &app) {
    return app.main_viewport_.cloudRevision();
  }

  static std::uint64_t trajectoryRevision(const App &app) {
    return app.trajectory_viewport_.cloudRevision();
  }

  static std::uint64_t beginNewSource(App &app) { return app.beginNewSource(); }

  static void seedSourceState(App &app) {
    app.playing_ = true;
    app.current_frame_ = 4;
    app.desired_frame_ = 5;
    app.frame_cache_.emplace(4, std::make_shared<PointCloudIRGB>());
    app.pending_frames_.insert(5);
  }

  static bool sourceStateReset(const App &app) {
    return !app.playing_ && !app.sequence_ && app.current_frame_ == 0 &&
           app.desired_frame_ == 0 && app.frame_cache_.empty() &&
           app.pending_frames_.empty();
  }

  static void requestStagedFrame(App &app) {
    workflow::SequenceOptions options;
    app.sequence_ = std::make_shared<workflow::SequenceSource>(
        std::move(options), std::vector<std::filesystem::path>{"0.bin"});
    app.requestFrame(0, true);
  }

  static void seedReplacementFrameState(App &app) {
    ++app.sequence_generation_;
    app.pending_frames_.insert(0);
    app.desired_frame_ = 0;
    app.playing_ = true;
    app.launch_state_ = App::LaunchState::Pending;
    app.launch_error_.reset();
  }

  static bool replacementFrameStatePreserved(const App &app) {
    return app.pending_frames_.contains(0) && app.desired_frame_ == 0 &&
           app.playing_ && app.launch_state_ == App::LaunchState::Pending &&
           !app.launch_error_;
  }

  static std::uint64_t seedMainViewport(App &app) {
    const auto revision = app.main_viewport_.beginRequest();
    REQUIRE(app.main_viewport_.accept(snapshot(revision)));
    return revision;
  }

  static bool acceptMain(App &app,
                         std::shared_ptr<const ViewportCloudSnapshot> value) {
    return app.main_viewport_.accept(std::move(value));
  }

  static void setViewportExtent(App &app, PixelExtent extent) {
    app.viewport_extent_override_for_tests_ = extent;
  }

  static std::vector<JobSnapshot> jobs(const App &app) {
    return app.jobs_.snapshots();
  }

  static void drainUi(App &app) { app.ui_.drain(); }

  static bool hasLogContaining(const App &app, std::string_view text) {
    return std::ranges::any_of(app.logs_, [text](const std::string &message) {
      return message.find(text) != std::string::npos;
    });
  }

  static bool playing(const App &app) { return app.playing_; }
  static bool playingReverse(const App &app) {
    return app.playback_direction_ == App::PlaybackDirection::Reverse;
  }
  static void togglePlayback(App &app, bool reverse) {
    app.togglePlayback(reverse ? App::PlaybackDirection::Reverse
                               : App::PlaybackDirection::Forward);
  }
  static void resetPlayback(App &app) { app.resetPlayback(); }
  static std::optional<std::size_t> nextPlaybackFrame(std::size_t current,
                                                      std::size_t frame_count,
                                                      bool reverse, bool loop) {
    return App::nextPlaybackFrame(current, frame_count,
                                  reverse ? App::PlaybackDirection::Reverse
                                          : App::PlaybackDirection::Forward,
                                  loop);
  }
  static bool launchReady(const App &app) {
    return app.launch_state_ == App::LaunchState::Ready;
  }
  static std::size_t currentFrame(const App &app) { return app.current_frame_; }
  static std::size_t desiredFrame(const App &app) { return app.desired_frame_; }
};

} // namespace kpt::gui

TEST_CASE("stale staging failure does not mutate replacement sequence",
          "[gui][web]") {
  auto stager = std::make_shared<FakeAssetStager>();
  kpt::gui::App app(std::make_unique<FakeRenderer>(),
                    std::make_unique<FakeRenderer>(), 0, stager);
  kpt::gui::AppTestAccess::requestStagedFrame(app);
  REQUIRE(stager->completion);

  kpt::gui::AppTestAccess::seedReplacementFrameState(app);
  stager->completion(std::string("injected staging failure"));

  REQUIRE(kpt::gui::AppTestAccess::replacementFrameStatePreserved(app));
  REQUIRE(stager->release_count == 0);
}

TEST_CASE("job system honors an explicit worker cap", "[jobs][web]") {
  kpt::gui::JobSystem jobs(4);
  REQUIRE(jobs.maxWorkers() == 4);
  REQUIRE(jobs.workerLimit() == 2);
  jobs.setWorkerLimit(99);
  REQUIRE(jobs.workerLimit() == 4);
}

TEST_CASE("viewport session orders GPU work and only uploads cloud revisions",
          "[gui]") {
  auto renderer = std::make_unique<FakeRenderer>();
  auto *fake = renderer.get();
  kpt::gui::ViewportSession session(std::move(renderer));
  const auto generation = session.beginRequest();
  REQUIRE(session.accept(snapshot(generation)));
  FakeFrameContext context;

  auto first = session.draw({640, 480}, context, kpt::gui::ViewportRole::Main);
  REQUIRE(first);
  REQUIRE(first.value().has_value());
  REQUIRE(fake->calls == std::vector<std::string>{"upload:1", "resize:640x480",
                                                  "render", "texture"});
  REQUIRE(fake->seen_context == &context);
  REQUIRE_FALSE(fake->last_interactive_lod);

  fake->calls.clear();
  session.orbit(320.0F, 240.0F, 321.0F, 242.0F, {640, 480});
  kpt::gui::ViewportStyle style;
  style.point_size = 8.0F;
  session.setStyle(style);
  REQUIRE(session.draw({640, 480}, context, kpt::gui::ViewportRole::Main));
  REQUIRE(fake->calls ==
          std::vector<std::string>{"resize:640x480", "render", "texture"});

  REQUIRE(
      session.draw({640, 480}, context, kpt::gui::ViewportRole::Main, true));
  REQUIRE(fake->last_interactive_lod);
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
  REQUIRE(
      trajectory.draw({160, 120}, context, kpt::gui::ViewportRole::Trajectory));
  REQUIRE(first->seen_context == &context);
  REQUIRE(second->seen_context == &context);
  REQUIRE(main.cloudRevision() == newest_generation);
}

TEST_CASE("viewport clear invalidates completions and clears GPU once",
          "[gui]") {
  auto renderer = std::make_unique<FakeRenderer>();
  auto *fake = renderer.get();
  kpt::gui::ViewportSession session(std::move(renderer));
  const auto loaded_revision = session.beginRequest();
  REQUIRE(session.accept(snapshot(loaded_revision)));
  FakeFrameContext context;
  REQUIRE(session.draw({320, 240}, context, kpt::gui::ViewportRole::Main));

  fake->calls.clear();
  fake->uploaded_sizes.clear();
  session.cancelAndClear();
  REQUIRE(session.cloudRevision() == 0);
  REQUIRE_FALSE(session.accept(snapshot(loaded_revision)));

  REQUIRE(session.draw({320, 240}, context, kpt::gui::ViewportRole::Main));
  REQUIRE(fake->calls == std::vector<std::string>{"upload:0", "resize:320x240",
                                                  "render", "texture"});
  REQUIRE(fake->uploaded_sizes == std::vector<std::size_t>{0});

  fake->calls.clear();
  REQUIRE(session.draw({320, 240}, context, kpt::gui::ViewportRole::Main));
  REQUIRE(fake->calls ==
          std::vector<std::string>{"resize:320x240", "render", "texture"});
}

TEST_CASE("new source resets playback and both stale viewports", "[gui][app]") {
  auto main_renderer = std::make_unique<FakeRenderer>();
  auto trajectory_renderer = std::make_unique<FakeRenderer>();
  kpt::gui::App app(std::move(main_renderer), std::move(trajectory_renderer));

  const auto old_source = kpt::gui::AppTestAccess::advanceSequence(app);
  const auto old_main = kpt::gui::AppTestAccess::seedMainViewport(app);
  REQUIRE(kpt::gui::AppTestAccess::setTrajectory(app, snapshot(1)));
  kpt::gui::AppTestAccess::seedSourceState(app);

  const auto new_source = kpt::gui::AppTestAccess::beginNewSource(app);
  REQUIRE(new_source == old_source + 1);
  REQUIRE(kpt::gui::AppTestAccess::sourceStateReset(app));
  REQUIRE(kpt::gui::AppTestAccess::mainRevision(app) == 0);
  REQUIRE(kpt::gui::AppTestAccess::trajectoryRevision(app) == 0);
  REQUIRE_FALSE(kpt::gui::AppTestAccess::acceptMain(app, snapshot(old_main)));
}

TEST_CASE("viewer load failure logs full path and job remains failed",
          "[gui][app]") {
  auto main_renderer = std::make_unique<FakeRenderer>();
  auto trajectory_renderer = std::make_unique<FakeRenderer>();
  kpt::gui::App app(std::move(main_renderer), std::move(trajectory_renderer));
  const auto nonce =
      std::chrono::steady_clock::now().time_since_epoch().count();
  const auto missing_native_path = std::filesystem::temp_directory_path() /
                                   ("kpt-missing-" + std::to_string(nonce)) /
                                   std::filesystem::path(u8"不存在.pcd");
  REQUIRE_FALSE(std::filesystem::exists(missing_native_path));
  const auto missing_path_result =
      kpt::platform::pathToUtf8(missing_native_path);
  REQUIRE(missing_path_result);
  const auto missing_path = missing_path_result.value();

  app.startViewer(missing_native_path);
  const auto deadline = std::chrono::steady_clock::now() + 5s;
  bool failed = false;
  while (std::chrono::steady_clock::now() < deadline) {
    const auto jobs = kpt::gui::AppTestAccess::jobs(app);
    failed = std::ranges::any_of(jobs, [](const kpt::gui::JobSnapshot &job) {
      return job.state == kpt::gui::JobState::Failed;
    });
    if (failed)
      break;
    std::this_thread::yield();
  }

  REQUIRE(failed);
  kpt::gui::AppTestAccess::drainUi(app);
  REQUIRE(kpt::gui::AppTestAccess::hasLogContaining(app, missing_path));
  REQUIRE(app.launchError());
  REQUIRE(app.launchError()->find(missing_path) != std::string::npos);
}

TEST_CASE("sequence autoplay starts only after frame zero is accepted",
          "[gui][app][player]") {
  auto main_renderer = std::make_unique<FakeRenderer>();
  auto trajectory_renderer = std::make_unique<FakeRenderer>();
  kpt::gui::App app(std::move(main_renderer), std::move(trajectory_renderer));

  kpt::workflow::SequenceOptions options;
  options.input_dir = "data";
  options.glob = "000123.pcd";
  options.poses = "data/missing-poses.csv";
  app.startSequence(std::move(options), 120, true);

  const auto deadline = std::chrono::steady_clock::now() + 5s;
  while (std::chrono::steady_clock::now() < deadline &&
         kpt::gui::AppTestAccess::mainRevision(app) == 0) {
    kpt::gui::AppTestAccess::drainUi(app);
    std::this_thread::yield();
  }

  REQUIRE_FALSE(app.launchError());
  REQUIRE(kpt::gui::AppTestAccess::mainRevision(app) != 0);
  REQUIRE(kpt::gui::AppTestAccess::currentFrame(app) == 0);
  REQUIRE(kpt::gui::AppTestAccess::desiredFrame(app) == 0);
  REQUIRE(kpt::gui::AppTestAccess::playing(app));
  REQUIRE(kpt::gui::AppTestAccess::launchReady(app));
  REQUIRE(kpt::gui::AppTestAccess::hasLogContaining(
      app, "Trajectory input ignored:"));
}

TEST_CASE("player supports forward reverse reset and boundary looping",
          "[gui][app][player]") {
  using Access = kpt::gui::AppTestAccess;
  REQUIRE(Access::nextPlaybackFrame(1, 3, false, false) == 2);
  REQUIRE_FALSE(Access::nextPlaybackFrame(2, 3, false, false));
  REQUIRE(Access::nextPlaybackFrame(2, 3, false, true) == 0);
  REQUIRE(Access::nextPlaybackFrame(1, 3, true, false) == 0);
  REQUIRE_FALSE(Access::nextPlaybackFrame(0, 3, true, false));
  REQUIRE(Access::nextPlaybackFrame(0, 3, true, true) == 2);
  REQUIRE_FALSE(Access::nextPlaybackFrame(0, 0, true, true));

  auto main_renderer = std::make_unique<FakeRenderer>();
  auto trajectory_renderer = std::make_unique<FakeRenderer>();
  kpt::gui::App app(std::move(main_renderer), std::move(trajectory_renderer));

  Access::togglePlayback(app, false);
  REQUIRE(Access::playing(app));
  REQUIRE_FALSE(Access::playingReverse(app));
  Access::togglePlayback(app, true);
  REQUIRE(Access::playing(app));
  REQUIRE(Access::playingReverse(app));
  Access::togglePlayback(app, true);
  REQUIRE_FALSE(Access::playing(app));
  Access::resetPlayback(app);
  REQUIRE_FALSE(Access::playing(app));
  REQUIRE_FALSE(Access::playingReverse(app));
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
  REQUIRE(fake->calls == std::vector<std::string>{"upload:1", "resize:0x12"});

  for (const auto stage :
       {kpt::gui::AppStage::Upload, kpt::gui::AppStage::Resize,
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

TEST_CASE("App drains completions before ordered dual viewport drawing",
          "[gui][app]") {
  ImGui::CreateContext();
  ImGui::GetIO().ConfigFlags |= ImGuiConfigFlags_DockingEnable;
  ImGui::GetIO().DisplaySize = {1024.0F, 768.0F};
  ImGui::GetIO().IniFilename = nullptr;
  unsigned char *font_pixels = nullptr;
  int font_width = 0;
  int font_height = 0;
  ImGui::GetIO().Fonts->GetTexDataAsRGBA32(&font_pixels, &font_width,
                                           &font_height);
  {
    std::vector<std::string> trace;
    auto main_renderer = std::make_unique<FakeRenderer>("main", &trace);
    auto trajectory_renderer =
        std::make_unique<FakeRenderer>("trajectory", &trace);
    auto *main = main_renderer.get();
    auto *trajectory = trajectory_renderer.get();
    kpt::gui::App app(std::move(main_renderer), std::move(trajectory_renderer));
    kpt::gui::AppTestAccess::setViewportExtent(app, {320, 240});

    const auto sequence = kpt::gui::AppTestAccess::advanceSequence(app);
    const auto stale_request = kpt::gui::AppTestAccess::beginMainRequest(app);
    const auto current_request = kpt::gui::AppTestAccess::beginMainRequest(app);
    kpt::gui::AppTestAccess::postMainCompletion(app, sequence,
                                                snapshot(stale_request), trace,
                                                "completion-stale-request");
    kpt::gui::AppTestAccess::postMainCompletion(
        app, sequence - 1, snapshot(current_request), trace,
        "completion-stale-sequence");
    kpt::gui::AppTestAccess::postMainCompletion(
        app, sequence, snapshot(current_request), trace, "completion-current");
    REQUIRE(kpt::gui::AppTestAccess::setTrajectory(app, snapshot(1)));

    FakeFrameContext context;
    ImGui::NewFrame();
    REQUIRE(app.draw(context, {{1024.0F, 768.0F}, {1024, 768}, {1.0F, 1.0F}}));
    ImGui::Render();
    // DockBuilder positions newly created windows for the following frame.
    ImGui::NewFrame();
    REQUIRE(app.draw(context, {{1024.0F, 768.0F}, {1024, 768}, {1.0F, 1.0F}}));
    ImGui::Render();

    REQUIRE(kpt::gui::AppTestAccess::mainRevision(app) == current_request);
    std::string rendered_trace;
    for (const auto &event : trace)
      rendered_trace += event + ",";
    INFO(rendered_trace);
    REQUIRE(main->uploaded_sizes == std::vector<std::size_t>{1});
    REQUIRE(trajectory->uploaded_sizes == std::vector<std::size_t>{1});
    REQUIRE(main->seen_context == &context);
    REQUIRE(trajectory->seen_context == &context);
    const auto completion =
        std::find(trace.begin(), trace.end(), "completion-current");
    const auto main_render =
        std::find(trace.begin(), trace.end(), "main-render");
    const auto trajectory_render =
        std::find(trace.begin(), trace.end(), "trajectory-render");
    REQUIRE(completion != trace.end());
    REQUIRE(main_render != trace.end());
    REQUIRE(trajectory_render != trace.end());
    REQUIRE(completion < main_render);
    REQUIRE(main_render < trajectory_render);

    trace.clear();
    main->calls.clear();
    trajectory->calls.clear();
    REQUIRE(kpt::gui::AppTestAccess::setTrajectory(app, snapshot(2)));
    auto empty = std::make_shared<kpt::gui::ViewportCloudSnapshot>();
    empty->revision = 3;
    REQUIRE(kpt::gui::AppTestAccess::setTrajectory(app, empty));
    ImGui::NewFrame();
    REQUIRE(app.draw(context, {{1024.0F, 768.0F}, {1024, 768}, {1.0F, 1.0F}}));
    ImGui::Render();
    REQUIRE(trajectory->uploaded_sizes.back() == 0);
    REQUIRE(std::find(trace.begin(), trace.end(), "trajectory-upload:3") !=
            trace.end());
    REQUIRE(std::find(trace.begin(), trace.end(), "trajectory-resize:0x0") !=
            trace.end());
    REQUIRE(std::find(trace.begin(), trace.end(), "trajectory-render") ==
            trace.end());
    REQUIRE(std::find(trace.begin(), trace.end(), "trajectory-texture") ==
            trace.end());
  }
  ImGui::DestroyContext();
}

TEST_CASE("compact dock layout preserves a usable viewport at 800 by 600",
          "[gui][app][layout]") {
  ImGui::CreateContext();
  ImGui::GetIO().ConfigFlags |= ImGuiConfigFlags_DockingEnable;
  ImGui::GetIO().DisplaySize = {800.0F, 600.0F};
  ImGui::GetIO().IniFilename = nullptr;
  unsigned char *font_pixels = nullptr;
  int font_width = 0;
  int font_height = 0;
  ImGui::GetIO().Fonts->GetTexDataAsRGBA32(&font_pixels, &font_width,
                                           &font_height);
  {
    auto renderer = std::make_unique<FakeRenderer>();
    auto *main = renderer.get();
    kpt::gui::App app(std::move(renderer), std::make_unique<FakeRenderer>());
    kpt::gui::AppTestAccess::seedMainViewport(app);
    FakeFrameContext context;
    for (int frame = 0; frame < 2; ++frame) {
      ImGui::NewFrame();
      REQUIRE(app.draw(context, {{800.0F, 600.0F}, {800, 600}, {1.0F, 1.0F}}));
      ImGui::Render();
    }
    REQUIRE(main->extent().width >= 480);
    REQUIRE(main->extent().height >= 350);
  }
  ImGui::DestroyContext();
}

TEST_CASE("GUI bounds ignore non-finite points and track scalar ranges",
          "[gui]") {
  kpt::PointCloudIRGB cloud;
  cloud.has_noise = true;
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
  second.noise = 1;
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
  REQUIRE(bounds.has_noise);
  REQUIRE(bounds.noise_points == 1);
  REQUIRE(bounds.radius > 0.0F);
}

TEST_CASE("GUI bounds are benign for empty cloud", "[gui]") {
  const auto bounds = kpt::gui::calculateBounds({});
  REQUIRE(bounds.finite_points == 0);
  REQUIRE(bounds.center.isZero());
  REQUIRE(bounds.radius == 1.0F);
}

TEST_CASE("GUI intensity CDF stretches skewed distributions", "[gui][cdf]") {
  kpt::PointCloudIRGB cloud;
  // 9 low-intensity points, 1 high -> distribution heavily skewed low.
  for (int i = 0; i < 9; ++i) {
    kpt::PointT p{};
    p.x = static_cast<float>(i);
    p.y = 0.0F;
    p.z = 0.0F;
    p.intensity = 0.05F;
    cloud.push_back(p);
  }
  kpt::PointT top{};
  top.x = 9.0F;
  top.y = 0.0F;
  top.z = 0.0F;
  top.intensity = 0.95F;
  cloud.push_back(top);

  const auto bounds = kpt::gui::calculateBounds(cloud);
  REQUIRE(bounds.intensity_cdf_valid);
  // ~90% of samples sit at the low end, so the midpoint bin must already be
  // close to the full cumulative mass rather than near 0.5.
  REQUIRE(bounds.intensity_cdf[128] > 0.8F);
  REQUIRE(bounds.intensity_cdf[255] == Approx(1.0F).margin(1e-3F));
}

TEST_CASE("GUI intensity CDF approximates identity for uniform spread",
          "[gui][cdf]") {
  kpt::PointCloudIRGB cloud;
  for (int i = 0; i < 256; ++i) {
    kpt::PointT p{};
    p.x = static_cast<float>(i);
    p.y = 0.0F;
    p.z = 0.0F;
    p.intensity = static_cast<float>(i) / 255.0F;
    cloud.push_back(p);
  }
  const auto bounds = kpt::gui::calculateBounds(cloud);
  REQUIRE(bounds.intensity_cdf_valid);
  REQUIRE(bounds.intensity_cdf[0] == Approx(0.0F).margin(2.0F / 256.0F));
  REQUIRE(bounds.intensity_cdf[128] == Approx(0.5F).margin(4.0F / 256.0F));
  REQUIRE(bounds.intensity_cdf[255] == Approx(1.0F).margin(1e-3F));
}

TEST_CASE("GUI intensity CDF is invalid for degenerate distribution",
          "[gui][cdf]") {
  kpt::PointCloudIRGB cloud;
  kpt::PointT p{};
  p.x = 1.0F;
  p.intensity = 0.42F;
  cloud.push_back(p);
  const auto bounds = kpt::gui::calculateBounds(cloud);
  REQUIRE_FALSE(bounds.intensity_cdf_valid);
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
