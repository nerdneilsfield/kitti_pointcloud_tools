#include <catch2/catch.hpp>

#include "gui/app.hpp"
#include "gui/inspection_share.hpp"
#include "gui/jobs/job_system.hpp"
#include "gui/scene/render_adapter.hpp"
#include "gui/viewport/cloud_adapter.hpp"
#include "gui/viewport/scene_compositor.hpp"
#include "gui/viewport/session.hpp"
#include "kpt/io/io.hpp"
#include "platform/utf8_path.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <filesystem>
#include <fstream>
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
  uploadLayers(std::span<const kpt::gui::ViewportLayerUpload> layers,
               std::uint64_t revision) override {
    calls.push_back("upload-layers:" + std::to_string(revision));
    layered_upload_sizes.push_back(layers.size());
    trace("upload-layers:" + std::to_string(revision));
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

  kpt::Result<void, kpt::gui::RendererError>
  renderLayers(const kpt::gui::ViewportFrame &frame,
               const kpt::gui::LayeredViewportFrame &layers,
               kpt::gui::FrameContext &context) override {
    seen_context = &context;
    last_interactive_lod = frame.interactive_lod;
    last_layered_revision = layers.revision;
    last_opaque_layer_count = layers.opaque_layers.size();
    last_transparent_layer_count = layers.transparent_layers.size();
    calls.push_back("render-layers");
    trace("render-layers");
    if (fail_stage == kpt::gui::AppStage::Render)
      return error();
    return {};
  }

  [[nodiscard]] kpt::Result<kpt::gui::Rgba8Image, kpt::gui::RendererError>
  captureRgba() const override {
    ++capture_calls;
    calls.push_back("capture-rgba");
    trace("capture-rgba");
    if (!capture_image.has_value())
      return error();
    return *capture_image;
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
  std::vector<std::size_t> layered_upload_sizes;
  kpt::gui::FrameContext *seen_context = nullptr;
  bool last_interactive_lod = false;
  std::uint64_t last_layered_revision = 0;
  std::size_t last_opaque_layer_count = 0;
  std::size_t last_transparent_layer_count = 0;
  std::optional<kpt::gui::AppStage> fail_stage;
  std::optional<kpt::gui::Rgba8Image> capture_image;
  mutable int capture_calls = 0;
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

std::shared_ptr<const kpt::gui::LayeredViewportSnapshot>
layeredSnapshot(std::uint64_t revision) {
  auto value = std::make_shared<kpt::gui::LayeredViewportSnapshot>();
  value->revision = revision;
  value->camera_cloud = snapshot(revision);
  kpt::gui::ViewportLayerSnapshot opaque;
  opaque.revision = revision;
  opaque.draw.layer_id = 11;
  opaque.vertices.push_back({});
  value->opaque_layers.push_back(std::move(opaque));
  kpt::gui::ViewportLayerSnapshot transparent;
  transparent.revision = revision;
  transparent.draw.layer_id = 12;
  transparent.draw.opacity = 0.5F;
  transparent.vertices.push_back({});
  value->transparent_layers.push_back(std::move(transparent));
  return value;
}

class TemporaryDirectory {
public:
  TemporaryDirectory() {
    path_ = std::filesystem::temp_directory_path() /
            ("kpt-gui-share-" + std::to_string(
                std::chrono::steady_clock::now().time_since_epoch().count()));
    std::filesystem::create_directories(path_);
  }
  ~TemporaryDirectory() {
    std::error_code error;
    std::filesystem::remove_all(path_, error);
  }

  [[nodiscard]] const std::filesystem::path &path() const noexcept {
    return path_;
  }

private:
  std::filesystem::path path_;
};

kpt::gui::CameraSnapshot reviewCamera() {
  kpt::gui::CameraSnapshot result;
  result.target = {1.0, 2.0, 3.0};
  result.rotation_center = {-4.0, 5.0, 6.0};
  result.distance = 17.5;
  result.fov_y_degrees = 53.0F;
  return result;
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
    app.playback_.toggle(App::PlaybackDirection::Forward);
    app.playback_.applied(4);
    app.playback_.request(5);
    app.frame_cache_.store(4, std::make_shared<PointCloudIRGB>(), 4, 5);
    static_cast<void>(app.frame_cache_.begin(5));
  }

  static bool sourceStateReset(const App &app) {
    return !app.playback_.playing() && !app.sequence_ &&
           app.playback_.current() == 0 && app.playback_.desired() == 0 &&
           app.frame_cache_.empty();
  }

  static void requestStagedFrame(App &app) {
    workflow::SequenceOptions options;
    app.sequence_ = std::make_shared<workflow::SequenceSource>(
        std::move(options), std::vector<std::filesystem::path>{"0.bin"});
    app.requestFrame(0, true);
  }

  static void queueCachedFrame(App &app) {
    auto cloud = std::make_shared<PointCloudIRGB>();
    cloud->push_back({});
    app.playback_.request(0);
    static_cast<void>(app.frame_cache_.begin(0));
    const auto request = app.main_viewport_.beginRequest();
    app.queueCachedFrame(0, std::move(cloud), false, request,
                         app.sequence_generation_);
  }

  static void seedReplacementFrameState(App &app) {
    ++app.sequence_generation_;
    static_cast<void>(app.frame_cache_.begin(0));
    app.playback_.request(0);
    app.playback_.toggle(App::PlaybackDirection::Forward);
    app.launch_state_ = App::LaunchState::Pending;
    app.launch_error_.reset();
  }

  static bool replacementFrameStatePreserved(const App &app) {
    return app.frame_cache_.isPending(0) && app.playback_.desired() == 0 &&
           app.playback_.playing() &&
           app.launch_state_ == App::LaunchState::Pending &&
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

  static void queueInspectionScreenshot(App &app,
                                        const std::filesystem::path &output,
                                        bool overwrite) {
    auto encoded = kpt::platform::pathToUtf8(output);
    REQUIRE(encoded);
    app.inspection_screenshot_output_ = std::move(encoded).value();
    app.inspection_screenshot_overwrite_ = overwrite;
    app.queueInspectionScreenshot();
  }

  static void queueInspectionShareSave(App &app,
                                       const std::filesystem::path &output,
                                       bool overwrite) {
    auto encoded = kpt::platform::pathToUtf8(output);
    REQUIRE(encoded);
    app.inspection_share_output_ = std::move(encoded).value();
    app.inspection_share_overwrite_ = overwrite;
    app.queueInspectionShareSave();
  }

  static void loadInspectionShare(App &app,
                                  const std::filesystem::path &share_path) {
    app.loadInspectionShareFile(share_path);
  }

  static const CloudLayer *inspectionLayer(const App &app,
                                           const std::string &source_key) {
    return app.inspection_scene_.findLayerBySourceKey(source_key);
  }

  static const std::optional<RoiBox> &inspectionRoi(const App &app) {
    return app.inspection_scene_.roi();
  }

  static std::size_t inspectionMeasurementCount(const App &app) {
    return app.inspection_scene_.measurements().size();
  }

  static const CameraBookmark *inspectionBookmark(const App &app,
                                                   const std::string &name) {
    return app.inspection_settings_.findBookmark(name);
  }

  static void drainUi(App &app) { app.ui_.drain(); }

  static bool hasLogContaining(const App &app, std::string_view text) {
    return std::ranges::any_of(app.logs_, [text](const std::string &message) {
      return message.find(text) != std::string::npos;
    });
  }

  static bool playing(const App &app) { return app.playback_.playing(); }
  static bool playingReverse(const App &app) {
    return app.playback_.direction() == App::PlaybackDirection::Reverse;
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
  static std::size_t currentFrame(const App &app) {
    return app.playback_.current();
  }
  static std::size_t desiredFrame(const App &app) {
    return app.playback_.desired();
  }

  static LayerId addInspectionLayer(
      App &app, std::string source_key,
      std::shared_ptr<const PointCloudIRGB> cloud,
      std::shared_ptr<const ViewportCloudSnapshot> snapshot) {
    app.registerInspectionLayer(std::move(source_key), std::move(cloud),
                                std::move(snapshot));
    return *app.inspection_scene_.activeLayer();
  }

  static LayerId addUnresolvedInspectionLayer(App &app,
                                              std::string source_key) {
    const LayerId layer_id = app.inspection_scene_.addLayer(std::move(source_key));
    app.inspection_scene_.clearHistory();
    return layer_id;
  }

  static LayerId addInspectionLayerWithoutSnapshot(
      App &app, std::string source_key,
      std::shared_ptr<const PointCloudIRGB> cloud) {
    const LayerId layer_id =
        app.inspection_scene_.addLayer(std::move(source_key), std::move(cloud));
    app.inspection_scene_.clearHistory();
    return layer_id;
  }

  static bool replaceInspectionLayerCloud(
      App &app, LayerId layer_id,
      std::shared_ptr<const PointCloudIRGB> cloud) {
    return app.inspection_scene_.setLayerCloud(layer_id, std::move(cloud));
  }

  static void refreshInspectionViewport(App &app) {
    app.refreshInspectionViewport(CameraUpdate::Preserve);
  }

  static std::optional<Scene::LayerCloudHydration>
  captureInspectionLayerHydration(const App &app, LayerId layer_id) {
    return app.inspection_scene_.captureLayerCloudHydration(layer_id);
  }

  static void completeInspectionShareLayerLoad(
      App &app, LayerId layer_id, const std::string &source_key,
      const Scene::LayerCloudHydration &hydration,
      std::shared_ptr<const PointCloudIRGB> cloud,
      std::shared_ptr<const ViewportCloudSnapshot> snapshot) {
    app.completeInspectionShareLayerLoad(
        layer_id, source_key, hydration, std::move(cloud), std::move(snapshot),
        "late-source.xyz", app.sequence_generation_);
  }

  static void completeInspectionSnapshotHydration(
      App &app, LayerId layer_id, const std::string &source_key,
      const Scene::LayerCloudHydration &hydration,
      std::shared_ptr<const ViewportCloudSnapshot> snapshot) {
    app.completeInspectionSnapshotHydration(
        layer_id, source_key, hydration, std::move(snapshot));
  }

  static void queueInspectionExport(App &app,
                                    const std::filesystem::path &output) {
    auto encoded = kpt::platform::pathToUtf8(output);
    REQUIRE(encoded);
    app.inspection_export_output_ = std::move(encoded).value();
    app.inspection_export_overwrite_ = false;
    app.inspection_export_scope_ = App::InspectionExportScope::ActiveLayer;
    app.queueInspectionExport();
  }

  static bool removeInspectionLayer(App &app, LayerId layer_id) {
    const bool removed = app.inspection_scene_.removeLayer(layer_id);
    if (removed)
      app.refreshInspectionViewport(CameraUpdate::Preserve);
    return removed;
  }

  static bool undoInspection(App &app) {
    if (!app.inspection_scene_.undo())
      return false;
    app.refreshAfterInspectionHistoryChange();
    return true;
  }

  static bool hasInspectionSnapshot(const App &app, LayerId layer_id) {
    return app.inspection_render_adapter_.hasSnapshot(app.inspection_scene_,
                                                      layer_id);
  }

  static std::shared_ptr<const ViewportCloudSnapshot>
  inspectionSnapshot(const App &app, LayerId layer_id) {
    const SceneRenderSnapshot scene =
        app.inspection_render_adapter_.capture(app.inspection_scene_);
    const auto iterator = std::ranges::find_if(
        scene.layers, [layer_id](const SceneRenderSource &source) {
          return source.layer_id == layer_id;
        });
    return iterator == scene.layers.end() ? nullptr : iterator->snapshot;
  }

  static void drainInspectionUi(App &app) { app.ui_.drain(); }

  static bool inspectionJobsActive(const App &app) {
    return app.jobs_.hasActiveJobs();
  }

  static void setInspectionRoi(App &app, std::optional<RoiBox> roi) {
    app.inspection_scene_.setRoi(std::move(roi));
  }

  static void hydrateInspectionRoi(App &app) {
    app.hydrateInspectionRoiControlsFromScene();
  }

  static bool inspectionRoiControlsEnabled(const App &app) {
    return app.inspection_roi_enabled_;
  }

  static Eigen::Vector3d inspectionRoiControlMinimum(const App &app) {
    return {app.inspection_roi_min_[0], app.inspection_roi_min_[1],
            app.inspection_roi_min_[2]};
  }

  static bool retryInspectionUpload(App &app) {
    return app.retryInspectionUpload(
        {ViewportRole::Main, AppStage::Upload,
         {RendererErrorCode::ResourceCreationFailed, "injected allocation"}});
  }

  static std::optional<std::size_t> inspectionGpuVertexCap(const App &app) {
    return app.inspection_gpu_vertex_cap_;
  }

  static std::size_t inspectionLayerCount(const App &app) {
    return app.inspection_scene_.layers().size();
  }

  static std::size_t renderedInspectionVertices(const App &app) {
    if (!app.inspection_render_list_) {
      return 0;
    }
    std::size_t result = 0;
    for (const LayerRenderItem &item : app.inspection_render_list_->layers) {
      result += item.vertex_selection.retained_vertex_count;
    }
    return result;
  }
};

} // namespace kpt::gui

TEST_CASE("playback engine owns transport transitions", "[gui][player]") {
  using Engine = kpt::gui::PlaybackEngine;
  const auto start = Engine::Clock::time_point{};
  Engine playback;
  playback.configure(10, false);
  playback.resetSource(start);
  playback.toggle(Engine::Direction::Forward, start);

  REQUIRE(playback.poll(3, start) == std::optional<std::size_t>(1));
  REQUIRE(playback.desired() == 0);
  playback.request(1);
  REQUIRE_FALSE(playback.poll(3, start + std::chrono::seconds(1)));
  playback.applied(1);
  REQUIRE(playback.poll(3, start + std::chrono::seconds(1)) ==
          std::optional<std::size_t>(2));
  playback.request(2);
  playback.applied(2);
  REQUIRE_FALSE(playback.poll(3, start + std::chrono::seconds(2)));
  REQUIRE_FALSE(playback.playing());
}

TEST_CASE("playback engine failure and autoplay transitions are atomic",
          "[gui][player]") {
  using Engine = kpt::gui::PlaybackEngine;
  const auto start = Engine::Clock::time_point{};
  Engine playback;
  playback.configure(20, true);
  playback.resetSource(start);
  REQUIRE(playback.autoplayArmed());
  REQUIRE(playback.startAutoplayIfArmed(0, start));
  REQUIRE(playback.playing());
  REQUIRE_FALSE(playback.autoplayArmed());
  playback.request(4);
  playback.applied(2);
  REQUIRE(playback.failIfDesired(4));
  REQUIRE(playback.desired() == 2);
  REQUIRE_FALSE(playback.playing());
}

TEST_CASE("stale staging failure does not mutate replacement sequence",
          "[gui][web]") {
  auto stager = std::make_shared<FakeAssetStager>();
  kpt::gui::App app(std::make_unique<FakeRenderer>(),
                    std::make_unique<FakeRenderer>(), 0, stager);
  kpt::gui::AppTestAccess::requestStagedFrame(app);
  REQUIRE(stager->completion);

  kpt::gui::AppTestAccess::seedReplacementFrameState(app);
  stager->completion(std::string("injected staging failure"));
  kpt::gui::AppTestAccess::drainUi(app);

  REQUIRE(kpt::gui::AppTestAccess::replacementFrameStatePreserved(app));
  REQUIRE(stager->release_count == 0);
}

TEST_CASE("stale cached completion does not clear replacement pending state",
          "[gui][player]") {
  kpt::gui::App app(std::make_unique<FakeRenderer>(),
                    std::make_unique<FakeRenderer>());
  kpt::gui::AppTestAccess::queueCachedFrame(app);
  const auto deadline = std::chrono::steady_clock::now() + 2s;
  while (std::chrono::steady_clock::now() < deadline) {
    const auto jobs = kpt::gui::AppTestAccess::jobs(app);
    if (std::ranges::any_of(jobs, [](const auto &job) {
          return job.state == kpt::gui::JobState::Succeeded;
        }))
      break;
    std::this_thread::yield();
  }
  kpt::gui::AppTestAccess::seedReplacementFrameState(app);
  kpt::gui::AppTestAccess::drainUi(app);
  REQUIRE(kpt::gui::AppTestAccess::replacementFrameStatePreserved(app));
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

TEST_CASE("viewport session uploads and renders native scene layers", "[gui]") {
  auto renderer = std::make_unique<FakeRenderer>();
  auto *fake = renderer.get();
  kpt::gui::ViewportSession session(std::move(renderer));
  const auto revision = session.beginRequest();
  REQUIRE(session.acceptLayered(layeredSnapshot(revision)));

  FakeFrameContext context;
  REQUIRE(session.draw({640, 480}, context, kpt::gui::ViewportRole::Main));
  REQUIRE(fake->calls == std::vector<std::string>{
                             "upload-layers:1", "resize:640x480",
                             "render-layers", "texture"});
  REQUIRE(fake->layered_upload_sizes == std::vector<std::size_t>{2});
  REQUIRE(fake->last_layered_revision == revision);
  REQUIRE(fake->last_opaque_layer_count == 1);
  REQUIRE(fake->last_transparent_layer_count == 1);

  // Browser interaction must propagate the per-frame LOD request through the
  // layered path too; the WebGL backend then chooses each layer's uniform EBO.
  REQUIRE(session.draw({640, 480}, context, kpt::gui::ViewportRole::Main,
                       true));
  REQUIRE(fake->last_interactive_lod);

  fake->calls.clear();
  const auto regular_revision = session.beginRequest();
  REQUIRE(session.accept(snapshot(regular_revision)));
  REQUIRE(session.draw({640, 480}, context, kpt::gui::ViewportRole::Main));
  REQUIRE(fake->calls == std::vector<std::string>{
                             "upload-layers:0", "upload:2", "resize:640x480",
                             "render", "texture"});
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

TEST_CASE("inspection screenshot waits one frame then captures completed viewport PNG",
          "[gui][inspection][screenshot]") {
  ImGui::CreateContext();
  ImGui::GetIO().ConfigFlags |= ImGuiConfigFlags_DockingEnable;
  ImGui::GetIO().DisplaySize = {1024.0F, 768.0F};
  ImGui::GetIO().IniFilename = nullptr;
  unsigned char *font_pixels = nullptr;
  int font_width = 0;
  int font_height = 0;
  ImGui::GetIO().Fonts->GetTexDataAsRGBA32(&font_pixels, &font_width,
                                           &font_height);
  const auto nonce =
      std::chrono::steady_clock::now().time_since_epoch().count();
  const auto output = std::filesystem::temp_directory_path() /
                      ("kpt-screenshot-" + std::to_string(nonce) + ".png");
  std::error_code cleanup_error;
  std::filesystem::remove(output, cleanup_error);
  {
    std::vector<std::string> trace;
    auto renderer = std::make_unique<FakeRenderer>("main", &trace);
    auto *main = renderer.get();
    main->capture_image = {{1, 1}, {0x12U, 0x34U, 0x56U, 0xffU}, 4};
    kpt::gui::App app(std::move(renderer), std::make_unique<FakeRenderer>(), 1);
    kpt::gui::AppTestAccess::setViewportExtent(app, {320, 240});
    kpt::gui::AppTestAccess::seedMainViewport(app);
    kpt::gui::AppTestAccess::queueInspectionScreenshot(app, output, false);

    FakeFrameContext context;
    for (int frame = 0; frame < 2; ++frame) {
      ImGui::NewFrame();
      REQUIRE(app.draw(context, {{1024.0F, 768.0F}, {1024, 768},
                                 {1.0F, 1.0F}}));
      ImGui::Render();
    }
    REQUIRE(main->capture_calls == 1);
    const auto render = std::find(trace.begin(), trace.end(), "main-render");
    const auto second_render =
        render == trace.end()
            ? trace.end()
            : std::find(std::next(render), trace.end(), "main-render");
    const auto capture =
        std::find(trace.begin(), trace.end(), "main-capture-rgba");
    REQUIRE(render != trace.end());
    REQUIRE(second_render != trace.end());
    REQUIRE(capture != trace.end());
    REQUIRE(render < capture);
    REQUIRE(capture < second_render);

    const auto deadline = std::chrono::steady_clock::now() + 2s;
    while (std::chrono::steady_clock::now() < deadline &&
           kpt::gui::AppTestAccess::jobs(app).front().state !=
               kpt::gui::JobState::Succeeded) {
      std::this_thread::sleep_for(2ms);
    }
    kpt::gui::AppTestAccess::drainUi(app);
    REQUIRE(std::filesystem::exists(output));
    REQUIRE(std::filesystem::file_size(output) > 8U);
    REQUIRE(kpt::gui::AppTestAccess::hasLogContaining(app, "Saved viewport PNG"));
  }
  std::filesystem::remove(output, cleanup_error);
  ImGui::DestroyContext();
}

TEST_CASE("native review-share save snapshots Scene to portable JSON",
          "[gui][inspection][share]") {
  TemporaryDirectory directory;
  const auto source = directory.path() / "reviews" / "clouds" / "scan.xyz";
  const auto share = directory.path() / "reviews" / "review.kpt-review.json";
  const auto source_key = kpt::gui::pathSourceKey(source, {});
  auto renderer = std::make_unique<FakeRenderer>();
  kpt::gui::App app(std::move(renderer), std::make_unique<FakeRenderer>(), 1);
  auto cloud = std::make_shared<kpt::PointCloudIRGB>();
  kpt::PointT point{};
  point.x = 3.0F;
  point.y = -2.0F;
  point.z = 1.0F;
  cloud->push_back(point);
  kpt::gui::AppTestAccess::addInspectionLayer(
      app, source_key, cloud, kpt::gui::makeViewportCloudSnapshot(cloud, 1));

  kpt::gui::AppTestAccess::queueInspectionShareSave(app, share, false);
  const auto deadline = std::chrono::steady_clock::now() + 2s;
  while (std::chrono::steady_clock::now() < deadline &&
         kpt::gui::AppTestAccess::inspectionJobsActive(app)) {
    std::this_thread::sleep_for(2ms);
  }
  kpt::gui::AppTestAccess::drainUi(app);
  const auto jobs = kpt::gui::AppTestAccess::jobs(app);
  REQUIRE(std::ranges::any_of(jobs, [](const kpt::gui::JobSnapshot &job) {
    return job.state == kpt::gui::JobState::Succeeded;
  }));
  REQUIRE(std::filesystem::exists(share));
  kpt::gui::InspectionShareDocument stored;
  REQUIRE(kpt::gui::InspectionShareFile(share).load(stored));
  REQUIRE(stored.layers.size() == 1);
  REQUIRE(stored.layers.front().source_key == source_key);
  REQUIRE(stored.layers.front().relative_source_path ==
          std::optional<std::filesystem::path>{"clouds/scan.xyz"});
  REQUIRE(kpt::gui::AppTestAccess::hasLogContaining(app,
                                                     "Saved review share"));
}

TEST_CASE("native review-share import hydrates relative sources and keeps misses",
          "[gui][inspection][share]") {
  TemporaryDirectory directory;
  const auto source = directory.path() / "reviews" / "clouds" / "scan.xyz";
  const auto missing = directory.path() / "reviews" / "clouds" / "missing.xyz";
  const auto share = directory.path() / "reviews" / "review.kpt-review.json";
  std::filesystem::create_directories(source.parent_path());
  {
    std::ofstream output(source);
    output << "1 2 3\n";
  }
  const auto source_key = kpt::gui::pathSourceKey(source, {});
  const auto missing_key = kpt::gui::pathSourceKey(missing, {});
  kpt::gui::LayerStyle style;
  style.color_by = kpt::ColorBy::RGB;
  style.point_size = 4.0F;
  Eigen::Affine3d transform = Eigen::Affine3d::Identity();
  transform.translation() = Eigen::Vector3d{10.0, 20.0, 30.0};
  kpt::gui::InspectionShareDocument document;
  document.layers.push_back(
      {source_key, std::filesystem::path{"clouds/scan.xyz"}, transform,
       style, false});
  document.layers.push_back(
      {missing_key, std::filesystem::path{"clouds/missing.xyz"},
       Eigen::Affine3d::Identity(), kpt::gui::LayerStyle{}, true});
  document.roi = kpt::gui::RoiBox({-1.0, -2.0, -3.0}, {4.0, 5.0, 6.0});
  document.measurements.push_back(
      {source_key, Eigen::Vector3d{1.0, 2.0, 3.0},
       std::optional<std::string>{missing_key},
       std::optional<Eigen::Vector3d>{Eigen::Vector3d{4.0, 5.0, 6.0}}});
  document.bookmarks.emplace_back("shared overview", reviewCamera());
  REQUIRE(kpt::gui::InspectionShareFile(share).save(document, true).status ==
          kpt::gui::InspectionShareSaveStatus::Written);

  kpt::gui::App app(std::make_unique<FakeRenderer>(),
                     std::make_unique<FakeRenderer>(), 1);
  kpt::gui::AppTestAccess::loadInspectionShare(app, share);
  const auto deadline = std::chrono::steady_clock::now() + 3s;
  while (std::chrono::steady_clock::now() < deadline) {
    kpt::gui::AppTestAccess::drainInspectionUi(app);
    const auto *loaded =
        kpt::gui::AppTestAccess::inspectionLayer(app, source_key);
    const auto *unresolved =
        kpt::gui::AppTestAccess::inspectionLayer(app, missing_key);
    if (loaded != nullptr && loaded->cloud() && unresolved != nullptr &&
        !kpt::gui::AppTestAccess::inspectionJobsActive(app)) {
      break;
    }
    std::this_thread::sleep_for(2ms);
  }
  kpt::gui::AppTestAccess::drainInspectionUi(app);

  const auto *loaded = kpt::gui::AppTestAccess::inspectionLayer(app, source_key);
  const auto *unresolved =
      kpt::gui::AppTestAccess::inspectionLayer(app, missing_key);
  REQUIRE(loaded != nullptr);
  REQUIRE(loaded->cloud() != nullptr);
  REQUIRE(loaded->cloud()->size() == 1);
  REQUIRE(loaded->localToWorld().isApprox(transform));
  REQUIRE_FALSE(loaded->visible());
  REQUIRE(unresolved != nullptr);
  REQUIRE_FALSE(unresolved->cloud());
  REQUIRE(kpt::gui::AppTestAccess::inspectionRoi(app).has_value());
  REQUIRE(kpt::gui::AppTestAccess::inspectionRoi(app)->contains(
      Eigen::Vector3d{4.0, 5.0, 6.0}));
  REQUIRE(kpt::gui::AppTestAccess::inspectionMeasurementCount(app) == 1);
  const auto *bookmark =
      kpt::gui::AppTestAccess::inspectionBookmark(app, "shared overview");
  REQUIRE(bookmark != nullptr);
  REQUIRE(bookmark->camera().target.isApprox(reviewCamera().target));
  REQUIRE(kpt::gui::AppTestAccess::hasLogContaining(app,
                                                     "Review layer unresolved"));
  REQUIRE_FALSE(kpt::gui::AppTestAccess::undoInspection(app));
  REQUIRE(loaded->cloud() != nullptr);
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
  REQUIRE(bounds.centroid.isApprox(Eigen::Vector3f(1.0F, 3.0F, 1.0F)));
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

TEST_CASE("scene compositor draws transformed visible review layers",
          "[gui][scene]") {
  auto make_cloud = [](float x, std::uint8_t red, std::uint8_t green,
                       std::uint8_t blue) {
    auto cloud = std::make_shared<kpt::PointCloudIRGB>();
    kpt::PointT point{};
    point.x = x;
    point.r = red;
    point.g = green;
    point.b = blue;
    cloud->push_back(point);
    return cloud;
  };

  const auto first_cloud = make_cloud(1.0F, 255, 0, 0);
  const auto second_cloud = make_cloud(2.0F, 0, 0, 255);
  kpt::gui::Scene scene;
  const auto first = scene.addLayer("first", first_cloud);
  const auto second = scene.addLayer("second", second_cloud);
  Eigen::Affine3d move = Eigen::Affine3d::Identity();
  move.translation() = Eigen::Vector3d{10.0, 0.0, 0.0};
  REQUIRE(scene.setLayerTransform(first, move));
  kpt::gui::LayerStyle first_style;
  first_style.color_by = kpt::ColorBy::None;
  first_style.fixed_color = {0.25F, 0.5F, 0.75F};
  REQUIRE(scene.setLayerStyle(first, first_style));
  REQUIRE(scene.setLayerVisible(second, false));

  kpt::gui::SceneRenderAdapter adapter;
  REQUIRE(adapter.acceptSnapshot(
      first, kpt::gui::makeViewportCloudSnapshot(first_cloud, 1)));
  REQUIRE(adapter.acceptSnapshot(
      second, kpt::gui::makeViewportCloudSnapshot(second_cloud, 1)));
  const auto list = adapter.build(scene);
  const auto composite = kpt::gui::composeSceneViewportSnapshot(list, 9);

  REQUIRE(composite->revision == 9);
  REQUIRE(composite->vertices.size() == 1);
  REQUIRE(composite->vertices.front().position.isApprox(
      Eigen::Vector3f{11.0F, 0.0F, 0.0F}));
  REQUIRE(composite->vertices.front().color.isApprox(
      Eigen::Vector3f{0.25F, 0.5F, 0.75F}));
  REQUIRE(composite->bounds.centroid.isApprox(
      Eigen::Vector3f{11.0F, 0.0F, 0.0F}));
}

TEST_CASE("inspection ROI controls rehydrate from undoable Scene state",
          "[gui][inspection]") {
  auto main_renderer = std::make_unique<FakeRenderer>();
  kpt::gui::App app(std::move(main_renderer), std::make_unique<FakeRenderer>(),
                     1);
  const kpt::gui::RoiBox roi{{1.0, 2.0, 3.0}, {4.0, 5.0, 6.0}};
  kpt::gui::AppTestAccess::setInspectionRoi(app, roi);
  kpt::gui::AppTestAccess::hydrateInspectionRoi(app);
  REQUIRE(kpt::gui::AppTestAccess::inspectionRoiControlsEnabled(app));
  REQUIRE(kpt::gui::AppTestAccess::inspectionRoiControlMinimum(app).isApprox(
      Eigen::Vector3d{1.0, 2.0, 3.0}));

  REQUIRE(kpt::gui::AppTestAccess::undoInspection(app));
  REQUIRE_FALSE(kpt::gui::AppTestAccess::inspectionRoiControlsEnabled(app));
}

TEST_CASE("inspection layer deletion prunes and undo rebuilds its snapshot",
          "[gui][inspection]") {
  auto main_renderer = std::make_unique<FakeRenderer>();
  kpt::gui::App app(std::move(main_renderer), std::make_unique<FakeRenderer>(),
                     1);
  auto cloud = std::make_shared<kpt::PointCloudIRGB>();
  kpt::PointT point{};
  point.x = 3.0F;
  cloud->push_back(point);
  const auto snapshot = kpt::gui::makeViewportCloudSnapshot(cloud, 1);
  const auto layer = kpt::gui::AppTestAccess::addInspectionLayer(
      app, "recoverable-layer", cloud, snapshot);
  REQUIRE(kpt::gui::AppTestAccess::hasInspectionSnapshot(app, layer));

  REQUIRE(kpt::gui::AppTestAccess::removeInspectionLayer(app, layer));
  REQUIRE_FALSE(kpt::gui::AppTestAccess::hasInspectionSnapshot(app, layer));
  REQUIRE(kpt::gui::AppTestAccess::undoInspection(app));

  const auto deadline = std::chrono::steady_clock::now() + 2s;
  while (std::chrono::steady_clock::now() < deadline) {
    kpt::gui::AppTestAccess::drainInspectionUi(app);
    if (!kpt::gui::AppTestAccess::inspectionJobsActive(app) &&
        kpt::gui::AppTestAccess::hasInspectionSnapshot(app, layer)) {
      break;
    }
    std::this_thread::sleep_for(2ms);
  }
  kpt::gui::AppTestAccess::drainInspectionUi(app);
  REQUIRE(kpt::gui::AppTestAccess::hasInspectionSnapshot(app, layer));
}

TEST_CASE("inspection snapshot rebuild rejects a stale COW layer binding",
          "[gui][inspection]") {
  kpt::gui::App app(std::make_unique<FakeRenderer>(),
                     std::make_unique<FakeRenderer>(), 1);
  auto original = std::make_shared<kpt::PointCloudIRGB>();
  kpt::PointT original_point{};
  original_point.x = 1.0F;
  original->push_back(original_point);
  const kpt::gui::LayerId layer =
      kpt::gui::AppTestAccess::addInspectionLayer(
          app, "snapshot-cow-layer", original,
          kpt::gui::makeViewportCloudSnapshot(original, 90));
  const auto original_hydration =
      kpt::gui::AppTestAccess::captureInspectionLayerHydration(app, layer);
  REQUIRE(original_hydration.has_value());

  auto replacement = std::make_shared<kpt::PointCloudIRGB>();
  kpt::PointT replacement_point{};
  replacement_point.x = 9.0F;
  replacement->push_back(replacement_point);
  REQUIRE(kpt::gui::AppTestAccess::replaceInspectionLayerCloud(
      app, layer, replacement));
  // A previously rendered snapshot is bound to the old cloud lease. COW must
  // hide it immediately; otherwise the stale completion below would see a
  // cache hit and never schedule the replacement cloud's rebuild.
  REQUIRE_FALSE(kpt::gui::AppTestAccess::hasInspectionSnapshot(app, layer));
  REQUIRE(kpt::gui::AppTestAccess::inspectionSnapshot(app, layer) == nullptr);

  // Simulate a worker built from the pre-replacement binding completing after
  // COW. It must be ignored, then schedule the live binding's snapshot.
  kpt::gui::AppTestAccess::completeInspectionSnapshotHydration(
      app, layer, "opaque:snapshot-cow-layer", *original_hydration,
      kpt::gui::makeViewportCloudSnapshot(original, 91));
  const auto deadline = std::chrono::steady_clock::now() + 2s;
  while (std::chrono::steady_clock::now() < deadline) {
    kpt::gui::AppTestAccess::drainInspectionUi(app);
    if (!kpt::gui::AppTestAccess::inspectionJobsActive(app) &&
        kpt::gui::AppTestAccess::hasInspectionSnapshot(app, layer)) {
      break;
    }
    std::this_thread::sleep_for(2ms);
  }
  kpt::gui::AppTestAccess::drainInspectionUi(app);
  const auto snapshot =
      kpt::gui::AppTestAccess::inspectionSnapshot(app, layer);
  REQUIRE(snapshot != nullptr);
  REQUIRE(snapshot->vertices.size() == 1);
  REQUIRE(snapshot->vertices.front().position.x() == Approx(replacement_point.x));
}

TEST_CASE("inspection COW replacement rebuilds snapshot without stale worker",
          "[gui][inspection]") {
  kpt::gui::App app(std::make_unique<FakeRenderer>(),
                     std::make_unique<FakeRenderer>(), 1);
  auto original = std::make_shared<kpt::PointCloudIRGB>();
  kpt::PointT original_point{};
  original_point.x = 1.0F;
  original->push_back(original_point);
  const kpt::gui::LayerId layer =
      kpt::gui::AppTestAccess::addInspectionLayer(
          app, "snapshot-cow-direct-rebuild", original,
          kpt::gui::makeViewportCloudSnapshot(original, 92));
  REQUIRE(kpt::gui::AppTestAccess::hasInspectionSnapshot(app, layer));

  auto replacement = std::make_shared<kpt::PointCloudIRGB>();
  kpt::PointT replacement_point{};
  replacement_point.x = 13.0F;
  replacement->push_back(replacement_point);
  REQUIRE(kpt::gui::AppTestAccess::replaceInspectionLayerCloud(
      app, layer, replacement));
  REQUIRE_FALSE(kpt::gui::AppTestAccess::hasInspectionSnapshot(app, layer));

  // No stale completion arrives in this path. A normal viewport refresh must
  // independently queue the replacement binding's snapshot.
  kpt::gui::AppTestAccess::refreshInspectionViewport(app);
  const auto deadline = std::chrono::steady_clock::now() + 2s;
  while (std::chrono::steady_clock::now() < deadline) {
    kpt::gui::AppTestAccess::drainInspectionUi(app);
    if (!kpt::gui::AppTestAccess::inspectionJobsActive(app) &&
        kpt::gui::AppTestAccess::hasInspectionSnapshot(app, layer)) {
      break;
    }
    std::this_thread::sleep_for(2ms);
  }
  kpt::gui::AppTestAccess::drainInspectionUi(app);
  const auto snapshot =
      kpt::gui::AppTestAccess::inspectionSnapshot(app, layer);
  REQUIRE(snapshot != nullptr);
  REQUIRE(snapshot->vertices.size() == 1);
  REQUIRE(snapshot->vertices.front().position.x() ==
          Approx(replacement_point.x));
}

TEST_CASE("late review hydration survives delete undo and restores export",
          "[gui][inspection][share]") {
  TemporaryDirectory directory;
  const auto source_key = kpt::gui::pathSourceKey(
      directory.path() / "clouds" / "late.xyz", {});
  const auto output = directory.path() / "restored.pcd";
  kpt::gui::App app(std::make_unique<FakeRenderer>(),
                     std::make_unique<FakeRenderer>(), 1);
  const kpt::gui::LayerId layer =
      kpt::gui::AppTestAccess::addUnresolvedInspectionLayer(app, source_key);
  const auto hydration =
      kpt::gui::AppTestAccess::captureInspectionLayerHydration(app, layer);
  REQUIRE(hydration.has_value());

  REQUIRE(kpt::gui::AppTestAccess::removeInspectionLayer(app, layer));
  auto cloud = std::make_shared<kpt::PointCloudIRGB>();
  kpt::PointT point{};
  // Keep this inside default export controls' [-1, 1] world-space ROI.
  point.x = 0.5F;
  point.y = -0.25F;
  point.z = 0.75F;
  cloud->push_back(point);
  kpt::gui::AppTestAccess::completeInspectionShareLayerLoad(
      app, layer, source_key, *hydration, cloud,
      kpt::gui::makeViewportCloudSnapshot(cloud, 41));
  REQUIRE_FALSE(kpt::gui::AppTestAccess::hasInspectionSnapshot(app, layer));

  REQUIRE(kpt::gui::AppTestAccess::undoInspection(app));
  const auto snapshot_deadline = std::chrono::steady_clock::now() + 2s;
  while (std::chrono::steady_clock::now() < snapshot_deadline) {
    kpt::gui::AppTestAccess::drainInspectionUi(app);
    if (!kpt::gui::AppTestAccess::inspectionJobsActive(app) &&
        kpt::gui::AppTestAccess::hasInspectionSnapshot(app, layer)) {
      break;
    }
    std::this_thread::sleep_for(2ms);
  }
  kpt::gui::AppTestAccess::drainInspectionUi(app);
  const auto *restored =
      kpt::gui::AppTestAccess::inspectionLayer(app, source_key);
  REQUIRE(restored != nullptr);
  REQUIRE(restored->cloud() == cloud);
  REQUIRE(kpt::gui::AppTestAccess::hasInspectionSnapshot(app, layer));

  kpt::gui::AppTestAccess::queueInspectionExport(app, output);
  const auto export_deadline = std::chrono::steady_clock::now() + 2s;
  while (std::chrono::steady_clock::now() < export_deadline &&
         kpt::gui::AppTestAccess::inspectionJobsActive(app)) {
    std::this_thread::sleep_for(2ms);
  }
  kpt::gui::AppTestAccess::drainInspectionUi(app);
  REQUIRE(std::filesystem::exists(output));
  const auto exported = kpt::load(output);
  REQUIRE(exported->size() == 1);
  REQUIRE(exported->points.front().x == Approx(point.x));
  REQUIRE(exported->points.front().y == Approx(point.y));
  REQUIRE(exported->points.front().z == Approx(point.z));
}

TEST_CASE("inspection upload allocation failure halves LOD then rejects minimum",
          "[gui][inspection]") {
  auto make_cloud = [](std::size_t count) {
    auto cloud = std::make_shared<kpt::PointCloudIRGB>();
    for (std::size_t index = 0; index < count; ++index) {
      kpt::PointT point{};
      point.x = static_cast<float>(index);
      cloud->push_back(point);
    }
    return cloud;
  };

  auto main_renderer = std::make_unique<FakeRenderer>();
  kpt::gui::App app(std::move(main_renderer), std::make_unique<FakeRenderer>(),
                     1);
  const auto cloud = make_cloud(8);
  const auto layer = kpt::gui::AppTestAccess::addInspectionLayer(
      app, "retry-layer", cloud, kpt::gui::makeViewportCloudSnapshot(cloud, 1));
  REQUIRE(layer != 0);
  REQUIRE(kpt::gui::AppTestAccess::retryInspectionUpload(app));
  REQUIRE(kpt::gui::AppTestAccess::inspectionGpuVertexCap(app) == 4);
  REQUIRE(kpt::gui::AppTestAccess::renderedInspectionVertices(app) == 4);

  auto second_main = std::make_unique<FakeRenderer>();
  kpt::gui::App minimum(std::move(second_main),
                         std::make_unique<FakeRenderer>(), 1);
  const auto single = make_cloud(1);
  kpt::gui::AppTestAccess::addInspectionLayer(
      minimum, "minimum-layer", single,
      kpt::gui::makeViewportCloudSnapshot(single, 1));
  REQUIRE(kpt::gui::AppTestAccess::retryInspectionUpload(minimum));
  REQUIRE(kpt::gui::AppTestAccess::inspectionLayerCount(minimum) == 0);
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
