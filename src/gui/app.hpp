#pragma once

#include "gui/jobs/job_system.hpp"
#include "gui/jobs/ui_events.hpp"
#include "gui/viewport/session.hpp"
#include "gui/web/asset_stager.hpp"
#include "kpt/types.hpp"
#include "kpt/workflow/workflow.hpp"

#include <chrono>
#include <cstdint>
#include <deque>
#include <filesystem>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace kpt::gui {

class AppTestAccess;

class App {
public:
  enum class Tool { Viewer, Player, Convert, Batch, Render };

  App(std::unique_ptr<ViewportRenderer> main_renderer,
      std::unique_ptr<ViewportRenderer> trajectory_renderer,
      unsigned max_workers = 0,
      std::shared_ptr<web::AssetStager> asset_stager = {});
  ~App();
  App(const App &) = delete;
  App &operator=(const App &) = delete;

  Result<void, AppError> draw(FrameContext &frame_context,
                              FramebufferMetrics metrics);
  [[nodiscard]] bool needsContinuousRedraw() const;
  void installSyntheticSmokeSnapshot();
  void setStartupStyle(const ViewportStyle &style);
  void startViewer(const std::filesystem::path &path);
  void startSequence(workflow::SequenceOptions options, int fps = 10,
                     bool autoplay = false);
  void startSequence(std::shared_ptr<workflow::SequenceSource> sequence,
                     int fps = 10, bool autoplay = false);
  [[nodiscard]] const std::optional<std::string> &launchError() const {
    return launch_error_;
  }
  [[nodiscard]] bool launchCompletedEmpty() const {
    return launch_state_ == LaunchState::Empty;
  }
  std::vector<std::string> takeLaunchWarnings();

private:
  friend class AppTestAccess;

  enum class DialogTarget {
    None,
    ViewerInput,
    PlayerInputDir,
    PlayerLabelDir,
    PlayerPoses,
    PlayerPoses2,
    PlayerSnapshotPrefix,
    ConvertInput,
    ConvertOutput,
    BatchInputDir,
    BatchOutputDir,
    RenderInput,
    RenderOutputPrefix
  };

  enum class LaunchState { None, Pending, Ready, Empty, Failed };
  enum class PlaybackDirection { Forward, Reverse };

  void drawDockspace();
  void drawTools();
  void drawInspector();
  Result<void, AppError> drawViewport(FrameContext &frame_context,
                                      FramebufferMetrics metrics);
  Result<void, AppError> drawTrajectory(FrameContext &frame_context,
                                        FramebufferMetrics metrics);
  void drawJobsAndLog();
  void drawFileDialog();
  void drawAboutPopup();
  void drawViewerControls();
  void drawPlayerControls();
  void drawConvertControls();
  void drawBatchControls();
  void drawRenderControls();
  void drawDisplayControls();

  void openDialog(DialogTarget target, const char *title, bool directory,
                  bool save, const std::string &current);
  void applyDialogResult(const std::string &value);
  void log(std::string message);
  std::optional<std::filesystem::path> decodeUiPath(std::string_view value,
                                                    std::string_view purpose);
  std::string displayPath(const std::filesystem::path &value);
  void loadViewerFile(const std::string &path);
  void loadViewerFile(const std::filesystem::path &path);
  void openSequence();
  void openSequence(workflow::SequenceOptions options);
  void openSequence(std::shared_ptr<workflow::SequenceSource> sequence);
  void queueSequence(
      std::function<std::shared_ptr<workflow::SequenceSource>()> create);
  [[nodiscard]] std::uint64_t beginNewSource();
  void requestFrame(std::size_t index, bool apply, bool fit_camera = false);
  void queueCachedFrame(std::size_t index, PointCloudIRGBConstPtr cloud,
                        bool fit_camera, std::uint64_t request_generation,
                        std::uint64_t sequence_generation);
  void queueFrameLoad(std::size_t index, bool apply, bool fit_camera,
                      std::uint64_t request_generation,
                      std::uint64_t sequence_generation,
                      std::vector<std::filesystem::path> staged_assets);
  void togglePlayback(PlaybackDirection direction);
  void resetPlayback();
  [[nodiscard]] static std::optional<std::size_t>
  nextPlaybackFrame(std::size_t current, std::size_t frame_count,
                    PlaybackDirection direction, bool loop);
  void updatePlayback();
  void queueSingleConversion();
  void queueBatchConversion();
  void queueRender(bool sequence);
  void queueSnapshotFrame(std::size_t index);

  // Destruction is reverse declaration order: jobs join first, then GPU
  // sessions, then the UI event queue captured by workers.
  UiEvents ui_;
  ViewportSession main_viewport_;
  ViewportSession trajectory_viewport_;
  JobSystem jobs_;
  std::shared_ptr<web::AssetStager> asset_stager_;

  Tool tool_ = Tool::Viewer;
  DialogTarget dialog_target_ = DialogTarget::None;
  bool dialog_directory_ = false;
  std::deque<std::string> logs_;
  std::vector<std::string> launch_warnings_;

  std::string viewer_input_;
  std::string player_input_dir_;
  std::string player_glob_ = "*";
  std::string player_label_dir_;
  std::string player_poses_;
  std::string player_poses2_;
  std::string player_snapshot_prefix_;
  std::shared_ptr<workflow::SequenceSource> sequence_;
  std::unordered_map<std::size_t, PointCloudIRGBConstPtr> frame_cache_;
  std::unordered_set<std::size_t> pending_frames_;
  double interaction_quality_until_ = 0.0;
  std::uint64_t sequence_generation_ = 0;
  std::size_t current_frame_ = 0;
  std::size_t desired_frame_ = 0;
  bool playing_ = false;
  PlaybackDirection playback_direction_ = PlaybackDirection::Forward;
  bool autoplay_when_sequence_ready_ = false;
  LaunchState launch_state_ = LaunchState::None;
  std::optional<std::string> launch_error_;
  bool loop_ = false;
  int fps_ = 10;
  std::chrono::steady_clock::time_point next_frame_time_;

  std::string convert_input_;
  std::string convert_output_;
  int convert_ascii_ = 0;
  bool convert_overwrite_ = false;

  std::string batch_input_dir_;
  std::string batch_output_dir_;
  std::string batch_glob_ = "*";
  int batch_format_ = 1;
  int batch_ascii_ = 0;
  bool batch_overwrite_ = false;

  std::string render_input_;
  std::string render_output_prefix_;
  int render_width_ = 640;
  int render_height_ = 480;
  float render_fov_ = 120.0F;
  int render_projection_ = 0;
  float render_trim_percent_ = 1.0F;
  int render_color_mode_ = 0;
  bool render_overwrite_ = false;
  bool render_views_[10] = {true, true, true, true, true,
                            true, true, true, true, true};

  ViewportStyle main_style_;
  int color_by_ = 0;
  int color_map_ = 0;
  bool equalize_ = true;
  float point_size_ = 3.0F;
  float background_[3] = {0.0F, 0.0F, 0.0F};
  bool show_viewport_controls_ = true;
  bool show_about_ = false;
  bool reset_dock_layout_ = false;
  std::optional<bool> compact_dock_layout_;
  std::optional<PixelExtent> viewport_extent_override_for_tests_;
};

} // namespace kpt::gui
