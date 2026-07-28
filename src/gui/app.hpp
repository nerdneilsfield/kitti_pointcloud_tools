#pragma once

#include "common/result.hpp"
#include "gui/jobs/job_system.hpp"
#include "gui/jobs/ui_events.hpp"
#include "gui/viewport/model.hpp"
#include "gui/viewport/renderer.hpp"
#include "kpt/types.hpp"
#include "kpt/workflow/workflow.hpp"

#include <chrono>
#include <cstdint>
#include <deque>
#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>

namespace kpt::gui {

enum class ViewportRole { Main, Trajectory };
enum class AppStage { Upload, Resize, Render };

struct AppError {
  ViewportRole role = ViewportRole::Main;
  AppStage stage = AppStage::Render;
  RendererError cause;
};

struct ViewportSession {
  explicit ViewportSession(std::unique_ptr<ViewportRenderer> renderer);

  ViewportModel model;
  std::unique_ptr<ViewportRenderer> renderer;
  std::uint64_t uploaded_revision = 0;
  std::uint64_t latest_requested_revision = 0;

  [[nodiscard]] std::uint64_t beginRequest();
  [[nodiscard]] bool
  accept(std::shared_ptr<const ViewportCloudSnapshot> snapshot,
         CameraUpdate camera_update = CameraUpdate::Fit);
  Result<std::optional<ViewportTexture>, AppError>
  draw(PixelExtent physical_extent, FrameContext &frame_context,
       ViewportRole role);
};

class App {
public:
  enum class Tool { Viewer, Player, Convert, Batch, Render };

  App(std::unique_ptr<ViewportRenderer> main_renderer,
      std::unique_ptr<ViewportRenderer> trajectory_renderer);
  ~App();
  App(const App &) = delete;
  App &operator=(const App &) = delete;

  Result<void, AppError> draw(FrameContext &frame_context);
  void installSyntheticSmokeSnapshot();

private:
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

  void drawDockspace();
  void drawTools();
  void drawInspector();
  Result<void, AppError> drawViewport(FrameContext &frame_context);
  Result<void, AppError> drawTrajectory(FrameContext &frame_context);
  void drawJobsAndLog();
  void drawFileDialog();
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
  void openSequence();
  void requestFrame(std::size_t index, bool apply, bool fit_camera = false);
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

  Tool tool_ = Tool::Viewer;
  DialogTarget dialog_target_ = DialogTarget::None;
  bool dialog_directory_ = false;
  std::deque<std::string> logs_;

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
  std::uint64_t sequence_generation_ = 0;
  std::size_t current_frame_ = 0;
  std::size_t desired_frame_ = 0;
  bool playing_ = false;
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
  bool render_overwrite_ = false;
  bool render_views_[10] = {true, true, true, true, true,
                            true, true, true, true, true};

  ViewportStyle main_style_;
  int color_by_ = 0;
  float point_size_ = 3.0F;
  float background_[3] = {0.0F, 0.0F, 0.0F};
  bool reset_dock_layout_ = false;
};

} // namespace kpt::gui
