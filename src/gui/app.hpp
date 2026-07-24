#pragma once

#include "gui/job_system.hpp"
#include "gui/point_renderer.hpp"
#include "kpt/workflow/workflow.hpp"

#include <chrono>
#include <deque>
#include <filesystem>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>

namespace kpt::gui {

class UiEvents {
public:
  void post(std::function<void()> event);
  void drain();

private:
  std::mutex mutex_;
  std::deque<std::function<void()>> events_;
};

class App {
public:
  enum class Tool { Viewer, Player, Convert, Batch, Render };

  App();
  ~App();
  App(const App &) = delete;
  App &operator=(const App &) = delete;

  void draw();
  bool runSmokeTest();

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
  void drawViewport();
  void drawTrajectory();
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
  void loadViewerFile(const std::string &path);
  void openSequence();
  void requestFrame(std::size_t index, bool apply, bool fit_camera = false);
  void updatePlayback();
  void queueSingleConversion();
  void queueBatchConversion();
  void queueRender(bool sequence);
  void queueSnapshotFrame(std::size_t index);

  UiEvents ui_;
  JobSystem jobs_;
  PointRenderer renderer_;
  PointRenderer trajectory_renderer_;

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
  std::unordered_map<std::size_t, PointCloudIRGBPtr> frame_cache_;
  std::unordered_set<std::size_t> pending_frames_;
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

  int color_by_ = 0;
  float point_size_ = 3.0F;
  float background_[3] = {0.0F, 0.0F, 0.0F};
  bool reset_dock_layout_ = false;
};

} // namespace kpt::gui
