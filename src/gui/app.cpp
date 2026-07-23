#include "gui/app.hpp"

#include "ImGuiFileDialog.h"
#include "imgui.h"
#include "imgui_internal.h"
#include "misc/cpp/imgui_stdlib.h"

#include "kpt/io/io.hpp"
#include "kpt/render/render.hpp"

#include <opencv2/imgcodecs.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <mutex>
#include <random>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace kpt::gui {
namespace {

constexpr std::array<Format, 7> kFormats = {
    Format::Bin,  Format::PCD,    Format::PLY,    Format::XYZ,
    Format::XYZI, Format::XYZRGB, Format::XYZRGBI};
constexpr std::array<Format, 4> kAsciiFormats = {
    Format::XYZ, Format::XYZI, Format::XYZRGB, Format::XYZRGBI};
constexpr std::array<View, 10> kViews = {
    View::Front,         View::Right,        View::Back,
    View::Left,          View::Top,          View::Bottom,
    View::TopRightFront, View::TopLeftFront, View::BotRightFront,
    View::BotLeftFront};
constexpr std::array<const char *, 10> kViewNames = {
    "front",  "right",         "back",         "left",          "top",
    "bottom", "toprightfront", "topleftfront", "botrightfront", "botleftfront"};

std::optional<Format> asciiFlavor(int selection) {
  if (selection <= 0)
    return std::nullopt;
  return kAsciiFormats[static_cast<std::size_t>(selection - 1)];
}

std::filesystem::path imageTemporaryPath(const std::filesystem::path &output) {
  static thread_local std::mt19937_64 generator(std::random_device{}());
  return output.parent_path() /
         (output.stem().string() + ".kpt-tmp-" + std::to_string(generator()) +
          output.extension().string());
}

bool writeImageAtomic(const std::filesystem::path &output, const cv::Mat &image,
                      bool overwrite) {
  static std::mutex commit_mutex;
  if (std::filesystem::exists(output) && !overwrite)
    return false;
  if (!output.parent_path().empty()) {
    std::filesystem::create_directories(output.parent_path());
  }
  const auto temporary = imageTemporaryPath(output);
  try {
    if (!cv::imwrite(temporary.string(), image)) {
      throw std::runtime_error("failed to write image: " + output.string());
    }
    {
      std::lock_guard commit_lock(commit_mutex);
      if (std::filesystem::exists(output) && !overwrite) {
        std::error_code ignored;
        std::filesystem::remove(temporary, ignored);
        return false;
      }
      std::filesystem::rename(temporary, output);
    }
  } catch (...) {
    std::error_code ignored;
    std::filesystem::remove(temporary, ignored);
    throw;
  }
  return true;
}

const char *toolName(App::Tool tool) {
  switch (tool) {
  case App::Tool::Viewer:
    return "Viewer";
  case App::Tool::Player:
    return "Player";
  case App::Tool::Convert:
    return "Convert";
  case App::Tool::Batch:
    return "Batch Convert";
  case App::Tool::Render:
    return "Render";
  }
  return "Unknown";
}

} // namespace

void UiEvents::post(std::function<void()> event) {
  std::lock_guard lock(mutex_);
  events_.push_back(std::move(event));
}

void UiEvents::drain() {
  std::deque<std::function<void()>> events;
  {
    std::lock_guard lock(mutex_);
    events.swap(events_);
  }
  for (auto &event : events)
    event();
}

App::App() { next_frame_time_ = std::chrono::steady_clock::now(); }

App::~App() {
  playing_ = false;
  jobs_.setPlayerActive(false);
  jobs_.cancelAll();
}

void App::draw() {
  ui_.drain();
  updatePlayback();
  drawDockspace();
  drawTools();
  drawInspector();
  drawViewport();
  drawTrajectory();
  drawJobsAndLog();
  drawFileDialog();
}

void App::drawDockspace() {
  const ImGuiViewport *viewport = ImGui::GetMainViewport();
  ImGui::SetNextWindowPos(viewport->WorkPos);
  ImGui::SetNextWindowSize(viewport->WorkSize);
  ImGui::SetNextWindowViewport(viewport->ID);
  constexpr ImGuiWindowFlags flags =
      ImGuiWindowFlags_NoDocking | ImGuiWindowFlags_NoTitleBar |
      ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoResize |
      ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoBringToFrontOnFocus |
      ImGuiWindowFlags_NoNavFocus | ImGuiWindowFlags_MenuBar;

  ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0F);
  ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0F);
  ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0.0F, 0.0F));
  ImGui::Begin("KPT Dockspace", nullptr, flags);
  ImGui::PopStyleVar(3);

  if (ImGui::BeginMenuBar()) {
    if (ImGui::BeginMenu("View")) {
      if (ImGui::MenuItem("Reset layout"))
        reset_dock_layout_ = true;
      ImGui::EndMenu();
    }
    ImGui::TextDisabled("KPT Dear ImGui Workbench");
    ImGui::EndMenuBar();
  }

  const ImGuiID dockspace_id = ImGui::GetID("KptMainDockspace");
  ImGui::DockSpace(dockspace_id, ImVec2(0.0F, 0.0F));
  if (reset_dock_layout_ ||
      ImGui::DockBuilderGetNode(dockspace_id) == nullptr) {
    reset_dock_layout_ = false;
    ImGui::DockBuilderRemoveNode(dockspace_id);
    ImGui::DockBuilderAddNode(dockspace_id, ImGuiDockNodeFlags_DockSpace);
    ImGui::DockBuilderSetNodeSize(dockspace_id, viewport->WorkSize);

    ImGuiID center = dockspace_id;
    const ImGuiID left = ImGui::DockBuilderSplitNode(center, ImGuiDir_Left,
                                                     0.20F, nullptr, &center);
    const ImGuiID right = ImGui::DockBuilderSplitNode(center, ImGuiDir_Right,
                                                      0.25F, nullptr, &center);
    const ImGuiID bottom = ImGui::DockBuilderSplitNode(center, ImGuiDir_Down,
                                                       0.24F, nullptr, &center);
    ImGui::DockBuilderDockWindow("Tools", left);
    ImGui::DockBuilderDockWindow("Inspector", right);
    ImGui::DockBuilderDockWindow("Jobs / Log", bottom);
    ImGui::DockBuilderDockWindow("3D Viewport", center);
    ImGui::DockBuilderFinish(dockspace_id);
  }
  ImGui::End();
}

void App::drawTools() {
  ImGui::Begin("Tools");
  constexpr std::array<Tool, 5> tools = {
      Tool::Viewer, Tool::Player, Tool::Convert, Tool::Batch, Tool::Render};
  for (const auto tool : tools) {
    if (ImGui::Selectable(toolName(tool), tool_ == tool))
      tool_ = tool;
  }
  ImGui::Separator();
  ImGui::TextWrapped("OpenGL viewport remains available while conversion and "
                     "render jobs run in background.");
  ImGui::End();
}

void App::drawInspector() {
  ImGui::Begin("Inspector");
  ImGui::TextUnformatted(toolName(tool_));
  ImGui::Separator();
  switch (tool_) {
  case Tool::Viewer:
    drawViewerControls();
    break;
  case Tool::Player:
    drawPlayerControls();
    break;
  case Tool::Convert:
    drawConvertControls();
    break;
  case Tool::Batch:
    drawBatchControls();
    break;
  case Tool::Render:
    drawRenderControls();
    break;
  }
  ImGui::Separator();
  drawDisplayControls();
  ImGui::End();
}

void App::drawViewerControls() {
  ImGui::InputText("Input", &viewer_input_);
  ImGui::SameLine();
  if (ImGui::Button("...##viewer")) {
    openDialog(DialogTarget::ViewerInput, "Open point cloud", false, false,
               viewer_input_);
  }
  if (ImGui::Button("Load") && !viewer_input_.empty()) {
    loadViewerFile(viewer_input_);
  }
}

void App::drawPlayerControls() {
  ImGui::InputText("Directory", &player_input_dir_);
  ImGui::SameLine();
  if (ImGui::Button("...##player-dir")) {
    openDialog(DialogTarget::PlayerInputDir, "Open sequence directory", true,
               false, player_input_dir_);
  }
  ImGui::InputText("Glob", &player_glob_);
  ImGui::InputText("Labels", &player_label_dir_);
  ImGui::SameLine();
  if (ImGui::Button("...##labels")) {
    openDialog(DialogTarget::PlayerLabelDir, "Open label directory", true,
               false, player_label_dir_);
  }
  ImGui::InputText("Poses", &player_poses_);
  ImGui::SameLine();
  if (ImGui::Button("...##poses")) {
    openDialog(DialogTarget::PlayerPoses, "Open poses", false, false,
               player_poses_);
  }
  ImGui::InputText("Poses 2", &player_poses2_);
  ImGui::SameLine();
  if (ImGui::Button("...##poses2")) {
    openDialog(DialogTarget::PlayerPoses2, "Open second poses", false, false,
               player_poses2_);
  }
  if (ImGui::Button("Open sequence") && !player_input_dir_.empty()) {
    openSequence();
  }

  if (!sequence_ || sequence_->empty())
    return;
  ImGui::Separator();
  if (ImGui::Button(playing_ ? "Pause" : "Play")) {
    playing_ = !playing_;
    jobs_.setPlayerActive(playing_);
    next_frame_time_ = std::chrono::steady_clock::now();
  }
  ImGui::SameLine();
  if (ImGui::Button("Previous") && current_frame_ > 0) {
    requestFrame(current_frame_ - 1, true);
  }
  ImGui::SameLine();
  if (ImGui::Button("Next") && current_frame_ + 1 < sequence_->size()) {
    requestFrame(current_frame_ + 1, true);
  }
  int frame = static_cast<int>(desired_frame_);
  const int maximum = static_cast<int>(sequence_->size() - 1);
  if (ImGui::SliderInt("Frame", &frame, 0, maximum)) {
    requestFrame(static_cast<std::size_t>(frame), true);
  }
  ImGui::SliderInt("FPS", &fps_, 1, 120);
  ImGui::Checkbox("Loop", &loop_);
  if (ImGui::CollapsingHeader("Snapshot export")) {
    ImGui::InputText("Prefix##player-snapshot", &player_snapshot_prefix_);
    ImGui::SameLine();
    if (ImGui::Button("...##player-snapshot")) {
      openDialog(DialogTarget::PlayerSnapshotPrefix, "Choose snapshot prefix",
                 false, true, player_snapshot_prefix_);
    }
    ImGui::InputInt("Width##player-snapshot", &render_width_);
    ImGui::InputInt("Height##player-snapshot", &render_height_);
    ImGui::InputFloat("FOV##player-snapshot", &render_fov_);
    ImGui::Checkbox("Overwrite##player-snapshot", &render_overwrite_);
    if (ImGui::Button("Export sequence snapshots") &&
        !player_snapshot_prefix_.empty()) {
      queueRender(true);
    }
  }
}

void App::drawConvertControls() {
  ImGui::InputText("Input", &convert_input_);
  ImGui::SameLine();
  if (ImGui::Button("...##convert-input")) {
    openDialog(DialogTarget::ConvertInput, "Open input", false, false,
               convert_input_);
  }
  ImGui::InputText("Output", &convert_output_);
  ImGui::SameLine();
  if (ImGui::Button("...##convert-output")) {
    openDialog(DialogTarget::ConvertOutput, "Save converted cloud", false, true,
               convert_output_);
  }
  constexpr const char *ascii_items =
      "From extension\0xyz\0xyzi\0xyzrgb\0xyzrgbi\0";
  ImGui::Combo("ASCII flavor", &convert_ascii_, ascii_items);
  ImGui::Checkbox("Overwrite existing", &convert_overwrite_);
  if (ImGui::Button("Queue conversion") && !convert_input_.empty() &&
      !convert_output_.empty()) {
    queueSingleConversion();
  }
}

void App::drawBatchControls() {
  ImGui::InputText("Input directory", &batch_input_dir_);
  ImGui::SameLine();
  if (ImGui::Button("...##batch-input")) {
    openDialog(DialogTarget::BatchInputDir, "Open input directory", true, false,
               batch_input_dir_);
  }
  ImGui::InputText("Output directory", &batch_output_dir_);
  ImGui::SameLine();
  if (ImGui::Button("...##batch-output")) {
    openDialog(DialogTarget::BatchOutputDir, "Open output directory", true,
               false, batch_output_dir_);
  }
  ImGui::InputText("Glob", &batch_glob_);
  constexpr const char *formats = "bin\0pcd\0ply\0xyz\0xyzi\0xyzrgb\0xyzrgbi\0";
  ImGui::Combo("Output format", &batch_format_, formats);
  constexpr const char *ascii_items =
      "From output format\0xyz\0xyzi\0xyzrgb\0xyzrgbi\0";
  ImGui::Combo("ASCII flavor", &batch_ascii_, ascii_items);
  ImGui::Checkbox("Overwrite existing", &batch_overwrite_);
  if (ImGui::Button("Queue batch") && !batch_input_dir_.empty() &&
      !batch_output_dir_.empty()) {
    queueBatchConversion();
  }
}

void App::drawRenderControls() {
  ImGui::InputText("Input", &render_input_);
  ImGui::SameLine();
  if (ImGui::Button("...##render-input")) {
    openDialog(DialogTarget::RenderInput, "Open point cloud", false, false,
               render_input_);
  }
  ImGui::InputText("Output prefix", &render_output_prefix_);
  ImGui::SameLine();
  if (ImGui::Button("...##render-prefix")) {
    openDialog(DialogTarget::RenderOutputPrefix, "Choose output prefix", false,
               true, render_output_prefix_);
  }
  ImGui::InputInt("Width", &render_width_);
  ImGui::InputInt("Height", &render_height_);
  ImGui::InputFloat("FOV", &render_fov_);
  ImGui::Checkbox("Overwrite existing", &render_overwrite_);
  for (std::size_t index = 0; index < kViews.size(); ++index) {
    ImGui::Checkbox(kViewNames[index], &render_views_[index]);
    if (index % 2 == 0)
      ImGui::SameLine();
  }
  if (ImGui::Button("Queue render") && !render_input_.empty() &&
      !render_output_prefix_.empty()) {
    queueRender(false);
  }
}

void App::drawDisplayControls() {
  constexpr const char *color_modes = "Intensity\0RGB\0Z\0Label\0None\0";
  if (ImGui::Combo("Color by", &color_by_, color_modes)) {
    renderer_.setColorBy(static_cast<ColorBy>(color_by_));
  }
  if (ImGui::SliderFloat("Point size", &point_size_, 1.0F, 20.0F)) {
    renderer_.setPointSize(point_size_);
  }
  if (ImGui::ColorEdit3("Background", background_)) {
    renderer_.setBackground(
        Eigen::Vector3f(background_[0], background_[1], background_[2]));
  }
  if (ImGui::Button("Fit cloud"))
    renderer_.fit();
  constexpr std::array<const char *, 10> labels = {
      "Front",  "Right", "Back", "Left", "Top",
      "Bottom", "TRF",   "TLF",  "BRF",  "BLF"};
  for (std::size_t index = 0; index < labels.size(); ++index) {
    if (ImGui::SmallButton(labels[index]))
      renderer_.setView(kViews[index]);
    if (index % 3 != 2)
      ImGui::SameLine();
  }
}

void App::drawViewport() {
  ImGui::Begin("3D Viewport");
  const ImVec2 available = ImGui::GetContentRegionAvail();
  const ImVec2 scale = ImGui::GetIO().DisplayFramebufferScale;
  renderer_.resize(static_cast<int>(available.x * scale.x),
                   static_cast<int>(available.y * scale.y));
  renderer_.render();
  ImGui::Image(static_cast<ImTextureID>(renderer_.texture()), available,
               ImVec2(0.0F, 1.0F), ImVec2(1.0F, 0.0F));
  if (ImGui::IsItemHovered()) {
    const ImGuiIO &io = ImGui::GetIO();
    if (ImGui::IsMouseDragging(ImGuiMouseButton_Left)) {
      renderer_.orbit(io.MouseDelta.x, io.MouseDelta.y);
    }
    if (ImGui::IsMouseDragging(ImGuiMouseButton_Middle) ||
        ImGui::IsMouseDragging(ImGuiMouseButton_Right)) {
      renderer_.pan(io.MouseDelta.x, -io.MouseDelta.y);
    }
    if (io.MouseWheel != 0.0F)
      renderer_.zoom(io.MouseWheel);
  }
  ImGui::SetItemTooltip("Left: orbit | Middle/Right: pan | Wheel: zoom");
  ImGui::End();
}

void App::drawTrajectory() {
  if (trajectory_renderer_.pointCount() == 0)
    return;
  ImGui::Begin("Trajectory");
  const ImVec2 available = ImGui::GetContentRegionAvail();
  const ImVec2 scale = ImGui::GetIO().DisplayFramebufferScale;
  trajectory_renderer_.resize(static_cast<int>(available.x * scale.x),
                              static_cast<int>(available.y * scale.y));
  trajectory_renderer_.render();
  ImGui::Image(static_cast<ImTextureID>(trajectory_renderer_.texture()),
               available, ImVec2(0.0F, 1.0F), ImVec2(1.0F, 0.0F));
  ImGui::End();
}

void App::drawJobsAndLog() {
  ImGui::Begin("Jobs / Log");
  unsigned worker_limit = jobs_.workerLimit();
  const unsigned minimum_workers = 1;
  const unsigned maximum_workers = jobs_.maxWorkers();
  if (ImGui::SliderScalar("Workers", ImGuiDataType_U32, &worker_limit,
                          &minimum_workers, &maximum_workers)) {
    jobs_.setWorkerLimit(worker_limit);
  }
  ImGui::SameLine();
  if (ImGui::Button("Cancel all"))
    jobs_.cancelAll();
  ImGui::SameLine();
  if (ImGui::Button("Clear finished"))
    jobs_.clearFinished();

  if (ImGui::BeginTable("jobs", 5,
                        ImGuiTableFlags_RowBg | ImGuiTableFlags_Borders |
                            ImGuiTableFlags_SizingStretchProp)) {
    ImGui::TableSetupColumn("Job");
    ImGui::TableSetupColumn("State");
    ImGui::TableSetupColumn("Progress");
    ImGui::TableSetupColumn("Message");
    ImGui::TableSetupColumn("Action");
    ImGui::TableHeadersRow();
    for (const auto &job : jobs_.snapshots()) {
      ImGui::TableNextRow();
      ImGui::TableNextColumn();
      ImGui::TextUnformatted(job.name.c_str());
      ImGui::TableNextColumn();
      ImGui::TextUnformatted(jobStateName(job.state));
      ImGui::TableNextColumn();
      ImGui::ProgressBar(job.progress, ImVec2(-1.0F, 0.0F));
      ImGui::TableNextColumn();
      ImGui::TextWrapped("%s", job.message.c_str());
      ImGui::TableNextColumn();
      if ((job.state == JobState::Queued || job.state == JobState::Running) &&
          ImGui::SmallButton(("Cancel##" + std::to_string(job.id)).c_str())) {
        jobs_.cancel(job.id);
      }
    }
    ImGui::EndTable();
  }
  ImGui::SeparatorText("Log");
  for (const auto &message : logs_)
    ImGui::TextWrapped("%s", message.c_str());
  ImGui::End();
}

void App::openDialog(DialogTarget target, const char *title, bool directory,
                     bool save, const std::string &current) {
  dialog_target_ = target;
  dialog_directory_ = directory;
  IGFD::FileDialogConfig config;
  const std::filesystem::path path(current);
  if (!current.empty()) {
    config.path = directory ? current : path.parent_path().string();
    if (save)
      config.fileName = path.filename().string();
  } else {
    config.path = ".";
  }
  config.flags =
      save ? ImGuiFileDialogFlags_ConfirmOverwrite : ImGuiFileDialogFlags_None;
  const char *filters =
      directory ? nullptr
                : ".bin,.pcd,.ply,.xyz,.xyzi,.xyzrgb,.xyzrgbi,.txt,.csv";
  ImGuiFileDialog::Instance()->OpenDialog("KptPathDialog", title, filters,
                                          config);
}

void App::drawFileDialog() {
  if (dialog_target_ == DialogTarget::None)
    return;
  if (ImGuiFileDialog::Instance()->Display("KptPathDialog")) {
    if (ImGuiFileDialog::Instance()->IsOk()) {
      applyDialogResult(dialog_directory_
                            ? ImGuiFileDialog::Instance()->GetCurrentPath()
                            : ImGuiFileDialog::Instance()->GetFilePathName());
    }
    ImGuiFileDialog::Instance()->Close();
    dialog_target_ = DialogTarget::None;
  }
}

void App::applyDialogResult(const std::string &value) {
  switch (dialog_target_) {
  case DialogTarget::ViewerInput:
    viewer_input_ = value;
    break;
  case DialogTarget::PlayerInputDir:
    player_input_dir_ = value;
    break;
  case DialogTarget::PlayerLabelDir:
    player_label_dir_ = value;
    break;
  case DialogTarget::PlayerPoses:
    player_poses_ = value;
    break;
  case DialogTarget::PlayerPoses2:
    player_poses2_ = value;
    break;
  case DialogTarget::PlayerSnapshotPrefix:
    player_snapshot_prefix_ = value;
    break;
  case DialogTarget::ConvertInput:
    convert_input_ = value;
    break;
  case DialogTarget::ConvertOutput:
    convert_output_ = value;
    break;
  case DialogTarget::BatchInputDir:
    batch_input_dir_ = value;
    break;
  case DialogTarget::BatchOutputDir:
    batch_output_dir_ = value;
    break;
  case DialogTarget::RenderInput:
    render_input_ = value;
    break;
  case DialogTarget::RenderOutputPrefix:
    render_output_prefix_ = value;
    break;
  case DialogTarget::None:
    break;
  }
}

void App::log(std::string message) {
  logs_.push_front(std::move(message));
  while (logs_.size() > 200)
    logs_.pop_back();
}

void App::loadViewerFile(const std::string &path) {
  jobs_.submit(
      "Load " + std::filesystem::path(path).filename().string(),
      JobPriority::High,
      [this, path](std::stop_token stop, const JobSystem::Reporter &report) {
        report(0.1F, "loading");
        const auto cloud = kpt::load(path);
        if (stop.stop_requested())
          return;
        ui_.post([this, cloud, path] {
          renderer_.setCloud(cloud);
          log("Loaded " + path + " (" + std::to_string(cloud->size()) +
              " points)");
        });
        report(1.0F, "loaded " + std::to_string(cloud->size()) + " points");
      });
}

void App::openSequence() {
  workflow::SequenceOptions options;
  options.input_dir = player_input_dir_;
  options.glob = player_glob_;
  if (!player_label_dir_.empty())
    options.label_dir = player_label_dir_;
  if (!player_poses_.empty())
    options.poses = player_poses_;
  if (!player_poses2_.empty())
    options.poses2 = player_poses2_;

  jobs_.submit(
      "Open sequence", JobPriority::High,
      [this, options = std::move(options)](std::stop_token stop,
                                           const JobSystem::Reporter &report) {
        report(0.1F, "enumerating");
        auto sequence = std::make_shared<workflow::SequenceSource>(options);
        PointCloudIRGBPtr trajectory = std::make_shared<PointCloudIRGB>();
        if (!stop.stop_requested())
          trajectory = sequence->trajectory();
        if (stop.stop_requested())
          return;
        ui_.post([this, sequence, trajectory] {
          sequence_ = sequence;
          frame_cache_.clear();
          pending_frames_.clear();
          current_frame_ = 0;
          desired_frame_ = 0;
          trajectory_renderer_.setCloud(trajectory);
          trajectory_renderer_.setColorBy(ColorBy::RGB);
          log("Opened sequence with " + std::to_string(sequence->size()) +
              " frames");
          if (!sequence->empty())
            requestFrame(0, true);
        });
        report(1.0F, "ready");
      });
}

void App::requestFrame(std::size_t index, bool apply) {
  if (!sequence_ || index >= sequence_->size())
    return;
  if (apply)
    desired_frame_ = index;
  if (const auto found = frame_cache_.find(index);
      found != frame_cache_.end()) {
    if (apply) {
      current_frame_ = index;
      renderer_.setCloud(found->second);
    }
    return;
  }
  if (!pending_frames_.insert(index).second)
    return;
  const auto sequence = sequence_;
  jobs_.submit("Load frame " + std::to_string(index), JobPriority::High,
               [this, sequence, index, apply](
                   std::stop_token stop, const JobSystem::Reporter &report) {
                 try {
                   report(0.1F, "loading");
                   auto frame = sequence->load(index);
                   if (stop.stop_requested()) {
                     ui_.post([this, index] { pending_frames_.erase(index); });
                     return;
                   }
                   ui_.post([this, index, apply, cloud = frame.cloud] {
                     pending_frames_.erase(index);
                     frame_cache_[index] = cloud;
                     while (frame_cache_.size() > 3) {
                       auto victim = frame_cache_.begin();
                       if (victim->first == current_frame_)
                         ++victim;
                       if (victim != frame_cache_.end()) {
                         frame_cache_.erase(victim);
                       } else {
                         break;
                       }
                     }
                     if (apply && desired_frame_ == index) {
                       current_frame_ = index;
                       renderer_.setCloud(cloud);
                       if (sequence_ && index + 1 < sequence_->size()) {
                         requestFrame(index + 1, false);
                       }
                     }
                   });
                   report(1.0F, "loaded");
                 } catch (...) {
                   ui_.post([this, index, apply] {
                     pending_frames_.erase(index);
                     if (apply && desired_frame_ == index) {
                       desired_frame_ = current_frame_;
                       playing_ = false;
                     }
                   });
                   throw;
                 }
               });
}

void App::updatePlayback() {
  jobs_.setPlayerActive(playing_);
  if (!playing_ || !sequence_ || sequence_->empty())
    return;
  if (desired_frame_ != current_frame_)
    return;
  const auto now = std::chrono::steady_clock::now();
  if (now < next_frame_time_)
    return;
  next_frame_time_ = now + std::chrono::milliseconds(1000 / std::max(1, fps_));

  std::size_t next = desired_frame_ + 1;
  if (next >= sequence_->size()) {
    if (loop_) {
      next = 0;
    } else {
      playing_ = false;
      jobs_.setPlayerActive(false);
      return;
    }
  }
  requestFrame(next, true);
}

void App::queueSingleConversion() {
  workflow::ConversionRequest request;
  request.input = convert_input_;
  request.output = convert_output_;
  request.ascii_flavor = asciiFlavor(convert_ascii_);
  request.overwrite = convert_overwrite_;
  jobs_.submit(
      "Convert " + request.input.filename().string(), JobPriority::Normal,
      [this, request](std::stop_token stop, const JobSystem::Reporter &report) {
        report(0.1F, "converting");
        if (stop.stop_requested())
          return;
        const auto result = workflow::convert(request);
        ui_.post([this, result] {
          log(result.input.string() + " -> " + result.output.string() + ": " +
              result.message);
        });
        if (result.status == workflow::OperationStatus::Failed) {
          throw std::runtime_error(result.message);
        }
        report(1.0F, result.message);
      });
}

void App::queueBatchConversion() {
  workflow::BatchConvertOptions options;
  options.input_dir = batch_input_dir_;
  options.output_dir = batch_output_dir_;
  options.glob = batch_glob_;
  options.output_format = kFormats[static_cast<std::size_t>(batch_format_)];
  options.ascii_flavor = asciiFlavor(batch_ascii_);
  options.overwrite = batch_overwrite_;

  try {
    const auto plan = workflow::makeBatchPlan(options);
    for (const auto &rejected : plan.rejected) {
      log(rejected.input.string() + ": " + rejected.message);
    }
    for (const auto &request : plan.requests) {
      jobs_.submit(
          "Batch " + request.input.filename().string(), JobPriority::Low,
          [this, request](std::stop_token stop,
                          const JobSystem::Reporter &report) {
            if (stop.stop_requested())
              return;
            report(0.1F, "converting");
            const auto result = workflow::convert(request);
            ui_.post([this, result] {
              log(result.input.filename().string() + ": " + result.message);
            });
            if (result.status == workflow::OperationStatus::Failed) {
              throw std::runtime_error(result.message);
            }
            report(1.0F, result.message);
          });
    }
    log("Queued " + std::to_string(plan.requests.size()) +
        " batch conversion jobs");
  } catch (const std::exception &error) {
    log(error.what());
  }
}

void App::queueRender(bool sequence) {
  if (sequence) {
    if (!sequence_)
      return;
    for (std::size_t index = 0; index < sequence_->size(); ++index) {
      queueSnapshotFrame(index);
    }
    log("Queued snapshots for " + std::to_string(sequence_->size()) +
        " frames");
    return;
  }

  const std::filesystem::path input = render_input_;
  const std::string prefix = render_output_prefix_;
  const int width = std::max(1, render_width_);
  const int height = std::max(1, render_height_);
  const float fov = render_fov_;
  const bool overwrite = render_overwrite_;
  for (std::size_t index = 0; index < kViews.size(); ++index) {
    if (!render_views_[index])
      continue;
    const View view = kViews[index];
    const std::string view_name = kViewNames[index];
    jobs_.submit(
        "Render " + view_name, JobPriority::Low,
        [this, input, prefix, view, view_name, width, height, fov,
         overwrite](std::stop_token stop, const JobSystem::Reporter &report) {
          if (stop.stop_requested())
            return;
          report(0.1F, "loading");
          const auto cloud = kpt::load(input);
          if (stop.stop_requested())
            return;
          RenderOpts options;
          options.width = width;
          options.height = height;
          options.fov = fov;
          options.views = {view};
          report(0.5F, "rendering");
          const auto results = kpt::renderMultiView(cloud, options);
          if (stop.stop_requested())
            return;
          const std::filesystem::path output =
              prefix + "_" + view_name + ".png";
          const bool written =
              writeImageAtomic(output, results.front().image, overwrite);
          ui_.post([this, output, written] {
            log(std::string(written ? "Wrote " : "Skipped existing ") +
                output.string());
          });
          report(1.0F, written ? "written" : "skipped existing");
        });
  }
}

void App::queueSnapshotFrame(std::size_t index) {
  const auto sequence = sequence_;
  const std::string prefix = player_snapshot_prefix_;
  const int width = std::max(1, render_width_);
  const int height = std::max(1, render_height_);
  const float fov = render_fov_;
  const bool overwrite = render_overwrite_;
  std::vector<View> views;
  for (std::size_t view_index = 0; view_index < kViews.size(); ++view_index) {
    if (render_views_[view_index])
      views.push_back(kViews[view_index]);
  }
  if (!sequence || prefix.empty()) {
    log("Set Render output prefix before exporting sequence snapshots");
    return;
  }
  jobs_.submit(
      "Snapshot frame " + std::to_string(index), JobPriority::Low,
      [this, sequence, prefix, index, width, height, fov, overwrite,
       views = std::move(views)](std::stop_token stop,
                                 const JobSystem::Reporter &report) {
        auto frame = sequence->load(index);
        if (stop.stop_requested())
          return;
        RenderOpts options;
        options.width = width;
        options.height = height;
        options.fov = fov;
        options.views = views;
        for (std::size_t result_index = 0; result_index < views.size();
             ++result_index) {
          if (stop.stop_requested())
            return;
          options.views = {views[result_index]};
          const auto results = kpt::renderMultiView(frame.cloud, options);
          const auto &result = results.front();
          const auto output =
              std::filesystem::path(prefix + "_" + frame.path.stem().string() +
                                    "_" + result.view_name + ".png");
          static_cast<void>(writeImageAtomic(output, result.image, overwrite));
          report(static_cast<float>(result_index + 1) /
                     static_cast<float>(std::max<std::size_t>(1, views.size())),
                 result.view_name);
        }
      });
}

bool App::runSmokeTest() {
  auto cloud = std::make_shared<PointCloudIRGB>();
  PointT center{};
  center.x = 0.0F;
  center.y = 0.0F;
  center.z = 0.0F;
  center.r = 255;
  center.g = 64;
  center.b = 32;
  center.intensity = 1.0F;
  cloud->push_back(center);
  renderer_.resize(128, 128);
  renderer_.setPointSize(20.0F);
  renderer_.setColorBy(ColorBy::RGB);
  renderer_.setCloud(cloud);
  renderer_.render();
  return renderer_.pointCount() == 1 && renderer_.centerPixelVisible();
}

} // namespace kpt::gui
