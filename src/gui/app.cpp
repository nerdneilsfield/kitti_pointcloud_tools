#define IMGUI_DEFINE_MATH_OPERATORS

#ifndef KPT_VERSION_STRING
#define KPT_VERSION_STRING "dev"
#endif
#ifndef KPT_GIT_HASH
#define KPT_GIT_HASH "unknown"
#endif
#ifndef KPT_BUILD_TIME
#define KPT_BUILD_TIME "unknown"
#endif

#include "gui/app.hpp"
#ifndef KPT_WEB_BUILD
#include "gui/dialog_paths.hpp"
#endif
#include "gui/viewport/cloud_adapter.hpp"
#include "i18n/i18n.hpp"

#ifndef KPT_WEB_BUILD
#include "ImGuiFileDialog.h"
#else
#include "gui/web/bridge.hpp"
#endif
#include "imgui.h"
#include "imgui_internal.h"
#include "misc/cpp/imgui_stdlib.h"

#include "kpt/io/conversion_options.hpp"
#include "kpt/io/io.hpp"
#include "kpt/render/render.hpp"
#include "platform/utf8_path.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <iomanip>
#include <spdlog/spdlog.h>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace kpt::gui {
namespace {

constexpr std::array<Format, 11> kFormats = {
    Format::Bin,  Format::PCD,    Format::PLY,    Format::LAS,
    Format::PTS,  Format::OBJ,     Format::NPY,
    Format::XYZ,  Format::XYZI,    Format::XYZRGB, Format::XYZRGBI};
constexpr std::array<Format, 4> kAsciiFormats = {
    Format::XYZ, Format::XYZI, Format::XYZRGB, Format::XYZRGBI};
constexpr std::array<View, 10> kViews = {
    View::Front,         View::Right,        View::Back,
    View::Left,          View::Top,          View::Bottom,
    View::TopRightFront, View::TopLeftFront, View::BotRightFront,
    View::BotLeftFront};
constexpr const char *kRenderColorModes = "Auto\0RGB\0Intensity\0Z\0Solid\0";
constexpr const char *kRenderProjections = "Orthographic\0Perspective\0";

std::string renderStatsSummary(const RenderCloudStats &stats) {
  const float retained_ratio =
      stats.finite_points == 0U
          ? 0.0F
          : 100.0F * static_cast<float>(stats.retained_points) /
                static_cast<float>(stats.finite_points);
  std::ostringstream output;
  output << std::fixed << std::setprecision(2)
         << "Bounds LxWxH input=" << stats.input_dimensions[0] << 'x'
         << stats.input_dimensions[1] << 'x' << stats.input_dimensions[2]
         << " m, framed=" << stats.framed_dimensions[0] << 'x'
         << stats.framed_dimensions[1] << 'x' << stats.framed_dimensions[2]
         << " m; retained " << stats.retained_points << '/'
         << stats.finite_points << " (" << std::setprecision(1)
         << retained_ratio << "%)";
  return output.str();
}

struct CameraPresetButton {
  CameraPreset preset;
  const char *label;
  const char *tooltip;
};

constexpr std::array<CameraPresetButton, 8> kCameraPresetButtons = {{
    {CameraPreset::Top, nullptr, nullptr},
    {CameraPreset::Front, nullptr, nullptr},
    {CameraPreset::Left, nullptr, nullptr},
    {CameraPreset::Back, nullptr, nullptr},
    {CameraPreset::Right, nullptr, nullptr},
    {CameraPreset::Bottom, nullptr, nullptr},
    {CameraPreset::Iso1, nullptr, nullptr},
    {CameraPreset::Iso2, nullptr, nullptr},
}};

std::chrono::steady_clock::duration frameInterval(int fps) {
  const auto nanoseconds =
      std::max<std::int64_t>(1, 1'000'000'000LL / std::max(1, fps));
  return std::chrono::nanoseconds(nanoseconds);
}

void drawViewportHelp(ImDrawList &draw_list, const ImVec2 &image_position,
                      const ImVec2 &image_size, float grid_spacing,
                      bool show_controls, const Eigen::Vector3f &background) {
  draw_list.PushClipRect(image_position, image_position + image_size, true);
  const float luminance =
      background.dot(Eigen::Vector3f{0.2126F, 0.7152F, 0.0722F});
  const ImU32 text_color = luminance > 0.55F ? IM_COL32(42, 48, 58, 235)
                                             : IM_COL32(235, 240, 248, 220);
  if (grid_spacing > 0.0F) {
    std::ostringstream scale;
    scale << kpt::i18n::tr("gui.viewport.grid_prefix") << std::setprecision(3) << grid_spacing
          << kpt::i18n::tr("gui.viewport.grid_suffix");
    draw_list.AddText(image_position + ImVec2(10.0F, 10.0F), text_color,
                      scale.str().c_str());
  }
  if (show_controls) {
    constexpr const char *help_key = "gui.viewport.controls_help";
    const char *help = kpt::i18n::tr(help_key);
    const ImVec2 padding{8.0F, 6.0F};
    const ImVec2 text_size = ImGui::CalcTextSize(help);
    const ImVec2 panel_min{image_position.x + 10.0F,
                           image_position.y + image_size.y - text_size.y -
                               padding.y * 2.0F - 10.0F};
    const ImVec2 panel_max = panel_min + text_size + padding * 2.0F;
    const ImU32 panel_color = luminance > 0.55F ? IM_COL32(245, 247, 250, 210)
                                                : IM_COL32(20, 24, 32, 190);
    draw_list.AddRectFilled(panel_min, panel_max, panel_color, 5.0F);
    draw_list.AddText(panel_min + padding, text_color, help);
  }
  draw_list.PopClipRect();
}

std::optional<Format> asciiFlavor(int selection) {
  if (selection <= 0)
    return std::nullopt;
  return kAsciiFormats[static_cast<std::size_t>(selection - 1)];
}

const char *toolName(App::Tool tool) {
  switch (tool) {
  case App::Tool::Viewer:
    return kpt::i18n::tr("gui.tool.viewer");
  case App::Tool::Player:
    return kpt::i18n::tr("gui.tool.player");
  case App::Tool::Convert:
    return kpt::i18n::tr("gui.tool.convert");
  case App::Tool::Batch:
    return kpt::i18n::tr("gui.tool.batch");
  case App::Tool::Render:
    return kpt::i18n::tr("gui.tool.render");
  }
  return kpt::i18n::tr("gui.tool.unknown");
}

bool pathInput(const char *label, const char *input_id, std::string &value,
               const char *browse_id) {
  ImGui::TextUnformatted(label);
  const ImGuiStyle &style = ImGui::GetStyle();
  const float browse_width =
      ImGui::CalcTextSize("...").x + style.FramePadding.x * 2.0F;
  ImGui::SetNextItemWidth(-(browse_width + style.ItemSpacing.x));
  ImGui::InputText(input_id, &value);
  ImGui::SameLine();
  return ImGui::Button(browse_id);
}

} // namespace

App::App(std::unique_ptr<ViewportRenderer> main_renderer,
         std::unique_ptr<ViewportRenderer> trajectory_renderer,
         unsigned max_workers, std::shared_ptr<web::AssetStager> asset_stager)
    : main_viewport_(std::move(main_renderer)),
      trajectory_viewport_(std::move(trajectory_renderer)), jobs_(max_workers),
      asset_stager_(std::move(asset_stager)) {
#ifdef KPT_WEB_BUILD
  jobs_.setWorkerLimit(jobs_.maxWorkers());
#endif
  reset_dock_layout_ = true;
  next_frame_time_ = std::chrono::steady_clock::now();
}

App::~App() {
  playing_ = false;
  jobs_.setPlayerActive(false);
  jobs_.cancelAll();
}

std::vector<std::string> App::takeLaunchWarnings() {
  auto warnings = std::move(launch_warnings_);
  launch_warnings_.clear();
  return warnings;
}

void App::setStartupStyle(const ViewportStyle &style) {
  main_style_ = style;
  color_by_ = static_cast<int>(style.color_by);
  color_map_ = static_cast<int>(style.color_map);
  equalize_ = style.intensity_equalize;
  point_size_ = style.point_size;
  background_[0] = style.background.x();
  background_[1] = style.background.y();
  background_[2] = style.background.z();
  main_viewport_.setStyle(main_style_);
}

void App::startViewer(const std::filesystem::path &path) {
  tool_ = Tool::Viewer;
  viewer_input_ = displayPath(path);
  launch_state_ = LaunchState::Pending;
  launch_error_.reset();
  loadViewerFile(path);
}

void App::startSequence(workflow::SequenceOptions options, int fps,
                        bool autoplay) {
  tool_ = Tool::Player;
  fps_ = std::max(1, fps);
  autoplay_when_sequence_ready_ = autoplay;
  launch_state_ = LaunchState::Pending;
  launch_error_.reset();
  player_input_dir_ = displayPath(options.input_dir);
  player_glob_ = options.glob;
  player_label_dir_ =
      options.label_dir ? displayPath(*options.label_dir) : std::string{};
  player_poses_ = options.poses ? displayPath(*options.poses) : std::string{};
  player_poses2_ =
      options.poses2 ? displayPath(*options.poses2) : std::string{};
  openSequence(std::move(options));
}

void App::startSequence(std::shared_ptr<workflow::SequenceSource> sequence,
                        int fps, bool autoplay) {
  if (!sequence)
    throw std::invalid_argument("sequence source must not be null");
  const auto &options = sequence->options();
  tool_ = Tool::Player;
  fps_ = std::max(1, fps);
  autoplay_when_sequence_ready_ = autoplay;
  launch_state_ = LaunchState::Pending;
  launch_error_.reset();
  player_input_dir_ = displayPath(options.input_dir);
  player_glob_ = options.glob;
  player_label_dir_ =
      options.label_dir ? displayPath(*options.label_dir) : std::string{};
  player_poses_ = options.poses ? displayPath(*options.poses) : std::string{};
  player_poses2_ =
      options.poses2 ? displayPath(*options.poses2) : std::string{};
  openSequence(std::move(sequence));
}

Result<void, AppError> App::draw(FrameContext &frame_context,
                                 FramebufferMetrics metrics) {
  ui_.drain();
  updatePlayback();
  drawDockspace();
  drawTools();
  drawInspector();
  auto main_draw = drawViewport(frame_context, metrics);
  if (!main_draw)
    return main_draw.error();
  auto trajectory_draw = drawTrajectory(frame_context, metrics);
  if (!trajectory_draw)
    return trajectory_draw.error();
  drawJobsAndLog();
  drawFileDialog();
  drawAboutPopup();
  return {};
}

bool App::needsContinuousRedraw() const {
  if (playing_ || launch_state_ == LaunchState::Pending ||
      !pending_frames_.empty()) {
    return true;
  }
  return jobs_.hasActiveJobs();
}

void App::drawDockspace() {
  const ImGuiViewport *viewport = ImGui::GetMainViewport();
  constexpr float compact_width = 1000.0F;
  constexpr float compact_height = 700.0F;
  const bool compact = viewport->WorkSize.x <= compact_width ||
                       viewport->WorkSize.y <= compact_height;
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
    if (ImGui::BeginMenu(kpt::i18n::tr("gui.menu.view"))) {
      if (ImGui::MenuItem(kpt::i18n::tr("gui.menu.reset_layout")))
        reset_dock_layout_ = true;
      ImGui::Separator();
      if (ImGui::BeginMenu(kpt::i18n::tr("gui.menu.language"))) {
        for (auto code : kpt::i18n::availableLanguages()) {
          bool selected = (code == kpt::i18n::currentLanguage());
          if (ImGui::MenuItem(kpt::i18n::languageDisplayName(code).data(),
                              nullptr, &selected)) {
            kpt::i18n::setLanguage(code);
            reset_dock_layout_ = true;
          }
        }
        ImGui::EndMenu();
      }
      ImGui::EndMenu();
    }
    if (ImGui::BeginMenu(kpt::i18n::tr("gui.menu.help"))) {
      if (ImGui::MenuItem(kpt::i18n::tr("gui.menu.about")))
        show_about_ = true;
      ImGui::EndMenu();
    }
    ImGui::TextDisabled("%s", kpt::i18n::tr("gui.dockspace.title"));
    ImGui::EndMenuBar();
  }

  const ImGuiID dockspace_id = ImGui::GetID("KptMainDockspace");
  // DockSpace() creates the node. Capture absence first so a fresh settings
  // file gets the default split without overwriting a restored custom layout.
  const bool compact_mode_changed =
      compact_dock_layout_.has_value() && *compact_dock_layout_ != compact;
  const bool restored_layout_needs_compacting =
      !compact_dock_layout_.has_value() && compact &&
      ImGui::DockBuilderGetNode(dockspace_id) != nullptr;
  compact_dock_layout_ = compact;
  const bool build_default_layout =
      reset_dock_layout_ || compact_mode_changed ||
      restored_layout_needs_compacting ||
      ImGui::DockBuilderGetNode(dockspace_id) == nullptr;
  ImGui::DockSpace(dockspace_id, ImVec2(0.0F, 0.0F));
  if (build_default_layout) {
    reset_dock_layout_ = false;
    ImGui::DockBuilderRemoveNode(dockspace_id);
    ImGui::DockBuilderAddNode(dockspace_id, ImGuiDockNodeFlags_DockSpace);
    ImGui::DockBuilderSetNodeSize(dockspace_id, viewport->WorkSize);

    ImGuiID center = dockspace_id;
    const ImGuiID side = ImGui::DockBuilderSplitNode(
        center, ImGuiDir_Right, compact ? 0.34F : 0.30F, nullptr, &center);
    ImGuiID tools = side;
    if (!compact) {
      tools = ImGui::DockBuilderSplitNode(center, ImGuiDir_Left, 0.20F, nullptr,
                                          &center);
    }
    const ImGuiID bottom = ImGui::DockBuilderSplitNode(
        center, ImGuiDir_Down, compact ? 0.22F : 0.24F, nullptr, &center);
    ImGui::DockBuilderDockWindow(kpt::i18n::tr("gui.panel.tools"), tools);
    ImGui::DockBuilderDockWindow(kpt::i18n::tr("gui.panel.inspector"), side);
    ImGui::DockBuilderDockWindow(kpt::i18n::tr("gui.panel.trajectory"), side);
    ImGui::DockBuilderDockWindow(kpt::i18n::tr("gui.panel.jobs_log"), bottom);
    ImGui::DockBuilderDockWindow(kpt::i18n::tr("gui.panel.viewport"), center);
    ImGui::DockBuilderFinish(dockspace_id);
  }
  ImGui::End();
}

void App::drawTools() {
  ImGui::Begin(kpt::i18n::tr("gui.panel.tools"));
#ifdef KPT_WEB_BUILD
  constexpr std::array<Tool, 2> tools = {Tool::Viewer, Tool::Player};
#else
  constexpr std::array<Tool, 5> tools = {
      Tool::Viewer, Tool::Player, Tool::Convert, Tool::Batch, Tool::Render};
#endif
  for (const auto tool : tools) {
    if (ImGui::Selectable(toolName(tool), tool_ == tool))
      tool_ = tool;
  }
  ImGui::Separator();
  ImGui::TextWrapped("%s", kpt::i18n::tr("gui.tools.description"));
  ImGui::End();
}

void App::drawInspector() {
  ImGui::Begin(kpt::i18n::tr("gui.panel.inspector"));
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
#ifdef KPT_WEB_BUILD
  const auto selection = web::selectionSnapshot();
  if (ImGui::Button(kpt::i18n::tr("gui.viewer.choose_cloud")))
    web::openPicker(web::PickerKind::Viewer);
  if (selection.viewer) {
    ImGui::TextWrapped(kpt::i18n::tr("gui.viewer.selected"),
                       displayPath(selection.viewer->filename()).c_str());
    if (ImGui::Button(kpt::i18n::tr("gui.viewer.load"))) {
      if (asset_stager_) {
        const auto path = *selection.viewer;
        asset_stager_->stage({path},
                             [this, path](std::optional<std::string> error) {
                               if (error)
                                 log("Point-cloud staging error: " + *error);
                               else
                                 startViewer(path);
                             });
      } else {
        startViewer(*selection.viewer);
      }
    }
  } else {
    ImGui::TextDisabled("%s", kpt::i18n::tr("gui.viewer.no_cloud"));
  }
  if (!selection.error.empty())
    ImGui::TextWrapped(kpt::i18n::tr("gui.viewer.selection_error"), selection.error.c_str());
#else
  if (pathInput(kpt::i18n::tr("gui.viewer.input_label"), "##viewer-input", viewer_input_, "...##viewer")) {
    openDialog(DialogTarget::ViewerInput, kpt::i18n::tr("gui.viewer.dialog_open"), false, false,
               viewer_input_);
  }
  if (ImGui::Button(kpt::i18n::tr("gui.viewer.load")) && !viewer_input_.empty()) {
    loadViewerFile(viewer_input_);
  }
#endif
}

void App::drawPlayerControls() {
#ifdef KPT_WEB_BUILD
  const auto selection = web::selectionSnapshot();
  if (ImGui::Button(kpt::i18n::tr("gui.player.choose_frames")))
    web::openPicker(web::PickerKind::Clouds);
  ImGui::SameLine();
  ImGui::Text(kpt::i18n::tr("gui.player.selected_count"), selection.cloud_count);
  if (ImGui::Button(kpt::i18n::tr("gui.player.choose_labels")))
    web::openPicker(web::PickerKind::Labels);
  ImGui::SameLine();
  ImGui::Text(kpt::i18n::tr("gui.player.selected_count"), selection.label_count);
  if (ImGui::Button(kpt::i18n::tr("gui.player.choose_poses")))
    web::openPicker(web::PickerKind::Poses);
  ImGui::SameLine();
  ImGui::TextDisabled("%s", selection.has_poses ? kpt::i18n::tr("gui.player.poses_selected") : kpt::i18n::tr("gui.player.poses_optional"));
  if (ImGui::Button(kpt::i18n::tr("gui.player.choose_poses2")))
    web::openPicker(web::PickerKind::Poses2);
  ImGui::SameLine();
  ImGui::TextDisabled("%s", selection.has_poses2 ? kpt::i18n::tr("gui.player.poses_selected") : kpt::i18n::tr("gui.player.poses_optional"));
  if (!selection.error.empty())
    ImGui::TextWrapped(kpt::i18n::tr("gui.viewer.selection_error"), selection.error.c_str());
  if (ImGui::Button(kpt::i18n::tr("gui.player.open_sequence"))) {
    auto built = web::buildSequence();
    if (!built.source) {
      log("Sequence selection error: " + built.error);
    } else if (asset_stager_) {
      std::vector<std::filesystem::path> trajectory_assets;
      if (built.source->options().poses)
        trajectory_assets.push_back(*built.source->options().poses);
      if (built.source->options().poses2)
        trajectory_assets.push_back(*built.source->options().poses2);
      if (trajectory_assets.empty()) {
        openSequence(std::move(built.source));
      } else {
        asset_stager_->stage(std::move(trajectory_assets),
                             [this, source = std::move(built.source)](
                                 std::optional<std::string> error) mutable {
                               if (error)
                                 log("Pose staging error: " + *error);
                               else
                                 openSequence(std::move(source));
                             });
      }
    } else {
      openSequence(std::move(built.source));
    }
  }
#else
  if (pathInput(kpt::i18n::tr("gui.player.directory_label"), "##player-dir-input", player_input_dir_,
                "...##player-dir")) {
    openDialog(DialogTarget::PlayerInputDir, kpt::i18n::tr("gui.player.dialog_open_dir"), true,
               false, player_input_dir_);
  }
  ImGui::InputText(kpt::i18n::tr("gui.player.glob"), &player_glob_);
  if (pathInput(kpt::i18n::tr("gui.player.labels_label"), "##player-labels-input", player_label_dir_,
                "...##labels")) {
    openDialog(DialogTarget::PlayerLabelDir, kpt::i18n::tr("gui.player.dialog_open_labels"), true,
               false, player_label_dir_);
  }
  if (pathInput(kpt::i18n::tr("gui.player.poses_label"), "##player-poses-input", player_poses_, "...##poses")) {
    openDialog(DialogTarget::PlayerPoses, kpt::i18n::tr("gui.player.dialog_open_poses"), false, false,
               player_poses_);
  }
  if (pathInput(kpt::i18n::tr("gui.player.poses2_label"), "##player-poses2-input", player_poses2_,
                "...##poses2")) {
    openDialog(DialogTarget::PlayerPoses2, kpt::i18n::tr("gui.player.dialog_open_poses2"), false, false,
               player_poses2_);
  }
  if (ImGui::Button(kpt::i18n::tr("gui.player.open_sequence")) && !player_input_dir_.empty()) {
    openSequence();
  }
#endif

  if (!sequence_ || sequence_->empty())
    return;
  ImGui::Separator();
  const bool playing_forward =
      playing_ && playback_direction_ == PlaybackDirection::Forward;
  const bool playing_reverse =
      playing_ && playback_direction_ == PlaybackDirection::Reverse;
  if (ImGui::Button(playing_forward ? kpt::i18n::tr("gui.player.pause_forward") : kpt::i18n::tr("gui.player.play"))) {
    togglePlayback(PlaybackDirection::Forward);
  }
  ImGui::SameLine();
  if (ImGui::Button(playing_reverse ? kpt::i18n::tr("gui.player.pause_reverse") : kpt::i18n::tr("gui.player.reverse"))) {
    togglePlayback(PlaybackDirection::Reverse);
  }
  ImGui::SameLine();
  if (ImGui::Button(kpt::i18n::tr("gui.player.reset"))) {
    resetPlayback();
  }
  if (ImGui::Button(kpt::i18n::tr("gui.player.previous")) && current_frame_ > 0) {
    requestFrame(current_frame_ - 1, true);
  }
  ImGui::SameLine();
  if (ImGui::Button(kpt::i18n::tr("gui.player.next")) && current_frame_ + 1 < sequence_->size()) {
    requestFrame(current_frame_ + 1, true);
  }
  int frame = static_cast<int>(desired_frame_);
  const int maximum = static_cast<int>(sequence_->size() - 1);
  if (ImGui::SliderInt(kpt::i18n::tr("gui.player.frame"), &frame, 0, maximum)) {
    requestFrame(static_cast<std::size_t>(frame), true);
  }
  ImGui::SliderInt(kpt::i18n::tr("gui.player.fps"), &fps_, 1, 120);
  ImGui::Checkbox(kpt::i18n::tr("gui.player.loop"), &loop_);
#ifndef KPT_WEB_BUILD
  if (ImGui::CollapsingHeader(kpt::i18n::tr("gui.player.snapshot_export"))) {
    if (pathInput(kpt::i18n::tr("gui.player.prefix"), "##player-snapshot-prefix", player_snapshot_prefix_,
                  "...##player-snapshot")) {
      openDialog(DialogTarget::PlayerSnapshotPrefix, kpt::i18n::tr("gui.player.snapshot_export"),
                 false, true, player_snapshot_prefix_);
    }
    ImGui::InputInt(kpt::i18n::tr("gui.player.snapshot_width"), &render_width_);
    ImGui::InputInt(kpt::i18n::tr("gui.player.snapshot_height"), &render_height_);
    ImGui::Combo(kpt::i18n::tr("gui.player.snapshot_projection"), &render_projection_,
                 kRenderProjections);
    ImGui::InputFloat(kpt::i18n::tr("gui.player.snapshot_trim"),
                     &render_trim_percent_);
    if (render_projection_ == 1)
      ImGui::InputFloat(kpt::i18n::tr("gui.player.snapshot_fov"), &render_fov_);
    ImGui::Combo(kpt::i18n::tr("gui.player.snapshot_color_by"), &render_color_mode_,
                 kRenderColorModes);
    ImGui::Checkbox(kpt::i18n::tr("gui.player.snapshot_overwrite"), &render_overwrite_);
    if (ImGui::Button(kpt::i18n::tr("gui.player.export_snapshots")) &&
        !player_snapshot_prefix_.empty()) {
      queueRender(true);
    }
  }
#endif
}

void App::drawConvertControls() {
  if (pathInput(kpt::i18n::tr("gui.convert.input_label"), "##convert-input-path", convert_input_,
                "...##convert-input")) {
    openDialog(DialogTarget::ConvertInput, kpt::i18n::tr("gui.convert.dialog_open"), false, false,
               convert_input_);
  }
  if (pathInput(kpt::i18n::tr("gui.convert.output_label"), "##convert-output-path", convert_output_,
                "...##convert-output")) {
    openDialog(DialogTarget::ConvertOutput, kpt::i18n::tr("gui.convert.dialog_save"), false, true,
               convert_output_);
  }
  constexpr const char *ascii_items =
      "From extension\0xyz\0xyzi\0xyzrgb\0xyzrgbi\0";
  ImGui::Combo(kpt::i18n::tr("gui.convert.ascii_flavor"), &convert_ascii_, ascii_items);
  ImGui::Checkbox(kpt::i18n::tr("gui.convert.overwrite"), &convert_overwrite_);
  if (ImGui::Button(kpt::i18n::tr("gui.convert.queue")) && !convert_input_.empty() &&
      !convert_output_.empty()) {
    queueSingleConversion();
  }
}

void App::drawBatchControls() {
  if (pathInput(kpt::i18n::tr("gui.batch.input_label"), "##batch-input-directory", batch_input_dir_,
                "...##batch-input")) {
    openDialog(DialogTarget::BatchInputDir, kpt::i18n::tr("gui.batch.dialog_open_input"), true, false,
               batch_input_dir_);
  }
  if (pathInput(kpt::i18n::tr("gui.batch.output_label"),
                "##batch-output-directory",
                batch_output_dir_, "...##batch-output")) {
    openDialog(DialogTarget::BatchOutputDir, kpt::i18n::tr("gui.batch.dialog_open_output"), true,
               false, batch_output_dir_);
  }
  ImGui::InputText(kpt::i18n::tr("gui.batch.glob"), &batch_glob_);
  constexpr const char *formats =
      "bin\0pcd\0ply\0las\0pts\0obj\0npy\0xyz\0xyzi\0xyzrgb\0xyzrgbi\0";
  ImGui::Combo(kpt::i18n::tr("gui.batch.output_format"), &batch_format_, formats);
  constexpr const char *ascii_items =
      "From output format\0xyz\0xyzi\0xyzrgb\0xyzrgbi\0";
  ImGui::Combo(kpt::i18n::tr("gui.batch.ascii_flavor"), &batch_ascii_, ascii_items);
  ImGui::Checkbox(kpt::i18n::tr("gui.batch.overwrite"), &batch_overwrite_);
  if (ImGui::Button(kpt::i18n::tr("gui.batch.queue")) && !batch_input_dir_.empty() &&
      !batch_output_dir_.empty()) {
    queueBatchConversion();
  }
}

void App::drawRenderControls() {
  if (pathInput(kpt::i18n::tr("gui.render.input_label"), "##render-input-path", render_input_,
                "...##render-input")) {
    openDialog(DialogTarget::RenderInput, kpt::i18n::tr("gui.render.dialog_open"), false, false,
               render_input_);
  }
  if (pathInput(kpt::i18n::tr("gui.render.output_prefix_label"), "##render-output-prefix",
                render_output_prefix_, "...##render-prefix")) {
    openDialog(DialogTarget::RenderOutputPrefix, kpt::i18n::tr("gui.render.dialog_save"), false,
               true, render_output_prefix_);
  }
  ImGui::InputInt(kpt::i18n::tr("gui.render.width"), &render_width_);
  ImGui::InputInt(kpt::i18n::tr("gui.render.height"), &render_height_);
  ImGui::Combo(kpt::i18n::tr("gui.render.projection"), &render_projection_, kRenderProjections);
  ImGui::InputFloat(kpt::i18n::tr("gui.render.trim"), &render_trim_percent_);
  if (render_projection_ == 1)
    ImGui::InputFloat(kpt::i18n::tr("gui.render.fov"), &render_fov_);
  ImGui::Combo(kpt::i18n::tr("gui.render.color_by"), &render_color_mode_, kRenderColorModes);
  ImGui::Checkbox(kpt::i18n::tr("gui.render.overwrite"), &render_overwrite_);
  for (std::size_t index = 0; index < kViews.size(); ++index) {
    const std::string view_name(kpt::viewName(kViews[index]));
    ImGui::Checkbox(view_name.c_str(), &render_views_[index]);
    if (index % 2 == 0)
      ImGui::SameLine();
  }
  if (ImGui::Button(kpt::i18n::tr("gui.render.queue")) && !render_input_.empty() &&
      !render_output_prefix_.empty()) {
    queueRender(false);
  }
}

void App::drawDisplayControls() {
  if (const auto cloud = main_viewport_.cloud()) {
    const auto &bounds = cloud->bounds;
    const Eigen::Vector3d size =
        bounds.maximum.cast<double>() - bounds.minimum.cast<double>();
    ImGui::SeparatorText(kpt::i18n::tr("gui.display.cloud_info"));
    ImGui::Text(kpt::i18n::tr("gui.display.finite_points"), bounds.finite_points);
    if (bounds.finite_points != 0) {
      const Eigen::Vector3d center =
          (bounds.minimum.cast<double>() + bounds.maximum.cast<double>()) * 0.5;
      if (ImGui::BeginTable("##aabb", 4,
                            ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg |
                                ImGuiTableFlags_SizingStretchProp)) {
        ImGui::TableSetupColumn("");
        ImGui::TableSetupColumn("X");
        ImGui::TableSetupColumn("Y");
        ImGui::TableSetupColumn("Z");
        ImGui::TableHeadersRow();

        auto row = [](const char *label, double x, double y, double z) {
          ImGui::TableNextRow();
          ImGui::TableNextColumn();
          ImGui::TextUnformatted(label);
          ImGui::TableNextColumn();
          ImGui::Text("%.4g", x);
          ImGui::TableNextColumn();
          ImGui::Text("%.4g", y);
          ImGui::TableNextColumn();
          ImGui::Text("%.4g", z);
        };

        row(kpt::i18n::tr("gui.display.row_min"),
            static_cast<double>(bounds.minimum.x()),
            static_cast<double>(bounds.minimum.y()),
            static_cast<double>(bounds.minimum.z()));
        row(kpt::i18n::tr("gui.display.row_max"),
            static_cast<double>(bounds.maximum.x()),
            static_cast<double>(bounds.maximum.y()),
            static_cast<double>(bounds.maximum.z()));
        row(kpt::i18n::tr("gui.display.row_size"),
            size.x(), size.y(), size.z());
        row(kpt::i18n::tr("gui.display.row_center"),
            center.x(), center.y(), center.z());
        ImGui::EndTable();
      }
      if (bounds.has_noise)
        ImGui::Text(kpt::i18n::tr("gui.display.noise"), bounds.noise_points,
                    bounds.finite_points);
    }
  }
  ImGui::SeparatorText(kpt::i18n::tr("gui.display.section"));
  constexpr const char *color_modes = "Intensity\0RGB\0Z\0Label\0Fixed\0";
  if (ImGui::Combo(kpt::i18n::tr("gui.display.color_by"), &color_by_, color_modes)) {
    main_style_.color_by = static_cast<ColorBy>(color_by_);
    main_viewport_.setStyle(main_style_);
  }
  if (main_style_.color_by == ColorBy::Intensity) {
    constexpr const char *color_maps =
        "Turbo\0Viridis\0Plasma\0Inferno\0Magma\0Grayscale\0Hot\0Jet\0Spring\0Autumn\0";
    if (ImGui::Combo(kpt::i18n::tr("gui.display.colormap"), &color_map_, color_maps)) {
      main_style_.color_map = static_cast<ColorMap>(color_map_);
      main_viewport_.setStyle(main_style_);
    }
    if (ImGui::Checkbox(kpt::i18n::tr("gui.display.equalize"), &equalize_)) {
      main_style_.intensity_equalize = equalize_;
      main_viewport_.setStyle(main_style_);
    }
  }
  if (ImGui::SliderFloat(kpt::i18n::tr("gui.display.point_size"), &point_size_, 1.0F, 20.0F)) {
    main_style_.point_size = point_size_;
    main_viewport_.setStyle(main_style_);
  }
  if (ImGui::ColorEdit3(kpt::i18n::tr("gui.display.background"), background_)) {
    main_style_.background =
        Eigen::Vector3f(background_[0], background_[1], background_[2]);
    main_viewport_.setStyle(main_style_);
  }
  bool style_changed = false;
  if (main_style_.color_by == ColorBy::None)
    style_changed |=
        ImGui::ColorEdit3(kpt::i18n::tr("gui.display.fixed_color"), main_style_.fixed_color.data());
  style_changed |=
      ImGui::Checkbox(kpt::i18n::tr("gui.display.highlight_noise"), &main_style_.highlight_noise);
  if (main_style_.highlight_noise)
    style_changed |=
        ImGui::ColorEdit3(kpt::i18n::tr("gui.display.noise_color"), main_style_.noise_color.data());
  style_changed |=
      ImGui::Checkbox(kpt::i18n::tr("gui.display.coordinate_axes"), &main_style_.show_coordinate_axes);
  style_changed |=
      ImGui::Checkbox(kpt::i18n::tr("gui.display.scale_grid"), &main_style_.show_scale_grid);
  if (style_changed)
    main_viewport_.setStyle(main_style_);
  ImGui::Checkbox(kpt::i18n::tr("gui.display.viewport_controls"), &show_viewport_controls_);
  if (ImGui::Button(kpt::i18n::tr("gui.display.fit_all"), {ImGui::GetContentRegionAvail().x, 0.0F}))
    main_viewport_.fit();
  if (ImGui::IsItemHovered())
    ImGui::SetTooltip("%s", kpt::i18n::tr("gui.display.fit_tooltip"));
  constexpr std::size_t columns = 3;
  constexpr std::array<std::string_view, 8> camera_labels = {
      "gui.camera.top", "gui.camera.front", "gui.camera.left",
      "gui.camera.back", "gui.camera.right", "gui.camera.bottom",
      "gui.camera.iso1", "gui.camera.iso2"};
  constexpr std::array<std::string_view, 8> camera_tooltips = {
      "gui.camera.top_tooltip", "gui.camera.front_tooltip",
      "gui.camera.left_tooltip", "gui.camera.back_tooltip",
      "gui.camera.right_tooltip", "gui.camera.bottom_tooltip",
      "gui.camera.iso1_tooltip", "gui.camera.iso2_tooltip"};
  const float button_width =
      std::max(1.0F, (ImGui::GetContentRegionAvail().x -
                      ImGui::GetStyle().ItemSpacing.x *
                          static_cast<float>(columns - 1)) /
                         static_cast<float>(columns));
  for (std::size_t index = 0; index < kCameraPresetButtons.size(); ++index) {
    const auto &button = kCameraPresetButtons[index];
    if (ImGui::Button(kpt::i18n::tr(camera_labels[index]),
                      {button_width, 0.0F}))
      main_viewport_.setView(button.preset);
    if (ImGui::IsItemHovered())
      ImGui::SetTooltip("%s", kpt::i18n::tr(camera_tooltips[index]));
    if ((index + 1) % columns != 0 && index + 1 < kCameraPresetButtons.size())
      ImGui::SameLine();
  }
}

Result<void, AppError> App::drawViewport(FrameContext &frame_context,
                                         FramebufferMetrics metrics) {
  ImGui::Begin(kpt::i18n::tr("gui.panel.viewport"));
  const ImVec2 available = ImGui::GetContentRegionAvail();
  constexpr float interaction_render_scale = 0.75F;
  const bool interaction_quality =
      ImGui::GetTime() < interaction_quality_until_;
  const float render_scale =
      interaction_quality ? interaction_render_scale : 1.0F;
  const PixelExtent physical_extent =
      viewport_extent_override_for_tests_.value_or(PixelExtent{
          static_cast<int>(available.x * metrics.scale.x * render_scale),
          static_cast<int>(available.y * metrics.scale.y * render_scale)});
  const bool interactive_lod =
      interaction_quality && frame_context.backendKind() == BackendKind::WebGL;
  auto drawn = main_viewport_.draw(physical_extent, frame_context,
                                   ViewportRole::Main, interactive_lod);
  if (!drawn) {
    ImGui::End();
    return drawn.error();
  }
  bool viewport_interacting = false;
  if (drawn.value()) {
    const auto &viewport_texture = *drawn.value();
    const ImVec2 image_position = ImGui::GetCursorScreenPos();
    ImGui::InvisibleButton("##main-viewport-input", available,
                           ImGuiButtonFlags_MouseButtonLeft |
                               ImGuiButtonFlags_MouseButtonRight |
                               ImGuiButtonFlags_MouseButtonMiddle);
    const bool viewport_hovered = ImGui::IsItemHovered();
    viewport_interacting = viewport_hovered || ImGui::IsItemActive();
    const ImGuiIO &io = ImGui::GetIO();
    const bool camera_interacting =
        (ImGui::IsItemActive() &&
         (ImGui::IsMouseDragging(ImGuiMouseButton_Left) ||
          ImGui::IsMouseDragging(ImGuiMouseButton_Right))) ||
        (viewport_hovered && io.MouseWheel != 0.0F);
    if (camera_interacting) {
      constexpr double quality_restore_delay_seconds = 0.15;
      interaction_quality_until_ =
          ImGui::GetTime() + quality_restore_delay_seconds;
    }
    ImGui::SetCursorScreenPos(image_position);
    ImGui::Image(viewport_texture.ref, available, viewport_texture.uv0,
                 viewport_texture.uv1);
    const float grid_spacing =
        main_style_.show_scale_grid ? main_viewport_.gridSpacing() : 0.0F;
    if (show_viewport_controls_ || grid_spacing > 0.0F)
      drawViewportHelp(*ImGui::GetWindowDrawList(), image_position, available,
                       grid_spacing, show_viewport_controls_,
                       main_style_.background);
  }
  if (viewport_interacting) {
    const ImGuiIO &io = ImGui::GetIO();
    const ImVec2 image_min = ImGui::GetItemRectMin();
    const ImVec2 current{io.MousePos.x - image_min.x,
                         io.MousePos.y - image_min.y};
    const ImVec2 previous{current.x - io.MouseDelta.x,
                          current.y - io.MouseDelta.y};
    const PixelExtent interaction_extent{
        std::max(1, static_cast<int>(available.x)),
        std::max(1, static_cast<int>(available.y))};
    if (ImGui::IsMouseDragging(ImGuiMouseButton_Left)) {
      if (io.KeyShift) {
        main_viewport_.roll(io.MouseDelta.x, interaction_extent);
      } else {
        main_viewport_.orbit(previous.x, previous.y, current.x, current.y,
                             interaction_extent);
      }
    }
    if (ImGui::IsMouseDragging(ImGuiMouseButton_Right)) {
      main_viewport_.pan(io.MouseDelta.x, io.MouseDelta.y, interaction_extent);
    }
    if (ImGui::IsMouseClicked(ImGuiMouseButton_Middle))
      static_cast<void>(main_viewport_.setRotationCenterFromScreen(
          current.x, current.y, interaction_extent));
    if (io.MouseWheel != 0.0F)
      main_viewport_.zoom(io.MouseWheel * 15.0F);
  }
  ImGui::End();
  return {};
}

Result<void, AppError> App::drawTrajectory(FrameContext &frame_context,
                                           FramebufferMetrics metrics) {
  const auto cloud = trajectory_viewport_.cloud();
  if (!cloud || cloud->vertices.empty()) {
    // Drawing the suspended session is intentional: after a source reset it
    // uploads one empty revision to release stale trajectory GPU data.
    auto suspended =
        trajectory_viewport_.draw({}, frame_context, ViewportRole::Trajectory);
    if (!suspended)
      return suspended.error();
    return {};
  }
  ImGui::Begin(kpt::i18n::tr("gui.panel.trajectory"));
  const ImVec2 available = ImGui::GetContentRegionAvail();
  const PixelExtent physical_extent =
      viewport_extent_override_for_tests_.value_or(
          PixelExtent{static_cast<int>(available.x * metrics.scale.x),
                      static_cast<int>(available.y * metrics.scale.y)});
  auto drawn = trajectory_viewport_.draw(physical_extent, frame_context,
                                         ViewportRole::Trajectory);
  if (!drawn) {
    ImGui::End();
    return drawn.error();
  }
  if (drawn.value()) {
    const auto &trajectory_texture = *drawn.value();
    ImGui::Image(trajectory_texture.ref, available, trajectory_texture.uv0,
                 trajectory_texture.uv1);
  }
  ImGui::End();
  return {};
}

void App::drawJobsAndLog() {
  ImGui::Begin(kpt::i18n::tr("gui.panel.jobs_log"));
#ifdef KPT_WEB_BUILD
  ImGui::Text(kpt::i18n::tr("gui.jobs.workers_count"), jobs_.maxWorkers());
#else
  unsigned worker_limit = jobs_.workerLimit();
  const unsigned minimum_workers = 1;
  const unsigned maximum_workers = jobs_.maxWorkers();
  if (ImGui::SliderScalar(kpt::i18n::tr("gui.jobs.workers"), ImGuiDataType_U32, &worker_limit,
                          &minimum_workers, &maximum_workers)) {
    jobs_.setWorkerLimit(worker_limit);
  }
#endif
  ImGui::SameLine();
  if (ImGui::Button(kpt::i18n::tr("gui.jobs.cancel_all")))
    jobs_.cancelAll();
  ImGui::SameLine();
  if (ImGui::Button(kpt::i18n::tr("gui.jobs.clear_finished")))
    jobs_.clearFinished();

  if (ImGui::BeginTable("jobs", 5,
                        ImGuiTableFlags_RowBg | ImGuiTableFlags_Borders |
                            ImGuiTableFlags_SizingStretchProp)) {
    ImGui::TableSetupColumn(kpt::i18n::tr("gui.jobs.col_job"));
    ImGui::TableSetupColumn(kpt::i18n::tr("gui.jobs.col_state"));
    ImGui::TableSetupColumn(kpt::i18n::tr("gui.jobs.col_progress"));
    ImGui::TableSetupColumn(kpt::i18n::tr("gui.jobs.col_message"));
    ImGui::TableSetupColumn(kpt::i18n::tr("gui.jobs.col_action"));
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
          ImGui::SmallButton((std::string(kpt::i18n::tr("gui.jobs.cancel")) + "##" + std::to_string(job.id)).c_str())) {
        jobs_.cancel(job.id);
      }
    }
    ImGui::EndTable();
  }
  ImGui::SeparatorText(kpt::i18n::tr("gui.jobs.log_section"));
  for (const auto &message : logs_)
    ImGui::TextWrapped("%s", message.c_str());
  ImGui::End();
}

void App::openDialog(DialogTarget target, const char *title, bool directory,
                     bool save, const std::string &current) {
#ifdef KPT_WEB_BUILD
  static_cast<void>(target);
  static_cast<void>(title);
  static_cast<void>(directory);
  static_cast<void>(save);
  static_cast<void>(current);
#else
  IGFD::FileDialogConfig config;
  auto initial_directory = dialogInitialDirectory(current, directory);
  if (!initial_directory) {
    log("File dialog path error: " + initial_directory.error().message);
    return;
  }
  auto initial_directory_utf8 = platform::pathToUtf8(initial_directory.value());
  if (!initial_directory_utf8) {
    log("File dialog path error: " + initial_directory_utf8.error().message);
    return;
  }
  config.path = std::move(initial_directory_utf8).value();

  if (!current.empty()) {
    auto current_path = platform::pathFromUtf8(current);
    if (!current_path) {
      log("File dialog path error: " + current_path.error().message);
      return;
    }
    if (save) {
      auto filename = platform::pathToUtf8(current_path.value().filename());
      if (!filename) {
        log("File dialog path error: " + filename.error().message);
        return;
      }
      config.fileName = std::move(filename).value();
    }
  }

  dialog_target_ = target;
  dialog_directory_ = directory;
  config.flags =
      save ? ImGuiFileDialogFlags_ConfirmOverwrite : ImGuiFileDialogFlags_None;
  const char *filters =
      directory ? nullptr
                : "Point clouds{.bin,.pcd,.ply,.las,.pts,.obj,.npy,.xyz,.xyzi,.xyzrgb,.xyzrgbi}";
  ImGuiFileDialog::Instance()->OpenDialog("KptPathDialog", title, filters,
                                          config);
#endif
}

void App::drawAboutPopup() {
  if (!show_about_)
    return;
  ImGui::OpenPopup("##KptAboutPopup");
  if (ImGui::BeginPopupModal("##KptAboutPopup", &show_about_,
                             ImGuiWindowFlags_AlwaysAutoResize)) {
    ImGui::TextUnformatted(kpt::i18n::tr("gui.about.title"));
    ImGui::Separator();
    ImGui::Text(kpt::i18n::tr("gui.about.version"), KPT_VERSION_STRING);
    ImGui::Text(kpt::i18n::tr("gui.about.commit"), KPT_GIT_HASH);
    ImGui::Text(kpt::i18n::tr("gui.about.build_time"), KPT_BUILD_TIME);
    ImGui::Spacing();
    ImGui::TextWrapped("%s", kpt::i18n::tr("gui.about.description"));
    ImGui::Spacing();
    ImGui::TextWrapped("%s",
      "https://github.com/nerdneilsfield/kitti_pointcloud_tools");
    ImGui::Separator();
    if (ImGui::Button(kpt::i18n::tr("gui.about.close"),
                      ImVec2(ImGui::GetContentRegionAvail().x, 0.0F)))
      show_about_ = false;
    ImGui::EndPopup();
  }
}

void App::drawFileDialog() {
#ifdef KPT_WEB_BUILD
  return;
#else
  if (dialog_target_ == DialogTarget::None)
    return;
  const ImGuiViewport *viewport = ImGui::GetMainViewport();
  const ImVec2 dialog_size(std::min(960.0F, viewport->WorkSize.x * 0.80F),
                           std::min(700.0F, viewport->WorkSize.y * 0.80F));
  const ImVec2 dialog_center(viewport->WorkPos.x + viewport->WorkSize.x * 0.5F,
                             viewport->WorkPos.y + viewport->WorkSize.y * 0.5F);
  ImGui::SetNextWindowPos(dialog_center, ImGuiCond_Appearing,
                          ImVec2(0.5F, 0.5F));
  ImGui::SetNextWindowSize(dialog_size, ImGuiCond_Appearing);
  if (ImGuiFileDialog::Instance()->Display("KptPathDialog")) {
    if (ImGuiFileDialog::Instance()->IsOk()) {
      const auto current_path = ImGuiFileDialog::Instance()->GetCurrentPath();
      auto value =
          dialog_directory_
              ? selectedDialogDirectory(
                    ImGuiFileDialog::Instance()->GetSelection(), current_path)
              : normalizeDialogPath(
                    ImGuiFileDialog::Instance()->GetFilePathName(),
                    current_path);
      if (!value) {
        log("File dialog path error: " + value.error().message);
      } else {
        auto value_utf8 = platform::pathToUtf8(value.value());
        if (!value_utf8)
          log("File dialog path error: " + value_utf8.error().message);
        else
          applyDialogResult(std::move(value_utf8).value());
      }
    }
    ImGuiFileDialog::Instance()->Close();
    dialog_target_ = DialogTarget::None;
  }
#endif
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
  spdlog::info("{}", message);
  logs_.push_front(std::move(message));
  while (logs_.size() > 200)
    logs_.pop_back();
}

std::optional<std::filesystem::path>
App::decodeUiPath(std::string_view value, std::string_view purpose) {
  auto decoded = platform::pathFromUtf8(value);
  if (!decoded) {
    log(std::string(purpose) + ": " + decoded.error().message);
    return std::nullopt;
  }
  return std::move(decoded).value();
}

std::string App::displayPath(const std::filesystem::path &value) {
  auto encoded = platform::pathToUtf8(value);
  if (!encoded) {
    log("Native path conversion failed: " + encoded.error().message);
    return "<invalid-native-path>";
  }
  return std::move(encoded).value();
}

void App::loadViewerFile(const std::string &path) {
  launch_state_ = LaunchState::None;
  launch_error_.reset();
  const auto native_path = decodeUiPath(path, "Viewer input path");
  if (!native_path)
    return;
  loadViewerFile(*native_path);
}

void App::loadViewerFile(const std::filesystem::path &native_path) {
  const auto filename = displayPath(native_path.filename());
  const auto display_path = displayPath(native_path);
  const auto source_generation = beginNewSource();
  const auto request_generation = main_viewport_.beginRequest();
  jobs_.submit(
      "Load " + filename, JobPriority::High,
      [this, native_path, display_path, source_generation, request_generation,
       stager = asset_stager_](std::stop_token stop,
                               const JobSystem::Reporter &report) {
        bool asset_released = false;
        const auto release = [this, stager, native_path, &asset_released] {
          if (asset_released || !stager)
            return;
          asset_released = true;
          ui_.post([stager, native_path] { stager->release({native_path}); });
        };
        try {
          report(0.1F, "loading");
          const auto cloud = kpt::load(native_path, stop);
          release();
          if (stop.stop_requested())
            return;
          const auto snapshot =
              makeViewportCloudSnapshot(cloud, request_generation);
          ui_.post([this, snapshot, display_path, source_generation] {
            if (source_generation != sequence_generation_)
              return;
            if (main_viewport_.accept(snapshot)) {
              log("Loaded " + display_path + " (" +
                  std::to_string(snapshot->vertices.size()) + " points)");
              if (launch_state_ == LaunchState::Pending)
                launch_state_ = LaunchState::Ready;
            }
          });
          report(1.0F, "loaded " + std::to_string(cloud->size()) + " points");
        } catch (const std::exception &error) {
          release();
          ui_.post([this, display_path, source_generation,
                    message = std::string(error.what())] {
            if (source_generation != sequence_generation_)
              return;
            const auto full_message =
                "Failed to load " + display_path + ": " + message;
            log(full_message);
            if (launch_state_ == LaunchState::Pending) {
              launch_error_ = full_message;
              launch_state_ = LaunchState::Failed;
            }
          });
          throw;
        } catch (...) {
          release();
          ui_.post([this, display_path, source_generation] {
            if (source_generation != sequence_generation_)
              return;
            const auto message =
                "Failed to load " + display_path + ": unknown error";
            log(message);
            if (launch_state_ == LaunchState::Pending) {
              launch_error_ = message;
              launch_state_ = LaunchState::Failed;
            }
          });
          throw;
        }
      });
}

void App::openSequence() {
  autoplay_when_sequence_ready_ = false;
  launch_state_ = LaunchState::None;
  launch_error_.reset();
  workflow::SequenceOptions options;
  const auto input_dir =
      decodeUiPath(player_input_dir_, "Sequence input directory");
  if (!input_dir)
    return;
  options.input_dir = *input_dir;
  options.glob = player_glob_;
  if (!player_label_dir_.empty()) {
    auto path = decodeUiPath(player_label_dir_, "Sequence label directory");
    if (!path)
      return;
    options.label_dir = *path;
  }
  if (!player_poses_.empty()) {
    auto path = decodeUiPath(player_poses_, "Sequence poses path");
    if (!path)
      return;
    options.poses = *path;
  }
  if (!player_poses2_.empty()) {
    auto path = decodeUiPath(player_poses2_, "Sequence poses 2 path");
    if (!path)
      return;
    options.poses2 = *path;
  }

  openSequence(std::move(options));
}

void App::openSequence(workflow::SequenceOptions options) {
  queueSequence([options = std::move(options)] {
    return std::make_shared<workflow::SequenceSource>(options);
  });
}

void App::openSequence(std::shared_ptr<workflow::SequenceSource> sequence) {
  queueSequence(
      [sequence = std::move(sequence)] { return std::move(sequence); });
}

void App::queueSequence(
    std::function<std::shared_ptr<workflow::SequenceSource>()> create) {
  const auto sequence_generation = beginNewSource();
  const auto trajectory_generation = trajectory_viewport_.beginRequest();
  jobs_.submit(
      "Open sequence", JobPriority::High,
      [this, create = std::move(create), sequence_generation,
       trajectory_generation](std::stop_token stop,
                              const JobSystem::Reporter &report) {
        try {
          report(0.1F, "enumerating");
          auto sequence = create();
          std::vector<std::filesystem::path> trajectory_assets;
          if (sequence->options().poses)
            trajectory_assets.push_back(*sequence->options().poses);
          if (sequence->options().poses2)
            trajectory_assets.push_back(*sequence->options().poses2);
          workflow::SequenceTrajectory trajectory{
              std::make_shared<PointCloudIRGB>(), {}};
          if (!stop.stop_requested()) {
            trajectory = sequence->trajectoryBestEffort(stop);
          }
          if (asset_stager_ && !trajectory_assets.empty()) {
            const auto stager = asset_stager_;
            ui_.post([stager, assets = std::move(trajectory_assets)] {
              stager->release(assets);
            });
          }
          if (stop.stop_requested())
            return;
          const auto trajectory_snapshot = makeViewportCloudSnapshot(
              trajectory.cloud, trajectory_generation);
          ui_.post([this, sequence, sequence_generation, trajectory_snapshot,
                    trajectory_warnings = std::move(trajectory.warnings)] {
            if (sequence_generation != sequence_generation_)
              return;
            sequence_ = sequence;
            frame_cache_.clear();
            pending_frames_.clear();
            current_frame_ = 0;
            desired_frame_ = 0;
            static_cast<void>(trajectory_viewport_.accept(trajectory_snapshot));
            ViewportStyle trajectory_style;
            trajectory_style.color_by = ColorBy::RGB;
            trajectory_viewport_.setStyle(trajectory_style);
            for (auto &warning : trajectory_warnings) {
              log(warning);
              if (launch_state_ == LaunchState::Pending)
                launch_warnings_.push_back(std::move(warning));
            }
            log("Opened sequence with " + std::to_string(sequence->size()) +
                " frames");
            if (!sequence->empty()) {
              requestFrame(0, true, true);
            } else if (launch_state_ == LaunchState::Pending) {
              launch_state_ = LaunchState::Empty;
            }
          });
          report(1.0F, "ready");
        } catch (const std::exception &error) {
          ui_.post(
              [this, sequence_generation, message = std::string(error.what())] {
                if (sequence_generation != sequence_generation_)
                  return;
                const auto full_message = "Failed to open sequence: " + message;
                log(full_message);
                if (launch_state_ == LaunchState::Pending) {
                  launch_error_ = full_message;
                  launch_state_ = LaunchState::Failed;
                }
              });
          throw;
        } catch (...) {
          ui_.post([this, sequence_generation] {
            if (sequence_generation != sequence_generation_)
              return;
            const std::string message =
                "Failed to open sequence: unknown error";
            log(message);
            if (launch_state_ == LaunchState::Pending) {
              launch_error_ = message;
              launch_state_ = LaunchState::Failed;
            }
          });
          throw;
        }
      });
}

std::uint64_t App::beginNewSource() {
  playing_ = false;
  playback_direction_ = PlaybackDirection::Forward;
  launch_warnings_.clear();
  jobs_.setPlayerActive(false);
  sequence_.reset();
  frame_cache_.clear();
  pending_frames_.clear();
  current_frame_ = 0;
  desired_frame_ = 0;
  next_frame_time_ = std::chrono::steady_clock::now();
  main_viewport_.cancelAndClear();
  trajectory_viewport_.cancelAndClear();
  return ++sequence_generation_;
}

void App::requestFrame(std::size_t index, bool apply, bool fit_camera) {
  if (!sequence_ || index >= sequence_->size())
    return;
  if (apply)
    desired_frame_ = index;
  if (pending_frames_.contains(index))
    return;

  std::uint64_t request_generation = 0;
  if (const auto found = frame_cache_.find(index);
      found != frame_cache_.end()) {
    if (apply) {
      pending_frames_.insert(index);
      request_generation = main_viewport_.beginRequest();
      queueCachedFrame(index, found->second, fit_camera, request_generation,
                       sequence_generation_);
    }
    return;
  }
  if (!pending_frames_.insert(index).second)
    return;
  if (apply)
    request_generation = main_viewport_.beginRequest();
  const auto sequence_generation = sequence_generation_;

  std::vector<std::filesystem::path> assets{sequence_->files()[index]};
  if (sequence_->options().label_dir) {
    auto label_name = sequence_->files()[index].stem();
    label_name += ".label";
    assets.push_back(*sequence_->options().label_dir / label_name);
  }
  if (asset_stager_) {
    const auto stager = asset_stager_;
    stager->stage(assets,
                  [this, index, apply, fit_camera, request_generation,
                   sequence_generation, stager, assets = std::move(assets)](
                      std::optional<std::string> stage_error) mutable {
                    if (stage_error) {
                      if (sequence_generation != sequence_generation_)
                        return;
                      pending_frames_.erase(index);
                      if (apply && desired_frame_ == index) {
                        desired_frame_ = current_frame_;
                        playing_ = false;
                        const std::string message =
                            "Failed to stage sequence frame " +
                            std::to_string(index) + ": " + *stage_error;
                        log(message);
                        if (launch_state_ == LaunchState::Pending) {
                          launch_error_ = message;
                          launch_state_ = LaunchState::Failed;
                        }
                      }
                      return;
                    }
                    if (sequence_generation != sequence_generation_) {
                      stager->release(assets);
                      return;
                    }
                    queueFrameLoad(index, apply, fit_camera, request_generation,
                                   sequence_generation, std::move(assets));
                  });
    return;
  }
  queueFrameLoad(index, apply, fit_camera, request_generation,
                 sequence_generation, {});
}

void App::queueCachedFrame(std::size_t index, PointCloudIRGBConstPtr cloud,
                           bool fit_camera,
                           std::uint64_t request_generation,
                           std::uint64_t sequence_generation) {
  jobs_.submit(
      "Prepare cached frame " + std::to_string(index), JobPriority::High,
      [this, index, cloud = std::move(cloud), fit_camera, request_generation,
       sequence_generation](std::stop_token stop,
                            const JobSystem::Reporter &report) {
        try {
          report(0.1F, "preparing cached frame");
          if (stop.stop_requested()) {
            ui_.post([this, index, sequence_generation] {
              if (sequence_generation == sequence_generation_)
                pending_frames_.erase(index);
            });
            return;
          }
          auto snapshot =
              makeViewportCloudSnapshot(cloud, request_generation);
          ui_.post([this, index, fit_camera, request_generation,
                    sequence_generation, snapshot = std::move(snapshot)] {
            pending_frames_.erase(index);
            if (sequence_generation != sequence_generation_ ||
                desired_frame_ != index)
              return;
            if (!main_viewport_.accept(
                    snapshot, fit_camera ? CameraUpdate::Fit
                                         : CameraUpdate::Preserve)) {
              return;
            }
            current_frame_ = index;
          });
          report(1.0F, "ready");
        } catch (...) {
          ui_.post([this, index, sequence_generation] {
            if (sequence_generation == sequence_generation_)
              pending_frames_.erase(index);
          });
          throw;
        }
      });
}

void App::queueFrameLoad(std::size_t index, bool apply, bool fit_camera,
                         std::uint64_t request_generation,
                         std::uint64_t sequence_generation,
                         std::vector<std::filesystem::path> staged_assets) {
  const auto sequence = sequence_;
  const auto stager = asset_stager_;
  jobs_.submit(
      "Load frame " + std::to_string(index), JobPriority::High,
      [this, sequence, index, apply, fit_camera, request_generation,
       sequence_generation, stager, staged_assets = std::move(staged_assets)](
          std::stop_token stop, const JobSystem::Reporter &report) {
        bool assets_released = false;
        const auto release = [this, stager, staged_assets, &assets_released] {
          if (assets_released)
            return;
          assets_released = true;
          if (stager && !staged_assets.empty()) {
            ui_.post(
                [stager, staged_assets] { stager->release(staged_assets); });
          }
        };
        try {
          report(0.1F, "loading");
          auto frame = sequence->load(index, stop);
          release();
          if (stop.stop_requested()) {
            ui_.post([this, index, sequence_generation] {
              if (sequence_generation == sequence_generation_)
                pending_frames_.erase(index);
            });
            return;
          }
          auto snapshot =
              apply ? makeViewportCloudSnapshot(frame.cloud, request_generation)
                    : std::shared_ptr<const ViewportCloudSnapshot>{};
          ui_.post([this, index, apply, fit_camera, cloud = frame.cloud,
                    snapshot = std::move(snapshot), sequence_generation] {
            if (sequence_generation != sequence_generation_)
              return;
            pending_frames_.erase(index);
            frame_cache_[index] = cloud;
            while (frame_cache_.size() > 3) {
              auto victim = frame_cache_.begin();
              if (victim->first == current_frame_ ||
                  victim->first == desired_frame_)
                ++victim;
              if (victim != frame_cache_.end()) {
                frame_cache_.erase(victim);
              } else {
                break;
              }
            }
            if (apply && desired_frame_ == index) {
              if (!main_viewport_.accept(snapshot,
                                         fit_camera ? CameraUpdate::Fit
                                                    : CameraUpdate::Preserve)) {
                return;
              }
              current_frame_ = index;
              if (index == 0 && launch_state_ == LaunchState::Pending)
                launch_state_ = LaunchState::Ready;
              if (index == 0 && autoplay_when_sequence_ready_) {
                autoplay_when_sequence_ready_ = false;
                playing_ = true;
                playback_direction_ = PlaybackDirection::Forward;
                next_frame_time_ =
                    std::chrono::steady_clock::now() + frameInterval(fps_);
              }
              if (sequence_) {
                const auto prefetched = nextPlaybackFrame(
                    index, sequence_->size(), playback_direction_, false);
                if (prefetched)
                  requestFrame(*prefetched, false);
              }
            } else if (!apply && desired_frame_ == index) {
              // A prefetch may already be in flight when scrub selects same
              // frame. Reuse its result; do not start second I/O job.
              requestFrame(index, true);
            }
          });
          report(1.0F, "loaded");
        } catch (const std::exception &error) {
          release();
          ui_.post([this, index, apply, sequence_generation,
                    message = std::string(error.what())] {
            if (sequence_generation != sequence_generation_)
              return;
            pending_frames_.erase(index);
            if (apply && desired_frame_ == index) {
              desired_frame_ = current_frame_;
              playing_ = false;
              autoplay_when_sequence_ready_ = false;
              if (launch_state_ == LaunchState::Pending) {
                launch_error_ = "Failed to load sequence frame " +
                                std::to_string(index) + ": " + message;
                launch_state_ = LaunchState::Failed;
              }
            }
          });
          throw;
        } catch (...) {
          release();
          ui_.post([this, index, apply, sequence_generation] {
            if (sequence_generation != sequence_generation_)
              return;
            pending_frames_.erase(index);
            if (apply && desired_frame_ == index) {
              desired_frame_ = current_frame_;
              playing_ = false;
              autoplay_when_sequence_ready_ = false;
              if (launch_state_ == LaunchState::Pending) {
                launch_error_ = "Failed to load sequence frame " +
                                std::to_string(index) + ": unknown error";
                launch_state_ = LaunchState::Failed;
              }
            }
          });
          throw;
        }
      });
}

void App::togglePlayback(PlaybackDirection direction) {
  if (playing_ && playback_direction_ == direction) {
    playing_ = false;
  } else {
    playing_ = true;
    playback_direction_ = direction;
  }
  jobs_.setPlayerActive(playing_);
  next_frame_time_ = std::chrono::steady_clock::now();
}

void App::resetPlayback() {
  playing_ = false;
  playback_direction_ = PlaybackDirection::Forward;
  jobs_.setPlayerActive(false);
  next_frame_time_ = std::chrono::steady_clock::now();
  if (sequence_ && !sequence_->empty())
    requestFrame(0, true);
}

std::optional<std::size_t> App::nextPlaybackFrame(std::size_t current,
                                                  std::size_t frame_count,
                                                  PlaybackDirection direction,
                                                  bool loop) {
  if (frame_count == 0 || current >= frame_count)
    return std::nullopt;
  if (direction == PlaybackDirection::Reverse) {
    if (current > 0)
      return current - 1;
    if (loop)
      return frame_count - 1;
    return std::nullopt;
  }
  if (current + 1 < frame_count)
    return current + 1;
  if (loop)
    return 0;
  return std::nullopt;
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
  next_frame_time_ = now + frameInterval(fps_);

  const auto next = nextPlaybackFrame(desired_frame_, sequence_->size(),
                                      playback_direction_, loop_);
  if (!next) {
    playing_ = false;
    jobs_.setPlayerActive(false);
    return;
  }
  requestFrame(*next, true);
}

void App::queueSingleConversion() {
  workflow::ConversionRequest request;
  const auto input = decodeUiPath(convert_input_, "Conversion input path");
  const auto output = decodeUiPath(convert_output_, "Conversion output path");
  if (!input || !output)
    return;
  const auto flavor = asciiFlavor(convert_ascii_);
  try {
    io::validateAsciiFlavor(*output, flavor);
  } catch (const std::invalid_argument &error) {
    log(error.what());
    return;
  }
  request.input = *input;
  request.output = *output;
  request.ascii_flavor = flavor;
  request.overwrite = convert_overwrite_;
  jobs_.submit(
      "Convert " + displayPath(request.input.filename()), JobPriority::Normal,
      [this, request](std::stop_token stop, const JobSystem::Reporter &report) {
        report(0.1F, "converting");
        if (stop.stop_requested())
          return;
        const auto result = workflow::convert(request, stop);
        if (result.status == workflow::OperationStatus::Cancelled)
          return;
        ui_.post([this, result] {
          log(displayPath(result.input) + " -> " + displayPath(result.output) +
              ": " + result.message);
        });
        if (result.status == workflow::OperationStatus::Failed) {
          throw std::runtime_error(result.message);
        }
        report(1.0F, result.message);
      });
}

void App::queueBatchConversion() {
  workflow::BatchConvertOptions options;
  const auto input = decodeUiPath(batch_input_dir_, "Batch input directory");
  const auto output = decodeUiPath(batch_output_dir_, "Batch output directory");
  if (!input || !output)
    return;
  options.input_dir = *input;
  options.output_dir = *output;
  options.glob = batch_glob_;
  options.output_format = kFormats[static_cast<std::size_t>(batch_format_)];
  options.ascii_flavor = asciiFlavor(batch_ascii_);
  options.overwrite = batch_overwrite_;

  try {
    io::validateAsciiFlavor(options.output_format, options.ascii_flavor);
    const auto plan = workflow::makeBatchPlan(options);
    if (plan.error) {
      log(*plan.error);
      return;
    }
    for (const auto &rejected : plan.rejected) {
      log(displayPath(rejected.input) + ": " + rejected.message);
    }
    for (const auto &request : plan.requests) {
      jobs_.submit(
          "Batch " + displayPath(request.input.filename()), JobPriority::Low,
          [this, request](std::stop_token stop,
                          const JobSystem::Reporter &report) {
            if (stop.stop_requested())
              return;
            report(0.1F, "converting");
            const auto result = workflow::convert(request, stop);
            if (result.status == workflow::OperationStatus::Cancelled)
              return;
            ui_.post([this, result] {
              log(displayPath(result.input.filename()) + ": " + result.message);
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
    if (std::none_of(std::begin(render_views_), std::end(render_views_),
                     [](bool selected) { return selected; })) {
      log("Select at least one render view");
      return;
    }
    for (std::size_t index = 0; index < sequence_->size(); ++index) {
      queueSnapshotFrame(index);
    }
    log("Queued snapshots for " + std::to_string(sequence_->size()) +
        " frames");
    return;
  }

  const auto input_path = decodeUiPath(render_input_, "Render input path");
  const auto output_prefix =
      decodeUiPath(render_output_prefix_, "Render output prefix");
  if (!input_path || !output_prefix)
    return;
  const std::filesystem::path input = *input_path;
  const std::filesystem::path prefix = *output_prefix;
  const int width = std::max(1, render_width_);
  const int height = std::max(1, render_height_);
  const float fov = render_fov_;
  const auto projection =
      static_cast<RenderProjection>(std::clamp(render_projection_, 0, 1));
  const float trim_percent = render_trim_percent_;
  if (!std::isfinite(trim_percent) || trim_percent < 0.0F ||
      trim_percent >= 50.0F) {
    log("Trim percent must be in [0, 50)");
    return;
  }
  const auto color_mode =
      static_cast<RenderColorMode>(std::clamp(render_color_mode_, 0, 4));
  const bool overwrite = render_overwrite_;
  std::vector<View> views;
  for (std::size_t index = 0; index < kViews.size(); ++index) {
    if (render_views_[index])
      views.push_back(kViews[index]);
  }
  if (views.empty()) {
    log("Select at least one render view");
    return;
  }
  jobs_.submit("Render " + displayPath(input.filename()), JobPriority::Low,
               [this, input, prefix, width, height, fov, projection,
                trim_percent, color_mode, overwrite, views = std::move(views)](
                   std::stop_token stop, const JobSystem::Reporter &report) {
                 report(0.05F, "loading");
                 const auto cloud = kpt::load(input, stop);
                 RenderOpts options;
                 options.width = width;
                 options.height = height;
                 options.fov = fov;
                 options.projection = projection;
                 options.trim_percent = trim_percent;
                 options.color_mode = color_mode;
                 options.views = views;
                 report(0.1F, "analyzing bounds");
                 const auto results =
                     kpt::renderMultiView(cloud, options, stop);
                 if (!results.empty()) {
                   const std::string summary =
                       renderStatsSummary(results.front().cloud_stats);
                   ui_.post([this, summary] { log(summary); });
                 }
                 for (std::size_t index = 0; index < results.size(); ++index) {
                   if (stop.stop_requested())
                     return;
                   const auto &result = results[index];
                   const std::string &view_name = result.view_name;
                   auto output = prefix;
                   output += "_" + view_name + ".png";
                   const auto status = kpt::writeImageAtomic(
                       output, result.image, overwrite, stop);
                   const bool written = status == ImageWriteStatus::Written;
                   ui_.post([this, output, written] {
                     log(std::string(written ? "Wrote " : "Skipped existing ") +
                         displayPath(output));
                   });
                   report(0.1F + 0.9F * static_cast<float>(index + 1) /
                                     static_cast<float>(results.size()),
                          written ? "written" : "skipped existing");
                 }
               });
}

void App::queueSnapshotFrame(std::size_t index) {
  const auto sequence = sequence_;
  const auto decoded_prefix =
      decodeUiPath(player_snapshot_prefix_, "Snapshot output prefix");
  if (!decoded_prefix)
    return;
  const std::filesystem::path prefix = *decoded_prefix;
  const int width = std::max(1, render_width_);
  const int height = std::max(1, render_height_);
  const float fov = render_fov_;
  const auto projection =
      static_cast<RenderProjection>(std::clamp(render_projection_, 0, 1));
  const float trim_percent = render_trim_percent_;
  if (!std::isfinite(trim_percent) || trim_percent < 0.0F ||
      trim_percent >= 50.0F) {
    log("Trim percent must be in [0, 50)");
    return;
  }
  const auto color_mode =
      static_cast<RenderColorMode>(std::clamp(render_color_mode_, 0, 4));
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
      [this, sequence, prefix, index, width, height, fov, projection,
       trim_percent, color_mode, overwrite, views = std::move(views)](
          std::stop_token stop, const JobSystem::Reporter &report) {
        auto frame = sequence->load(index, stop);
        if (stop.stop_requested())
          return;
        RenderOpts options;
        options.width = width;
        options.height = height;
        options.fov = fov;
        options.projection = projection;
        options.trim_percent = trim_percent;
        options.color_mode = color_mode;
        options.views = views;
        const auto results = kpt::renderMultiView(frame.cloud, options, stop);
        if (!results.empty()) {
          const std::string summary =
              renderStatsSummary(results.front().cloud_stats);
          ui_.post([this, index, summary] {
            log("Frame " + std::to_string(index) + ": " + summary);
          });
        }
        for (std::size_t result_index = 0; result_index < results.size();
             ++result_index) {
          if (stop.stop_requested())
            return;
          const auto &result = results[result_index];
          auto output = prefix;
          output += "_";
          output += frame.path.stem().native();
          output += "_" + result.view_name + ".png";
          static_cast<void>(
              kpt::writeImageAtomic(output, result.image, overwrite, stop));
          report(static_cast<float>(result_index + 1) /
                     static_cast<float>(std::max<std::size_t>(1, views.size())),
                 result.view_name);
        }
      });
}

void App::installSyntheticSmokeSnapshot() {
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
  main_style_.point_size = 20.0F;
  main_style_.color_by = ColorBy::RGB;
  main_viewport_.setStyle(main_style_);
  const auto generation = main_viewport_.beginRequest();
  static_cast<void>(
      main_viewport_.accept(makeViewportCloudSnapshot(cloud, generation)));
}

} // namespace kpt::gui
