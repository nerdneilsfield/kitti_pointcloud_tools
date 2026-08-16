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
#include "gui/inspection_share.hpp"
#include "gui/measurement_overlay.hpp"
#ifndef KPT_WEB_BUILD
#include "gui/dialog_paths.hpp"
#include "gui/inspection_export.hpp"
#include "gui/roi_filter.hpp"
#include "gui/viewport/capture.hpp"
#endif
#include "gui/viewport/cloud_adapter.hpp"
#include "gui/viewport/scene_compositor.hpp"
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
#include "kpt/cancellation.hpp"
#include "kpt/render/render.hpp"
#include "platform/utf8_path.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <iomanip>
#include <limits>
#include <Eigen/SVD>
#include <spdlog/spdlog.h>
#include <sstream>
#include <stdexcept>
#include <utility>

#if defined(__linux__) && !defined(KPT_WEB_BUILD)
#include <sys/sysinfo.h>
#endif

namespace kpt::gui {
namespace {

constexpr std::array<Format, 11> kFormats = {
    Format::Bin,  Format::PCD,    Format::PLY,    Format::LAS,
    Format::PTS,  Format::OBJ,    Format::NPY,    Format::XYZ,
    Format::XYZI, Format::XYZRGB, Format::XYZRGBI};
constexpr std::array<Format, 4> kAsciiFormats = {
    Format::XYZ, Format::XYZI, Format::XYZRGB, Format::XYZRGBI};
constexpr std::array<View, 10> kViews = {
    View::Front,         View::Right,        View::Back,
    View::Left,          View::Top,          View::Bottom,
    View::TopRightFront, View::TopLeftFront, View::BotRightFront,
    View::BotLeftFront};
constexpr const char *kRenderColorModes = "Auto\0RGB\0Intensity\0Z\0Solid\0";
constexpr const char *kRenderProjections = "Orthographic\0Perspective\0";
constexpr std::array<ColorBy, 4> kReviewColorModes = {
    ColorBy::Intensity, ColorBy::RGB, ColorBy::Z, ColorBy::None};
constexpr const char *kReviewColorModeNames = "Intensity\0RGB\0Z\0Fixed\0";

[[nodiscard]] int reviewColorModeIndex(ColorBy color_by) noexcept {
  for (std::size_t index = 0; index < kReviewColorModes.size(); ++index) {
    if (kReviewColorModes[index] == color_by) {
      return static_cast<int>(index);
    }
  }
  // PointT stores no label field. Legacy Label styling is already RGB, so
  // preserve the closest visible result rather than exposing a false option.
  return 1;
}

[[nodiscard]] ColorBy reviewColorMode(int index) noexcept {
  const auto clamped = std::clamp(index, 0,
                                  static_cast<int>(kReviewColorModes.size() - 1));
  return kReviewColorModes[static_cast<std::size_t>(clamped)];
}

struct LayerTransformControls {
  Eigen::Vector3d translation = Eigen::Vector3d::Zero();
  Eigen::Vector3d rotation_degrees = Eigen::Vector3d::Zero();
  Eigen::Vector3d scale = Eigen::Vector3d::Ones();
};

[[nodiscard]] LayerTransformControls
decomposeLayerTransform(const Eigen::Affine3d &transform) {
  LayerTransformControls result;
  result.translation = transform.translation();
  const Eigen::Matrix3d linear = transform.linear();
  Eigen::JacobiSVD<Eigen::Matrix3d> decomposition(
      linear, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix3d rotation = decomposition.matrixU() *
                             decomposition.matrixV().transpose();
  if (rotation.determinant() < 0.0) {
    Eigen::Matrix3d u = decomposition.matrixU();
    u.col(2) *= -1.0;
    rotation = u * decomposition.matrixV().transpose();
  }
  const Eigen::Matrix3d scale_shear = rotation.transpose() * linear;
  result.scale = scale_shear.diagonal();
  for (Eigen::Index axis = 0; axis < result.scale.size(); ++axis) {
    if (!std::isfinite(result.scale[axis])) {
      result.scale[axis] = 1.0;
    }
  }
  const Eigen::Vector3d zyx = rotation.eulerAngles(2, 1, 0);
  constexpr double radians_to_degrees = 180.0 / 3.14159265358979323846;
  result.rotation_degrees = {zyx.z() * radians_to_degrees,
                             zyx.y() * radians_to_degrees,
                             zyx.x() * radians_to_degrees};
  return result;
}

[[nodiscard]] Eigen::Affine3d
composeLayerTransform(const LayerTransformControls &controls) {
  constexpr double degrees_to_radians = 3.14159265358979323846 / 180.0;
  const Eigen::Vector3d radians = controls.rotation_degrees * degrees_to_radians;
  const Eigen::Matrix3d rotation =
      (Eigen::AngleAxisd(radians.z(), Eigen::Vector3d::UnitZ()) *
       Eigen::AngleAxisd(radians.y(), Eigen::Vector3d::UnitY()) *
       Eigen::AngleAxisd(radians.x(), Eigen::Vector3d::UnitX()))
          .toRotationMatrix();
  Eigen::Affine3d result = Eigen::Affine3d::Identity();
  result.linear() = rotation * controls.scale.asDiagonal();
  result.translation() = controls.translation;
  return result;
}

void beginSceneTransactionForActiveWidget(Scene &scene) {
  if (ImGui::IsItemActivated() && !scene.transactionActive()) {
    static_cast<void>(scene.beginTransaction());
  }
}

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

std::string sourceKeyForPath(const std::filesystem::path &path) {
  std::error_code error;
  const auto absolute = std::filesystem::absolute(path, error);
  const auto candidate = error ? path : absolute;
  const auto canonical = std::filesystem::weakly_canonical(candidate, error);
  const auto normalized = error ? candidate.lexically_normal() : canonical;
  return pathSourceKey(normalized, {});
}

[[nodiscard]] std::optional<std::uint64_t>
availableSystemMemoryBytes() noexcept {
#if defined(__linux__) && !defined(KPT_WEB_BUILD)
  struct sysinfo info {};
  if (sysinfo(&info) != 0 || info.mem_unit == 0) {
    return std::nullopt;
  }
  const auto unit = static_cast<std::uint64_t>(info.mem_unit);
  const auto free_bytes = static_cast<std::uint64_t>(info.freeram);
  const auto cache_bytes = static_cast<std::uint64_t>(info.bufferram);
  if (free_bytes > std::numeric_limits<std::uint64_t>::max() - cache_bytes ||
      free_bytes + cache_bytes > std::numeric_limits<std::uint64_t>::max() / unit) {
    return std::numeric_limits<std::uint64_t>::max();
  }
  return (free_bytes + cache_bytes) * unit;
#else
  // Other native backends deliberately fall back to LayerAdmissionConfig's
  // conservative 512 MiB policy rather than misreporting total RAM as free.
  return std::nullopt;
#endif
}

#ifndef KPT_WEB_BUILD
RoiBox wholeFinitePointWorldRoi() {
  constexpr double coordinate_limit =
      static_cast<double>(std::numeric_limits<float>::max());
  return {Eigen::Vector3d::Constant(-coordinate_limit),
          Eigen::Vector3d::Constant(coordinate_limit)};
}
#endif

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

std::string substitute(std::string text, std::string_view placeholder,
                       std::string_view value) {
  if (const auto position = text.find(placeholder);
      position != std::string::npos)
    text.replace(position, placeholder.size(), value);
  return text;
}

template <typename Value>
std::string translatedValue(std::string_view key, std::string_view placeholder,
                            const Value &value) {
  std::ostringstream rendered;
  rendered << value;
  return substitute(kpt::i18n::tr(key), placeholder, rendered.str());
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
    scale << kpt::i18n::tr("gui.viewport.grid_prefix") << std::setprecision(3)
          << grid_spacing << kpt::i18n::tr("gui.viewport.grid_suffix");
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

void drawMeasurementOverlay(const MeasurementOverlay &overlay,
                            ImDrawList &draw_list,
                            const ImVec2 &image_position,
                            const ImVec2 &image_size) {
  if ((overlay.markers.empty() && overlay.segments.empty()) ||
      image_size.x <= 0.0F || image_size.y <= 0.0F) {
    return;
  }
  const auto screen = [&image_position, &image_size](
                          const Eigen::Vector2f &normalized) {
    return image_position +
           ImVec2(normalized.x() * image_size.x, normalized.y() * image_size.y);
  };
  constexpr ImU32 kAttachedColour = IM_COL32(91, 210, 255, 245);
  constexpr ImU32 kDetachedColour = IM_COL32(255, 174, 63, 245);
  constexpr ImU32 kOutlineColour = IM_COL32(8, 14, 22, 230);
  constexpr ImU32 kTextColour = IM_COL32(245, 250, 255, 245);

  draw_list.PushClipRect(image_position, image_position + image_size, true);
  for (const MeasurementOverlaySegment &segment : overlay.segments) {
    const ImVec2 first = screen(segment.first_normalized_position);
    const ImVec2 second = screen(segment.second_normalized_position);

    std::ostringstream label;
    label << std::setprecision(5) << segment.distance;
    label << " m";
    const ImVec2 center{(first.x + second.x) * 0.5F,
                        (first.y + second.y) * 0.5F};
    const ImVec2 text_size = ImGui::CalcTextSize(label.str().c_str());
    const ImVec2 label_min = center - text_size * 0.5F - ImVec2(4.0F, 2.0F);
    const ImVec2 label_max = center + text_size * 0.5F + ImVec2(4.0F, 2.0F);
    draw_list.AddRectFilled(label_min, label_max, kOutlineColour, 3.0F);
    draw_list.AddText(label_min + ImVec2(4.0F, 2.0F), kTextColour,
                      label.str().c_str());
  }
  for (const MeasurementOverlayMarker &marker : overlay.markers) {
    const ImVec2 position = screen(marker.normalized_position);
    const ImU32 colour = marker.detached ? kDetachedColour : kAttachedColour;
    if (marker.pending || marker.detached) {
      std::string label = marker.second_endpoint ? "P2" : "P1";
      if (marker.pending)
        label += " (pick P2)";
      else if (marker.detached)
        label += " (detached)";
      draw_list.AddText(position + ImVec2(8.0F, -18.0F), colour,
                        label.c_str());
    }
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
         unsigned max_workers, std::shared_ptr<web::AssetStager> asset_stager,
         std::filesystem::path inspection_settings_path)
    : main_viewport_(std::move(main_renderer)),
      trajectory_viewport_(std::move(trajectory_renderer)), jobs_(max_workers),
      asset_stager_(std::move(asset_stager)),
      inspection_settings_file_(std::move(inspection_settings_path)),
      inspection_settings_enabled_(!inspection_settings_file_.path().empty()) {
#ifdef KPT_WEB_BUILD
  jobs_.setWorkerLimit(jobs_.maxWorkers());
#endif
  reset_dock_layout_ = true;
  loadInspectionSettings();
}

App::~App() {
  persistInspectionSettings();
  playback_.stop();
  jobs_.setPlayerActive(false);
  jobs_.cancelAll();
}

void App::loadInspectionSettings() {
  if (!inspection_settings_enabled_)
    return;
  std::string error;
  if (!inspection_settings_file_.load(inspection_settings_, &error)) {
    log("Inspection settings load failed: " + error);
    return;
  }
  const auto &bookmarks = inspection_settings_.bookmarks();
  if (bookmarks.empty())
    return;
  // Restore only after an accepted cloud has performed its default fit.  The
  // viewport validates this untrusted persisted input at that boundary.
  pending_initial_camera_snapshot_ = bookmarks.back().camera();
}

void App::persistInspectionSettings() {
  if (!inspection_settings_enabled_)
    return;
  std::string error;
  if (!inspection_settings_file_.save(inspection_settings_, &error))
    log("Inspection settings save failed: " + error);
}

std::vector<std::string> App::takeLaunchWarnings() {
  auto warnings = std::move(launch_warnings_);
  launch_warnings_.clear();
  return warnings;
}

void App::setStartupStyle(const ViewportStyle &style) {
  main_style_ = style;
  color_by_ = reviewColorModeIndex(style.color_by);
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
  playback_.configure(fps, autoplay);
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
  playback_.configure(fps, autoplay);
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
#ifdef KPT_WEB_BUILD
  for (const web::ViewportPngDownloadResult &result :
       web::takeViewportPngDownloadResults()) {
    if (result.error) {
      log("Viewport PNG download failed " + result.filename + ": " +
          *result.error);
    } else {
      // Browser handoff completed. The browser still owns final download
      // policy and naming, so do not claim that a file was written here.
      log("Viewport PNG download started: " + result.filename);
    }
  }
#endif
  updatePlayback();
  drawDockspace();
  drawTools();
  drawInspector();
  handleInspectionUndoRedo();
  refreshInspectionViewportIfRoiDue();
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
  if (playback_.playing() || launch_state_ == LaunchState::Pending ||
      !frame_cache_.pendingEmpty()) {
    return true;
  }
  const bool inspection_work = inspection_roi_preview_pending_ ||
                               inspection_upload_retry_pending_ ||
                               inspection_screenshot_request_.has_value();
#ifdef KPT_WEB_BUILD
  return inspection_work || web::hasViewportPngDownloadActivity() ||
         jobs_.hasActiveJobs();
#else
  return inspection_work || jobs_.hasActiveJobs();
#endif
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
    const auto selected = substitute(kpt::i18n::tr("gui.viewer.selected"), "%s",
                                     displayPath(selection.viewer->filename()));
    ImGui::TextWrapped("%s", selected.c_str());
    if (ImGui::Button(kpt::i18n::tr("gui.viewer.load"))) {
      if (asset_stager_) {
        const auto path = *selection.viewer;
        asset_stager_->stage({path},
                             [this, path](std::optional<std::string> error) {
                               ui_.post([this, path, error = std::move(error)] {
                                 if (error)
                                   log("Point-cloud staging error: " + *error);
                                 else
                                   startViewer(path);
                               });
                             });
      } else {
        startViewer(*selection.viewer);
      }
    }
  } else {
    ImGui::TextDisabled("%s", kpt::i18n::tr("gui.viewer.no_cloud"));
  }
  if (!selection.error.empty()) {
    const auto error = substitute(kpt::i18n::tr("gui.viewer.selection_error"),
                                  "%s", selection.error);
    ImGui::TextWrapped("%s", error.c_str());
  }
#else
  if (pathInput(kpt::i18n::tr("gui.viewer.input_label"), "##viewer-input",
                viewer_input_, "...##viewer")) {
    openDialog(DialogTarget::ViewerInput,
               kpt::i18n::tr("gui.viewer.dialog_open"), false, false,
               viewer_input_);
  }
  if (ImGui::Button(kpt::i18n::tr("gui.viewer.load")) &&
      !viewer_input_.empty()) {
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
  ImGui::TextUnformatted(
      translatedValue("gui.player.selected_count", "%zu", selection.cloud_count)
          .c_str());
  if (ImGui::Button(kpt::i18n::tr("gui.player.choose_labels")))
    web::openPicker(web::PickerKind::Labels);
  ImGui::SameLine();
  ImGui::TextUnformatted(
      translatedValue("gui.player.selected_count", "%zu", selection.label_count)
          .c_str());
  if (ImGui::Button(kpt::i18n::tr("gui.player.choose_poses")))
    web::openPicker(web::PickerKind::Poses);
  ImGui::SameLine();
  ImGui::TextDisabled("%s", selection.has_poses
                                ? kpt::i18n::tr("gui.player.poses_selected")
                                : kpt::i18n::tr("gui.player.poses_optional"));
  if (ImGui::Button(kpt::i18n::tr("gui.player.choose_poses2")))
    web::openPicker(web::PickerKind::Poses2);
  ImGui::SameLine();
  ImGui::TextDisabled("%s", selection.has_poses2
                                ? kpt::i18n::tr("gui.player.poses_selected")
                                : kpt::i18n::tr("gui.player.poses_optional"));
  if (!selection.error.empty()) {
    const auto error = substitute(kpt::i18n::tr("gui.viewer.selection_error"),
                                  "%s", selection.error);
    ImGui::TextWrapped("%s", error.c_str());
  }
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
                               ui_.post([this, source = std::move(source),
                                         error = std::move(error)]() mutable {
                                 if (error)
                                   log("Pose staging error: " + *error);
                                 else
                                   openSequence(std::move(source));
                               });
                             });
      }
    } else {
      openSequence(std::move(built.source));
    }
  }
#else
  if (pathInput(kpt::i18n::tr("gui.player.directory_label"),
                "##player-dir-input", player_input_dir_, "...##player-dir")) {
    openDialog(DialogTarget::PlayerInputDir,
               kpt::i18n::tr("gui.player.dialog_open_dir"), true, false,
               player_input_dir_);
  }
  ImGui::InputText(kpt::i18n::tr("gui.player.glob"), &player_glob_);
  if (pathInput(kpt::i18n::tr("gui.player.labels_label"),
                "##player-labels-input", player_label_dir_, "...##labels")) {
    openDialog(DialogTarget::PlayerLabelDir,
               kpt::i18n::tr("gui.player.dialog_open_labels"), true, false,
               player_label_dir_);
  }
  if (pathInput(kpt::i18n::tr("gui.player.poses_label"), "##player-poses-input",
                player_poses_, "...##poses")) {
    openDialog(DialogTarget::PlayerPoses,
               kpt::i18n::tr("gui.player.dialog_open_poses"), false, false,
               player_poses_);
  }
  if (pathInput(kpt::i18n::tr("gui.player.poses2_label"),
                "##player-poses2-input", player_poses2_, "...##poses2")) {
    openDialog(DialogTarget::PlayerPoses2,
               kpt::i18n::tr("gui.player.dialog_open_poses2"), false, false,
               player_poses2_);
  }
  if (ImGui::Button(kpt::i18n::tr("gui.player.open_sequence")) &&
      !player_input_dir_.empty()) {
    openSequence();
  }
#endif

  if (!sequence_ || sequence_->empty())
    return;
  ImGui::Separator();
  const bool playing_forward =
      playback_.playing() &&
      playback_.direction() == PlaybackDirection::Forward;
  const bool playing_reverse =
      playback_.playing() &&
      playback_.direction() == PlaybackDirection::Reverse;
  if (ImGui::Button(playing_forward ? kpt::i18n::tr("gui.player.pause_forward")
                                    : kpt::i18n::tr("gui.player.play"))) {
    togglePlayback(PlaybackDirection::Forward);
  }
  ImGui::SameLine();
  if (ImGui::Button(playing_reverse ? kpt::i18n::tr("gui.player.pause_reverse")
                                    : kpt::i18n::tr("gui.player.reverse"))) {
    togglePlayback(PlaybackDirection::Reverse);
  }
  ImGui::SameLine();
  if (ImGui::Button(kpt::i18n::tr("gui.player.reset"))) {
    resetPlayback();
  }
  if (ImGui::Button(kpt::i18n::tr("gui.player.previous")) &&
      playback_.current() > 0) {
    requestFrame(playback_.current() - 1, true);
  }
  ImGui::SameLine();
  if (ImGui::Button(kpt::i18n::tr("gui.player.next")) &&
      playback_.current() + 1 < sequence_->size()) {
    requestFrame(playback_.current() + 1, true);
  }
  int frame = static_cast<int>(playback_.desired());
  const int maximum = static_cast<int>(sequence_->size() - 1);
  if (ImGui::SliderInt(kpt::i18n::tr("gui.player.frame"), &frame, 0, maximum)) {
    requestFrame(static_cast<std::size_t>(frame), true);
  }
  ImGui::SliderInt(kpt::i18n::tr("gui.player.fps"),
                   &playback_.fpsControl(), 1, 120);
  ImGui::Checkbox(kpt::i18n::tr("gui.player.loop"),
                  &playback_.loopControl());
#ifndef KPT_WEB_BUILD
  if (ImGui::CollapsingHeader(kpt::i18n::tr("gui.player.snapshot_export"))) {
    if (pathInput(kpt::i18n::tr("gui.player.prefix"),
                  "##player-snapshot-prefix", player_snapshot_prefix_,
                  "...##player-snapshot")) {
      openDialog(DialogTarget::PlayerSnapshotPrefix,
                 kpt::i18n::tr("gui.player.snapshot_export"), false, true,
                 player_snapshot_prefix_);
    }
    ImGui::InputInt(kpt::i18n::tr("gui.player.snapshot_width"), &render_width_);
    ImGui::InputInt(kpt::i18n::tr("gui.player.snapshot_height"),
                    &render_height_);
    ImGui::Combo(kpt::i18n::tr("gui.player.snapshot_projection"),
                 &render_projection_, kRenderProjections);
    ImGui::InputFloat(kpt::i18n::tr("gui.player.snapshot_trim"),
                      &render_trim_percent_);
    if (render_projection_ == 1)
      ImGui::InputFloat(kpt::i18n::tr("gui.player.snapshot_fov"), &render_fov_);
    ImGui::Combo(kpt::i18n::tr("gui.player.snapshot_color_by"),
                 &render_color_mode_, kRenderColorModes);
    ImGui::Checkbox(kpt::i18n::tr("gui.player.snapshot_overwrite"),
                    &render_overwrite_);
    if (ImGui::Button(kpt::i18n::tr("gui.player.export_snapshots")) &&
        !player_snapshot_prefix_.empty()) {
      queueRender(true);
    }
  }
#endif
}

void App::drawConvertControls() {
  if (pathInput(kpt::i18n::tr("gui.convert.input_label"),
                "##convert-input-path", convert_input_, "...##convert-input")) {
    openDialog(DialogTarget::ConvertInput,
               kpt::i18n::tr("gui.convert.dialog_open"), false, false,
               convert_input_);
  }
  if (pathInput(kpt::i18n::tr("gui.convert.output_label"),
                "##convert-output-path", convert_output_,
                "...##convert-output")) {
    openDialog(DialogTarget::ConvertOutput,
               kpt::i18n::tr("gui.convert.dialog_save"), false, true,
               convert_output_);
  }
  constexpr const char *ascii_items =
      "From extension\0xyz\0xyzi\0xyzrgb\0xyzrgbi\0";
  ImGui::Combo(kpt::i18n::tr("gui.convert.ascii_flavor"), &convert_ascii_,
               ascii_items);
  ImGui::Checkbox(kpt::i18n::tr("gui.convert.overwrite"), &convert_overwrite_);
  if (ImGui::Button(kpt::i18n::tr("gui.convert.queue")) &&
      !convert_input_.empty() && !convert_output_.empty()) {
    queueSingleConversion();
  }
}

void App::drawBatchControls() {
  if (pathInput(kpt::i18n::tr("gui.batch.input_label"),
                "##batch-input-directory", batch_input_dir_,
                "...##batch-input")) {
    openDialog(DialogTarget::BatchInputDir,
               kpt::i18n::tr("gui.batch.dialog_open_input"), true, false,
               batch_input_dir_);
  }
  if (pathInput(kpt::i18n::tr("gui.batch.output_label"),
                "##batch-output-directory", batch_output_dir_,
                "...##batch-output")) {
    openDialog(DialogTarget::BatchOutputDir,
               kpt::i18n::tr("gui.batch.dialog_open_output"), true, false,
               batch_output_dir_);
  }
  ImGui::InputText(kpt::i18n::tr("gui.batch.glob"), &batch_glob_);
  constexpr const char *formats =
      "bin\0pcd\0ply\0las\0pts\0obj\0npy\0xyz\0xyzi\0xyzrgb\0xyzrgbi\0";
  ImGui::Combo(kpt::i18n::tr("gui.batch.output_format"), &batch_format_,
               formats);
  constexpr const char *ascii_items =
      "From output format\0xyz\0xyzi\0xyzrgb\0xyzrgbi\0";
  ImGui::Combo(kpt::i18n::tr("gui.batch.ascii_flavor"), &batch_ascii_,
               ascii_items);
  ImGui::Checkbox(kpt::i18n::tr("gui.batch.overwrite"), &batch_overwrite_);
  if (ImGui::Button(kpt::i18n::tr("gui.batch.queue")) &&
      !batch_input_dir_.empty() && !batch_output_dir_.empty()) {
    queueBatchConversion();
  }
}

void App::drawRenderControls() {
  if (pathInput(kpt::i18n::tr("gui.render.input_label"), "##render-input-path",
                render_input_, "...##render-input")) {
    openDialog(DialogTarget::RenderInput,
               kpt::i18n::tr("gui.render.dialog_open"), false, false,
               render_input_);
  }
  if (pathInput(kpt::i18n::tr("gui.render.output_prefix_label"),
                "##render-output-prefix", render_output_prefix_,
                "...##render-prefix")) {
    openDialog(DialogTarget::RenderOutputPrefix,
               kpt::i18n::tr("gui.render.dialog_save"), false, true,
               render_output_prefix_);
  }
  ImGui::InputInt(kpt::i18n::tr("gui.render.width"), &render_width_);
  ImGui::InputInt(kpt::i18n::tr("gui.render.height"), &render_height_);
  ImGui::Combo(kpt::i18n::tr("gui.render.projection"), &render_projection_,
               kRenderProjections);
  ImGui::InputFloat(kpt::i18n::tr("gui.render.trim"), &render_trim_percent_);
  if (render_projection_ == 1)
    ImGui::InputFloat(kpt::i18n::tr("gui.render.fov"), &render_fov_);
  ImGui::Combo(kpt::i18n::tr("gui.render.color_by"), &render_color_mode_,
               kRenderColorModes);
  ImGui::Checkbox(kpt::i18n::tr("gui.render.overwrite"), &render_overwrite_);
  for (std::size_t index = 0; index < kViews.size(); ++index) {
    const std::string view_name(kpt::viewName(kViews[index]));
    ImGui::Checkbox(view_name.c_str(), &render_views_[index]);
    if (index % 2 == 0)
      ImGui::SameLine();
  }
  if (ImGui::Button(kpt::i18n::tr("gui.render.queue")) &&
      !render_input_.empty() && !render_output_prefix_.empty()) {
    queueRender(false);
  }
}

void App::drawLayerControls() {
  ImGui::SeparatorText("Layers");
#ifndef KPT_WEB_BUILD
  if (ImGui::Button("Add cloud...##inspection-layer")) {
    openDialog(DialogTarget::InspectionLayerInput, "Add point-cloud layer",
               false, false, viewer_input_);
  }
#endif
  ImGui::SameLine();
  if (ImGui::Button("Fit visible##inspection-layer"))
    fitInspectionVisible();
  ImGui::SameLine();
  if (ImGui::Button("Fit active##inspection-layer"))
    fitInspectionActive();

  if (ImGui::Button("Undo##inspection-layer")) {
    if (inspection_scene_.undo())
      refreshAfterInspectionHistoryChange();
  }
  ImGui::SameLine();
  if (ImGui::Button("Redo##inspection-layer")) {
    if (inspection_scene_.redo())
      refreshAfterInspectionHistoryChange();
  }

  const auto active_layer = inspection_scene_.activeLayer();
  ImGui::TextDisabled("%zu layer(s); %s picking", inspection_scene_.layers().size(),
                      inspection_render_list_ &&
                              inspection_render_list_->pick_scope ==
                                  LayerPickScope::ActiveLayerOnly
                          ? "active-only"
                          : "visible-layer");
  if (inspection_render_list_) {
    ImGui::TextDisabled("GPU estimate: %.1f MiB",
                        static_cast<double>(inspection_render_list_->estimated_gpu_bytes) /
                            (1024.0 * 1024.0));
  }

  std::optional<LayerId> remove_layer;
  bool refresh = false;
  for (const CloudLayer &layer : inspection_scene_.layers()) {
    const std::string id = std::to_string(layer.id());
    ImGui::PushID(id.c_str());
    bool visible = layer.visible();
    if (ImGui::Checkbox("##visible", &visible)) {
      beginSceneTransactionForActiveWidget(inspection_scene_);
      static_cast<void>(inspection_scene_.setLayerVisible(layer.id(), visible));
      refresh = true;
    }
    ImGui::SameLine();
    const bool selected = active_layer && *active_layer == layer.id();
    if (ImGui::Selectable(layer.sourceKey().c_str(), selected,
                          ImGuiSelectableFlags_SpanAvailWidth)) {
      beginSceneTransactionForActiveWidget(inspection_scene_);
      static_cast<void>(inspection_scene_.setActiveLayer(layer.id()));
      refresh = true;
    }
    if (!layer.cloud()) {
      ImGui::TextDisabled("Unresolved source (loading or unavailable)");
    }

    if (ImGui::TreeNode("Layer settings")) {
      LayerStyle style = layer.style();
      bool style_changed = false;
      int color_by = reviewColorModeIndex(style.color_by);
      if (ImGui::Combo("Color", &color_by, kReviewColorModeNames)) {
        beginSceneTransactionForActiveWidget(inspection_scene_);
        style.color_by = reviewColorMode(color_by);
        style_changed = true;
      }
      constexpr const char *color_maps =
          "Turbo\0Viridis\0Plasma\0Inferno\0Magma\0Grayscale\0Hot\0Jet\0Spring\0Autumn\0";
      int color_map = static_cast<int>(style.color_map);
      if (ImGui::Combo("Color map", &color_map, color_maps)) {
        beginSceneTransactionForActiveWidget(inspection_scene_);
        style.color_map = static_cast<ColorMap>(color_map);
        style_changed = true;
      }
      const bool point_size_changed = ImGui::SliderFloat(
          "Point size", &style.point_size, 0.1F, 5.0F, "%.2f");
      beginSceneTransactionForActiveWidget(inspection_scene_);
      style_changed |= point_size_changed;
      const bool opacity_changed =
          ImGui::SliderFloat("Opacity", &style.opacity, 0.0F, 1.0F, "%.2f");
      beginSceneTransactionForActiveWidget(inspection_scene_);
      style_changed |= opacity_changed;
      const bool fixed_colour_changed =
          ImGui::ColorEdit3("Fixed colour", style.fixed_color.data());
      beginSceneTransactionForActiveWidget(inspection_scene_);
      style_changed |= fixed_colour_changed;
      const bool highlight_noise_changed =
          ImGui::Checkbox("Highlight noise", &style.highlight_noise);
      beginSceneTransactionForActiveWidget(inspection_scene_);
      style_changed |= highlight_noise_changed;
      if (style.highlight_noise) {
        const bool noise_colour_changed =
            ImGui::ColorEdit3("Noise colour", style.noise_color.data());
        beginSceneTransactionForActiveWidget(inspection_scene_);
        style_changed |= noise_colour_changed;
      }
      const bool equalize_changed =
          ImGui::Checkbox("Equalize intensity", &style.intensity_equalize);
      beginSceneTransactionForActiveWidget(inspection_scene_);
      style_changed |= equalize_changed;
      if (style_changed) {
        try {
          refresh |= inspection_scene_.setLayerStyle(layer.id(), style);
        } catch (const std::exception &error) {
          log("Invalid layer style: " + std::string(error.what()));
        }
      }

      LayerTransformControls transform =
          decomposeLayerTransform(layer.localToWorld());
      bool transform_changed = false;
      ImGui::TextUnformatted("Translation (world)");
      const bool translation_x =
          ImGui::InputDouble("X##translation", &transform.translation.x());
      beginSceneTransactionForActiveWidget(inspection_scene_);
      transform_changed |= translation_x;
      ImGui::SameLine();
      const bool translation_y =
          ImGui::InputDouble("Y##translation", &transform.translation.y());
      beginSceneTransactionForActiveWidget(inspection_scene_);
      transform_changed |= translation_y;
      ImGui::SameLine();
      const bool translation_z =
          ImGui::InputDouble("Z##translation", &transform.translation.z());
      beginSceneTransactionForActiveWidget(inspection_scene_);
      transform_changed |= translation_z;
      ImGui::TextUnformatted("Rotation (XYZ degrees)");
      const bool rotation_x =
          ImGui::InputDouble("X##rotation", &transform.rotation_degrees.x());
      beginSceneTransactionForActiveWidget(inspection_scene_);
      transform_changed |= rotation_x;
      ImGui::SameLine();
      const bool rotation_y =
          ImGui::InputDouble("Y##rotation", &transform.rotation_degrees.y());
      beginSceneTransactionForActiveWidget(inspection_scene_);
      transform_changed |= rotation_y;
      ImGui::SameLine();
      const bool rotation_z =
          ImGui::InputDouble("Z##rotation", &transform.rotation_degrees.z());
      beginSceneTransactionForActiveWidget(inspection_scene_);
      transform_changed |= rotation_z;
      ImGui::TextUnformatted("Scale (local axes)");
      const bool scale_x = ImGui::InputDouble("X##scale", &transform.scale.x());
      beginSceneTransactionForActiveWidget(inspection_scene_);
      transform_changed |= scale_x;
      ImGui::SameLine();
      const bool scale_y = ImGui::InputDouble("Y##scale", &transform.scale.y());
      beginSceneTransactionForActiveWidget(inspection_scene_);
      transform_changed |= scale_y;
      ImGui::SameLine();
      const bool scale_z = ImGui::InputDouble("Z##scale", &transform.scale.z());
      beginSceneTransactionForActiveWidget(inspection_scene_);
      transform_changed |= scale_z;
      if (transform_changed) {
        try {
          refresh |= inspection_scene_.setLayerTransform(
              layer.id(), composeLayerTransform(transform));
        } catch (const std::exception &error) {
          log("Invalid layer transform: " + std::string(error.what()));
        }
      }
      if (ImGui::SmallButton("Reset transform")) {
        beginSceneTransactionForActiveWidget(inspection_scene_);
        refresh |= inspection_scene_.setLayerTransform(
            layer.id(), Eigen::Affine3d::Identity());
      }
      ImGui::SameLine();
      if (ImGui::SmallButton("Remove layer")) {
        if (inspection_scene_.transactionActive())
          static_cast<void>(inspection_scene_.commitTransaction());
        remove_layer = layer.id();
      }
      ImGui::TreePop();
    }
    ImGui::PopID();
    if (remove_layer)
      break;
  }
  if (remove_layer) {
    if (inspection_scene_.removeLayer(*remove_layer)) {
      refresh = true;
    }
  }
  if (inspection_scene_.transactionActive() && !ImGui::IsAnyItemActive()) {
    static_cast<void>(inspection_scene_.commitTransaction());
    refresh = true;
  }
  if (refresh) {
    inspection_undo_domain_ = InspectionUndoDomain::Scene;
    refreshInspectionViewport(CameraUpdate::Preserve);
  }
}

void App::drawDisplayControls() {
  drawLayerControls();
  drawBookmarkControls();
  drawInspectionScreenshotControls();
  drawInspectionShareControls();
  drawInspectionRoiAndExportControls();
  ImGui::SeparatorText("Measurement");
  ImGui::TextDisabled("Ctrl + left click: pick up to two points");
  for (const auto &measurement : inspection_scene_.measurements()) {
    const auto &first = measurement.firstWorld();
    ImGui::TextDisabled("P1 source: %s", measurement.firstSourceKey().c_str());
    ImGui::Text("P1: %.5g, %.5g, %.5g", first.x(), first.y(), first.z());
    if (!measurement.secondWorld()) {
      ImGui::TextDisabled("Awaiting second point (any visible layer)");
      continue;
    }

    const auto &second = *measurement.secondWorld();
    const double distance = *measurement.distance();
    if (measurement.secondSourceKey()) {
      ImGui::TextDisabled("P2 source: %s",
                          measurement.secondSourceKey()->c_str());
    }
    ImGui::Text("P2: %.5g, %.5g, %.5g", second.x(), second.y(), second.z());
    ImGui::Text("Distance: %.6g", distance);
    if (inspection_scene_.measurementDetached(measurement)) {
      ImGui::TextColored(ImVec4(1.0F, 0.65F, 0.2F, 1.0F),
                         "Detached: one or more source layers are unavailable");
    }
    ImGui::PushID(static_cast<int>(measurement.id()));
    if (ImGui::Button("Copy measurement")) {
      std::ostringstream text;
      text << std::setprecision(10) << "P1 " << first.x() << ' ' << first.y()
           << ' ' << first.z() << "\nP2 " << second.x() << ' ' << second.y()
           << ' ' << second.z() << "\nDistance " << distance;
      ImGui::SetClipboardText(text.str().c_str());
    }
    ImGui::PopID();
  }
  if (!inspection_scene_.measurements().empty() &&
      ImGui::Button("Clear measurements")) {
    if (inspection_scene_.clearMeasurements())
      inspection_undo_domain_ = InspectionUndoDomain::Scene;
  }
  if (const auto cloud = main_viewport_.cloud()) {
    const auto &bounds = cloud->bounds;
    const Eigen::Vector3d size =
        bounds.maximum.cast<double>() - bounds.minimum.cast<double>();
    ImGui::SeparatorText(kpt::i18n::tr("gui.display.cloud_info"));
    ImGui::TextUnformatted(translatedValue("gui.display.finite_points", "%zu",
                                           bounds.finite_points)
                               .c_str());
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
        row(kpt::i18n::tr("gui.display.row_size"), size.x(), size.y(),
            size.z());
        row(kpt::i18n::tr("gui.display.row_center"), center.x(), center.y(),
            center.z());
        ImGui::EndTable();
      }
      if (bounds.has_noise) {
        auto noise =
            translatedValue("gui.display.noise", "%zu", bounds.noise_points);
        noise = substitute(std::move(noise), "%zu",
                           std::to_string(bounds.finite_points));
        ImGui::TextUnformatted(noise.c_str());
      }
    }
  }
  ImGui::SeparatorText(kpt::i18n::tr("gui.display.section"));
  color_by_ = reviewColorModeIndex(main_style_.color_by);
  if (ImGui::Combo(kpt::i18n::tr("gui.display.color_by"), &color_by_,
                   kReviewColorModeNames)) {
    main_style_.color_by = reviewColorMode(color_by_);
    main_viewport_.setStyle(main_style_);
  }
  if (main_style_.color_by == ColorBy::Intensity) {
    constexpr const char *color_maps =
        "Turbo\0Viridis\0Plasma\0Inferno\0Magma\0Grayscale\0Hot\0Jet\0Spring\0A"
        "utumn\0";
    if (ImGui::Combo(kpt::i18n::tr("gui.display.colormap"), &color_map_,
                     color_maps)) {
      main_style_.color_map = static_cast<ColorMap>(color_map_);
      main_viewport_.setStyle(main_style_);
    }
    if (ImGui::Checkbox(kpt::i18n::tr("gui.display.equalize"), &equalize_)) {
      main_style_.intensity_equalize = equalize_;
      main_viewport_.setStyle(main_style_);
    }
  }
  if (ImGui::SliderFloat(kpt::i18n::tr("gui.display.point_size"), &point_size_,
                         0.0F, 5.0F, "%.2f")) {
    main_style_.point_size = point_size_;
    main_viewport_.setStyle(main_style_);
  }
  if (ImGui::ColorEdit3(kpt::i18n::tr("gui.display.background"), background_)) {
    main_style_.background =
        Eigen::Vector3f(background_[0], background_[1], background_[2]);
    main_viewport_.setStyle(main_style_);
    if (!inspection_scene_.layers().empty())
      refreshInspectionViewport(CameraUpdate::Preserve);
  }
  bool style_changed = false;
  if (main_style_.color_by == ColorBy::None)
    style_changed |= ImGui::ColorEdit3(kpt::i18n::tr("gui.display.fixed_color"),
                                       main_style_.fixed_color.data());
  style_changed |= ImGui::Checkbox(kpt::i18n::tr("gui.display.highlight_noise"),
                                   &main_style_.highlight_noise);
  if (main_style_.highlight_noise)
    style_changed |= ImGui::ColorEdit3(kpt::i18n::tr("gui.display.noise_color"),
                                       main_style_.noise_color.data());
  style_changed |= ImGui::Checkbox(kpt::i18n::tr("gui.display.coordinate_axes"),
                                   &main_style_.show_coordinate_axes);
  style_changed |= ImGui::Checkbox(kpt::i18n::tr("gui.display.scale_grid"),
                                   &main_style_.show_scale_grid);
  if (style_changed)
    main_viewport_.setStyle(main_style_);
  ImGui::Checkbox(kpt::i18n::tr("gui.display.viewport_controls"),
                  &show_viewport_controls_);
  if (ImGui::Button(kpt::i18n::tr("gui.display.fit_all"),
                    {ImGui::GetContentRegionAvail().x, 0.0F})) {
    if (inspection_scene_.layers().empty()) {
      main_viewport_.fit();
    } else {
      fitInspectionVisible();
    }
  }
  if (ImGui::IsItemHovered())
    ImGui::SetTooltip("%s", kpt::i18n::tr("gui.display.fit_tooltip"));
  constexpr std::size_t columns = 3;
  constexpr std::array<std::string_view, 8> camera_labels = {
      "gui.camera.top",  "gui.camera.front", "gui.camera.left",
      "gui.camera.back", "gui.camera.right", "gui.camera.bottom",
      "gui.camera.iso1", "gui.camera.iso2"};
  constexpr std::array<std::string_view, 8> camera_tooltips = {
      "gui.camera.top_tooltip",   "gui.camera.front_tooltip",
      "gui.camera.left_tooltip",  "gui.camera.back_tooltip",
      "gui.camera.right_tooltip", "gui.camera.bottom_tooltip",
      "gui.camera.iso1_tooltip",  "gui.camera.iso2_tooltip"};
  const float button_width =
      std::max(1.0F, (ImGui::GetContentRegionAvail().x -
                      ImGui::GetStyle().ItemSpacing.x *
                          static_cast<float>(columns - 1)) /
                         static_cast<float>(columns));
  for (std::size_t index = 0; index < kCameraPresetButtons.size(); ++index) {
    const auto &button = kCameraPresetButtons[index];
    if (ImGui::Button(kpt::i18n::tr(camera_labels[index]),
                      {button_width, 0.0F})) {
      main_viewport_.setView(button.preset);
      if (!inspection_scene_.layers().empty())
        inspection_layer_order_dirty_ = true;
    }
    if (ImGui::IsItemHovered())
      ImGui::SetTooltip("%s", kpt::i18n::tr(camera_tooltips[index]));
    if ((index + 1) % columns != 0 && index + 1 < kCameraPresetButtons.size())
      ImGui::SameLine();
  }
  if (!inspection_scene_.layers().empty()) {
    // Layer-specific colour/noise settings were baked by the compatibility
    // compositor.  Keep the legacy renderer in RGB mode for the final draw.
    ViewportStyle composed_style = main_style_;
    composed_style.color_by = ColorBy::RGB;
    composed_style.highlight_noise = false;
    main_viewport_.setStyle(composed_style);
  }
}

std::optional<RoiBox> App::inspectionRoiFromControls() const {
  const Eigen::Vector3d minimum{inspection_roi_min_[0], inspection_roi_min_[1],
                                inspection_roi_min_[2]};
  const Eigen::Vector3d maximum{inspection_roi_max_[0], inspection_roi_max_[1],
                                inspection_roi_max_[2]};
  if (!minimum.allFinite() || !maximum.allFinite() ||
      (minimum.array() > maximum.array()).any()) {
    return std::nullopt;
  }
  return RoiBox{minimum, maximum};
}

void App::drawInspectionRoiAndExportControls() {
  if (inspection_roi_controls_need_hydrate_) {
    hydrateInspectionRoiControlsFromScene();
  }
  ImGui::SeparatorText("ROI export");
  const bool enabled_changed =
      ImGui::Checkbox("Enable ROI##inspection", &inspection_roi_enabled_);
  beginSceneTransactionForActiveWidget(inspection_scene_);
  bool roi_changed = false;
  bool roi_final_edit = ImGui::IsItemDeactivatedAfterEdit();
  if (inspection_roi_enabled_) {
    ImGui::TextUnformatted("Min (world)");
    const bool min_x =
        ImGui::InputDouble("X##inspection-roi-min", &inspection_roi_min_[0]);
    beginSceneTransactionForActiveWidget(inspection_scene_);
    roi_changed |= min_x;
    roi_final_edit |= ImGui::IsItemDeactivatedAfterEdit();
    ImGui::SameLine();
    const bool min_y =
        ImGui::InputDouble("Y##inspection-roi-min", &inspection_roi_min_[1]);
    beginSceneTransactionForActiveWidget(inspection_scene_);
    roi_changed |= min_y;
    roi_final_edit |= ImGui::IsItemDeactivatedAfterEdit();
    ImGui::SameLine();
    const bool min_z =
        ImGui::InputDouble("Z##inspection-roi-min", &inspection_roi_min_[2]);
    beginSceneTransactionForActiveWidget(inspection_scene_);
    roi_changed |= min_z;
    roi_final_edit |= ImGui::IsItemDeactivatedAfterEdit();
    ImGui::TextUnformatted("Max (world)");
    const bool max_x =
        ImGui::InputDouble("X##inspection-roi-max", &inspection_roi_max_[0]);
    beginSceneTransactionForActiveWidget(inspection_scene_);
    roi_changed |= max_x;
    roi_final_edit |= ImGui::IsItemDeactivatedAfterEdit();
    ImGui::SameLine();
    const bool max_y =
        ImGui::InputDouble("Y##inspection-roi-max", &inspection_roi_max_[1]);
    beginSceneTransactionForActiveWidget(inspection_scene_);
    roi_changed |= max_y;
    roi_final_edit |= ImGui::IsItemDeactivatedAfterEdit();
    ImGui::SameLine();
    const bool max_z =
        ImGui::InputDouble("Z##inspection-roi-max", &inspection_roi_max_[2]);
    beginSceneTransactionForActiveWidget(inspection_scene_);
    roi_changed |= max_z;
    roi_final_edit |= ImGui::IsItemDeactivatedAfterEdit();
  }

  const auto roi = inspectionRoiFromControls();
  bool scene_roi_changed = false;
  if (!inspection_roi_enabled_) {
    if (enabled_changed) {
      inspection_scene_.setRoi(std::nullopt);
      scene_roi_changed = true;
      scheduleInspectionRoiPreview(roi_final_edit);
    }
  } else if (roi) {
    if (enabled_changed || roi_changed || !inspection_scene_.roi()) {
      inspection_scene_.setRoi(*roi);
      scene_roi_changed = true;
      scheduleInspectionRoiPreview(roi_final_edit);
    }
  } else {
    // Do not leave a stale valid ROI active after a malformed edit.
    if (enabled_changed || roi_changed || inspection_scene_.roi()) {
      inspection_scene_.setRoi(std::nullopt);
      scene_roi_changed = true;
      scheduleInspectionRoiPreview(roi_final_edit);
    }
    ImGui::TextColored(ImVec4(1.0F, 0.35F, 0.35F, 1.0F),
                       "ROI requires finite min <= max on every axis");
  }

  if (inspection_scene_.transactionActive() && !ImGui::IsAnyItemActive()) {
    static_cast<void>(inspection_scene_.commitTransaction());
  }
  if (scene_roi_changed) {
    inspection_undo_domain_ = InspectionUndoDomain::Scene;
  }
  if (roi_final_edit) {
    scheduleInspectionRoiPreview(true);
  }

#ifndef KPT_WEB_BUILD
  if (pathInput("Export point cloud", "##inspection-export-output",
                inspection_export_output_, "...##inspection-export")) {
    openDialog(DialogTarget::InspectionExportOutput, "Export point cloud",
               false, true, inspection_export_output_);
  }
  ImGui::Checkbox("Overwrite existing file##inspection-export",
                  &inspection_export_overwrite_);
  constexpr const char *export_scopes =
      "Active layer\0Visible layers (merged)\0All layers (merged)\0";
  int export_scope = static_cast<int>(inspection_export_scope_);
  if (ImGui::Combo("Export scope##inspection", &export_scope, export_scopes)) {
    inspection_export_scope_ = static_cast<InspectionExportScope>(
        std::clamp(export_scope, 0, 2));
  }

  const auto active_layer_id = inspection_scene_.activeLayer();
  std::size_t selected_cloud_count = 0;
  for (const CloudLayer &layer : inspection_scene_.layers()) {
    const bool selected =
        inspection_export_scope_ == InspectionExportScope::AllLayers ||
        (inspection_export_scope_ == InspectionExportScope::VisibleLayers &&
         layer.visible()) ||
        (inspection_export_scope_ == InspectionExportScope::ActiveLayer &&
         active_layer_id && layer.id() == *active_layer_id);
    if (selected && layer.cloud()) {
      ++selected_cloud_count;
    }
  }
  const bool can_export = selected_cloud_count != 0 &&
                          !inspection_export_output_.empty() &&
                          (!inspection_roi_enabled_ || roi.has_value());
  if (!can_export)
    ImGui::BeginDisabled();
  if (ImGui::Button("Export selected layers##inspection-export"))
    queueInspectionExport();
  if (!can_export)
    ImGui::EndDisabled();
  if (selected_cloud_count == 0) {
    ImGui::TextDisabled("No selected layer has a loaded point cloud");
  } else if (!inspection_roi_enabled_) {
    ImGui::TextDisabled("ROI disabled: exports all finite world-space points");
  }
#else
  ImGui::TextDisabled(
      "Point-cloud export is available in native desktop builds");
#endif
}

void App::drawBookmarkControls() {
  ImGui::SeparatorText("View bookmarks");
  ImGui::InputText("Name##bookmark", &bookmark_name_);
  ImGui::SameLine();
  if (ImGui::Button("Save view##bookmark") && !bookmark_name_.empty()) {
    try {
      inspection_settings_.saveBookmark(
          CameraBookmark(bookmark_name_, main_viewport_.cameraSnapshot()));
      inspection_undo_domain_ = InspectionUndoDomain::Bookmarks;
      persistInspectionSettings();
    } catch (const std::exception &error) {
      log("Bookmark save failed: " + std::string(error.what()));
    }
  }
  ImGui::SameLine();
  const bool can_undo_bookmark = inspection_settings_.canUndo();
  if (!can_undo_bookmark)
    ImGui::BeginDisabled();
  if (ImGui::SmallButton("Undo bookmark")) {
    if (inspection_settings_.undo()) {
      inspection_undo_domain_ = InspectionUndoDomain::Bookmarks;
      persistInspectionSettings();
    }
  }
  if (!can_undo_bookmark)
    ImGui::EndDisabled();
  ImGui::SameLine();
  const bool can_redo_bookmark = inspection_settings_.canRedo();
  if (!can_redo_bookmark)
    ImGui::BeginDisabled();
  if (ImGui::SmallButton("Redo bookmark")) {
    if (inspection_settings_.redo()) {
      inspection_undo_domain_ = InspectionUndoDomain::Bookmarks;
      persistInspectionSettings();
    }
  }
  if (!can_redo_bookmark)
    ImGui::EndDisabled();

  for (const CameraBookmark &bookmark : inspection_settings_.bookmarks()) {
    ImGui::PushID(bookmark.name().c_str());
    ImGui::TextUnformatted(bookmark.name().c_str());
    ImGui::SameLine();
    if (ImGui::SmallButton("Restore")) {
      if (!main_viewport_.setCameraSnapshot(bookmark.camera()))
        log("Bookmark camera is not renderable: " + bookmark.name());
    }
    ImGui::SameLine();
    if (ImGui::SmallButton("Delete")) {
      const std::string name = bookmark.name();
      try {
        if (inspection_settings_.removeBookmark(name)) {
          inspection_undo_domain_ = InspectionUndoDomain::Bookmarks;
          persistInspectionSettings();
        }
      } catch (const std::exception &error) {
        log("Bookmark delete failed: " + std::string(error.what()));
      }
      ImGui::PopID();
      break;
    }
    ImGui::PopID();
  }
}

void App::drawInspectionScreenshotControls() {
  ImGui::SeparatorText("Screenshot");
#ifdef KPT_WEB_BUILD
  const bool capture_pending = inspection_screenshot_request_.has_value() ||
                               web::hasViewportPngDownloadActivity();
  if (capture_pending)
    ImGui::BeginDisabled();
  if (ImGui::Button("Download viewport PNG##inspection-screenshot"))
    queueInspectionScreenshot();
  if (capture_pending)
    ImGui::EndDisabled();
  ImGui::TextDisabled(
      "Captures the rendered viewport; PNG downloads through this browser");
#else
  if (pathInput("Viewport PNG", "##inspection-screenshot-output",
                inspection_screenshot_output_, "...##inspection-screenshot")) {
    openDialog(DialogTarget::InspectionScreenshotOutput, "Save viewport PNG",
               false, true, inspection_screenshot_output_);
  }
  ImGui::Checkbox("Overwrite existing file##inspection-screenshot",
                  &inspection_screenshot_overwrite_);
  const bool can_capture = !inspection_screenshot_output_.empty();
  if (!can_capture)
    ImGui::BeginDisabled();
  if (ImGui::Button("Save viewport PNG##inspection-screenshot"))
    queueInspectionScreenshot();
  if (!can_capture)
    ImGui::EndDisabled();
  ImGui::TextDisabled("Captures rendered viewport; PNG encoding runs in background");
#endif
}

void App::drawInspectionShareControls() {
  ImGui::SeparatorText("Review share");
#ifdef KPT_WEB_BUILD
  ImGui::TextDisabled("Review-share files are available in native desktop builds");
#else
  if (ImGui::Button("Open review share...##inspection-share")) {
    openDialog(DialogTarget::InspectionShareInput, "Open review share", false,
               false, inspection_share_output_);
  }
  if (pathInput("Review share JSON", "##inspection-share-output",
                inspection_share_output_, "...##inspection-share-output")) {
    openDialog(DialogTarget::InspectionShareOutput, "Save review share", false,
               true, inspection_share_output_);
  }
  ImGui::Checkbox("Overwrite existing share##inspection-share",
                  &inspection_share_overwrite_);
  const bool can_save = !inspection_share_output_.empty();
  if (!can_save)
    ImGui::BeginDisabled();
  if (ImGui::Button("Save review share##inspection-share"))
    queueInspectionShareSave();
  if (!can_save)
    ImGui::EndDisabled();
  ImGui::TextDisabled(
      "JSON stores review state only; paths resolve relative to this file");
#endif
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
  if (physical_extent.width > 0 && physical_extent.height > 0)
    main_viewport_extent_ = physical_extent;
  // Capture before encoding this frame.  The image then belongs to the prior
  // viewport pass, which renderAndPresent() committed after the previous
  // App::draw().  In particular, Metal never sees its current runtime-owned
  // command buffer in MTLCommandBufferStatusNotEnqueued here.
  if (inspection_screenshot_request_ &&
      inspection_screenshot_request_->capture_after_viewport_frame <=
          inspection_viewport_render_count_) {
    capturePendingInspectionScreenshot();
  }
  if (inspection_scene_.measurements().empty()) {
    main_viewport_.setSupplementalGuides({});
  } else {
    main_viewport_.setSupplementalGuides(buildMeasurementRenderGuides(
        inspection_scene_, main_viewport_.frameForPicking(physical_extent)));
  }
  const bool interactive_lod =
      interaction_quality && frame_context.backendKind() == BackendKind::WebGL;
  auto drawn = main_viewport_.draw(physical_extent, frame_context,
                                   ViewportRole::Main, interactive_lod);
  if (!drawn) {
    if (retryInspectionUpload(drawn.error())) {
      ImGui::End();
      return {};
    }
    ImGui::End();
    return drawn.error();
  }
  if (!inspection_scene_.layers().empty()) {
    inspection_upload_retry_pending_ = false;
    inspection_last_added_layer_.reset();
  }
  if (drawn.value())
    ++inspection_viewport_render_count_;
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
    if (!inspection_scene_.measurements().empty()) {
      const MeasurementOverlay overlay = buildMeasurementOverlay(
          inspection_scene_, main_viewport_.frameForPicking(physical_extent));
      drawMeasurementOverlay(overlay, *ImGui::GetWindowDrawList(),
                             image_position, available);
    }
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
    bool camera_changed = false;
    if (ImGui::IsMouseDragging(ImGuiMouseButton_Left)) {
      if (io.KeyShift) {
        main_viewport_.roll(io.MouseDelta.x, interaction_extent);
      } else {
        main_viewport_.orbit(previous.x, previous.y, current.x, current.y,
                             interaction_extent);
      }
      camera_changed = true;
    }
    if (ImGui::IsMouseDragging(ImGuiMouseButton_Right)) {
      main_viewport_.pan(io.MouseDelta.x, io.MouseDelta.y, interaction_extent);
      camera_changed = true;
    }
    if (ImGui::IsMouseClicked(ImGuiMouseButton_Middle)) {
      camera_changed = main_viewport_.setRotationCenterFromScreen(
          current.x, current.y, interaction_extent);
    }
    if (io.KeyCtrl && ImGui::IsMouseClicked(ImGuiMouseButton_Left) &&
        !ImGui::IsMouseDragging(ImGuiMouseButton_Left)) {
      if (!inspection_scene_.layers().empty()) {
        if (const auto picked = pickInspectionLayerFromScreen(
                current.x, current.y, interaction_extent)) {
          addMeasurementFromLayerPick(*picked);
        }
      } else if (const auto picked = main_viewport_.pickCloudFromScreen(
                     current.x, current.y, interaction_extent)) {
        addMeasurementFromLocalPick(*picked);
      }
    }
    if (io.MouseWheel != 0.0F) {
      main_viewport_.zoom(io.MouseWheel * 15.0F);
      camera_changed = true;
    }
    if (camera_changed && !inspection_scene_.layers().empty()) {
      // Re-sort transparent layer passes once on the next frame.  The rebuild
      // preserves the user camera and never runs merely because a scene was
      // already accepted, avoiding a revision/render loop.
      inspection_layer_order_dirty_ = true;
    }
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
  ImGui::TextUnformatted(
      translatedValue("gui.jobs.workers_count", "%u", jobs_.maxWorkers())
          .c_str());
#else
  unsigned worker_limit = jobs_.workerLimit();
  const unsigned minimum_workers = 1;
  const unsigned maximum_workers = jobs_.maxWorkers();
  if (ImGui::SliderScalar(kpt::i18n::tr("gui.jobs.workers"), ImGuiDataType_U32,
                          &worker_limit, &minimum_workers, &maximum_workers)) {
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
          ImGui::SmallButton((std::string(kpt::i18n::tr("gui.jobs.cancel")) +
                              "##" + std::to_string(job.id))
                                 .c_str())) {
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
  const char *filters = nullptr;
  if (!directory) {
    switch (target) {
    case DialogTarget::InspectionScreenshotOutput:
      filters = "PNG image{.png}";
      break;
    case DialogTarget::InspectionShareInput:
    case DialogTarget::InspectionShareOutput:
      filters = "Review shares{.kpt-review.json,.json}";
      break;
    default:
      filters = "Point "
                "clouds{.bin,.pcd,.ply,.las,.pts,.obj,.npy,"
                ".xyz,.xyzi,.xyzrgb,.xyzrgbi}";
      break;
    }
  }
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
    ImGui::TextUnformatted(
        substitute(kpt::i18n::tr("gui.about.version"), "%s", KPT_VERSION_STRING)
            .c_str());
    ImGui::TextUnformatted(
        substitute(kpt::i18n::tr("gui.about.commit"), "%s", KPT_GIT_HASH)
            .c_str());
    ImGui::TextUnformatted(
        substitute(kpt::i18n::tr("gui.about.build_time"), "%s", KPT_BUILD_TIME)
            .c_str());
    ImGui::Spacing();
    ImGui::TextWrapped("%s", kpt::i18n::tr("gui.about.description"));
    ImGui::Spacing();
    ImGui::TextWrapped(
        "%s", "https://github.com/nerdneilsfield/kitti_pointcloud_tools");
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
  case DialogTarget::InspectionLayerInput: {
    const auto path = decodeUiPath(value, "Inspection layer path");
    if (path)
      loadInspectionLayerFile(*path);
    break;
  }
  case DialogTarget::InspectionExportOutput:
    inspection_export_output_ = value;
    break;
  case DialogTarget::InspectionScreenshotOutput:
    inspection_screenshot_output_ = value;
    break;
  case DialogTarget::InspectionShareInput: {
    const auto path = decodeUiPath(value, "Review share path");
    if (path) {
      inspection_share_output_ = value;
      loadInspectionShareFile(*path);
    }
    break;
  }
  case DialogTarget::InspectionShareOutput:
    inspection_share_output_ = value;
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
  const auto layer_snapshot_revision = ++inspection_layer_snapshot_revision_;
  const auto source_key = sourceKeyForPath(native_path);
  jobs_.submit(
      "Load " + filename, JobPriority::High,
       [this, native_path, display_path, source_key, source_generation,
       layer_snapshot_revision,
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
              makeViewportCloudSnapshot(cloud, layer_snapshot_revision, stop);
          if (stop.stop_requested())
            return;
          ui_.post([this, snapshot, cloud, source_key, display_path, source_generation] {
            if (source_generation != sequence_generation_)
              return;
            registerInspectionLayer(source_key, cloud, snapshot,
                                    CameraUpdate::Fit);
            restorePendingCameraAfterInitialFit();
            log("Loaded " + display_path + " (" +
                std::to_string(snapshot->vertices.size()) + " points)");
            if (launch_state_ == LaunchState::Pending)
              launch_state_ = LaunchState::Ready;
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

void App::loadInspectionLayerFile(const std::filesystem::path &native_path) {
  const auto filename = displayPath(native_path.filename());
  const auto display_path = displayPath(native_path);
  const auto source_key = sourceKeyForPath(native_path);
  if (inspection_scene_.findLayerBySourceKey(source_key) != nullptr) {
    log("Layer is already loaded: " + display_path);
    return;
  }

  const std::uint64_t source_generation = sequence_generation_;
  const std::uint64_t layer_snapshot_revision =
      ++inspection_layer_snapshot_revision_;
  jobs_.submit(
      "Add layer " + filename, JobPriority::High,
      [this, native_path, display_path, source_key, source_generation,
       layer_snapshot_revision](std::stop_token stop,
                                const JobSystem::Reporter &report) {
        try {
          report(0.1F, "loading");
          const auto cloud = kpt::load(native_path, stop);
          if (stop.stop_requested())
            return;
          const auto snapshot = makeViewportCloudSnapshot(
              cloud, layer_snapshot_revision, stop);
          if (stop.stop_requested())
            return;
          ui_.post([this, cloud, snapshot, source_key, display_path,
                    source_generation] {
            if (source_generation != sequence_generation_ ||
                inspection_scene_.findLayerBySourceKey(source_key) != nullptr) {
              return;
            }
            registerInspectionLayer(source_key, cloud, snapshot,
                                    CameraUpdate::Preserve);
            log("Added layer " + display_path + " (" +
                std::to_string(snapshot->vertices.size()) + " points)");
          });
          report(1.0F, "loaded " + std::to_string(cloud->size()) + " points");
        } catch (const std::exception &error) {
          ui_.post([this, display_path, message = std::string(error.what())] {
            log("Failed to add layer " + display_path + ": " + message);
          });
          throw;
        }
      });
}

void App::loadInspectionShareFile(const std::filesystem::path &share_path) {
#ifdef KPT_WEB_BUILD
  static_cast<void>(share_path);
  log("Review-share files are unavailable in web builds");
#else
  const std::string display_path = displayPath(share_path);
  const std::string filename = displayPath(share_path.filename());
  const std::uint64_t import_generation =
      ++inspection_share_import_generation_;
  jobs_.submit(
      "Open review share " + filename, JobPriority::High,
      [this, share_path, display_path,
       import_generation](std::stop_token stop,
                          const JobSystem::Reporter &report) {
        if (stop.stop_requested())
          return;
        report(0.1F, "reading JSON");
        InspectionShareDocument document;
        std::string error;
        if (!InspectionShareFile(share_path).load(document, &error)) {
          ui_.post([this, display_path, import_generation, error] {
            if (import_generation != inspection_share_import_generation_)
              return;
            log("Review share load failed " + display_path + ": " + error);
          });
          throw std::runtime_error(error);
        }
        if (stop.stop_requested())
          return;
        report(0.8F, "validating review state");
        ui_.post([this, document = std::move(document), share_path,
                  import_generation]() mutable {
          if (import_generation != inspection_share_import_generation_)
            return;
          applyInspectionShare(std::move(document), share_path);
        });
        report(1.0F, "ready");
      });
#endif
}

void App::openSequence() {
  playback_.disarmAutoplay();
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
              trajectory.cloud, trajectory_generation, stop);
          if (stop.stop_requested())
            return;
          ui_.post([this, sequence, sequence_generation, trajectory_snapshot,
                    trajectory_warnings = std::move(trajectory.warnings)] {
            if (sequence_generation != sequence_generation_)
              return;
            sequence_ = sequence;
            frame_cache_.clear();
            playback_.resetSource();
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
  ++inspection_share_import_generation_;
  jobs_.cancelAll();
  playback_.resetSource();
  launch_warnings_.clear();
  jobs_.setPlayerActive(false);
  sequence_.reset();
  frame_cache_.clear();
  main_viewport_.cancelAndClear();
  trajectory_viewport_.cancelAndClear();
  inspection_scene_.clearLayers();
  inspection_render_adapter_.clearSnapshots();
  inspection_render_list_.reset();
  inspection_layer_order_dirty_ = false;
  inspection_roi_preview_pending_ = false;
  inspection_roi_controls_need_hydrate_ = true;
  inspection_snapshot_hydration_layers_.clear();
  inspection_gpu_vertex_cap_.reset();
  inspection_last_added_layer_.reset();
  inspection_upload_retry_pending_ = false;
  inspection_screenshot_request_.reset();
  return ++sequence_generation_;
}

void App::registerInspectionLayer(
    std::string source_key, std::shared_ptr<const PointCloudIRGB> cloud,
    std::shared_ptr<const ViewportCloudSnapshot> snapshot,
    CameraUpdate camera_update) {
  if (source_key.empty() || !cloud || !snapshot)
    return;
  if (inspection_scene_.findLayerBySourceKey(source_key) != nullptr)
    return;
  const auto layer_id = inspection_scene_.addLayer(std::move(source_key),
                                                   std::move(cloud));
  if (!inspection_render_adapter_.acceptSnapshot(layer_id, std::move(snapshot))) {
    static_cast<void>(inspection_scene_.removeLayer(layer_id));
    log("Inspection layer snapshot was rejected");
    return;
  }
  static_cast<void>(inspection_scene_.setActiveLayer(layer_id));
  inspection_undo_domain_ = InspectionUndoDomain::Scene;
  inspection_last_added_layer_ = layer_id;
  refreshInspectionViewport(camera_update);
}

SceneRenderOptions App::inspectionSceneRenderOptions(
    const CameraSnapshot &camera,
    std::optional<std::size_t> transient_vertex_cap) const {
  SceneRenderOptions options;
  options.admission.available_system_memory_bytes = availableSystemMemoryBytes();
  options.camera_position = camera.target +
                            camera.camera_to_world.col(2).cast<double>() *
                                camera.distance;
  options.camera_forward = -camera.camera_to_world.col(2).cast<double>();
  if (transient_vertex_cap && inspection_gpu_vertex_cap_) {
    options.maximum_render_vertices = std::min(*transient_vertex_cap,
                                               *inspection_gpu_vertex_cap_);
  } else if (transient_vertex_cap) {
    options.maximum_render_vertices = *transient_vertex_cap;
  } else if (inspection_gpu_vertex_cap_) {
    options.maximum_render_vertices = *inspection_gpu_vertex_cap_;
  }
  return options;
}

bool App::retryInspectionUpload(const AppError &failure) {
  if (failure.role != ViewportRole::Main || failure.stage != AppStage::Upload ||
      failure.cause.code != RendererErrorCode::ResourceCreationFailed ||
      !inspection_render_list_) {
    return false;
  }

  std::size_t current_vertex_count = 0;
  for (const LayerRenderItem &item : inspection_render_list_->layers) {
    if (item.visible) {
      current_vertex_count += item.vertex_selection.retained_vertex_count;
    }
  }
  if (current_vertex_count > 1) {
    const std::size_t next_cap = std::max<std::size_t>(1,
                                                       current_vertex_count / 2U);
    if (!inspection_gpu_vertex_cap_ || next_cap < *inspection_gpu_vertex_cap_) {
      inspection_gpu_vertex_cap_ = next_cap;
      inspection_upload_retry_pending_ = true;
      log("GPU upload failed; retrying review scene at " +
          std::to_string(next_cap) + " uniformly sampled points");
      refreshInspectionViewport(CameraUpdate::Preserve);
      return true;
    }
  }

  // A new layer is the only recoverable admission target once even a minimal
  // LOD fails.  Remove it through Scene so Ctrl+Z retains the CPU cloud and
  // the snapshot hydration path can restore it later; never discard user data.
  if (inspection_last_added_layer_ &&
      inspection_scene_.removeLayer(*inspection_last_added_layer_)) {
    const std::string id = std::to_string(*inspection_last_added_layer_);
    inspection_render_adapter_.pruneMissingLayers(inspection_scene_);
    inspection_last_added_layer_.reset();
    inspection_upload_retry_pending_ = true;
    log("Rejected review layer " + id +
        ": GPU cannot allocate even minimum LOD (undo restores it)");
    refreshInspectionViewport(CameraUpdate::Preserve);
    return true;
  }
  return false;
}

void App::refreshInspectionViewport(CameraUpdate camera_update) {
  // A synchronous scene edit supersedes any worker-built ROI preview.  The
  // generation check in its UI completion prevents an older filter from
  // replacing this newer render list.
  invalidateInspectionRoiPreview();
  inspection_render_adapter_.pruneMissingLayers(inspection_scene_);
  if (inspection_scene_.layers().empty()) {
    inspection_render_list_.reset();
    main_viewport_.cancelAndClear();
    return;
  }

  const CameraSnapshot camera = main_viewport_.cameraSnapshot();
  const SceneRenderOptions options = inspectionSceneRenderOptions(camera);
  const LayerRenderList render_list =
      inspection_render_adapter_.build(inspection_scene_, options);
  const auto request = main_viewport_.beginRequest();
  SceneCompositeOptions composite_options;
  const auto composite = composeLayeredSceneViewportSnapshot(
      render_list, request, composite_options);
  inspection_render_list_ = render_list;
  inspection_layer_order_dirty_ = false;
  if (!main_viewport_.acceptLayered(composite, camera_update)) {
    log("Inspection scene viewport snapshot was rejected");
  }
}

void App::refreshInspectionViewportIfRoiDue() {
  if (inspection_layer_order_dirty_) {
    refreshInspectionViewport(CameraUpdate::Preserve);
    return;
  }
  if (inspection_roi_preview_pending_) {
    const double now = ImGui::GetTime();
    if (now >= inspection_roi_preview_due_seconds_) {
      inspection_roi_preview_pending_ = false;
      inspection_roi_preview_last_seconds_ = now;
      dispatchInspectionRoiPreview(inspection_roi_preview_full_resolution_);
    }
  }
}

void App::scheduleInspectionRoiPreview(bool final_edit) {
  invalidateInspectionRoiPreview();
  const double now = ImGui::GetTime();
  constexpr double preview_interval_seconds = 0.1;
  constexpr double final_settle_seconds = 0.15;
  inspection_roi_preview_pending_ = true;
  inspection_roi_preview_full_resolution_ = final_edit;
  if (final_edit) {
    inspection_roi_preview_due_seconds_ = now + final_settle_seconds;
    return;
  }
  const double earliest = inspection_roi_preview_last_seconds_ < 0.0
                              ? now
                              : inspection_roi_preview_last_seconds_ +
                                    preview_interval_seconds;
  inspection_roi_preview_due_seconds_ = std::max(now, earliest);
}

void App::invalidateInspectionRoiPreview() noexcept {
  ++inspection_roi_preview_generation_;
  inspection_roi_preview_pending_ = false;
  inspection_roi_preview_full_resolution_ = false;
  if (inspection_roi_preview_job_) {
    jobs_.cancel(*inspection_roi_preview_job_);
    inspection_roi_preview_job_.reset();
  }
}

void App::dispatchInspectionRoiPreview(bool full_resolution) {
  if (inspection_scene_.layers().empty()) {
    return;
  }

  constexpr std::size_t drag_preview_vertex_cap = 500'000U;
  const std::uint64_t generation = inspection_roi_preview_generation_;
  const std::uint64_t request = main_viewport_.beginRequest();
  const SceneRenderSnapshot scene_snapshot =
      inspection_render_adapter_.capture(inspection_scene_);
  const CameraSnapshot camera = main_viewport_.cameraSnapshot();
  const SceneRenderOptions options = inspectionSceneRenderOptions(
      camera, full_resolution ? std::optional<std::size_t>{}
                              : std::optional<std::size_t>{
                                    drag_preview_vertex_cap});

  inspection_roi_preview_job_ = jobs_.submit(
      full_resolution ? "Refine ROI preview" : "Preview ROI",
      JobPriority::Normal,
      [this, generation, request, scene_snapshot, options,
       full_resolution](std::stop_token stop,
                        const JobSystem::Reporter &report) {
        try {
          report(0.05F, "filtering ROI");
          LayerRenderList render_list =
              SceneRenderAdapter::build(scene_snapshot, options, stop);
          if (stop.stop_requested()) {
            return;
          }
          report(0.75F, "building layered preview");
          const auto composite = composeLayeredSceneViewportSnapshot(
              render_list, request, {}, stop);
          if (stop.stop_requested()) {
            return;
          }
          ui_.post([this, generation, request,
                    render_list = std::move(render_list), composite,
                    full_resolution]() mutable {
            if (generation != inspection_roi_preview_generation_) {
              return;
            }
            inspection_roi_preview_job_.reset();
            inspection_render_list_ = std::move(render_list);
            inspection_layer_order_dirty_ = false;
            if (!main_viewport_.acceptLayered(composite,
                                              CameraUpdate::Preserve)) {
              return;
            }
            log(full_resolution ? "ROI preview refined"
                                : "ROI preview updated");
          });
          report(1.0F, "preview ready");
        } catch (const OperationCancelled &) {
          // A newer ROI generation owns the UI; cancellation is expected.
        }
      });
}

void App::hydrateInspectionRoiControlsFromScene() {
  const auto &roi = inspection_scene_.roi();
  inspection_roi_enabled_ = roi.has_value();
  if (roi) {
    for (Eigen::Index axis = 0; axis < 3; ++axis) {
      inspection_roi_min_[static_cast<std::size_t>(axis)] = roi->minimum()[axis];
      inspection_roi_max_[static_cast<std::size_t>(axis)] = roi->maximum()[axis];
    }
  }
  inspection_roi_controls_need_hydrate_ = false;
}

void App::hydrateInspectionSnapshotsForScene() {
  for (const CloudLayer &layer : inspection_scene_.layers()) {
    if (!layer.cloud() || inspection_render_adapter_.hasSnapshot(layer.id()) ||
        inspection_snapshot_hydration_layers_.contains(layer.id())) {
      continue;
    }

    const LayerId layer_id = layer.id();
    const std::string source_key = layer.sourceKey();
    const auto cloud = layer.cloud();
    const std::uint64_t revision = ++inspection_layer_snapshot_revision_;
    inspection_snapshot_hydration_layers_.insert(layer_id);
    jobs_.submit(
        "Restore layer " + source_key, JobPriority::Normal,
        [this, layer_id, source_key, cloud, revision](
            std::stop_token stop, const JobSystem::Reporter &report) {
          try {
            report(0.1F, "building snapshot");
            const auto snapshot = makeViewportCloudSnapshot(cloud, revision, stop);
            if (stop.stop_requested()) {
              return;
            }
            ui_.post([this, layer_id, source_key, snapshot] {
              inspection_snapshot_hydration_layers_.erase(layer_id);
              const CloudLayer *current = inspection_scene_.findLayer(layer_id);
              if (current == nullptr || current->sourceKey() != source_key ||
                  !inspection_render_adapter_.acceptSnapshot(layer_id, snapshot)) {
                return;
              }
              refreshInspectionViewport(CameraUpdate::Preserve);
            });
            report(1.0F, "restored");
          } catch (const OperationCancelled &) {
            ui_.post([this, layer_id] {
              inspection_snapshot_hydration_layers_.erase(layer_id);
            });
          }
        });
  }
}

void App::refreshAfterInspectionHistoryChange() {
  inspection_roi_controls_need_hydrate_ = true;
  hydrateInspectionRoiControlsFromScene();
  inspection_render_adapter_.pruneMissingLayers(inspection_scene_);
  hydrateInspectionSnapshotsForScene();
  if (inspection_scene_.layers().empty()) {
    inspection_render_list_.reset();
    main_viewport_.cancelAndClear();
    return;
  }
  if (inspection_scene_.roi()) {
    scheduleInspectionRoiPreview(true);
  } else {
    refreshInspectionViewport(CameraUpdate::Preserve);
  }
}

void App::handleInspectionUndoRedo() {
  const ImGuiIO &io = ImGui::GetIO();
  if (!io.KeyCtrl || io.WantTextInput || inspection_scene_.transactionActive()) {
    return;
  }
  const bool redo = ImGui::IsKeyPressed(ImGuiKey_Y, false) ||
                    (io.KeyShift && ImGui::IsKeyPressed(ImGuiKey_Z, false));
  const bool undo = !redo && ImGui::IsKeyPressed(ImGuiKey_Z, false);
  if (!undo && !redo) {
    return;
  }
  if (inspection_undo_domain_ == InspectionUndoDomain::Bookmarks) {
    const bool changed = redo ? inspection_settings_.redo() : inspection_settings_.undo();
    if (changed) {
      persistInspectionSettings();
      return;
    }
  }
  const bool changed = redo ? inspection_scene_.redo() : inspection_scene_.undo();
  if (changed) {
    inspection_undo_domain_ = InspectionUndoDomain::Scene;
    refreshAfterInspectionHistoryChange();
    return;
  }
  // If the active scene history has no matching operation, make saved view
  // changes reachable from the same standard shortcut instead of requiring a
  // separate modal workflow.
  if (inspection_undo_domain_ != InspectionUndoDomain::Bookmarks &&
      (redo ? inspection_settings_.redo() : inspection_settings_.undo())) {
    inspection_undo_domain_ = InspectionUndoDomain::Bookmarks;
    persistInspectionSettings();
  }
}

void App::fitInspectionVisible() {
  if (!inspection_render_list_)
    return;
  const auto probe = composeSceneFitViewportSnapshot(
      *inspection_render_list_, 1);
  if (const auto fitted =
          main_viewport_.fitCameraFor(probe, main_viewport_extent_)) {
    static_cast<void>(main_viewport_.setCameraSnapshot(*fitted));
    inspection_layer_order_dirty_ = true;
  }
}

void App::fitInspectionActive() {
  if (!inspection_render_list_)
    return;
  SceneCompositeOptions options;
  options.only_layer = inspection_scene_.activeLayer();
  const auto probe = composeSceneFitViewportSnapshot(
      *inspection_render_list_, 1, options);
  if (const auto fitted =
          main_viewport_.fitCameraFor(probe, main_viewport_extent_)) {
    static_cast<void>(main_viewport_.setCameraSnapshot(*fitted));
    inspection_layer_order_dirty_ = true;
  }
}

std::optional<LayerPickResult>
App::pickInspectionLayerFromScreen(float x, float y, PixelExtent viewport) {
  if (!inspection_render_list_ || viewport.width <= 0 || viewport.height <= 0 ||
      !std::isfinite(x) || !std::isfinite(y)) {
    return std::nullopt;
  }
  const ViewportFrame frame = main_viewport_.frameForPicking(viewport);
  const auto active = inspection_scene_.activeLayer();
  constexpr float pick_radius = 8.0F;
  float best_distance_squared = pick_radius * pick_radius;
  float best_depth = std::numeric_limits<float>::infinity();
  std::optional<LayerPickResult> result;
  const auto &world_roi = inspection_scene_.roi();

  for (const LayerRenderItem &item : inspection_render_list_->layers) {
    if (!item.visible || !item.snapshot ||
        (inspection_render_list_->pick_scope == LayerPickScope::ActiveLayerOnly &&
         (!active || item.layer_id != *active))) {
      continue;
    }
    const auto &all_candidates = item.snapshot->picking_vertices.empty()
                                     ? item.snapshot->vertices
                                     : item.snapshot->picking_vertices;
    const std::size_t candidate_count = std::min(
        all_candidates.size(), SceneRenderAdapter::kMaximumPickingCandidatesPerLayer);
    for (std::size_t candidate = 0; candidate < candidate_count; ++candidate) {
      const std::size_t index =
          (candidate * all_candidates.size()) / candidate_count;
      const ViewportVertex &local = all_candidates[index];
      const auto world = transformLocalToWorld(local.position.cast<double>(),
                                               item.local_to_world);
      if (!world || (world->array().abs() >
                     static_cast<double>(std::numeric_limits<float>::max()))
                        .any()) {
        continue;
      }
      if (world_roi && !world_roi->contains(*world)) {
        continue;
      }
      const Eigen::Vector3f render_local =
          (world->cast<float>() - frame.world_origin) * frame.world_scale;
      const Eigen::Vector4f clip = frame.view_projection *
                                   Eigen::Vector4f(render_local.x(),
                                                   render_local.y(),
                                                   render_local.z(), 1.0F);
      if (!clip.allFinite() || clip.w() <= 0.0F)
        continue;
      const Eigen::Vector3f ndc = clip.head<3>() / clip.w();
      if (!ndc.allFinite() || ndc.z() < -1.0F || ndc.z() > 1.0F)
        continue;
      const float screen_x =
          (ndc.x() * 0.5F + 0.5F) * static_cast<float>(viewport.width);
      const float screen_y =
          (0.5F - ndc.y() * 0.5F) * static_cast<float>(viewport.height);
      const float dx = screen_x - x;
      const float dy = screen_y - y;
      const float distance_squared = dx * dx + dy * dy;
      if (distance_squared > best_distance_squared ||
          (distance_squared == best_distance_squared && ndc.z() >= best_depth)) {
        continue;
      }
      PickResult local_pick;
      local_pick.cloud_position = local.position;
      local_pick.world_position = local.position;
      local_pick.intensity = local.intensity;
      local_pick.noise = local.noise;
      const auto resolved = inspection_render_adapter_.resolvePick(
          inspection_scene_, item.layer_id, local_pick);
      if (!resolved)
        continue;
      best_distance_squared = distance_squared;
      best_depth = ndc.z();
      result = resolved;
    }
  }
  return result;
}

void App::addMeasurementFromLayerPick(const LayerPickResult &pick) {
  const auto &measurements = inspection_scene_.measurements();
  if (!measurements.empty()) {
    const Measurement &pending = measurements.back();
    if (!pending.secondWorld()) {
      if (inspection_scene_.completeMeasurement(pending.id(), pick.source_key,
                                                pick.world_position)) {
        inspection_undo_domain_ = InspectionUndoDomain::Scene;
      }
      return;
    }
  }
  static_cast<void>(inspection_scene_.beginMeasurement(pick.source_key,
                                                        pick.world_position));
  inspection_undo_domain_ = InspectionUndoDomain::Scene;
}

void App::queueInspectionExport() {
#ifdef KPT_WEB_BUILD
  log("Point-cloud export is unavailable in web builds");
#else
  const auto output =
      decodeUiPath(inspection_export_output_, "Inspection export output path");
  if (!output)
    return;
  const auto roi = inspectionRoiFromControls();
  if (inspection_roi_enabled_ && !roi) {
    log("Inspection export ROI is invalid");
    return;
  }

  struct ExportLayer {
    std::shared_ptr<const PointCloudIRGB> cloud;
    Eigen::Affine3d local_to_world = Eigen::Affine3d::Identity();
  };
  std::vector<ExportLayer> layers;
  const auto active_layer_id = inspection_scene_.activeLayer();
  for (const CloudLayer &layer : inspection_scene_.layers()) {
    const bool selected =
        inspection_export_scope_ == InspectionExportScope::AllLayers ||
        (inspection_export_scope_ == InspectionExportScope::VisibleLayers &&
         layer.visible()) ||
        (inspection_export_scope_ == InspectionExportScope::ActiveLayer &&
         active_layer_id && layer.id() == *active_layer_id);
    if (selected && layer.cloud()) {
      layers.push_back({layer.cloud(), layer.localToWorld()});
    }
  }
  if (layers.empty()) {
    log("Inspection export has no selected loaded layers");
    return;
  }

  // All worker inputs are immutable snapshots. In particular, later layer,
  // ROI, style, or file-dialog edits cannot race this export job.
  const RoiBox world_roi = roi.value_or(wholeFinitePointWorldRoi());
  const std::filesystem::path output_path = *output;
  const std::string output_display = displayPath(output_path);
  const std::string output_name = displayPath(output_path.filename());
  const bool overwrite = inspection_export_overwrite_;

  jobs_.submit(
      "Export " + output_name, JobPriority::Normal,
      [this, layers = std::move(layers), world_roi, output_path, output_display,
       overwrite](std::stop_token stop, const JobSystem::Reporter &report) {
        if (stop.stop_requested())
          return;
        report(0.05F, "filtering ROI");

        std::vector<PointCloudIRGB> filtered_layers;
        filtered_layers.reserve(layers.size());
        try {
          for (std::size_t index = 0; index < layers.size(); ++index) {
            if (stop.stop_requested())
              return;
            filtered_layers.push_back(filterCloudToWorldRoi(
                *layers[index].cloud, layers[index].local_to_world, world_roi,
                stop));
            report(0.05F + 0.45F *
                               static_cast<float>(index + 1) /
                                   static_cast<float>(layers.size()),
                   "filtered layer " + std::to_string(index + 1) + "/" +
                       std::to_string(layers.size()));
          }
        } catch (const OperationCancelled &) {
          ui_.post([this, output_display] {
            log("Inspection export cancelled " + output_display);
          });
          return;
        } catch (const std::exception &error) {
          const std::string message = error.what();
          ui_.post([this, output_display, message] {
            log("Inspection export failed " + output_display + ": " + message);
          });
          throw;
        }
        if (stop.stop_requested())
          return;

        std::vector<WorldCloudView> clouds;
        clouds.reserve(filtered_layers.size());
        std::size_t point_count = 0;
        for (const PointCloudIRGB &cloud : filtered_layers) {
          clouds.push_back({cloud});
          point_count += cloud.size();
        }
        report(0.55F, "writing " + std::to_string(point_count) + " points");
        const InspectionExportResult result = exportWorldClouds(
            output_path, clouds, overwrite, std::nullopt, stop);

        std::string message;
        switch (result.status) {
        case InspectionExportStatus::Written:
          message = "Exported " + std::to_string(point_count) +
                    " world-space points to " + output_display;
          break;
        case InspectionExportStatus::Skipped:
          message = "Inspection export skipped " + output_display + ": " +
                    result.message;
          break;
        case InspectionExportStatus::Empty:
          message = "Inspection export produced no points " + output_display +
                    "; no file was written";
          break;
        case InspectionExportStatus::Cancelled:
          message = "Inspection export cancelled " + output_display;
          break;
        case InspectionExportStatus::Failed:
          message = "Inspection export failed " + output_display + ": " +
                    result.message;
          break;
        }
        ui_.post([this, message] { log(message); });

        if (result.status == InspectionExportStatus::Failed)
          throw std::runtime_error(result.message);
        if (result.status == InspectionExportStatus::Cancelled ||
            stop.stop_requested())
          return;
        report(1.0F, result.status == InspectionExportStatus::Written
                         ? "exported"
                         : result.status == InspectionExportStatus::Empty
                               ? "empty"
                               : "skipped");
      });
#endif
}

void App::queueInspectionScreenshot() {
#ifdef KPT_WEB_BUILD
  if (inspection_screenshot_request_ ||
      web::hasViewportPngDownloadActivity()) {
    log("Viewport PNG download is already pending or encoding");
    return;
  }
  inspection_screenshot_request_ = InspectionScreenshotRequest{
      {}, "kpt-viewport.png", false, inspection_viewport_render_count_ + 1U};
  log("Viewport PNG will capture from the next completed frame");
#else
  const auto output = decodeUiPath(inspection_screenshot_output_,
                                   "Viewport PNG output path");
  if (!output)
    return;
  if (output->extension() != ".png") {
    log("Viewport screenshot output must use a .png extension");
    return;
  }
  inspection_screenshot_request_ = InspectionScreenshotRequest{
      *output, displayPath(*output), inspection_screenshot_overwrite_,
      inspection_viewport_render_count_ + 1U};
  log("Viewport screenshot will capture from the next completed frame");
#endif
}

void App::capturePendingInspectionScreenshot() {
  if (!inspection_screenshot_request_)
    return;
  InspectionScreenshotRequest request =
      std::move(*inspection_screenshot_request_);
  inspection_screenshot_request_.reset();

  auto captured = main_viewport_.captureRgba();
  if (!captured) {
    log("Viewport screenshot capture failed: " + captured.error().message);
    return;
  }
  Rgba8Image pixels = std::move(captured).value();
#ifdef KPT_WEB_BUILD
  std::string download_error;
  if (!web::downloadViewportPng(request.output_display, pixels,
                                &download_error)) {
    log("Viewport screenshot download failed: " + download_error);
    return;
  }
  log("Viewport PNG encoding queued: " + request.output_display);
#else
  const std::string output_name = displayPath(request.output.filename());
  jobs_.submit(
      "Save screenshot " + output_name, JobPriority::Normal,
      [this, request = std::move(request), pixels = std::move(pixels)](
          std::stop_token stop, const JobSystem::Reporter &report) {
        report(0.1F, "encoding PNG");
        const ViewportCaptureResult result = writeViewportCapturePng(
            request.output, pixels, request.overwrite, stop);
        std::string message;
        switch (result.status) {
        case ViewportCaptureStatus::Written:
          message = "Saved viewport PNG " + request.output_display;
          break;
        case ViewportCaptureStatus::Skipped:
          message = "Viewport PNG skipped " + request.output_display + ": " +
                    result.message;
          break;
        case ViewportCaptureStatus::Cancelled:
          message = "Viewport PNG cancelled " + request.output_display;
          break;
        case ViewportCaptureStatus::Failed:
          message = "Viewport PNG failed " + request.output_display + ": " +
                    result.message;
          break;
        }
        ui_.post([this, message] { log(message); });
        if (result.status == ViewportCaptureStatus::Failed) {
          throw std::runtime_error(result.message);
        }
        if (result.status == ViewportCaptureStatus::Cancelled ||
            stop.stop_requested()) {
          return;
        }
        report(1.0F, result.status == ViewportCaptureStatus::Written
                         ? "saved"
                         : "skipped");
      });
#endif
}

void App::queueInspectionShareSave() {
#ifdef KPT_WEB_BUILD
  log("Review-share files are unavailable in web builds");
#else
  const auto output =
      decodeUiPath(inspection_share_output_, "Review share output path");
  if (!output)
    return;

  InspectionShareDocument document;
  try {
    // Capture all mutable Scene/settings state on the UI thread. The worker
    // receives no live App objects except its UI completion callback.
    document = InspectionShareFile::capture(inspection_scene_,
                                            inspection_settings_, *output);
  } catch (const std::exception &error) {
    log("Review share capture failed: " + std::string(error.what()));
    return;
  }

  const std::filesystem::path output_path = *output;
  const std::string output_display = displayPath(output_path);
  const std::string output_name = displayPath(output_path.filename());
  const bool overwrite = inspection_share_overwrite_;
  jobs_.submit(
      "Save review share " + output_name, JobPriority::Normal,
      [this, document = std::move(document), output_path,
       output_display, overwrite](std::stop_token stop,
                       const JobSystem::Reporter &report) {
        report(0.1F, "writing JSON");
        const InspectionShareSaveResult result =
            InspectionShareFile(output_path).save(document, overwrite, stop);
        std::string message;
        switch (result.status) {
        case InspectionShareSaveStatus::Written:
          message = "Saved review share " + output_display;
          break;
        case InspectionShareSaveStatus::Skipped:
          message = "Review share skipped " + output_display + ": " +
                    result.message;
          break;
        case InspectionShareSaveStatus::Cancelled:
          message = "Review share cancelled " + output_display;
          break;
        case InspectionShareSaveStatus::Failed:
          message = "Review share save failed " + output_display + ": " +
                    result.message;
          break;
        }
        ui_.post([this, message] { log(message); });
        if (result.status == InspectionShareSaveStatus::Failed) {
          throw std::runtime_error(result.message);
        }
        if (result.status == InspectionShareSaveStatus::Cancelled ||
            stop.stop_requested()) {
          return;
        }
        report(1.0F, result.status == InspectionShareSaveStatus::Written
                         ? "saved"
                         : "skipped");
      });
#endif
}

void App::applyInspectionShare(InspectionShareDocument document,
                               std::filesystem::path share_path) {
#ifdef KPT_WEB_BUILD
  static_cast<void>(document);
  static_cast<void>(share_path);
  log("Review-share files are unavailable in web builds");
#else
  // Do not mutate the active review until the worker completely parsed and
  // validated the share document. beginNewSource then invalidates all old
  // point-cloud completions before fresh runtime IDs are allocated.
  const std::uint64_t source_generation = beginNewSource();
  inspection_scene_.resetForImport();
  pending_initial_camera_snapshot_.reset();

  std::size_t queued_source_count = 0;
  std::size_t unresolved_source_count = 0;
  try {
    for (const InspectionShareLayer &layer : document.layers) {
      const LayerId layer_id = inspection_scene_.addLayer(layer.source_key);
      static_cast<void>(
          inspection_scene_.setLayerTransform(layer_id, layer.local_to_world));
      static_cast<void>(inspection_scene_.setLayerStyle(layer_id, layer.style));
      static_cast<void>(inspection_scene_.setLayerVisible(layer_id,
                                                           layer.visible));

      const auto source_path =
          InspectionShareFile::resolveSourcePath(share_path, layer);
      if (!source_path.has_value()) {
        ++unresolved_source_count;
        log("Review layer remains unresolved: " + layer.source_key);
        continue;
      }
      ++queued_source_count;
      queueInspectionShareLayerLoad(layer_id, layer.source_key, *source_path,
                                    source_generation);
    }

    inspection_scene_.setRoi(document.roi);
    for (const InspectionShareMeasurement &measurement : document.measurements) {
      if (measurement.second_source_key && measurement.second_world) {
        static_cast<void>(inspection_scene_.addMeasurement(
            measurement.first_source_key, measurement.first_world,
            *measurement.second_source_key, *measurement.second_world));
      } else {
        static_cast<void>(inspection_scene_.beginMeasurement(
            measurement.first_source_key, measurement.first_world));
      }
    }
    // InspectionSettings is application-level state. Merge imported names so
    // opening a review cannot discard unrelated user bookmarks; share names
    // intentionally replace same-name views. Import itself becomes a history
    // root rather than hundreds of undo steps.
    for (const CameraBookmark &bookmark : document.bookmarks) {
      inspection_settings_.saveBookmark(bookmark);
    }
    inspection_settings_.clearHistory();
    persistInspectionSettings();

    inspection_scene_.clearHistory();
    inspection_roi_controls_need_hydrate_ = true;
    hydrateInspectionRoiControlsFromScene();
    inspection_undo_domain_ = InspectionUndoDomain::Scene;
    log("Review share restored " + std::to_string(document.layers.size()) +
        " layer(s); " + std::to_string(queued_source_count) +
        " source(s) queued, " + std::to_string(unresolved_source_count) +
        " unresolved");
  } catch (const std::exception &error) {
    // A loaded document should already satisfy every semantic invariant. This
    // rollback still avoids exposing a partly constructed scene if allocation
    // or a future mutator validation fails.
    inspection_scene_.resetForImport();
    inspection_render_adapter_.clearSnapshots();
    inspection_render_list_.reset();
    main_viewport_.cancelAndClear();
    log("Review share import failed: " + std::string(error.what()));
  }
#endif
}

void App::queueInspectionShareLayerLoad(
    LayerId layer_id, std::string source_key, std::filesystem::path source_path,
    std::uint64_t source_generation) {
#ifdef KPT_WEB_BUILD
  static_cast<void>(layer_id);
  static_cast<void>(source_key);
  static_cast<void>(source_path);
  static_cast<void>(source_generation);
#else
  const std::string display_path = displayPath(source_path);
  const std::string filename = displayPath(source_path.filename());
  const std::uint64_t snapshot_revision =
      ++inspection_layer_snapshot_revision_;
  jobs_.submit(
      "Load review layer " + filename, JobPriority::High,
      [this, layer_id, source_key = std::move(source_key), source_path,
       display_path, source_generation,
       snapshot_revision](std::stop_token stop,
                          const JobSystem::Reporter &report) {
        try {
          report(0.1F, "loading source");
          const auto cloud = kpt::load(source_path, stop);
          if (stop.stop_requested())
            return;
          const auto snapshot =
              makeViewportCloudSnapshot(cloud, snapshot_revision, stop);
          if (stop.stop_requested())
            return;
          ui_.post([this, layer_id, source_key, cloud, snapshot, display_path,
                    source_generation] {
            if (source_generation != sequence_generation_)
              return;
            const CloudLayer *current = inspection_scene_.findLayer(layer_id);
            if (current == nullptr || current->sourceKey() != source_key)
              return;
            if (!inspection_render_adapter_.acceptSnapshot(layer_id, snapshot)) {
              log("Review layer snapshot rejected: " + display_path);
              return;
            }
            if (!inspection_scene_.hydrateLayerCloud(layer_id, cloud)) {
              inspection_render_adapter_.removeSnapshot(layer_id);
              return;
            }
            inspection_last_added_layer_ = layer_id;
            inspection_undo_domain_ = InspectionUndoDomain::Scene;
            refreshInspectionViewport(CameraUpdate::Preserve);
            log("Loaded review layer " + display_path + " (" +
                std::to_string(snapshot->vertices.size()) + " points)");
          });
          report(1.0F, "loaded " + std::to_string(cloud->size()) + " points");
        } catch (const OperationCancelled &) {
          // Cancellation is expected when a newer review replaces this one.
        } catch (const std::exception &error) {
          const std::string message = error.what();
          ui_.post([this, display_path, source_generation, message] {
            if (source_generation != sequence_generation_)
              return;
            log("Review layer unresolved " + display_path + ": " + message);
          });
          report(1.0F, "unresolved");
        }
      });
#endif
}

void App::addMeasurementFromLocalPick(const PickResult &pick) {
  const auto active_layer_id = inspection_scene_.activeLayer();
  if (!active_layer_id)
    return;
  const auto *layer = inspection_scene_.findLayer(*active_layer_id);
  if (layer == nullptr)
    return;

  // ViewportModel owns one local cloud only. Scene owns the transform boundary,
  // so all persisted Measurement positions are immutable world coordinates.
  const auto world = transformLocalToWorld(pick.cloud_position.cast<double>(),
                                           layer->localToWorld());
  if (!world)
    return;

  const auto &measurements = inspection_scene_.measurements();
  if (!measurements.empty()) {
    const auto &pending = measurements.back();
    if (!pending.secondWorld()) {
      if (inspection_scene_.completeMeasurement(pending.id(),
                                                layer->sourceKey(), *world)) {
        inspection_undo_domain_ = InspectionUndoDomain::Scene;
      }
      return;
    }
  }
  static_cast<void>(
      inspection_scene_.beginMeasurement(layer->sourceKey(), *world));
  inspection_undo_domain_ = InspectionUndoDomain::Scene;
}

void App::restorePendingCameraAfterInitialFit() {
  if (!pending_initial_camera_snapshot_)
    return;
  const CameraSnapshot snapshot = *pending_initial_camera_snapshot_;
  pending_initial_camera_snapshot_.reset();
  if (!main_viewport_.setCameraSnapshot(snapshot))
    log("Inspection settings ignored invalid saved camera");
}

void App::requestFrame(std::size_t index, bool apply, bool fit_camera) {
  if (!sequence_ || index >= sequence_->size())
    return;
  if (apply)
    playback_.request(index);
  if (frame_cache_.isPending(index))
    return;

  std::uint64_t request_generation = 0;
  if (const auto cached = frame_cache_.find(index)) {
    if (apply) {
      static_cast<void>(frame_cache_.begin(index));
      request_generation = main_viewport_.beginRequest();
      queueCachedFrame(index, cached, fit_camera, request_generation,
                       sequence_generation_);
    }
    return;
  }
  if (!frame_cache_.begin(index))
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
    stager->stage(assets, [this, index, apply, fit_camera, request_generation,
                           sequence_generation, stager,
                           assets = std::move(assets)](
                              std::optional<std::string> stage_error) mutable {
      ui_.post([this, index, apply, fit_camera, request_generation,
                sequence_generation, stager, assets = std::move(assets),
                stage_error = std::move(stage_error)]() mutable {
        if (stage_error) {
          if (sequence_generation != sequence_generation_)
            return;
          frame_cache_.finish(index);
          if (apply && playback_.failIfDesired(index)) {
            const std::string message = "Failed to stage sequence frame " +
                                        std::to_string(index) + ": " +
                                        *stage_error;
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
    });
    return;
  }
  queueFrameLoad(index, apply, fit_camera, request_generation,
                 sequence_generation, {});
}

void App::queueCachedFrame(std::size_t index, PointCloudIRGBConstPtr cloud,
                           bool fit_camera, std::uint64_t request_generation,
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
                frame_cache_.finish(index);
            });
            return;
          }
          auto snapshot =
              makeViewportCloudSnapshot(cloud, request_generation, stop);
          if (stop.stop_requested()) {
            ui_.post([this, index, sequence_generation] {
              if (sequence_generation == sequence_generation_)
                frame_cache_.finish(index);
            });
            return;
          }
          ui_.post([this, index, fit_camera, sequence_generation,
                    snapshot = std::move(snapshot)] {
            if (sequence_generation != sequence_generation_)
              return;
            frame_cache_.finish(index);
            if (playback_.desired() != index)
              return;
            if (!main_viewport_.accept(snapshot,
                                       fit_camera ? CameraUpdate::Fit
                                                  : CameraUpdate::Preserve)) {
              return;
            }
            restorePendingCameraAfterInitialFit();
            playback_.applied(index);
          });
          report(1.0F, "ready");
        } catch (...) {
          ui_.post([this, index, sequence_generation] {
            if (sequence_generation == sequence_generation_)
              frame_cache_.finish(index);
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
                frame_cache_.finish(index);
            });
            return;
          }
          auto snapshot =
              apply ? makeViewportCloudSnapshot(frame.cloud, request_generation,
                                                stop)
                    : std::shared_ptr<const ViewportCloudSnapshot>{};
          if (stop.stop_requested()) {
            ui_.post([this, index, sequence_generation] {
              if (sequence_generation == sequence_generation_)
                frame_cache_.finish(index);
            });
            return;
          }
          ui_.post([this, index, apply, fit_camera, cloud = frame.cloud,
                    snapshot = std::move(snapshot), sequence_generation] {
            if (sequence_generation != sequence_generation_)
              return;
            frame_cache_.finish(index);
            frame_cache_.store(index, cloud, playback_.current(),
                               playback_.desired());
            if (apply && playback_.desired() == index) {
              if (!main_viewport_.accept(snapshot,
                                         fit_camera ? CameraUpdate::Fit
                                                    : CameraUpdate::Preserve)) {
                return;
              }
              restorePendingCameraAfterInitialFit();
              playback_.applied(index);
              if (index == 0 && launch_state_ == LaunchState::Pending)
                launch_state_ = LaunchState::Ready;
              static_cast<void>(playback_.startAutoplayIfArmed(index));
              if (sequence_) {
                const auto prefetched = nextPlaybackFrame(
                    index, sequence_->size(), playback_.direction(), false);
                if (prefetched)
                  requestFrame(*prefetched, false);
              }
            } else if (!apply && playback_.desired() == index) {
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
            frame_cache_.finish(index);
            if (apply && playback_.failIfDesired(index)) {
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
            frame_cache_.finish(index);
            if (apply && playback_.failIfDesired(index)) {
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
  playback_.toggle(direction);
  jobs_.setPlayerActive(playback_.playing());
}

void App::resetPlayback() {
  playback_.resetTransport();
  jobs_.setPlayerActive(false);
  if (sequence_ && !sequence_->empty())
    requestFrame(0, true);
}

std::optional<std::size_t> App::nextPlaybackFrame(std::size_t current,
                                                  std::size_t frame_count,
                                                  PlaybackDirection direction,
                                                  bool loop) {
  return PlaybackEngine::nextFrame(current, frame_count, direction, loop);
}

void App::updatePlayback() {
  jobs_.setPlayerActive(playback_.playing());
  if (!sequence_ || sequence_->empty())
    return;
  const auto next = playback_.poll(sequence_->size());
  if (!next) {
    jobs_.setPlayerActive(playback_.playing());
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
  main_style_.point_size = 5.0F;
  main_style_.color_by = ColorBy::RGB;
  const auto snapshot = makeViewportCloudSnapshot(
      cloud, ++inspection_layer_snapshot_revision_);
  registerInspectionLayer("synthetic-smoke", cloud, snapshot,
                          CameraUpdate::Fit);
  restorePendingCameraAfterInitialFit();
}

} // namespace kpt::gui
