#pragma once

#include "gui/runtime/runtime.hpp"
#include "i18n/i18n.hpp"
#include "platform/utf8_path.hpp"

#include "imgui.h"

#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>

#include <algorithm>
#include <iostream>
#include <string_view>

namespace kpt::gui::detail {

enum class CjkGlyphRange { Full, SimplifiedCommon };

inline void logPlatformError(
    std::string_view operation,
    const platform::PlatformError &platform_error) {
  std::cerr << operation << ": " << platform_error.message;
  if (platform_error.system_error)
    std::cerr << " (" << platform_error.system_error.message() << ')';
  std::cerr << '\n';
}

inline FramebufferMetrics glfwFramebufferMetrics(GLFWwindow *window) noexcept {
  FramebufferMetrics metrics;
  if (window == nullptr)
    return metrics;

  int logical_width = 0;
  int logical_height = 0;
  int framebuffer_width = 0;
  int framebuffer_height = 0;
  glfwGetWindowSize(window, &logical_width, &logical_height);
  glfwGetFramebufferSize(window, &framebuffer_width, &framebuffer_height);
  metrics.logical_size = {static_cast<float>(logical_width),
                          static_cast<float>(logical_height)};
  metrics.framebuffer_size = {std::max(0, framebuffer_width),
                              std::max(0, framebuffer_height)};
  float scale_x = 1.0F;
  float scale_y = 1.0F;
  glfwGetWindowContentScale(window, &scale_x, &scale_y);
  if (logical_width > 0 && framebuffer_width > 0)
    scale_x = static_cast<float>(framebuffer_width) / logical_width;
  if (logical_height > 0 && framebuffer_height > 0)
    scale_y = static_cast<float>(framebuffer_height) / logical_height;
  metrics.scale = {scale_x, scale_y};
  return metrics;
}

inline void configureRuntimeFonts(ImGuiIO &io,
                                  const GuiRuntimeOptions &options,
                                  CjkGlyphRange glyph_range) {
  ImFontConfig default_config;
  default_config.SizePixels = 16.0F;
  ImFont *default_font = io.Fonts->AddFontDefault(&default_config);
  if (options.fonts == nullptr)
    return;

  const auto cjk_font =
      options.fonts->matchUiFont(U"中文路径文件选择点云轨迹标签");
  if (!cjk_font) {
    logPlatformError("CJK font lookup disabled", cjk_font.error());
    if (kpt::i18n::needsCJK())
      std::cerr << "Warning: CJK language selected but no CJK font available; "
                   "UI may show missing glyphs\n";
    return;
  }
  if (!cjk_font.value()) {
    if (kpt::i18n::needsCJK())
      std::cerr << "Warning: CJK language selected but no CJK font found; UI "
                   "may show missing glyphs\n";
    return;
  }
  const auto utf8_path = platform::pathToUtf8(cjk_font.value()->file);
  if (!utf8_path) {
    logPlatformError("CJK font path conversion failed", utf8_path.error());
    return;
  }

  ImFontConfig config;
  config.MergeMode = true;
  config.PixelSnapH = true;
  config.FontNo = static_cast<ImU32>(cjk_font.value()->face_index);
  config.DstFont = default_font;
  const ImWchar *ranges = glyph_range == CjkGlyphRange::Full
                              ? io.Fonts->GetGlyphRangesChineseFull()
                              : io.Fonts->GetGlyphRangesChineseSimplifiedCommon();
  if (io.Fonts->AddFontFromFileTTF(utf8_path.value().c_str(), 16.0F, &config,
                                   ranges) == nullptr)
    std::cerr << "CJK font load failed: " << utf8_path.value() << '\n';
}

inline bool loadRuntimeSettings(const GuiRuntimeOptions &options) {
  const auto loaded = options.settings->loadIni();
  if (!loaded) {
    logPlatformError("ImGui settings disabled", loaded.error());
    return false;
  }
  if (loaded.value())
    ImGui::LoadIniSettingsFromMemory(loaded.value()->data(),
                                     loaded.value()->size());
  return true;
}

inline bool flushRuntimeSettings(const GuiRuntimeOptions &options) {
  std::size_t size = 0;
  const char *contents = ImGui::SaveIniSettingsToMemory(&size);
  const auto saved =
      options.settings->saveIniAtomically(std::string_view(contents, size));
  if (!saved) {
    logPlatformError("ImGui settings save failed; will retry", saved.error());
    return false;
  }
  ImGui::GetIO().WantSaveIniSettings = false;
  return true;
}

} // namespace kpt::gui::detail
