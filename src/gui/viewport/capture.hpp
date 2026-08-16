#pragma once

#include "gui/viewport/renderer.hpp"

#include "kpt/render/image.hpp"

#include <filesystem>
#include <stop_token>
#include <string>

namespace kpt::gui {

enum class ViewportCaptureStatus { Written, Skipped, Cancelled, Failed };

struct ViewportCaptureResult {
  ViewportCaptureStatus status = ViewportCaptureStatus::Failed;
  std::string message;

  [[nodiscard]] bool completed() const noexcept {
    return status == ViewportCaptureStatus::Written ||
           status == ViewportCaptureStatus::Skipped;
  }
};

// Converts top-left RGBA8 pixels into the RGB layout accepted by kpt's PNG
// writer. `bytes_per_row` may include backend alignment padding.
[[nodiscard]] ImageRGB8 rgba8ToRgbImage(const Rgba8Image &image,
                                         std::stop_token stop = {});

// Encodes a CPU-owned screenshot through kpt::writeImageAtomic. It is safe to
// call from a worker after captureRgba() returned on the render thread.
[[nodiscard]] ViewportCaptureResult writeViewportCapturePng(
    const std::filesystem::path &output, const Rgba8Image &image,
    bool overwrite, std::stop_token stop = {});

} // namespace kpt::gui
