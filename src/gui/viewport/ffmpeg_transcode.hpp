#pragma once

#include "gui/viewport/recording.hpp"

#include <filesystem>
#include <optional>
#include <stop_token>

namespace kpt::gui {

// Resolves the ffmpeg CLI once, without assuming the GUI process inherited a
// shell PATH: Homebrew directories are included on macOS because a packaged
// .app launched from Finder sees neither. Empty means the optional MP4
// conversion is unavailable; recording itself never depends on it.
[[nodiscard]] std::optional<std::filesystem::path> findFfmpegExecutable();

// Converts a recorded MJPEG AVI into an H.264 MP4 through the ffmpeg CLI.
// Writes a unique .part.mp4 next to `mp4`, publishes it with the same atomic
// replace helper the encoders use, and never touches `avi`. A zero exit status
// alone is not success: the output must also exist and be non-empty, because
// ffmpeg exits 0 when it refuses to overwrite without -y.
[[nodiscard]] RecordingResult
transcodeAviToMp4(const std::filesystem::path &ffmpeg,
                  const std::filesystem::path &avi,
                  const std::filesystem::path &mp4, bool overwrite,
                  std::stop_token stop = {});

} // namespace kpt::gui
