#include "gui/viewport/recording.hpp"

#include "platform/utf8_path.hpp"

#include <gif.h>

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <system_error>
#include <vector>

namespace kpt::gui {
namespace {

// gif.h is a single non-inline header: every function is defined here, so it
// must be included by exactly one translation unit in the program.
class GifEncoder final : public RecordingEncoder {
public:
  explicit GifEncoder(const RecordingConfig &config) : config_(config) {}
  ~GifEncoder() override {
    if (open_)
      abort();
  }

  Result<void, std::string> begin(PixelExtent extent, int fps) override {
    if (open_)
      return std::string("GIF encoder already began");
    if (extent.width <= 0 || extent.height <= 0)
      return std::string("GIF extent must have positive dimensions");
    if (fps < 1)
      return std::string("GIF fps must be positive");
    extent_ = extent;
    fps_ = fps;
    output_index_ = 0;
    temporary_ = config_.output;
    temporary_ += ".tmp";
    const auto name = platform::pathToUtf8(temporary_);
    if (!name)
      return std::string(name.error().message);
    // A non-zero delay in GifBegin writes the NETSCAPE2.0 loop-forever block;
    // per-frame delays come from gifDelayCentiseconds in writeFrame.
    if (!GifBegin(&writer_, name.value().c_str(),
                  static_cast<std::uint32_t>(extent.width),
                  static_cast<std::uint32_t>(extent.height),
                  gifDelayCentiseconds(fps, 0), 8, false)) {
      writer_ = {};
      return std::string("cannot open GIF output file");
    }
    open_ = true;
    return {};
  }

  Result<void, std::string> writeFrame(const Rgba8Image &frame,
                                       unsigned repeat) override {
    if (!open_)
      return std::string("GIF encoder has no open file");
    if (frame.extent.width != extent_.width ||
        frame.extent.height != extent_.height)
      return std::string("GIF frame extent changed mid-recording");
    const auto width = static_cast<std::size_t>(extent_.width);
    const auto height = static_cast<std::size_t>(extent_.height);
    // gif.h indexes rows as width * 4; repack when the capture has padding.
    const std::uint8_t *rgba = frame.pixels.data();
    if (frame.bytes_per_row != width * 4U) {
      packed_.assign(width * height * 4U, 0);
      for (std::size_t row = 0; row < height; ++row) {
        std::copy(frame.pixels.data() + row * frame.bytes_per_row,
                  frame.pixels.data() + row * frame.bytes_per_row + width * 4U,
                  packed_.data() + row * width * 4U);
      }
      rgba = packed_.data();
    }
    for (unsigned output_frame = 0; output_frame < repeat; ++output_frame) {
      const unsigned delay =
          gifDelayCentiseconds(fps_, output_index_++);
      if (!GifWriteFrame(&writer_, rgba,
                         static_cast<std::uint32_t>(extent_.width),
                         static_cast<std::uint32_t>(extent_.height), delay, 8,
                         false))
        return std::string("GIF frame write failed");
    }
    return {};
  }

  RecordingResult finish() override {
    if (!open_)
      return {RecordingStatus::Failed, "GIF encoder has no open file"};
    open_ = false;
    if (!GifEnd(&writer_))
      return {RecordingStatus::Failed, "GIF finalize failed"};
    return publishRecordedFile(temporary_, config_.output, config_.overwrite);
  }

  void abort() noexcept override {
    if (!open_)
      return;
    open_ = false;
    (void)GifEnd(&writer_);
    std::error_code ignored;
    std::filesystem::remove(temporary_, ignored);
  }

private:
  RecordingConfig config_;
  PixelExtent extent_{};
  int fps_ = 0;
  unsigned output_index_ = 0;
  std::filesystem::path temporary_;
  GifWriter writer_{};
  bool open_ = false;
  std::vector<std::uint8_t> packed_;
};

} // namespace

std::unique_ptr<RecordingEncoder> createGifEncoder(const RecordingConfig &config) {
  return std::make_unique<GifEncoder>(config);
}

} // namespace kpt::gui
