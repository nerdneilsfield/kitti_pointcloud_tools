#include "gui/viewport/recording.hpp"

#include "gui/viewport/capture.hpp"
#include "kpt/render/detail/stb_jpg.hpp"

#include <array>
#include <cstdint>
#include <filesystem>
#include <ostream>
#include <fstream>
#include <stdexcept>
#include <string>
#include <system_error>
#include <vector>

namespace kpt::gui {
namespace {

constexpr std::uint32_t kAviHasIndex = 0x00000010U;
constexpr std::uint32_t kAviKeyFrame = 0x00000010U;
// stb enables 4:2:0 chroma subsampling at quality <= 90 and edge-pads odd
// dimensions into 16x16 MCUs; 95 keeps 4:4:4 so odd viewport sizes are clean.
constexpr int kAviJpegQuality = 95;

void writeU16(std::ostream &out, std::uint16_t value) {
  const std::array<std::uint8_t, 2> bytes{static_cast<std::uint8_t>(value),
                                          static_cast<std::uint8_t>(value >> 8)};
  out.write(reinterpret_cast<const char *>(bytes.data()), bytes.size());
}

void writeU32(std::ostream &out, std::uint32_t value) {
  const std::array<std::uint8_t, 4> bytes{
      static_cast<std::uint8_t>(value),
      static_cast<std::uint8_t>(value >> 8U),
      static_cast<std::uint8_t>(value >> 16U),
      static_cast<std::uint8_t>(value >> 24U)};
  out.write(reinterpret_cast<const char *>(bytes.data()), bytes.size());
}

void writeFourcc(std::ostream &out, const char fourcc[4]) {
  out.write(fourcc, 4);
}

class AviMjpegEncoder final : public RecordingEncoder {
public:
  explicit AviMjpegEncoder(const RecordingConfig &config)
      : config_(config) {}
  ~AviMjpegEncoder() override {
    if (open_)
      abort();
  }

  Result<void, std::string> begin(PixelExtent extent, int fps) override {
    if (open_)
      return std::string("AVI encoder already began");
    if (extent.width <= 0 || extent.height <= 0)
      return std::string("AVI extent must have positive dimensions");
    if (fps < 1)
      return std::string("AVI fps must be positive");
    extent_ = extent;
    fps_ = fps;
    frames_ = 0;
    movi_offset_ = 4;
    index_.clear();
    temporary_ = config_.output;
    temporary_ += ".tmp";
    output_.open(temporary_, std::ios::binary | std::ios::trunc);
    if (!output_)
      return std::string("cannot open AVI output file");
    open_ = true;
    writeHeader();
    return {};
  }

  Result<void, std::string> writeFrame(const Rgba8Image &frame,
                                       unsigned repeat) override {
    if (!open_)
      return std::string("AVI encoder has no open file");
    if (frame.extent.width != extent_.width ||
        frame.extent.height != extent_.height)
      return std::string("AVI frame extent changed mid-recording");
    try {
      // The same JPEG chunk is written `repeat` times with one idx1 entry per
      // chunk, so a merged queue entry never shifts the total duration.
      const ImageRGB8 rgb = rgba8ToRgbImage(frame);
      const std::vector<std::uint8_t> jpeg =
          render_detail::writeJpegToMemory(rgb.view(), kAviJpegQuality);
      const std::uint32_t size = static_cast<std::uint32_t>(jpeg.size());
      // RIFF chunks are padded to even length; the pad byte is excluded from
      // the chunk size but shifts every following chunk offset.
      const std::uint32_t padded = size + (size & 1U);
      for (unsigned output_frame = 0; output_frame < repeat; ++output_frame) {
        index_.push_back({movi_offset_, size});
        writeFourcc(output_, "00db");
        writeU32(output_, size);
        output_.write(reinterpret_cast<const char *>(jpeg.data()),
                      static_cast<std::streamsize>(size));
        if (size & 1U)
          output_.put('\0');
        movi_offset_ += 8U + padded;
        ++frames_;
      }
    } catch (const std::exception &error) {
      return std::string(error.what());
    }
    if (!output_)
      return std::string("AVI frame write failed");
    return {};
  }

  RecordingResult finish() override {
    if (!open_)
      return {RecordingStatus::Failed, "AVI encoder has no open file"};
    open_ = false;
    if (frames_ == 0) {
      abort();
      return {RecordingStatus::Failed, "AVI recording produced no frames"};
    }
    writeIndex();
    backfillSizes();
    output_.flush();
    if (!output_) {
      abort();
      return {RecordingStatus::Failed, "AVI finalize failed"};
    }
    output_.close();
    return publishRecordedFile(temporary_, config_.output, config_.overwrite);
  }

  void abort() noexcept override {
    if (!output_.is_open())
      return;
    open_ = false;
    output_.close();
    std::error_code ignored;
    std::filesystem::remove(temporary_, ignored);
  }

private:
  struct IndexEntry {
    std::uint32_t offset;
    std::uint32_t size;
  };

  void writeHeader() {
    // RIFF header; the size is only known after idx1 and is backfilled.
    writeFourcc(output_, "RIFF");
    riff_size_position_ = output_.tellp();
    writeU32(output_, 0);
    writeFourcc(output_, "AVI ");

    // hdrl: 'hdrl' fourcc + avih chunk (8 + 56) + strl list (8 + 4 + 64 + 48)
    writeFourcc(output_, "LIST");
    writeU32(output_, 4U + (8U + 56U) + (8U + 4U + (8U + 56U) + (8U + 40U)));
    writeFourcc(output_, "hdrl");

    writeFourcc(output_, "avih");
    writeU32(output_, 56U);
    writeU32(output_, 1'000'000U / static_cast<std::uint32_t>(fps_));
    writeU32(output_, 0);                       // dwMaxBytesPerSec
    writeU32(output_, 0);                       // dwPaddingGranularity
    writeU32(output_, kAviHasIndex);            // dwFlags
    avih_frames_position_ = output_.tellp();
    writeU32(output_, 0);                       // dwTotalFrames (backfilled)
    writeU32(output_, 0);                       // dwInitialFrames
    writeU32(output_, 1);                       // dwStreams
    writeU32(output_, 0);                       // dwSuggestedBufferSize
    writeU32(output_, static_cast<std::uint32_t>(extent_.width));
    writeU32(output_, static_cast<std::uint32_t>(extent_.height));
    for (int reserved = 0; reserved < 4; ++reserved)
      writeU32(output_, 0);

    // strl: 'strl' fourcc + strh chunk (8 + 56) + strf chunk (8 + 40)
    writeFourcc(output_, "LIST");
    writeU32(output_, 4U + (8U + 56U) + (8U + 40U));
    writeFourcc(output_, "strl");

    writeFourcc(output_, "strh");
    writeU32(output_, 56U);
    writeFourcc(output_, "vids");
    writeFourcc(output_, "MJPG");
    writeU32(output_, 0);                       // dwFlags
    writeU16(output_, 0);                       // wPriority
    writeU16(output_, 0);                       // wLanguage
    writeU32(output_, 0);                       // dwInitialFrames
    writeU32(output_, 1);                       // dwScale
    writeU32(output_, static_cast<std::uint32_t>(fps_)); // dwRate
    writeU32(output_, 0);                       // dwStart
    strh_length_position_ = output_.tellp();
    writeU32(output_, 0);                       // dwLength (backfilled)
    writeU32(output_, 0);                       // dwSuggestedBufferSize
    writeU32(output_, 0);                       // dwQuality
    writeU32(output_, 0);                       // dwSampleSize
    // rcFrame (left, top, right, bottom) completes the 56-byte strh record.
    for (int corner = 0; corner < 4; ++corner)
      writeU16(output_, 0);

    writeFourcc(output_, "strf");
    writeU32(output_, 40U);                     // BITMAPINFOHEADER
    writeU32(output_, 40U);                     // biSize
    writeU32(output_, static_cast<std::uint32_t>(extent_.width));
    writeU32(output_, static_cast<std::uint32_t>(extent_.height));
    writeU16(output_, 1);                       // biPlanes
    writeU16(output_, 24);                      // biBitCount
    writeFourcc(output_, "MJPG");               // biCompression
    writeU32(output_, static_cast<std::uint32_t>(extent_.width) *
                          static_cast<std::uint32_t>(extent_.height) * 3U);
    writeU32(output_, 0);                       // biXPelsPerMeter
    writeU32(output_, 0);                       // biYPelsPerMeter
    writeU32(output_, 0);                       // biClrUsed
    writeU32(output_, 0);                       // biClrImportant

    // movi list; its size is backfilled after the last frame.
    writeFourcc(output_, "LIST");
    movi_size_position_ = output_.tellp();
    writeU32(output_, 0);
    writeFourcc(output_, "movi");
  }

  void writeIndex() {
    writeFourcc(output_, "idx1");
    writeU32(output_, static_cast<std::uint32_t>(index_.size() * 16U));
    for (const IndexEntry &entry : index_) {
      writeFourcc(output_, "00db");
      writeU32(output_, kAviKeyFrame);
      writeU32(output_, entry.offset);
      writeU32(output_, entry.size);
    }
  }

  // Frame counts, the movi list size, and the RIFF size are only known after
  // the last frame; seek back to the recorded positions, patch, and leave the
  // stream positioned at the end.
  void backfillSizes() {
    const std::uint32_t frames = static_cast<std::uint32_t>(frames_);
    const std::streampos end = output_.tellp();
    output_.seekp(riff_size_position_);
    writeU32(output_, static_cast<std::uint32_t>(end) - 8U);
    output_.seekp(avih_frames_position_);
    writeU32(output_, frames);
    output_.seekp(strh_length_position_);
    writeU32(output_, frames);
    output_.seekp(movi_size_position_);
    writeU32(output_, movi_offset_);
    output_.seekp(end);
  }

  RecordingConfig config_;
  PixelExtent extent_{};
  int fps_ = 0;
  std::uint32_t frames_ = 0;
  std::uint32_t movi_offset_ = 0;
  std::vector<IndexEntry> index_;
  std::filesystem::path temporary_;
  std::ofstream output_;
  std::streampos riff_size_position_{};
  std::streampos avih_frames_position_{};
  std::streampos strh_length_position_{};
  std::streampos movi_size_position_{};
  bool open_ = false;
};

} // namespace

std::unique_ptr<RecordingEncoder>
createAviMjpegEncoder(const RecordingConfig &config) {
  return std::make_unique<AviMjpegEncoder>(config);
}

} // namespace kpt::gui
