#pragma once

#include "common/result.hpp"
#include "gui/jobs/job_system.hpp"
#include "gui/jobs/ui_events.hpp"
#include "gui/viewport/renderer.hpp"

#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <filesystem>
#include <functional>
#include <memory>
#include <mutex>
#include <stop_token>
#include <string>

namespace kpt::gui {

enum class RecordingFormat { Gif, Avi };

enum class RecordingStatus { Written, Skipped, Cancelled, Failed };

// What `RecordingConfig::limit_value` counts down: played-back seconds or
// displayed point-cloud sequence frames. A value of -1 means the take runs
// until the current playback pass reaches its natural boundary.
enum class RecordingLimit { Seconds, Frames };

struct RecordingResult {
  RecordingStatus status = RecordingStatus::Failed;
  std::string message;

  [[nodiscard]] bool completed() const noexcept {
    return status == RecordingStatus::Written ||
           status == RecordingStatus::Skipped;
  }
};

struct RecordingConfig {
  RecordingFormat format = RecordingFormat::Gif;
  // Output video fps, independent of the player's playback fps. A captured
  // sequence frame is held for the time it was displayed on the playhead.
  int fps = 10;
  RecordingLimit limit = RecordingLimit::Seconds;
  // Positive seconds or point-cloud frames, or -1 for "until the pass ends".
  int limit_value = 3;
  std::filesystem::path output;
  bool overwrite = false;
};

// Throws std::invalid_argument unless fps stays inside the encoder range
// (1..60), limit_value is positive or -1, and the output extension matches the
// selected format.
void validateRecordingConfig(const RecordingConfig &config);

// Per-output-frame GIF delay in centiseconds. The Bresenham form keeps the
// accumulated delay exact for any fps: 7 fps alternates 14/15 cs instead of a
// constant 14 cs that accumulates error.
[[nodiscard]] unsigned gifDelayCentiseconds(int fps,
                                            unsigned output_frame);

struct QueuedFrame {
  Rgba8Image frame;
  unsigned repeat = 1;
  // Displayed point-cloud frames represented by this entry, for progress.
  unsigned source_frames = 1;
};

// Bounded capture->encoder queue. The capture side never blocks and never
// drops a logical frame: when the queue is full the new repeat count merges
// into the tail entry, so the worker still accounts for every displayed
// frame and the output keeps the played duration.
class RecordingFrameQueue {
public:
  static constexpr std::size_t kMaxDistinctFrames = 8;

  // Returns false once the queue is closed; a closed queue still drains.
  bool push(Rgba8Image frame, unsigned repeat, unsigned source_frames = 1);
  enum class Pop { Entry, Closed, Interrupted };
  // Blocks until an entry is available, the queue is closed and drained, or
  // the stop token fires (external JobSystem cancellation).
  Pop pop(QueuedFrame &out, std::stop_token stop = {});
  void close();
  [[nodiscard]] bool closed() const;
  [[nodiscard]] std::size_t size() const;
  // Count of repeat/source counts merged into an existing tail entry.
  [[nodiscard]] std::uint64_t mergedFrames() const;

private:
  mutable std::mutex mutex_;
  std::condition_variable_any ready_;
  std::deque<QueuedFrame> frames_;
  bool closed_ = false;
  std::uint64_t merged_ = 0;
};

class RecordingEncoder {
public:
  virtual ~RecordingEncoder() = default;
  virtual Result<void, std::string> begin(PixelExtent extent, int fps) = 0;
  // Writes one queue entry: the frame emitted `repeat` times. Per-output-frame
  // timing is derived by the encoder from fps and its own output index, so a
  // merged entry never shifts the total duration.
  virtual Result<void, std::string> writeFrame(const Rgba8Image &frame,
                                               unsigned repeat) = 0;
  // Flush, backfill, close, then publish <output>.tmp through the atomic
  // replace helper. overwrite=false and an existing destination remove the
  // temporary and yield Skipped, mirroring writeImageAtomic.
  virtual RecordingResult finish() = 0;
  // Cancel path: close the file and delete the .tmp; never publishes.
  virtual void abort() noexcept = 0;
};

[[nodiscard]] std::unique_ptr<RecordingEncoder>
createGifEncoder(const RecordingConfig &config);
[[nodiscard]] std::unique_ptr<RecordingEncoder>
createAviMjpegEncoder(const RecordingConfig &config);
[[nodiscard]] std::unique_ptr<RecordingEncoder>
createRecordingEncoder(const RecordingConfig &config);

// Publishes a fully written temporary through the platform atomic replace
// helper. Shared by both encoders' finish() paths.
[[nodiscard]] RecordingResult
publishRecordedFile(const std::filesystem::path &temporary,
                    const std::filesystem::path &output, bool overwrite);

// UI-thread recording state machine:
//   Idle -> Waiting -> Recording -> Finalizing -> Idle
// prepare() only validates and freezes the take (Waiting): no encoder job,
// temporary file, capture, or budget consumption happens yet. begin() submits
// the encoding job on the first displayed playback frame. Each further
// displayed point-cloud frame is added through addFrame(), which measures how
// long the previous frame was on screen while the playhead advanced and turns
// that into output-frame repeats. finish() closes the take normally; stop()
// aborts it without publishing.
class RecordingSession {
public:
  enum class State { Idle, Waiting, Recording, Finalizing };

  RecordingSession(JobSystem &jobs, UiEvents &ui);
  ~RecordingSession();
  RecordingSession(const RecordingSession &) = delete;
  RecordingSession &operator=(const RecordingSession &) = delete;

  // Validates the config and arms the take. `extent` is frozen for the whole
  // take (the caller computes it once with the render scale pinned to 1.0).
  // `completion` runs on the UI thread with the final result.
  Result<void, std::string>
  prepare(const RecordingConfig &config, PixelExtent extent,
          std::function<void(const RecordingResult &)> completion = {});

  // Starts the encoder now that a capturable playback frame exists.
  // `playback_active` seeds the played-time clock used by a seconds budget.
  void begin(std::chrono::steady_clock::time_point now, bool playback_active);

  // Adds one displayed point-cloud frame. Returns true once the frame budget
  // is satisfied, at which point the caller must call finish(). Output slots
  // are due by played time, so at a low output fps some displayed point-cloud
  // frames may be dropped instead of stretching the output duration.
  bool addFrame(Rgba8Image frame, std::chrono::steady_clock::time_point now);

  // Pauses/resumes the played-time clock. Paused playback displays no new
  // sequence frames and consumes neither seconds nor point-cloud frames.
  void setPlaybackActive(bool active,
                         std::chrono::steady_clock::time_point now);

  // True once the played time has reached a positive seconds budget. The
  // caller finishes the take before capturing further frames.
  [[nodiscard]] bool secondsLimitReached(
      std::chrono::steady_clock::time_point now) const;

  // Natural end (pass boundary or satisfied budget): the remaining due output
  // slots are pushed, the queue closes, and the encoder publishes normally.
  // When the final frame has no following capture, its effective display
  // interval is the previously measured one, so it reaches the video whenever
  // that interval covers an output sample. The played duration is preserved
  // to within one output frame.
  void finish(std::chrono::steady_clock::time_point now);

  // Capture failure: the take aborts; completion reports Failed.
  void fail(std::string message);

  // User cancellation: the queue closes, the worker aborts the encoder, and
  // the completion callback reports Cancelled. While Waiting nothing has been
  // opened, so the completion is delivered directly.
  void stop();

  [[nodiscard]] State state() const noexcept { return state_; }
  [[nodiscard]] RecordingConfig config() const noexcept;
  [[nodiscard]] PixelExtent extent() const noexcept;
  [[nodiscard]] std::uint64_t capturedFrames() const noexcept;
  [[nodiscard]] double playedSeconds(std::chrono::steady_clock::time_point now) const;
  [[nodiscard]] std::uint64_t mergedFrames() const noexcept;

private:
  struct Take;
  friend void runEncodingJob(std::shared_ptr<Take>, RecordingEncoder &,
                             UiEvents &, std::stop_token,
                             const JobSystem::Reporter &);

  JobSystem &jobs_;
  UiEvents &ui_;
  std::shared_ptr<Take> take_;
  State state_ = State::Idle;
};

} // namespace kpt::gui
