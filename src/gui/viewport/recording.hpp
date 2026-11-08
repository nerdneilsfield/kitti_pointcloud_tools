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
  int fps = 10;
  int duration_seconds = 3;
  std::filesystem::path output;
  bool overwrite = false;
};

// Throws std::invalid_argument unless fps and duration stay inside the UI
// ranges (1..60) and the output extension matches the selected format.
void validateRecordingConfig(const RecordingConfig &config);

// Per-output-frame GIF delay in centiseconds. The Bresenham form keeps the
// accumulated delay exact for any fps: 7 fps alternates 14/15 cs instead of a
// constant 14 cs that accumulates error.
[[nodiscard]] unsigned gifDelayCentiseconds(int fps,
                                            unsigned output_frame);

// Wall-clock pacer. The first consume() anchors the start time, so the first
// emitted frame is the one the user saw when Record was pressed. A frame slot
// counts as due once it has begun, capped at fps * duration; a slow display
// loop therefore yields one capture with a larger repeat count, never a lost
// slot. Integer nanosecond math avoids float drift at slot boundaries.
class RecordingPacer {
public:
  RecordingPacer(int fps, int duration_seconds);

  [[nodiscard]] int fps() const noexcept { return fps_; }
  [[nodiscard]] int totalFrames() const noexcept { return total_frames_; }
  [[nodiscard]] bool started() const noexcept { return started_; }
  [[nodiscard]] bool finished() const noexcept {
    return emitted_ >= static_cast<unsigned>(total_frames_);
  }
  [[nodiscard]] unsigned emittedFrames() const noexcept { return emitted_; }
  [[nodiscard]] double
  elapsedSeconds(std::chrono::steady_clock::time_point now) const;

  // Number of frames due at `now`; advances the emitted count. Returns 0
  // when nothing new is due.
  [[nodiscard]] unsigned consume(std::chrono::steady_clock::time_point now);

private:
  int fps_;
  int total_frames_;
  bool started_ = false;
  std::chrono::steady_clock::time_point start_{};
  unsigned emitted_ = 0;
};

struct QueuedFrame {
  Rgba8Image frame;
  unsigned repeat = 1;
};

// Bounded capture->encoder queue. The capture side never blocks and never
// drops a logical frame: when the queue is full the new due count merges into
// the tail entry's repeat, so the worker always accounts for every due slot
// and the output keeps exactly fps * duration frames.
class RecordingFrameQueue {
public:
  static constexpr std::size_t kMaxDistinctFrames = 8;

  // Returns false once the queue is closed; a closed queue still drains.
  bool push(Rgba8Image frame, unsigned due);
  enum class Pop { Entry, Closed, Interrupted };
  // Blocks until an entry is available, the queue is closed and drained, or
  // the stop token fires (external JobSystem cancellation).
  Pop pop(QueuedFrame &out, std::stop_token stop = {});
  void close();
  [[nodiscard]] bool closed() const;
  [[nodiscard]] std::size_t size() const;
  // Count of due counts merged into an existing tail entry.
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

// UI-thread recording state machine: Idle -> Recording -> Finalizing -> Idle.
// tick() paces captures on the wall clock and hands (frame, due) entries to
// the bounded queue; an ExportJob drains the queue into a RecordingEncoder at
// JobPriority::High. The worker owns the take through shared_ptr and never
// touches the session, so teardown never races the encoder.
class RecordingSession {
public:
  enum class State { Idle, Recording, Finalizing };

  RecordingSession(JobSystem &jobs, UiEvents &ui);
  ~RecordingSession();
  RecordingSession(const RecordingSession &) = delete;
  RecordingSession &operator=(const RecordingSession &) = delete;

  // Validates the config and submits the encoding job. `extent` is frozen for
  // the whole take (the caller computes it once with the render scale pinned
  // to 1.0). `completion` runs on the UI thread with the final result.
  Result<void, std::string>
  start(const RecordingConfig &config, PixelExtent extent,
        std::function<void(const RecordingResult &)> completion = {});

  // One call per App draw while Recording. `capture` returns the last
  // committed viewport frame; a genuine capture error aborts the take.
  void tick(std::chrono::steady_clock::time_point now,
            const std::function<Result<Rgba8Image, std::string>()> &capture);

  // User cancellation. The queue closes, the worker aborts the encoder, and
  // the completion callback reports Cancelled.
  void stop();

  [[nodiscard]] State state() const noexcept { return state_; }
  [[nodiscard]] RecordingConfig config() const noexcept;
  [[nodiscard]] PixelExtent extent() const noexcept;
  [[nodiscard]] unsigned framesLeft() const noexcept;
  [[nodiscard]] double
  secondsLeft(std::chrono::steady_clock::time_point now) const;
  [[nodiscard]] std::uint64_t mergedFrames() const noexcept;

private:
  struct Take;
  friend void runEncodingJob(std::shared_ptr<Take>, RecordingEncoder &,
                             UiEvents &, std::stop_token,
                             const JobSystem::Reporter &);
  void finishTake();

  JobSystem &jobs_;
  UiEvents &ui_;
  std::shared_ptr<Take> take_;
  State state_ = State::Idle;
};

} // namespace kpt::gui
