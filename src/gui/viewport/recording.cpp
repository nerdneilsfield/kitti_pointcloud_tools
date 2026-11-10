#include "gui/viewport/recording.hpp"

#include "platform/detail/atomic_replace.hpp"
#include "platform/utf8_path.hpp"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <limits>
#include <optional>
#include <stdexcept>
#include <utility>

namespace kpt::gui {

struct RecordingSession::Take {
  RecordingConfig config;
  PixelExtent extent;
  RecordingFrameQueue queue;
  std::atomic<bool> cancelled{false};
  std::mutex error_mutex;
  std::optional<std::string> capture_error;
  std::function<void(const RecordingResult &)> completion;

  // Everything below is UI-thread only. The encoder worker reads the config
  // once and otherwise only touches the mutex-protected queue.
  bool clock_active = false;
  std::chrono::steady_clock::time_point clock_start{};
  double played_accumulated = 0.0;
  // The last captured sequence frame is held until the next capture or the
  // end of the take, when the output slots due by then are written.
  std::optional<Rgba8Image> pending;
  double pending_played = 0.0;
  // How long the previous source frame stayed on screen; the final frame has
  // no following capture, so its effective display interval is this one.
  double last_interval = 0.0;
  // Output frames already handed to the encoder. Played time, not source
  // frames, owns the output duration: each new capture writes the output
  // slots that became due since the previous one.
  std::uint64_t emitted_slots = 0;
  std::uint64_t frames = 0;

  Take(const RecordingConfig &config_value, PixelExtent extent_value)
      : config(config_value), extent(extent_value) {}

  [[nodiscard]] double
  playedSeconds(std::chrono::steady_clock::time_point now) const {
    if (!clock_active)
      return played_accumulated;
    const double running =
        std::chrono::duration<double>(now - clock_start).count();
    return played_accumulated + std::max(0.0, running);
  }
};

namespace {

[[nodiscard]] std::string
composePlatformError(const platform::PlatformError &error) {
  const std::string detail = error.system_error.message();
  return detail.empty() ? error.message : error.message + ": " + detail;
}

} // namespace

// Drains the bounded queue into the encoder on a JobSystem worker. Runs fully
// on the Take snapshot; nothing here touches the session itself.
void runEncodingJob(std::shared_ptr<RecordingSession::Take> take,
                    RecordingEncoder &encoder, UiEvents &ui,
                    std::stop_token stop, const JobSystem::Reporter &report) {
  RecordingResult result;
  unsigned written = 0;
  std::uint64_t sources = 0;
  bool fatal = false;
  const double expected_seconds =
      take->config.limit == RecordingLimit::Seconds &&
              take->config.limit_value > 0
          ? static_cast<double>(take->config.limit_value)
          : 0.0;
  const double expected_sources =
      take->config.limit == RecordingLimit::Frames &&
              take->config.limit_value > 0
          ? static_cast<double>(take->config.limit_value)
          : 0.0;
  if (auto begun = encoder.begin(take->extent, take->config.fps); !begun) {
    result = {RecordingStatus::Failed, begun.error()};
  } else {
    QueuedFrame entry;
    while (true) {
      const auto popped = take->queue.pop(entry, stop);
      if (popped == RecordingFrameQueue::Pop::Interrupted)
        break;
      if (popped == RecordingFrameQueue::Pop::Closed)
        break;
      if (auto written_frame = encoder.writeFrame(entry.frame, entry.repeat);
          !written_frame) {
        result = {RecordingStatus::Failed, written_frame.error()};
        fatal = true;
        break;
      }
      written += entry.repeat;
      sources += entry.source_frames;
      float progress = 0.0F;
      if (expected_seconds > 0.0) {
        progress = static_cast<float>(
            static_cast<double>(written) /
            (expected_seconds * static_cast<double>(take->config.fps)));
      } else if (expected_sources > 0.0) {
        progress = static_cast<float>(static_cast<double>(sources) /
                                      expected_sources);
      }
      report(std::min(progress, 0.99F), "recording frames");
    }
    if (fatal) {
      encoder.abort();
    } else if (stop.stop_requested() || take->cancelled.load()) {
      encoder.abort();
      result = {RecordingStatus::Cancelled, "operation cancelled"};
      std::lock_guard lock(take->error_mutex);
      if (take->capture_error)
        result = {RecordingStatus::Failed, *take->capture_error};
    } else {
      result = encoder.finish();
    }
  }
  if (take->completion)
    ui.post([take, result] { take->completion(result); });
}

void validateRecordingConfig(const RecordingConfig &config) {
  if (config.fps < 1 || config.fps > 60)
    throw std::invalid_argument("recording fps must be in [1, 60]");
  if (config.limit_value < -1 || config.limit_value == 0)
    throw std::invalid_argument(
        "recording limit must be a positive count or -1");
  if (config.output.empty())
    throw std::invalid_argument("recording output path is empty");
  const std::filesystem::path extension = config.output.extension();
  if (config.format == RecordingFormat::Gif ? extension != ".gif"
                                            : extension != ".avi")
    throw std::invalid_argument(
        "recording output extension must match the format");
}

unsigned gifDelayCentiseconds(int fps, unsigned output_frame) {
  const auto frames = static_cast<unsigned>(fps);
  const unsigned next =
      ((output_frame + 1U) * 100U + frames / 2U) / frames;
  const unsigned current = (output_frame * 100U + frames / 2U) / frames;
  return next - current;
}

// Output frame slots that are due after `played_seconds` of playback. Slots
// are counted from the start of the take and rounded once here, so the total
// never accumulates per-source-frame rounding error; the output duration
// stays within one output frame of the played time.
[[nodiscard]] std::uint64_t outputSlotsAt(double played_seconds, int fps) {
  if (!(played_seconds > 0.0) || fps < 1)
    return 0U;
  const double slots =
      std::floor(played_seconds * static_cast<double>(fps) + 0.5);
  if (!(slots > 0.0))
    return 0U;
  return static_cast<std::uint64_t>(std::min(
      slots, static_cast<double>(std::numeric_limits<std::uint64_t>::max())));
}

[[nodiscard]] unsigned repeatForSlots(std::uint64_t due) {
  return static_cast<unsigned>(
      std::min<std::uint64_t>(due, std::numeric_limits<unsigned>::max()));
}

bool RecordingFrameQueue::push(Rgba8Image frame, unsigned repeat,
                               unsigned source_frames) {
  const std::lock_guard lock(mutex_);
  if (closed_)
    return false;
  if (repeat == 0)
    return true;
  if (frames_.size() < kMaxDistinctFrames) {
    frames_.push_back({std::move(frame), repeat, source_frames});
  } else {
    frames_.back().repeat += repeat;
    frames_.back().source_frames += source_frames;
    ++merged_;
  }
  ready_.notify_one();
  return true;
}

RecordingFrameQueue::Pop RecordingFrameQueue::pop(QueuedFrame &out,
                                                  std::stop_token stop) {
  std::unique_lock lock(mutex_);
  ready_.wait(lock, stop,
              [&] { return !frames_.empty() || closed_ || stop.stop_requested(); });
  if (!frames_.empty()) {
    out = std::move(frames_.front());
    frames_.pop_front();
    return Pop::Entry;
  }
  return closed_ ? Pop::Closed : Pop::Interrupted;
}

void RecordingFrameQueue::close() {
  {
    const std::lock_guard lock(mutex_);
    closed_ = true;
  }
  ready_.notify_all();
}

bool RecordingFrameQueue::closed() const {
  const std::lock_guard lock(mutex_);
  return closed_;
}

std::size_t RecordingFrameQueue::size() const {
  const std::lock_guard lock(mutex_);
  return frames_.size();
}

std::uint64_t RecordingFrameQueue::mergedFrames() const {
  const std::lock_guard lock(mutex_);
  return merged_;
}

std::unique_ptr<RecordingEncoder>
createRecordingEncoder(const RecordingConfig &config) {
  return config.format == RecordingFormat::Gif ? createGifEncoder(config)
                                               : createAviMjpegEncoder(config);
}

RecordingResult publishRecordedFile(const std::filesystem::path &temporary,
                                    const std::filesystem::path &output,
                                    bool overwrite) {
  std::error_code error;
  if (!overwrite && std::filesystem::exists(output, error)) {
    std::filesystem::remove(temporary, error);
    return {RecordingStatus::Skipped, "destination already exists"};
  }
  auto replacer = platform::detail::createAtomicReplace();
  if (auto replaced = replacer->replace(temporary, output); !replaced)
    return {RecordingStatus::Failed,
            composePlatformError(replaced.error())};
  return {RecordingStatus::Written, {}};
}

RecordingSession::RecordingSession(JobSystem &jobs, UiEvents &ui)
    : jobs_(jobs), ui_(ui) {}

RecordingSession::~RecordingSession() {
  if (state_ == State::Recording)
    stop();
}

Result<void, std::string> RecordingSession::prepare(
    const RecordingConfig &config, PixelExtent extent,
    std::function<void(const RecordingResult &)> completion) {
  if (state_ != State::Idle)
    return std::string("a recording is already prepared or active");
  if (extent.width <= 0 || extent.height <= 0)
    return std::string("recording extent must have positive dimensions");
  try {
    validateRecordingConfig(config);
  } catch (const std::exception &error) {
    return std::string(error.what());
  }

  auto take = std::make_shared<Take>(config, extent);
  take->completion = [this, completion = std::move(completion)](
                         const RecordingResult &result) {
    state_ = State::Idle;
    take_.reset();
    if (completion)
      completion(result);
  };
  take_ = std::move(take);
  state_ = State::Waiting;
  return {};
}

void RecordingSession::begin(std::chrono::steady_clock::time_point now,
                             bool playback_active) {
  if (state_ != State::Waiting || !take_)
    return;
  take_->played_accumulated = 0.0;
  take_->clock_start = now;
  take_->clock_active = playback_active;

  auto encoder = std::shared_ptr<RecordingEncoder>(
      createRecordingEncoder(take_->config));
  auto encoded_name = platform::pathToUtf8(take_->config.output.filename());
  const std::string name =
      "Record viewport " +
      std::string(take_->config.format == RecordingFormat::Gif ? "GIF "
                                                               : "AVI ") +
      (encoded_name ? encoded_name.value()
                    : take_->config.output.filename().string());
  UiEvents &ui = ui_;
  auto take = take_;
  jobs_.submit(name, JobPriority::High,
               [take, encoder, &ui](
                   std::stop_token stop, const JobSystem::Reporter &report) {
                 runEncodingJob(take, *encoder, ui, stop, report);
               });
  state_ = State::Recording;
}

void RecordingSession::setPlaybackActive(
    bool active, std::chrono::steady_clock::time_point now) {
  if (state_ != State::Recording || !take_)
    return;
  if (active == take_->clock_active)
    return;
  if (active) {
    take_->clock_start = now;
    take_->clock_active = true;
  } else {
    const double running =
        std::chrono::duration<double>(now - take_->clock_start).count();
    take_->played_accumulated += std::max(0.0, running);
    take_->clock_active = false;
  }
}

bool RecordingSession::addFrame(Rgba8Image frame,
                                std::chrono::steady_clock::time_point now) {
  if (state_ != State::Recording || !take_)
    return false;
  const double played = take_->playedSeconds(now);
  if (take_->pending) {
    take_->last_interval = std::max(0.0, played - take_->pending_played);
    const std::uint64_t target = outputSlotsAt(played, take_->config.fps);
    if (target > take_->emitted_slots) {
      take_->queue.push(std::move(*take_->pending),
                        repeatForSlots(target - take_->emitted_slots), 1U);
      take_->emitted_slots = target;
    }
    // Otherwise this source frame fell inside an existing output slot: at
    // low output fps not every displayed point-cloud frame is encoded, but
    // the played duration is still accounted for by the next due slot.
  }
  take_->pending = std::move(frame);
  take_->pending_played = played;
  ++take_->frames;
  return take_->config.limit == RecordingLimit::Frames &&
         take_->config.limit_value > 0 &&
         take_->frames >=
             static_cast<std::uint64_t>(take_->config.limit_value);
}

void RecordingSession::finish(std::chrono::steady_clock::time_point now) {
  if (state_ != State::Recording || !take_)
    return;
  double played = take_->playedSeconds(now);
  if (take_->config.limit == RecordingLimit::Seconds &&
      take_->config.limit_value > 0) {
    played = std::min(played, static_cast<double>(take_->config.limit_value));
  }
  std::uint64_t target = outputSlotsAt(played, take_->config.fps);
  std::uint64_t due =
      target > take_->emitted_slots ? target - take_->emitted_slots : 0U;
  if (due == 0U && take_->pending) {
    // The final source frame has no following capture to measure it. Its
    // effective display interval is the previous frame's measured one, so it
    // reaches the video whenever that interval covers an output sample (the
    // frame budget stops on the same clock tick as the last capture).
    double displayed_until = take_->pending_played + take_->last_interval;
    if (take_->config.limit == RecordingLimit::Seconds &&
        take_->config.limit_value > 0) {
      displayed_until = std::min(
          displayed_until, static_cast<double>(take_->config.limit_value));
    }
    target = outputSlotsAt(std::max(played, displayed_until),
                           take_->config.fps);
    due = target > take_->emitted_slots ? target - take_->emitted_slots : 0U;
  }
  // A recorded take is never empty: even a pass that ends before one output
  // slot elapses still publishes a single frame.
  if (due == 0U && take_->emitted_slots == 0U && take_->pending)
    due = 1U;
  if (due > 0U && take_->pending) {
    take_->queue.push(std::move(*take_->pending), repeatForSlots(due), 1U);
    take_->emitted_slots += due;
  }
  take_->pending.reset();
  take_->queue.close();
  state_ = State::Finalizing;
}

void RecordingSession::fail(std::string message) {
  if (!take_)
    return;
  if (state_ == State::Waiting) {
    auto take = take_;
    state_ = State::Idle;
    take_.reset();
    if (take->completion)
      take->completion({RecordingStatus::Failed, std::move(message)});
    return;
  }
  if (state_ != State::Recording)
    return;
  {
    std::lock_guard lock(take_->error_mutex);
    take_->capture_error = std::move(message);
  }
  take_->cancelled.store(true);
  finish(std::chrono::steady_clock::now());
}

void RecordingSession::stop() {
  if (!take_)
    return;
  if (state_ == State::Waiting) {
    auto take = take_;
    state_ = State::Idle;
    take_.reset();
    if (take->completion)
      take->completion({RecordingStatus::Cancelled, "operation cancelled"});
    return;
  }
  if (state_ != State::Recording)
    return;
  take_->cancelled.store(true);
  finish(std::chrono::steady_clock::now());
}

bool RecordingSession::secondsLimitReached(
    std::chrono::steady_clock::time_point now) const {
  if (!take_ || state_ != State::Recording)
    return false;
  if (take_->config.limit != RecordingLimit::Seconds ||
      take_->config.limit_value <= 0)
    return false;
  return take_->playedSeconds(now) >=
         static_cast<double>(take_->config.limit_value);
}

RecordingConfig RecordingSession::config() const noexcept {
  return take_ ? take_->config : RecordingConfig{};
}

PixelExtent RecordingSession::extent() const noexcept {
  return take_ ? take_->extent : PixelExtent{};
}

std::uint64_t RecordingSession::capturedFrames() const noexcept {
  return take_ ? take_->frames : 0U;
}

double RecordingSession::playedSeconds(
    std::chrono::steady_clock::time_point now) const {
  return take_ ? take_->playedSeconds(now) : 0.0;
}

std::uint64_t RecordingSession::mergedFrames() const noexcept {
  return take_ ? take_->queue.mergedFrames() : 0U;
}

} // namespace kpt::gui
