#include "gui/viewport/recording.hpp"

#include "platform/detail/atomic_replace.hpp"
#include "platform/utf8_path.hpp"

#include <algorithm>
#include <atomic>
#include <optional>
#include <stdexcept>
#include <utility>

namespace kpt::gui {

struct RecordingSession::Take {
  RecordingConfig config;
  PixelExtent extent;
  RecordingPacer pacer;
  RecordingFrameQueue queue;
  std::atomic<bool> cancelled{false};
  std::mutex error_mutex;
  std::optional<std::string> capture_error;
  std::function<void(const RecordingResult &)> completion;

  Take(const RecordingConfig &config_value, PixelExtent extent_value)
      : config(config_value), extent(extent_value),
        pacer(config_value.fps, config_value.duration_seconds) {}
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
  const int total = take->pacer.totalFrames();
  unsigned written = 0;
  bool fatal = false;
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
      report(static_cast<float>(written) / static_cast<float>(total),
             "encoding frames");
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
  if (config.duration_seconds < 1 || config.duration_seconds > 60)
    throw std::invalid_argument(
        "recording duration must be in [1, 60] seconds");
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

RecordingPacer::RecordingPacer(int fps, int duration_seconds)
    : fps_(fps), total_frames_(fps * duration_seconds) {
  if (fps < 1 || duration_seconds < 1)
    throw std::invalid_argument("pacer requires positive fps and duration");
}

double RecordingPacer::elapsedSeconds(
    std::chrono::steady_clock::time_point now) const {
  if (!started_)
    return 0.0;
  return std::chrono::duration<double>(now - start_).count();
}

unsigned RecordingPacer::consume(std::chrono::steady_clock::time_point now) {
  if (finished())
    return 0;
  if (!started_) {
    start_ = now;
    started_ = true;
  }
  const std::int64_t elapsed =
      std::chrono::duration_cast<std::chrono::nanoseconds>(now - start_)
          .count();
  if (elapsed < 0)
    return 0;
  // Frame slot n covers [n / fps, (n + 1) / fps); a slot is due once it has
  // begun, so the anchor tick emits the frame visible when Record was pressed
  // and the total cap keeps the take at exactly fps * duration frames.
  const std::int64_t begun = elapsed * fps_ / 1'000'000'000 + 1;
  const std::int64_t target = std::min(begun, std::int64_t{total_frames_});
  if (target <= static_cast<std::int64_t>(emitted_))
    return 0;
  const unsigned due = static_cast<unsigned>(target) - emitted_;
  emitted_ = static_cast<unsigned>(target);
  return due;
}

bool RecordingFrameQueue::push(Rgba8Image frame, unsigned due) {
  const std::lock_guard lock(mutex_);
  if (closed_)
    return false;
  if (due == 0)
    return true;
  if (frames_.size() < kMaxDistinctFrames) {
    frames_.push_back({std::move(frame), due});
  } else {
    frames_.back().repeat += due;
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

Result<void, std::string> RecordingSession::start(
    const RecordingConfig &config, PixelExtent extent,
    std::function<void(const RecordingResult &)> completion) {
  if (state_ != State::Idle)
    return std::string("a recording is already active");
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

  auto encoder = std::shared_ptr<RecordingEncoder>(
      createRecordingEncoder(config));
  auto encoded_name = platform::pathToUtf8(config.output.filename());
  const std::string name =
      "Record viewport " +
      std::string(config.format == RecordingFormat::Gif ? "GIF " : "AVI ") +
      (encoded_name ? encoded_name.value()
                    : config.output.filename().string());
  UiEvents &ui = ui_;
  jobs_.submit(name, JobPriority::High,
               [take, encoder, &ui](
                   std::stop_token stop, const JobSystem::Reporter &report) {
                 runEncodingJob(take, *encoder, ui, stop, report);
               });
  take_ = std::move(take);
  state_ = State::Recording;
  return {};
}

void RecordingSession::tick(
    std::chrono::steady_clock::time_point now,
    const std::function<Result<Rgba8Image, std::string>()> &capture) {
  if (state_ != State::Recording || !take_)
    return;
  const unsigned due = take_->pacer.consume(now);
  if (due == 0)
    return;
  auto frame = capture();
  if (!frame) {
    // A genuine capture error is fatal for the whole take.
    {
      std::lock_guard lock(take_->error_mutex);
      take_->capture_error = frame.error();
    }
    take_->cancelled.store(true);
    finishTake();
    return;
  }
  take_->queue.push(std::move(frame).value(), due);
  if (take_->pacer.finished())
    finishTake();
}

void RecordingSession::stop() {
  if (state_ != State::Recording || !take_)
    return;
  take_->cancelled.store(true);
  finishTake();
}

void RecordingSession::finishTake() {
  take_->queue.close();
  state_ = State::Finalizing;
}

RecordingConfig RecordingSession::config() const noexcept {
  return take_ ? take_->config : RecordingConfig{};
}

PixelExtent RecordingSession::extent() const noexcept {
  return take_ ? take_->extent : PixelExtent{};
}

unsigned RecordingSession::framesLeft() const noexcept {
  if (!take_)
    return 0;
  const unsigned emitted = take_->pacer.emittedFrames();
  const unsigned total = static_cast<unsigned>(take_->pacer.totalFrames());
  return emitted >= total ? 0U : total - emitted;
}

double RecordingSession::secondsLeft(
    std::chrono::steady_clock::time_point now) const {
  if (!take_)
    return 0.0;
  const double elapsed = take_->pacer.elapsedSeconds(now);
  const double duration = static_cast<double>(take_->config.duration_seconds);
  return std::max(0.0, duration - elapsed);
}

std::uint64_t RecordingSession::mergedFrames() const noexcept {
  return take_ ? take_->queue.mergedFrames() : 0U;
}

} // namespace kpt::gui
