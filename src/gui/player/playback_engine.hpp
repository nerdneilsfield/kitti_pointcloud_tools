#pragma once

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <optional>

namespace kpt::gui {

// Owns playback transitions and timing. App supplies frame availability and
// performs I/O; this class keeps UI events, async completions, and timer ticks
// from each implementing subtly different state transitions.
class PlaybackEngine {
public:
  enum class Direction { Forward, Reverse };
  using Clock = std::chrono::steady_clock;

  void configure(int frames_per_second, bool autoplay) noexcept {
    fps_ = std::max(1, frames_per_second);
    autoplay_armed_ = autoplay;
  }

  void resetSource(Clock::time_point now = Clock::now()) noexcept {
    playing_ = false;
    direction_ = Direction::Forward;
    current_ = 0;
    desired_ = 0;
    next_frame_time_ = now;
  }

  void stop() noexcept { playing_ = false; }
  void disarmAutoplay() noexcept { autoplay_armed_ = false; }

  void toggle(Direction direction,
              Clock::time_point now = Clock::now()) noexcept {
    if (playing_ && direction_ == direction) {
      playing_ = false;
    } else {
      playing_ = true;
      direction_ = direction;
    }
    next_frame_time_ = now;
  }

  void resetTransport(Clock::time_point now = Clock::now()) noexcept {
    playing_ = false;
    direction_ = Direction::Forward;
    next_frame_time_ = now;
  }

  void request(std::size_t index) noexcept { desired_ = index; }
  void applied(std::size_t index) noexcept { current_ = index; }

  [[nodiscard]] bool failIfDesired(std::size_t index) noexcept {
    if (desired_ != index)
      return false;
    desired_ = current_;
    playing_ = false;
    autoplay_armed_ = false;
    return true;
  }

  [[nodiscard]] bool startAutoplayIfArmed(
      std::size_t index, Clock::time_point now = Clock::now()) noexcept {
    if (index != 0 || !autoplay_armed_)
      return false;
    autoplay_armed_ = false;
    playing_ = true;
    direction_ = Direction::Forward;
    next_frame_time_ = now + frameInterval();
    return true;
  }

  [[nodiscard]] std::optional<std::size_t>
  poll(std::size_t frame_count,
       Clock::time_point now = Clock::now()) noexcept {
    if (!playing_ || frame_count == 0 || desired_ != current_ ||
        now < next_frame_time_) {
      return std::nullopt;
    }
    next_frame_time_ = now + frameInterval();
    const auto next = nextFrame(desired_, frame_count, direction_, loop_);
    if (!next)
      playing_ = false;
    return next;
  }

  [[nodiscard]] static std::optional<std::size_t>
  nextFrame(std::size_t current, std::size_t frame_count,
            Direction direction, bool loop) noexcept {
    if (frame_count == 0 || current >= frame_count)
      return std::nullopt;
    if (direction == Direction::Reverse) {
      if (current > 0)
        return current - 1;
      return loop ? std::optional<std::size_t>(frame_count - 1)
                  : std::nullopt;
    }
    if (current + 1 < frame_count)
      return current + 1;
    return loop ? std::optional<std::size_t>(0) : std::nullopt;
  }

  [[nodiscard]] bool playing() const noexcept { return playing_; }
  [[nodiscard]] Direction direction() const noexcept { return direction_; }
  [[nodiscard]] std::size_t current() const noexcept { return current_; }
  [[nodiscard]] std::size_t desired() const noexcept { return desired_; }
  [[nodiscard]] bool autoplayArmed() const noexcept { return autoplay_armed_; }
  [[nodiscard]] int fps() const noexcept { return fps_; }
  [[nodiscard]] bool loop() const noexcept { return loop_; }
  int &fpsControl() noexcept { return fps_; }
  bool &loopControl() noexcept { return loop_; }

private:
  [[nodiscard]] Clock::duration frameInterval() const noexcept {
    return std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<double>(1.0 / static_cast<double>(fps_)));
  }

  std::size_t current_ = 0;
  std::size_t desired_ = 0;
  bool playing_ = false;
  Direction direction_ = Direction::Forward;
  bool autoplay_armed_ = false;
  bool loop_ = false;
  int fps_ = 10;
  Clock::time_point next_frame_time_ = Clock::now();
};

} // namespace kpt::gui
