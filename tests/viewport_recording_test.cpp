#include "gui/viewport/recording.hpp"
#include "kpt/render/detail/stb_jpg.hpp"

#include <catch2/catch.hpp>

#define STB_IMAGE_IMPLEMENTATION
#include <stb_image.h>

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <stop_token>
#include <thread>
#include <utility>
#include <vector>

namespace {

using namespace kpt::gui;

class TemporaryDirectory {
public:
  TemporaryDirectory() {
    path_ = std::filesystem::temp_directory_path() /
            ("kpt-viewport-recording-" +
             std::to_string(
                 std::chrono::steady_clock::now().time_since_epoch().count()));
    std::filesystem::create_directories(path_);
  }
  ~TemporaryDirectory() {
    std::error_code error;
    std::filesystem::remove_all(path_, error);
  }
  [[nodiscard]] const std::filesystem::path &path() const { return path_; }

private:
  std::filesystem::path path_;
};

Rgba8Image solidFrame(int width, int height, std::uint8_t red,
                      std::uint8_t green, std::uint8_t blue) {
  Rgba8Image image;
  image.extent = {width, height};
  image.bytes_per_row = static_cast<std::size_t>(width) * 4U;
  image.pixels.assign(static_cast<std::size_t>(width) * height * 4U, 0);
  for (std::size_t pixel = 0;
       pixel < static_cast<std::size_t>(width) * height; ++pixel) {
    image.pixels[pixel * 4U] = red;
    image.pixels[pixel * 4U + 1U] = green;
    image.pixels[pixel * 4U + 2U] = blue;
    image.pixels[pixel * 4U + 3U] = 255;
  }
  return image;
}

std::vector<std::uint8_t> readFile(const std::filesystem::path &path) {
  std::ifstream input(path, std::ios::binary);
  return {std::istreambuf_iterator<char>(input), {}};
}

std::uint32_t u32At(const std::vector<std::uint8_t> &bytes, std::size_t at) {
  return static_cast<std::uint32_t>(bytes[at]) |
         (static_cast<std::uint32_t>(bytes[at + 1]) << 8U) |
         (static_cast<std::uint32_t>(bytes[at + 2]) << 16U) |
         (static_cast<std::uint32_t>(bytes[at + 3]) << 24U);
}

std::string stringAt(const std::vector<std::uint8_t> &bytes, std::size_t at,
                     std::size_t length) {
  return {reinterpret_cast<const char *>(bytes.data()) + at, length};
}

std::string fourccAt(const std::vector<std::uint8_t> &bytes, std::size_t at) {
  return stringAt(bytes, at, 4);
}

struct AviIndexEntry {
  std::uint32_t offset = 0;
  std::uint32_t size = 0;
};

struct ParsedAvi {
  std::uint32_t riff_size = 0;
  std::uint32_t micro_sec_per_frame = 0;
  std::uint32_t total_frames = 0;
  std::uint32_t width = 0;
  std::uint32_t height = 0;
  std::uint32_t scale = 0;
  std::uint32_t rate = 0;
  std::uint32_t length = 0;
  std::string handler;
  std::uint32_t movi_list_size = 0;
  std::vector<std::vector<std::uint8_t>> chunks;
  std::vector<AviIndexEntry> index;
};

void walkAvi(ParsedAvi &parsed, const std::vector<std::uint8_t> &bytes,
             std::size_t at, std::size_t end, bool inside_movi) {
  while (at + 8 <= end) {
    const std::string id = fourccAt(bytes, at);
    const std::uint32_t size = u32At(bytes, at + 4);
    const std::size_t body = at + 8;
    REQUIRE(body + size <= end);
    if (inside_movi && id == "00db") {
      parsed.chunks.emplace_back(bytes.begin() + body,
                                 bytes.begin() + body + size);
    } else if (id == "idx1") {
      for (std::uint32_t entry = 0; entry < size / 16U; ++entry) {
        const std::size_t base = body + entry * 16U;
        REQUIRE(fourccAt(bytes, base) == "00db");
        parsed.index.push_back({u32At(bytes, base + 8),
                                u32At(bytes, base + 12)});
      }
    } else if (id == "LIST") {
      const std::string list = fourccAt(bytes, body);
      if (list == "movi") {
        parsed.movi_list_size = size;
        walkAvi(parsed, bytes, body + 4, body + size, true);
      } else if (list == "hdrl" || list == "strl") {
        walkAvi(parsed, bytes, body + 4, body + size, false);
      }
    } else if (id == "avih") {
      parsed.micro_sec_per_frame = u32At(bytes, body);
      parsed.total_frames = u32At(bytes, body + 16);
      parsed.width = u32At(bytes, body + 32);
      parsed.height = u32At(bytes, body + 36);
    } else if (id == "strh") {
      parsed.handler = fourccAt(bytes, body + 4);
      parsed.scale = u32At(bytes, body + 20);
      parsed.rate = u32At(bytes, body + 24);
      parsed.length = u32At(bytes, body + 32);
    }
    at = body + size + (size & 1U);
  }
}

ParsedAvi parseAvi(const std::vector<std::uint8_t> &bytes) {
  ParsedAvi parsed;
  parsed.riff_size = u32At(bytes, 4);
  walkAvi(parsed, bytes, 12, bytes.size(), false);
  return parsed;
}

void waitForCompletion(UiEvents &ui, std::atomic<bool> &done) {
  for (int waited = 0; waited < 5000 && !done.load(); waited += 2) {
    ui.drain();
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  REQUIRE(done.load());
}

} // namespace

TEST_CASE("recording frame queue merges repeats instead of dropping",
          "[recording]") {
  RecordingFrameQueue queue;
  for (int frame = 0; frame < 8; ++frame)
    REQUIRE(queue.push(solidFrame(2, 2, static_cast<std::uint8_t>(frame), 0, 0),
                       1U));
  REQUIRE(queue.size() == 8U);
  // The queue is full: the new due count merges into the tail entry.
  CHECK(queue.push(solidFrame(2, 2, 9, 0, 0), 3U));
  CHECK(queue.size() == 8U);
  CHECK(queue.mergedFrames() == 1U);
  CHECK(!queue.closed());
  queue.close();
  CHECK(queue.closed());
  CHECK(!queue.push(solidFrame(2, 2, 10, 0, 0), 1U));

  unsigned accounted = 0;
  unsigned tail_repeat = 0;
  QueuedFrame entry;
  while (queue.pop(entry) == RecordingFrameQueue::Pop::Entry) {
    accounted += entry.repeat;
    tail_repeat = entry.repeat;
  }
  // 8 pushed frames + 3 merged due slots: nothing is lost.
  CHECK(accounted == 11U);
  CHECK(tail_repeat == 4U);
  CHECK(queue.pop(entry) == RecordingFrameQueue::Pop::Closed);
}

TEST_CASE("recording frame queue pop honors close and stop tokens",
          "[recording]") {
  SECTION("close wakes a blocked pop") {
    RecordingFrameQueue queue;
    std::thread waiter([&] {
      QueuedFrame entry;
      CHECK(queue.pop(entry) == RecordingFrameQueue::Pop::Closed);
    });
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    queue.close();
    waiter.join();
  }

  SECTION("a stop token interrupts a blocked pop") {
    RecordingFrameQueue queue;
    std::stop_source source;
    std::thread waiter([&] {
      QueuedFrame entry;
      CHECK(queue.pop(entry, source.get_token()) ==
            RecordingFrameQueue::Pop::Interrupted);
    });
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    source.request_stop();
    waiter.join();
  }
}

TEST_CASE("queue accounting preserves every pushed output frame",
          "[recording]") {
  // A slow encoder saturates the queue; merged repeats still account for
  // every output frame and every displayed sequence frame.
  RecordingFrameQueue queue;
  for (int frame = 0; frame < 20; ++frame)
    queue.push(solidFrame(4, 4, 1, 2, 3), 3U, 1U);
  queue.close();
  unsigned accounted = 0;
  std::uint64_t sources = 0;
  QueuedFrame entry;
  while (queue.pop(entry) == RecordingFrameQueue::Pop::Entry) {
    accounted += entry.repeat;
    sources += entry.source_frames;
  }
  CHECK(accounted == 60U);
  CHECK(sources == 20U);
  CHECK(queue.mergedFrames() > 0U);
}

TEST_CASE("gif delays follow the Bresenham pattern", "[recording]") {
  // 7 fps: round(n * 100 / 7) advances by 15 exactly twice per 7 frames, so
  // the sequence is 14,15,14,14,14,15,14 and sums to exactly one second.
  // (A constant 14 cs would drift; the design's "alternate" wording is the
  // formula, not strict alternation.)
  const std::array<unsigned, 7> seven_fps{14, 15, 14, 14, 14, 15, 14};
  unsigned sum = 0;
  for (unsigned frame = 0; frame < 7; ++frame) {
    const unsigned delay = gifDelayCentiseconds(7, frame);
    CHECK(delay == seven_fps[frame]);
    sum += delay;
  }
  CHECK(sum == 100U);
  for (unsigned frame = 0; frame < 10; ++frame)
    CHECK(gifDelayCentiseconds(10, frame) == 10U);
  sum = 0;
  for (unsigned frame = 0; frame < 60; ++frame)
    sum += gifDelayCentiseconds(60, frame);
  CHECK(sum == 100U);
}

TEST_CASE("gif encoder writes a GIF89a with trailer", "[recording]") {
  TemporaryDirectory directory;
  RecordingConfig config;
  config.format = RecordingFormat::Gif;
  config.fps = 10;
  config.output = directory.path() / "smoke.gif";
  config.overwrite = false;

  auto encoder = createGifEncoder(config);
  REQUIRE(encoder->begin({64, 64}, 10));
  REQUIRE(encoder->writeFrame(solidFrame(64, 64, 255, 0, 0), 1U));
  REQUIRE(encoder->writeFrame(solidFrame(64, 64, 0, 0, 255), 1U));
  const RecordingResult result = encoder->finish();
  REQUIRE(result.status == RecordingStatus::Written);

  const std::vector<std::uint8_t> bytes = readFile(config.output);
  REQUIRE(bytes.size() > 500U);
  CHECK(stringAt(bytes, 0, 6) == "GIF89a");
  CHECK(bytes.back() == 0x3bU);

  SECTION("existing destination without overwrite is skipped") {
    auto repeat = createGifEncoder(config);
    REQUIRE(repeat->begin({64, 64}, 10));
    REQUIRE(repeat->writeFrame(solidFrame(64, 64, 0, 255, 0), 1U));
    const RecordingResult skipped = repeat->finish();
    CHECK(skipped.status == RecordingStatus::Skipped);
    CHECK(readFile(config.output).size() == bytes.size());
  }
}

TEST_CASE("gif encoder abort removes the temporary", "[recording]") {
  TemporaryDirectory directory;
  RecordingConfig config;
  config.format = RecordingFormat::Gif;
  config.output = directory.path() / "aborted.gif";
  auto encoder = createGifEncoder(config);
  REQUIRE(encoder->begin({8, 8}, 10));
  REQUIRE(encoder->writeFrame(solidFrame(8, 8, 1, 2, 3), 1U));
  std::filesystem::path temporary = config.output;
  temporary += ".tmp";
  REQUIRE(std::filesystem::exists(temporary));
  encoder->abort();
  CHECK(!std::filesystem::exists(temporary));
  CHECK(!std::filesystem::exists(config.output));
}

TEST_CASE("avi mjpeg encoder writes a parseable RIFF AVI", "[recording]") {
  TemporaryDirectory directory;
  RecordingConfig config;
  config.format = RecordingFormat::Avi;
  config.fps = 10;
  config.output = directory.path() / "smoke.avi";
  config.overwrite = false;

  auto encoder = createAviMjpegEncoder(config);
  REQUIRE(encoder->begin({64, 64}, 10));
  REQUIRE(encoder->writeFrame(solidFrame(64, 64, 255, 0, 0), 1U));
  // A merged queue entry rewrites the same JPEG chunk with its own index
  // entry, so the total duration is unaffected.
  REQUIRE(encoder->writeFrame(solidFrame(64, 64, 0, 0, 255), 2U));
  const RecordingResult result = encoder->finish();
  REQUIRE(result.status == RecordingStatus::Written);

  const std::vector<std::uint8_t> bytes = readFile(config.output);
  REQUIRE(bytes.size() > 8U);
  REQUIRE(fourccAt(bytes, 0) == "RIFF");
  REQUIRE(fourccAt(bytes, 8) == "AVI ");
  const ParsedAvi parsed = parseAvi(bytes);

  CHECK(parsed.riff_size == bytes.size() - 8U);
  CHECK(parsed.micro_sec_per_frame == 100'000U);
  CHECK(parsed.total_frames == 3U);
  CHECK(parsed.width == 64U);
  CHECK(parsed.height == 64U);
  CHECK(parsed.handler == "MJPG");
  CHECK(parsed.scale == 1U);
  CHECK(parsed.rate == 10U);
  CHECK(parsed.length == 3U);
  REQUIRE(parsed.chunks.size() == 3U);
  CHECK(parsed.chunks[1] == parsed.chunks[2]);
  CHECK(parsed.chunks[0] != parsed.chunks[1]);

  REQUIRE(parsed.index.size() == 3U);
  CHECK(parsed.index[0].offset == 4U);
  CHECK(parsed.index[1].offset == 4U + 8U + parsed.chunks[0].size() +
                                     (parsed.chunks[0].size() & 1U));
  CHECK(parsed.index[2].offset == parsed.index[1].offset + 8U +
                                     parsed.chunks[1].size() +
                                     (parsed.chunks[1].size() & 1U));
  CHECK(parsed.index[0].size == parsed.chunks[0].size());
  std::uint32_t movi_bytes = 4U;
  for (const auto &chunk : parsed.chunks)
    movi_bytes += 8U + chunk.size() + (chunk.size() & 1U);
  CHECK(parsed.movi_list_size == movi_bytes);

  // One embedded JPEG frame decodes back to the viewport extent.
  int width = 0;
  int height = 0;
  int components = 0;
  stbi_uc *decoded = stbi_load_from_memory(
      parsed.chunks[0].data(), static_cast<int>(parsed.chunks[0].size()),
      &width, &height, &components, 3);
  REQUIRE(decoded != nullptr);
  CHECK(width == 64);
  CHECK(height == 64);
  stbi_image_free(decoded);

  SECTION("existing destination without overwrite is skipped") {
    auto repeat = createAviMjpegEncoder(config);
    REQUIRE(repeat->begin({64, 64}, 10));
    REQUIRE(repeat->writeFrame(solidFrame(64, 64, 0, 255, 0), 1U));
    const RecordingResult skipped = repeat->finish();
    CHECK(skipped.status == RecordingStatus::Skipped);
    CHECK(readFile(config.output).size() == bytes.size());
  }
}

TEST_CASE("avi mjpeg encoder handles odd viewport dimensions",
          "[recording]") {
  TemporaryDirectory directory;
  RecordingConfig config;
  config.format = RecordingFormat::Avi;
  config.fps = 25;
  config.output = directory.path() / "odd.avi";
  auto encoder = createAviMjpegEncoder(config);
  REQUIRE(encoder->begin({63, 41}, 25));
  REQUIRE(encoder->writeFrame(solidFrame(63, 41, 10, 200, 30), 2U));
  const RecordingResult result = encoder->finish();
  REQUIRE(result.status == RecordingStatus::Written);

  const ParsedAvi parsed = parseAvi(readFile(config.output));
  CHECK(parsed.width == 63U);
  CHECK(parsed.height == 41U);
  CHECK(parsed.total_frames == 2U);
  REQUIRE(parsed.chunks.size() == 2U);
  int width = 0;
  int height = 0;
  int components = 0;
  stbi_uc *decoded = stbi_load_from_memory(
      parsed.chunks[0].data(), static_cast<int>(parsed.chunks[0].size()),
      &width, &height, &components, 3);
  REQUIRE(decoded != nullptr);
  CHECK(width == 63);
  CHECK(height == 41);
  stbi_image_free(decoded);
}

TEST_CASE("writeJpegToMemory round-trips through stb_image", "[recording]") {
  kpt::ImageRGB8 image(9, 5);
  for (int y = 0; y < 5; ++y)
    for (int x = 0; x < 9; ++x) {
      auto *pixel = image.pixel(x, y);
      pixel[0] = static_cast<std::uint8_t>(x * 25);
      pixel[1] = static_cast<std::uint8_t>(y * 40);
      pixel[2] = static_cast<std::uint8_t>(x * y * 5);
    }
  const std::vector<std::uint8_t> jpeg =
      kpt::render_detail::writeJpegToMemory(image.view(), 95);
  REQUIRE(!jpeg.empty());
  int width = 0;
  int height = 0;
  int components = 0;
  stbi_uc *decoded = stbi_load_from_memory(
      jpeg.data(), static_cast<int>(jpeg.size()), &width, &height, &components,
      3);
  REQUIRE(decoded != nullptr);
  CHECK(width == 9);
  CHECK(height == 5);
  stbi_image_free(decoded);

  REQUIRE_THROWS_AS(kpt::render_detail::writeJpegToMemory(
                        kpt::ImageView{9, 5, 27 - 1, image.pixels()}, 95),
                    std::invalid_argument);
}

TEST_CASE("recording session waits for playback before opening the encoder",
          "[recording]") {
  TemporaryDirectory directory;
  JobSystem jobs(1);
  UiEvents ui;
  RecordingSession session(jobs, ui);
  RecordingConfig config;
  config.format = RecordingFormat::Avi;
  config.fps = 10;
  config.limit = RecordingLimit::Frames;
  config.limit_value = 4;
  config.output = directory.path() / "prepared.avi";

  std::atomic<bool> done{false};
  std::optional<RecordingResult> completion;
  REQUIRE(session.prepare(config, {16, 16},
                          [&](const RecordingResult &result) {
                            completion = result;
                            done.store(true);
                          }));
  CHECK(session.state() == RecordingSession::State::Waiting);
  // Only one take may be armed.
  CHECK(!session.prepare(config, {16, 16}, {}));
  const std::filesystem::path temporary = config.output.string() + ".tmp";
  CHECK(!std::filesystem::exists(temporary));
  CHECK(!std::filesystem::exists(config.output));
  CHECK(!done.load());

  // Cancelling a waiting take publishes nothing and reports Cancelled.
  session.stop();
  CHECK(session.state() == RecordingSession::State::Idle);
  REQUIRE(completion.has_value());
  CHECK(completion->status == RecordingStatus::Cancelled);
  CHECK(done.load());
  CHECK(!std::filesystem::exists(config.output));
  CHECK(!std::filesystem::exists(temporary));
}

TEST_CASE("output duration follows played time, not source-frame count",
          "[recording]") {
  struct Case {
    int output_fps;
    int source_frames;
    int display_ms;
    unsigned expected_frames;
  };
  // expected_frames comes from the played duration only:
  // round(source_frames * display_ms / 1000 * output_fps).
  const Case cases[] = {
      {10, 4, 100, 4U},
      {30, 4, 100, 12U},
      {24, 5, 100, 12U},
      {1, 100, 10, 1U},
      {7, 4, 300, 8U},
  };
  for (const Case &test : cases) {
    TemporaryDirectory directory;
    JobSystem jobs(1);
    UiEvents ui;
    RecordingSession session(jobs, ui);
    RecordingConfig config;
    config.format = RecordingFormat::Avi;
    config.fps = test.output_fps;
    config.limit = RecordingLimit::Frames;
    config.limit_value = test.source_frames;
    config.output = directory.path() / "duration.avi";

    std::atomic<bool> done{false};
    std::optional<RecordingResult> completion;
    REQUIRE(session.prepare(config, {16, 16},
                            [&](const RecordingResult &result) {
                              completion = result;
                              done.store(true);
                            }));
    const std::chrono::steady_clock::time_point start =
        std::chrono::steady_clock::now();
    session.begin(start, true);
    CHECK(session.state() == RecordingSession::State::Recording);

    bool budget = false;
    for (int frame = 0; frame < test.source_frames; ++frame) {
      budget = session.addFrame(
          solidFrame(16, 16, static_cast<std::uint8_t>(frame & 0xffU), 30,
                     200),
          start + std::chrono::milliseconds(test.display_ms * frame));
    }
    CHECK(session.capturedFrames() ==
          static_cast<std::uint64_t>(test.source_frames));
    CHECK(budget);
    // The App finishes on the same clock tick that satisfies the frame
    // budget; the final source frame must still reach the output.
    session.finish(start +
                   std::chrono::milliseconds(test.display_ms *
                                             (test.source_frames - 1)));
    CHECK(session.state() == RecordingSession::State::Finalizing);
    waitForCompletion(ui, done);
    REQUIRE(completion.has_value());
    CHECK(completion->status == RecordingStatus::Written);

    const ParsedAvi parsed = parseAvi(readFile(config.output));
    CHECK(parsed.total_frames == test.expected_frames);
    CHECK(parsed.micro_sec_per_frame ==
          1000000U / static_cast<unsigned>(test.output_fps));
  }
}

TEST_CASE("frame budget stop keeps the final source frame",
          "[recording]") {
  TemporaryDirectory directory;
  JobSystem jobs(1);
  UiEvents ui;
  RecordingSession session(jobs, ui);
  RecordingConfig config;
  config.format = RecordingFormat::Avi;
  config.fps = 10;
  config.limit = RecordingLimit::Frames;
  config.limit_value = 2;
  config.output = directory.path() / "last-frame.avi";

  std::atomic<bool> done{false};
  std::optional<RecordingResult> completion;
  REQUIRE(session.prepare(config, {16, 16},
                          [&](const RecordingResult &result) {
                            completion = result;
                            done.store(true);
                          }));
  const std::chrono::steady_clock::time_point start =
      std::chrono::steady_clock::now();
  session.begin(start, true);
  CHECK(!session.addFrame(solidFrame(16, 16, 220, 20, 20), start));
  // The second source frame satisfies the budget; the App then calls finish
  // on this same clock tick.
  CHECK(session.addFrame(solidFrame(16, 16, 20, 220, 20),
                         start + std::chrono::milliseconds(100)));
  session.finish(start + std::chrono::milliseconds(100));
  waitForCompletion(ui, done);
  REQUIRE(completion.has_value());
  CHECK(completion->status == RecordingStatus::Written);

  const ParsedAvi parsed = parseAvi(readFile(config.output));
  CHECK(parsed.total_frames == 2U);
  CHECK(parsed.micro_sec_per_frame == 100000U);
  REQUIRE(parsed.chunks.size() == 2U);
  CHECK(parsed.chunks[0] != parsed.chunks[1]);
}

TEST_CASE("natural pass end keeps the last source frame", "[recording]") {
  TemporaryDirectory directory;
  JobSystem jobs(1);
  UiEvents ui;
  RecordingSession session(jobs, ui);
  RecordingConfig config;
  config.format = RecordingFormat::Avi;
  config.fps = 10;
  config.limit = RecordingLimit::Seconds;
  config.limit_value = -1; // until the current pass ends
  config.output = directory.path() / "pass-end.avi";

  std::atomic<bool> done{false};
  std::optional<RecordingResult> completion;
  REQUIRE(session.prepare(config, {16, 16},
                          [&](const RecordingResult &result) {
                            completion = result;
                            done.store(true);
                          }));
  const std::chrono::steady_clock::time_point start =
      std::chrono::steady_clock::now();
  session.begin(start, true);
  for (int frame = 0; frame < 3; ++frame)
    CHECK(!session.addFrame(
        solidFrame(16, 16, static_cast<std::uint8_t>(20 + frame * 80), 40, 40),
        start + std::chrono::milliseconds(100 * frame)));
  // The pass ended after the last frame was displayed for one interval.
  session.finish(start + std::chrono::milliseconds(200));
  waitForCompletion(ui, done);
  REQUIRE(completion.has_value());
  CHECK(completion->status == RecordingStatus::Written);
  const ParsedAvi parsed = parseAvi(readFile(config.output));
  CHECK(parsed.total_frames == 3U);
  CHECK(parsed.micro_sec_per_frame == 100000U);
  REQUIRE(parsed.chunks.size() == 3U);
  CHECK(parsed.chunks[0] != parsed.chunks[2]);
}

TEST_CASE("recording session seconds budget skips paused time", "[recording]") {
  TemporaryDirectory directory;
  JobSystem jobs(1);
  UiEvents ui;
  RecordingSession session(jobs, ui);
  RecordingConfig config;
  config.format = RecordingFormat::Avi;
  config.fps = 10;
  config.limit = RecordingLimit::Seconds;
  config.limit_value = 1;
  config.output = directory.path() / "seconds.avi";

  std::atomic<bool> done{false};
  std::optional<RecordingResult> completion;
  REQUIRE(session.prepare(config, {16, 16},
                          [&](const RecordingResult &result) {
                            completion = result;
                            done.store(true);
                          }));
  const std::chrono::steady_clock::time_point start =
      std::chrono::steady_clock::now();
  session.begin(start, true);
  session.addFrame(solidFrame(16, 16, 1, 2, 3), start);
  CHECK(session.capturedFrames() == 1U);

  // The viewer pauses for 1.4 s: the played clock must not advance.
  session.setPlaybackActive(false, start + std::chrono::milliseconds(100));
  CHECK(session.playedSeconds(start + std::chrono::milliseconds(1500)) < 0.11);
  CHECK(!session.secondsLimitReached(start + std::chrono::milliseconds(1500)));

  session.setPlaybackActive(true, start + std::chrono::milliseconds(1500));
  // Resume advances the budget from the paused point, never from wall time.
  session.addFrame(solidFrame(16, 16, 4, 5, 6),
                   start + std::chrono::milliseconds(1600));
  CHECK(session.playedSeconds(start + std::chrono::milliseconds(1600)) < 0.21);
  CHECK(!session.secondsLimitReached(start + std::chrono::milliseconds(1600)));

  auto now = start + std::chrono::milliseconds(1600);
  while (!session.secondsLimitReached(now)) {
    now += std::chrono::milliseconds(100);
    session.addFrame(solidFrame(16, 16, 7, 8, 9), now);
  }
  session.finish(now);
  CHECK(session.state() == RecordingSession::State::Finalizing);
  waitForCompletion(ui, done);
  REQUIRE(completion.has_value());
  CHECK(completion->status == RecordingStatus::Written);
  const ParsedAvi parsed = parseAvi(readFile(config.output));
  CHECK(parsed.micro_sec_per_frame == 100000U);
  // The played clock stops at 1.0 s, so 10 fps yields exactly 10 frames
  // even though the take was paused in between.
  CHECK(parsed.total_frames == 10U);
}

TEST_CASE("recording session publishes a single frame for a very short take",
          "[recording]") {
  TemporaryDirectory directory;
  JobSystem jobs(1);
  UiEvents ui;
  RecordingSession session(jobs, ui);
  RecordingConfig config;
  config.format = RecordingFormat::Avi;
  config.fps = 10;
  config.limit = RecordingLimit::Frames;
  config.limit_value = 1;
  config.output = directory.path() / "short.avi";

  std::atomic<bool> done{false};
  std::optional<RecordingResult> completion;
  REQUIRE(session.prepare(config, {16, 16},
                          [&](const RecordingResult &result) {
                            completion = result;
                            done.store(true);
                          }));
  const std::chrono::steady_clock::time_point start =
      std::chrono::steady_clock::now();
  session.begin(start, true);
  CHECK(session.addFrame(solidFrame(16, 16, 4, 8, 12), start));
  session.finish(start);
  waitForCompletion(ui, done);
  REQUIRE(completion.has_value());
  CHECK(completion->status == RecordingStatus::Written);
  const ParsedAvi parsed = parseAvi(readFile(config.output));
  // Zero elapsed output slots still produce a non-empty, playable file.
  CHECK(parsed.total_frames == 1U);
  CHECK(parsed.micro_sec_per_frame == 100000U);
}

TEST_CASE("recording session cancellation aborts without publishing",
          "[recording]") {
  TemporaryDirectory directory;
  JobSystem jobs(1);
  UiEvents ui;
  RecordingSession session(jobs, ui);
  RecordingConfig config;
  config.format = RecordingFormat::Avi;
  config.fps = 10;
  config.limit = RecordingLimit::Frames;
  config.limit_value = 100;
  config.output = directory.path() / "cancelled.avi";

  std::atomic<bool> done{false};
  std::optional<RecordingResult> completion;
  REQUIRE(session.prepare(config, {8, 8}, [&](const RecordingResult &result) {
    completion = result;
    done.store(true);
  }));
  const std::chrono::steady_clock::time_point start =
      std::chrono::steady_clock::now();
  session.begin(start, true);
  for (int frame = 0; frame < 2; ++frame)
    session.addFrame(solidFrame(8, 8, 1, static_cast<std::uint8_t>(frame), 3),
                     start + std::chrono::milliseconds(100 * frame));
  session.stop();
  CHECK(session.state() == RecordingSession::State::Finalizing);
  waitForCompletion(ui, done);
  CHECK(session.state() == RecordingSession::State::Idle);
  REQUIRE(completion.has_value());
  CHECK(completion->status == RecordingStatus::Cancelled);
  const std::filesystem::path temporary = config.output.string() + ".tmp";
  CHECK(!std::filesystem::exists(config.output));
  CHECK(!std::filesystem::exists(temporary));
}

TEST_CASE("recording session reports a capture failure without publishing",
          "[recording]") {
  TemporaryDirectory directory;
  JobSystem jobs(1);
  UiEvents ui;
  RecordingSession session(jobs, ui);
  RecordingConfig config;
  config.format = RecordingFormat::Gif;
  config.fps = 10;
  config.limit = RecordingLimit::Frames;
  config.limit_value = 2;
  config.output = directory.path() / "failed.gif";

  std::atomic<bool> done{false};
  std::optional<RecordingResult> completion;
  REQUIRE(session.prepare(config, {8, 8}, [&](const RecordingResult &result) {
    completion = result;
    done.store(true);
  }));
  const std::chrono::steady_clock::time_point start =
      std::chrono::steady_clock::now();
  session.begin(start, true);
  session.addFrame(solidFrame(8, 8, 1, 2, 3), start);
  session.fail("viewport capture failed");
  CHECK(session.state() == RecordingSession::State::Finalizing);
  waitForCompletion(ui, done);
  REQUIRE(completion.has_value());
  CHECK(completion->status == RecordingStatus::Failed);
  CHECK(completion->message == "viewport capture failed");
  CHECK(!std::filesystem::exists(config.output));
}
