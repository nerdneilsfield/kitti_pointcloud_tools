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

TEST_CASE("recording pacer anchors and emits exact frame counts",
          "[recording]") {
  SECTION("first tick emits the visible frame") {
    RecordingPacer pacer(10, 3);
    REQUIRE(pacer.totalFrames() == 30);
    const std::chrono::steady_clock::time_point start{};
    REQUIRE(!pacer.started());
    CHECK(pacer.consume(start) == 1U);
    CHECK(pacer.consume(start + std::chrono::milliseconds(50)) == 0U);
    CHECK(pacer.consume(start + std::chrono::milliseconds(100)) == 1U);
    CHECK(pacer.emittedFrames() == 2U);
  }

  SECTION("integer math keeps slot boundaries exact") {
    // 700 ms at 10 fps is exactly slot 7 in binary-imprecise floating point.
    RecordingPacer pacer(10, 1);
    const std::chrono::steady_clock::time_point start{};
    CHECK(pacer.consume(start) == 1U);
    CHECK(pacer.consume(start + std::chrono::milliseconds(700)) == 7U);
  }

  SECTION("slow display loop pads with repeats") {
    RecordingPacer pacer(10, 3);
    const std::chrono::steady_clock::time_point start{};
    CHECK(pacer.consume(start) == 1U);
    // A 1.5 s gap must account for 15 slots at once, not lose them.
    CHECK(pacer.consume(start + std::chrono::milliseconds(1500)) == 15U);
    CHECK(pacer.consume(start + std::chrono::milliseconds(3000)) == 14U);
    CHECK(pacer.finished());
    CHECK(pacer.consume(start + std::chrono::milliseconds(4000)) == 0U);
  }

  SECTION("fps above display rate still totals fps * duration") {
    RecordingPacer pacer(60, 1);
    const std::chrono::steady_clock::time_point start{};
    unsigned total = 0;
    for (int tick = 0; tick <= 10; ++tick)
      total += pacer.consume(start + std::chrono::milliseconds(100 * tick));
    CHECK(total == 60U);
    CHECK(pacer.finished());
  }
}

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

TEST_CASE("queue accounting keeps fps * duration frames under a slow encoder",
          "[recording]") {
  // Simulate a display loop much faster than the encoder: every due count is
  // pushed, the queue saturates, and the worker still sees exactly the
  // pacer's total.
  RecordingPacer pacer(60, 2);
  RecordingFrameQueue queue;
  const std::chrono::steady_clock::time_point start{};
  for (int tick = 0; tick <= 20; ++tick)
    queue.push(solidFrame(4, 4, 1, 2, 3),
               pacer.consume(start + std::chrono::milliseconds(100 * tick)));
  queue.close();
  unsigned accounted = 0;
  QueuedFrame entry;
  while (queue.pop(entry) == RecordingFrameQueue::Pop::Entry)
    accounted += entry.repeat;
  CHECK(accounted == 120U);
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
  config.duration_seconds = 3;
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
  config.duration_seconds = 3;
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

TEST_CASE("recording session runs a take end to end", "[recording]") {
  TemporaryDirectory directory;
  JobSystem jobs(1);
  UiEvents ui;
  RecordingSession session(jobs, ui);
  RecordingConfig config;
  config.format = RecordingFormat::Gif;
  config.fps = 50;
  config.duration_seconds = 1;
  config.output = directory.path() / "session.gif";

  std::atomic<int> captures{0};
  const auto capture = [&]() -> kpt::Result<Rgba8Image, std::string> {
    const int index = captures.fetch_add(1) + 1;
    return solidFrame(16, 16, static_cast<std::uint8_t>(index), 0, 0);
  };
  std::atomic<bool> done{false};
  std::optional<RecordingResult> completion;

  REQUIRE(session.start(config, {16, 16},
                        [&](const RecordingResult &result) {
                          completion = result;
                          done.store(true);
                        }));
  CHECK(session.state() == RecordingSession::State::Recording);
  // One recording at a time.
  CHECK(!session.start(config, {16, 16}, {}));

  // 1 s at 50 fps: the anchor tick plus one slot per 20 ms tick.
  const std::chrono::steady_clock::time_point start{};
  for (int tick = 0; tick <= 50; ++tick)
    session.tick(start + std::chrono::milliseconds(20 * tick), capture);
  CHECK(session.state() == RecordingSession::State::Finalizing);
  waitForCompletion(ui, done);
  CHECK(session.state() == RecordingSession::State::Idle);
  REQUIRE(completion.has_value());
  CHECK(completion->status == RecordingStatus::Written);
  CHECK(std::filesystem::exists(config.output));
  CHECK(stringAt(readFile(config.output), 0, 6) == "GIF89a");
  // Exactly fps * duration captures, never more.
  CHECK(captures.load() == 50);
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
  config.duration_seconds = 3;
  config.output = directory.path() / "cancelled.avi";

  std::atomic<int> captures{0};
  const auto capture = [&]() -> kpt::Result<Rgba8Image, std::string> {
    const int index = captures.fetch_add(1);
    return solidFrame(8, 8, 1, static_cast<std::uint8_t>(index), 3);
  };
  std::atomic<bool> done{false};
  std::optional<RecordingResult> completion;
  REQUIRE(session.start(config, {8, 8}, [&](const RecordingResult &result) {
    completion = result;
    done.store(true);
  }));

  const std::chrono::steady_clock::time_point start{};
  for (int tick = 0; tick <= 2; ++tick)
    session.tick(start + std::chrono::milliseconds(100 * tick), capture);
  session.stop();
  CHECK(session.state() == RecordingSession::State::Finalizing);
  waitForCompletion(ui, done);
  CHECK(session.state() == RecordingSession::State::Idle);
  REQUIRE(completion.has_value());
  CHECK(completion->status == RecordingStatus::Cancelled);
  std::filesystem::path temporary = config.output;
  temporary += ".tmp";
  CHECK(!std::filesystem::exists(config.output));
  CHECK(!std::filesystem::exists(temporary));
}
