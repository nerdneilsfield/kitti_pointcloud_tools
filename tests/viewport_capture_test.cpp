#include "gui/viewport/capture.hpp"
#include "kpt/cancellation.hpp"

#include <catch2/catch.hpp>

#include <chrono>
#include <algorithm>
#include <cstdint>
#include <filesystem>
#include <stop_token>
#include <vector>

namespace {

class TemporaryDirectory {
public:
  TemporaryDirectory() {
    path_ = std::filesystem::temp_directory_path() /
            ("kpt-viewport-capture-" + std::to_string(
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

kpt::gui::Rgba8Image rgbaImage() {
  kpt::gui::Rgba8Image image;
  image.extent = {2, 2};
  image.bytes_per_row = 12;
  image.pixels = {
      1, 2, 3, 90, 4, 5, 6, 91, 250, 250, 250, 250,
      7, 8, 9, 92, 10, 11, 12, 93, 251, 251, 251, 251,
  };
  return image;
}

TEST_CASE("viewport RGBA capture converts padded top-left rows to RGB",
          "[viewport_capture]") {
  const auto rgb = kpt::gui::rgba8ToRgbImage(rgbaImage());
  REQUIRE(rgb.width() == 2);
  REQUIRE(rgb.height() == 2);
  REQUIRE(rgb.strideBytes() == 6);
  const std::vector<std::uint8_t> expected{
      1, 2, 3, 4, 5, 6,
      7, 8, 9, 10, 11, 12,
  };
  REQUIRE(rgb.pixels().size() == expected.size());
  REQUIRE(std::equal(rgb.pixels().begin(), rgb.pixels().end(),
                     expected.begin()));
}

TEST_CASE("viewport capture validates storage and cancellation", "[viewport_capture]") {
  auto truncated = rgbaImage();
  truncated.pixels.resize(7);
  REQUIRE_THROWS_AS(kpt::gui::rgba8ToRgbImage(truncated), std::invalid_argument);

  std::stop_source cancelled;
  cancelled.request_stop();
  REQUIRE_THROWS_AS(kpt::gui::rgba8ToRgbImage(rgbaImage(), cancelled.get_token()),
                    kpt::OperationCancelled);
}

TEST_CASE("viewport capture writes PNG asynchronously-safe image snapshots",
          "[viewport_capture]") {
  TemporaryDirectory directory;
  const auto output = directory.path() / "capture.png";
  const auto first =
      kpt::gui::writeViewportCapturePng(output, rgbaImage(), false);
  REQUIRE(first.status == kpt::gui::ViewportCaptureStatus::Written);
  REQUIRE(std::filesystem::exists(output));
  REQUIRE(std::filesystem::file_size(output) > 8);

  const auto skipped =
      kpt::gui::writeViewportCapturePng(output, rgbaImage(), false);
  REQUIRE(skipped.status == kpt::gui::ViewportCaptureStatus::Skipped);

  std::stop_source cancelled;
  cancelled.request_stop();
  const auto cancelled_result = kpt::gui::writeViewportCapturePng(
      directory.path() / "cancelled.png", rgbaImage(), true,
      cancelled.get_token());
  REQUIRE(cancelled_result.status == kpt::gui::ViewportCaptureStatus::Cancelled);

  const auto invalid_extension = kpt::gui::writeViewportCapturePng(
      directory.path() / "capture.jpg", rgbaImage(), true);
  REQUIRE(invalid_extension.status == kpt::gui::ViewportCaptureStatus::Failed);
}

} // namespace
