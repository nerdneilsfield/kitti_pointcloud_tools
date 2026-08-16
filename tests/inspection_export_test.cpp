#include "gui/inspection_export.hpp"

#include "kpt/io/io.hpp"

#include <catch2/catch.hpp>

#include <filesystem>
#include <random>
#include <stop_token>
#include <vector>

namespace {

namespace fs = std::filesystem;

fs::path temporaryPath(const char *stem) {
  static std::mt19937_64 generator(std::random_device{}());
  return fs::temp_directory_path() /
         (std::string(stem) + "-" + std::to_string(generator()) + ".pcd");
}

struct RemoveFile {
  fs::path path;
  ~RemoveFile() {
    std::error_code error;
    fs::remove(path, error);
  }
};

kpt::PointCloudIRGB cloudWithPoint(float x, std::uint8_t noise,
                                   bool has_noise) {
  kpt::PointCloudIRGB cloud;
  cloud.points.push_back({x, x + 1.0F, x + 2.0F, 7, 8, 9, noise, x + 3.0F});
  cloud.width = 1;
  cloud.height = 1;
  cloud.has_noise = has_noise;
  return cloud;
}

TEST_CASE("inspection export merges supplied world clouds without losing PointT fields") {
  const auto first = cloudWithPoint(1.0F, 1, false);
  const auto second = cloudWithPoint(10.0F, 0, true);
  const std::vector<kpt::gui::WorldCloudView> views{{first}, {second}};

  const auto merged = kpt::gui::mergeWorldClouds(views);

  REQUIRE(merged.size() == 2);
  REQUIRE(merged.width == 2);
  REQUIRE(merged.height == 1);
  REQUIRE(merged.has_noise);
  REQUIRE(merged.points[0].x == Approx(1.0F));
  REQUIRE(merged.points[0].r == 7);
  REQUIRE(merged.points[0].noise == 1);
  REQUIRE(merged.points[0].intensity == Approx(4.0F));
  REQUIRE(merged.points[1].x == Approx(10.0F));
}

TEST_CASE("inspection export writes, skips, and overwrites through saveAtomic") {
  const RemoveFile output{temporaryPath("kpt-inspection-export")};
  const auto first = cloudWithPoint(1.0F, 1, true);
  const std::vector<kpt::gui::WorldCloudView> first_view{{first}};

  const auto written = kpt::gui::exportWorldClouds(output.path, first_view, false);
  REQUIRE(written.status == kpt::gui::InspectionExportStatus::Written);
  REQUIRE(written.completed());
  REQUIRE(fs::exists(output.path));
  REQUIRE(kpt::load(output.path)->points.front().x == Approx(1.0F));

  const auto replacement = cloudWithPoint(9.0F, 0, false);
  const std::vector<kpt::gui::WorldCloudView> replacement_view{{replacement}};
  const auto skipped =
      kpt::gui::exportWorldClouds(output.path, replacement_view, false);
  REQUIRE(skipped.status == kpt::gui::InspectionExportStatus::Skipped);
  REQUIRE(skipped.completed());
  REQUIRE(kpt::load(output.path)->points.front().x == Approx(1.0F));

  const auto overwritten =
      kpt::gui::exportWorldClouds(output.path, replacement_view, true);
  REQUIRE(overwritten.status == kpt::gui::InspectionExportStatus::Written);
  REQUIRE(kpt::load(output.path)->points.front().x == Approx(9.0F));
}

TEST_CASE("inspection export does not publish an empty ROI result") {
  const RemoveFile output{temporaryPath("kpt-inspection-empty")};
  kpt::PointCloudIRGB empty;
  const std::vector<kpt::gui::WorldCloudView> views{{empty}};

  const auto result = kpt::gui::exportWorldClouds(output.path, views, true);

  REQUIRE(result.status == kpt::gui::InspectionExportStatus::Empty);
  REQUIRE(result.completed());
  REQUIRE_FALSE(std::filesystem::exists(output.path));
}

TEST_CASE("inspection export reports cancellation and I/O errors") {
  const RemoveFile cancelled_output{temporaryPath("kpt-inspection-cancelled")};
  const auto cloud = cloudWithPoint(1.0F, 0, false);
  const std::vector<kpt::gui::WorldCloudView> views{{cloud}};
  std::stop_source cancellation;
  cancellation.request_stop();

  const auto cancelled = kpt::gui::exportWorldClouds(
      cancelled_output.path, views, true, std::nullopt, cancellation.get_token());
  REQUIRE(cancelled.status == kpt::gui::InspectionExportStatus::Cancelled);
  REQUIRE_FALSE(cancelled.completed());
  REQUIRE_FALSE(fs::exists(cancelled_output.path));

  const auto bad_path = temporaryPath("kpt-inspection-bad-extension").replace_extension(".bad");
  const auto failed = kpt::gui::exportWorldClouds(bad_path, views, true);
  REQUIRE(failed.status == kpt::gui::InspectionExportStatus::Failed);
  REQUIRE_FALSE(failed.message.empty());
}

} // namespace
