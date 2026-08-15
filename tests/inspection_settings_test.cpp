#include "gui/inspection_settings.hpp"

#include <catch2/catch.hpp>

#include <chrono>
#include <filesystem>
#include <fstream>

namespace {

class TemporaryDirectory {
public:
  TemporaryDirectory() {
    path_ = std::filesystem::temp_directory_path() /
            ("kpt-inspection-settings-" + std::to_string(
                std::chrono::steady_clock::now().time_since_epoch().count()));
    std::filesystem::create_directories(path_);
  }
  ~TemporaryDirectory() { std::error_code error; std::filesystem::remove_all(path_, error); }
  const std::filesystem::path &path() const { return path_; }
private:
  std::filesystem::path path_;
};

kpt::gui::CameraSnapshot snapshot() {
  kpt::gui::CameraSnapshot value;
  value.target = {1.0, 2.0, 3.0};
  value.rotation_center = {-4.0, 5.0, 6.0};
  value.camera_to_world(0, 1) = 0.25F;
  value.distance = 17.5;
  value.fov_y_degrees = 53.0F;
  return value;
}

TEST_CASE("inspection settings persist versioned bookmarks atomically") {
  TemporaryDirectory directory;
  kpt::gui::InspectionSettings settings;
  settings.saveBookmark(kpt::gui::CameraBookmark("inspection \"A\"", snapshot()));
  kpt::gui::InspectionSettingsFile store(directory.path() / "nested" / "inspection.json");

  std::string error;
  REQUIRE(store.save(settings, &error));
  REQUIRE(error.empty());
  REQUIRE(std::filesystem::exists(store.path()));

  kpt::gui::InspectionSettings loaded;
  REQUIRE(store.load(loaded, &error));
  REQUIRE(loaded.bookmarks().size() == 1);
  const auto *bookmark = loaded.findBookmark("inspection \"A\"");
  REQUIRE(bookmark != nullptr);
  REQUIRE(bookmark->camera().target.isApprox(snapshot().target));
  REQUIRE(bookmark->camera().rotation_center.isApprox(snapshot().rotation_center));
  REQUIRE(bookmark->camera().camera_to_world.isApprox(snapshot().camera_to_world));
  REQUIRE(bookmark->camera().distance == Approx(17.5));
  REQUIRE(bookmark->camera().fov_y_degrees == Approx(53.0F));
}

TEST_CASE("inspection settings leave state untouched on malformed or unsupported files") {
  TemporaryDirectory directory;
  const auto file = directory.path() / "inspection.json";
  kpt::gui::InspectionSettingsFile store(file);
  kpt::gui::InspectionSettings settings;
  settings.saveBookmark(kpt::gui::CameraBookmark("keep", snapshot()));
  std::string error;

  { std::ofstream output(file); output << "{invalid}"; }
  REQUIRE_FALSE(store.load(settings, &error));
  REQUIRE_FALSE(error.empty());
  REQUIRE(settings.findBookmark("keep") != nullptr);

  { std::ofstream output(file); output << "{\"schema_version\":2,\"bookmarks\":[]}"; }
  REQUIRE_FALSE(store.load(settings, &error));
  REQUIRE(settings.findBookmark("keep") != nullptr);
}

TEST_CASE("missing inspection settings file loads empty settings") {
  TemporaryDirectory directory;
  kpt::gui::InspectionSettingsFile store(directory.path() / "missing.json");
  kpt::gui::InspectionSettings settings;
  settings.saveBookmark(kpt::gui::CameraBookmark("stale", snapshot()));
  REQUIRE(store.load(settings));
  REQUIRE(settings.bookmarks().empty());
}

} // namespace
