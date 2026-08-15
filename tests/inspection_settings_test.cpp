#include "gui/inspection_settings.hpp"

#include <catch2/catch.hpp>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <limits>

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

TEST_CASE("inspection settings replace an existing file") {
  TemporaryDirectory directory;
  kpt::gui::InspectionSettingsFile store(directory.path() / "inspection.json");
  kpt::gui::InspectionSettings first;
  first.saveBookmark(kpt::gui::CameraBookmark("first", snapshot()));
  kpt::gui::InspectionSettings second;
  auto updated = snapshot();
  updated.distance = 31.0;
  second.saveBookmark(kpt::gui::CameraBookmark("second", updated));

  REQUIRE(store.save(first));
  REQUIRE(store.save(second));

  kpt::gui::InspectionSettings loaded;
  REQUIRE(store.load(loaded));
  REQUIRE(loaded.findBookmark("first") == nullptr);
  const auto *bookmark = loaded.findBookmark("second");
  REQUIRE(bookmark != nullptr);
  REQUIRE(bookmark->camera().distance == Approx(31.0));
}

TEST_CASE("inspection settings round trip escaped controls and Unicode escapes") {
  TemporaryDirectory directory;
  kpt::gui::InspectionSettingsFile store(directory.path() / "inspection.json");
  const std::string name{"A\x01\n\t\0B", 6};
  kpt::gui::InspectionSettings settings;
  settings.saveBookmark(kpt::gui::CameraBookmark(name, snapshot()));
  REQUIRE(store.save(settings));

  kpt::gui::InspectionSettings loaded;
  REQUIRE(store.load(loaded));
  REQUIRE(loaded.findBookmark(name) != nullptr);

  std::ofstream output(store.path(), std::ios::binary | std::ios::trunc);
  output << "{\"schema_version\":1,\"bookmarks\":[{\"name\":\"A\\u0001B\\u4e2d\",\"camera\":{"
            "\"target\":[1,2,3],\"rotation_center\":[-4,5,6],"
            "\"camera_to_world\":[[1,0,0],[0,1,0],[0,0,1]],"
            "\"distance\":17.5,\"fov_y_degrees\":53}}]}";
  output.close();
  REQUIRE(store.load(loaded));
  REQUIRE(loaded.findBookmark(std::string{"A\x01" "B\xe4\xb8\xad"}) != nullptr);
}

TEST_CASE("inspection settings reject invalid viewport cameras and surrogate escapes") {
  TemporaryDirectory directory;
  kpt::gui::InspectionSettingsFile store(directory.path() / "inspection.json");
  kpt::gui::InspectionSettings settings;
  auto invalid = snapshot();
  invalid.distance = std::numeric_limits<double>::max();
  settings.saveBookmark(kpt::gui::CameraBookmark("invalid", invalid));
  std::string error;
  REQUIRE_FALSE(store.save(settings, &error));
  REQUIRE_FALSE(error.empty());

  std::ofstream output(store.path(), std::ios::binary | std::ios::trunc);
  output << "{\"schema_version\":1,\"bookmarks\":[{\"name\":\"\\ud800\",\"camera\":{"
            "\"target\":[1,2,3],\"rotation_center\":[-4,5,6],"
            "\"camera_to_world\":[[1,0,0],[0,1,0],[0,0,1]],"
            "\"distance\":17.5,\"fov_y_degrees\":53}}]}";
  output.close();
  REQUIRE_FALSE(store.load(settings, &error));
  REQUIRE_FALSE(error.empty());
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
