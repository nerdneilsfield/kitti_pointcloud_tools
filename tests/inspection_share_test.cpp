#include "gui/inspection_share.hpp"

#include <catch2/catch.hpp>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <future>
#include <limits>
#include <stop_token>
#include <string>

namespace {

class TemporaryDirectory {
public:
  TemporaryDirectory() {
    path_ = std::filesystem::temp_directory_path() /
            ("kpt-inspection-share-" + std::to_string(
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

Eigen::Vector3d point(double x, double y, double z) { return {x, y, z}; }

kpt::gui::CameraSnapshot snapshot() {
  kpt::gui::CameraSnapshot value;
  value.target = {1.0, 2.0, 3.0};
  value.rotation_center = {-4.0, 5.0, 6.0};
  value.distance = 17.5;
  value.fov_y_degrees = 53.0F;
  return value;
}

TEST_CASE("inspection share round trips review semantics without runtime IDs",
          "[inspection_share]") {
  TemporaryDirectory directory;
  const auto session = directory.path() / "session";
  const auto source = session / "reviews" / "clouds" / "scan.pcd";
  const auto share_path = session / "reviews" / "review.kpt-review.json";
  std::filesystem::create_directories(source.parent_path());

  kpt::gui::Scene scene;
  const auto source_key = kpt::gui::pathSourceKey(source, {});
  const auto layer_id = scene.addLayer(source_key);
  kpt::gui::LayerStyle style;
  style.color_by = kpt::ColorBy::RGB;
  style.color_map = kpt::gui::ColorMap::Viridis;
  style.point_size = 4.5F;
  style.opacity = 0.6F;
  style.fixed_color = {0.1F, 0.2F, 0.3F};
  REQUIRE(scene.setLayerStyle(layer_id, style));
  Eigen::Affine3d transform = Eigen::Affine3d::Identity();
  transform.translation() = point(10.0, 20.0, 30.0);
  REQUIRE(scene.setLayerTransform(layer_id, transform));
  REQUIRE(scene.setLayerVisible(layer_id, false));
  scene.setRoi(kpt::gui::RoiBox({-1.0, -2.0, -3.0}, {4.0, 5.0, 6.0}));
  static_cast<void>(scene.addMeasurement(
      source_key, point(1.0, 2.0, 3.0), kpt::gui::opaqueSourceKey("detached"),
      point(4.0, 6.0, 3.0)));

  kpt::gui::InspectionSettings settings;
  settings.saveBookmark({"overview", snapshot()});
  const auto document =
      kpt::gui::InspectionShareFile::capture(scene, settings, share_path);
  REQUIRE(document.layers.size() == 1);
  REQUIRE(document.layers.front().source_key == source_key);
  REQUIRE(document.layers.front().relative_source_path.has_value());
  REQUIRE(document.layers.front().relative_source_path->generic_string() ==
          "clouds/scan.pcd");
  REQUIRE(document.measurements.size() == 1);
  REQUIRE(document.bookmarks.size() == 1);

  kpt::gui::InspectionShareFile store(share_path);
  REQUIRE(store.save(document, true).status ==
          kpt::gui::InspectionShareSaveStatus::Written);
  REQUIRE(std::filesystem::exists(share_path));
  kpt::gui::InspectionShareDocument loaded;
  REQUIRE(store.load(loaded));
  REQUIRE(loaded.layers.size() == 1);
  REQUIRE(loaded.layers.front().source_key == source_key);
  REQUIRE(loaded.layers.front().local_to_world.isApprox(transform));
  REQUIRE(loaded.layers.front().style.opacity == Approx(0.6F));
  REQUIRE_FALSE(loaded.layers.front().visible);
  REQUIRE(loaded.roi.has_value());
  REQUIRE(loaded.roi->contains(point(4.0, 5.0, 6.0)));
  REQUIRE(loaded.measurements.front().first_world.isApprox(point(1.0, 2.0, 3.0)));
  REQUIRE(loaded.measurements.front().second_source_key ==
          std::optional<std::string>{"opaque:detached"});
  REQUIRE(loaded.bookmarks.front().name() == "overview");

  const auto resolved =
      kpt::gui::InspectionShareFile::resolveSourcePath(share_path,
                                                        loaded.layers.front());
  REQUIRE(resolved.has_value());
  REQUIRE(*resolved == source.lexically_normal());
}

TEST_CASE("inspection share keeps missing source paths unresolved", "[inspection_share]") {
  TemporaryDirectory directory;
  const auto share_path = directory.path() / "review.kpt-review.json";
  kpt::gui::InspectionShareDocument document;
  const auto missing = directory.path() / "clouds" / "missing.pcd";
  document.layers.push_back(
      {kpt::gui::pathSourceKey(missing, {}), std::filesystem::path{"clouds/missing.pcd"},
       Eigen::Affine3d::Identity(), {}, true});

  kpt::gui::InspectionShareFile store(share_path);
  REQUIRE(store.save(document, true).status ==
          kpt::gui::InspectionShareSaveStatus::Written);
  kpt::gui::InspectionShareDocument loaded;
  REQUIRE(store.load(loaded));
  const auto resolved =
      kpt::gui::InspectionShareFile::resolveSourcePath(share_path,
                                                        loaded.layers.front());
  REQUIRE(resolved == std::optional<std::filesystem::path>{missing});
  REQUIRE_FALSE(std::filesystem::exists(*resolved));

  kpt::gui::InspectionShareLayer opaque{
      kpt::gui::opaqueSourceKey("stream:42"), std::nullopt,
      Eigen::Affine3d::Identity(), {}, true};
  REQUIRE_FALSE(kpt::gui::InspectionShareFile::resolveSourcePath(share_path,
                                                                   opaque));
}

TEST_CASE("inspection share preserves cross-runtime source-key union",
          "[inspection_share]") {
  const std::filesystem::path fixture{
      "tests/data/review-share-v1-cross-runtime.json"};
  kpt::gui::InspectionShareDocument document;
  REQUIRE(kpt::gui::InspectionShareFile(fixture).load(document));
  REQUIRE(document.layers.size() == 3);
  REQUIRE(document.layers[0].source_key ==
          "sha256:0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef");
  REQUIRE(document.layers[1].source_key == "opaque:remote/capture-7");
  REQUIRE(document.layers[2].source_key == "path:/portable/scan.xyz");

  const auto base = std::filesystem::absolute(fixture).parent_path();
  REQUIRE(kpt::gui::InspectionShareFile::resolveSourcePath(
              fixture, document.layers[0]) ==
          std::optional<std::filesystem::path>{base / "sources/hashed.xyz"});
  REQUIRE(kpt::gui::InspectionShareFile::resolveSourcePath(
              fixture, document.layers[1]) ==
          std::optional<std::filesystem::path>{base / "sources/capture.xyz"});
  REQUIRE(kpt::gui::InspectionShareFile::resolveSourcePath(
              fixture, document.layers[2]) ==
          std::optional<std::filesystem::path>{base / "sources/native.xyz"});
}

TEST_CASE("inspection share rejects source paths escaping its directory",
          "[inspection_share]") {
  TemporaryDirectory directory;
  const auto share_path = directory.path() / "review.kpt-review.json";
  const auto source = directory.path() / "inside.pcd";
  kpt::gui::InspectionShareDocument document;
  document.layers.push_back(
      {kpt::gui::pathSourceKey(source, {}), std::filesystem::path{"inside.pcd"},
       Eigen::Affine3d::Identity(), {}, true});

  kpt::gui::InspectionShareFile store(share_path);
  REQUIRE(store.save(document, true).status ==
          kpt::gui::InspectionShareSaveStatus::Written);

  std::ifstream input(share_path, std::ios::binary);
  input.seekg(0, std::ios::end);
  const auto length = input.tellg();
  REQUIRE(length > 0);
  std::string encoded(static_cast<std::size_t>(length), '\0');
  input.seekg(0);
  input.read(encoded.data(), static_cast<std::streamsize>(length));
  REQUIRE(input);
  REQUIRE_FALSE(encoded.empty());
  const auto source_path = encoded.find("inside.pcd");
  REQUIRE(source_path != std::string::npos);
  std::ofstream output(share_path, std::ios::binary | std::ios::trunc);
  output << encoded.substr(0, source_path) << "../outside.pcd"
         << encoded.substr(source_path + std::string_view{"inside.pcd"}.size());
  output.close();

  kpt::gui::InspectionShareDocument preserved;
  preserved.layers.push_back(
      {kpt::gui::opaqueSourceKey("preserved"), std::nullopt,
       Eigen::Affine3d::Identity(), {}, true});
  std::string error;
  REQUIRE_FALSE(store.load(preserved, &error));
  REQUIRE_FALSE(error.empty());
  REQUIRE(preserved.layers.size() == 1);
  REQUIRE(preserved.layers.front().source_key == "opaque:preserved");

  document.layers.front().relative_source_path = "../outside.pcd";
  REQUIRE(store.save(document, true).status ==
          kpt::gui::InspectionShareSaveStatus::Failed);
}

TEST_CASE("inspection share save atomically refuses replacement and honours cancellation",
          "[inspection_share]") {
  TemporaryDirectory directory;
  const auto share_path = directory.path() / "review.kpt-review.json";
  kpt::gui::InspectionShareDocument first;
  first.layers.push_back(
      {kpt::gui::opaqueSourceKey("first"), std::nullopt,
       Eigen::Affine3d::Identity(), {}, true});
  kpt::gui::InspectionShareDocument second;
  second.layers.push_back(
      {kpt::gui::opaqueSourceKey("second"), std::nullopt,
       Eigen::Affine3d::Identity(), {}, true});

  auto write_first = std::async(std::launch::async, [&] {
    return kpt::gui::InspectionShareFile(share_path).save(first, false);
  });
  auto write_second = std::async(std::launch::async, [&] {
    return kpt::gui::InspectionShareFile(share_path).save(second, false);
  });
  const auto first_result = write_first.get();
  const auto second_result = write_second.get();
  const unsigned writes =
      (first_result.status == kpt::gui::InspectionShareSaveStatus::Written ? 1U
                                                                            : 0U) +
      (second_result.status == kpt::gui::InspectionShareSaveStatus::Written ? 1U
                                                                             : 0U);
  const unsigned skips =
      (first_result.status == kpt::gui::InspectionShareSaveStatus::Skipped ? 1U
                                                                            : 0U) +
      (second_result.status == kpt::gui::InspectionShareSaveStatus::Skipped ? 1U
                                                                             : 0U);
  REQUIRE(writes == 1);
  REQUIRE(skips == 1);

  kpt::gui::InspectionShareDocument stored;
  REQUIRE(kpt::gui::InspectionShareFile(share_path).load(stored));
  REQUIRE(stored.layers.size() == 1);
  const auto preserved_key = stored.layers.front().source_key;
  REQUIRE((preserved_key == "opaque:first" || preserved_key == "opaque:second"));

  std::stop_source cancelled;
  cancelled.request_stop();
  const auto cancelled_result = kpt::gui::InspectionShareFile(share_path).save(
      first, true, cancelled.get_token());
  REQUIRE(cancelled_result.status ==
          kpt::gui::InspectionShareSaveStatus::Cancelled);
  REQUIRE(kpt::gui::InspectionShareFile(share_path).load(stored));
  REQUIRE(stored.layers.front().source_key == preserved_key);
}

TEST_CASE("inspection share rejects malformed state without replacing caller data",
          "[inspection_share]") {
  TemporaryDirectory directory;
  const auto share_path = directory.path() / "review.kpt-review.json";
  kpt::gui::InspectionShareFile store(share_path);

  kpt::gui::InspectionShareDocument keep;
  keep.layers.push_back(
      {kpt::gui::opaqueSourceKey("keep"), std::nullopt,
       Eigen::Affine3d::Identity(), {}, true});
  std::string error;
  REQUIRE(store.save(keep, true).status ==
          kpt::gui::InspectionShareSaveStatus::Written);

  {
    std::ofstream output(share_path, std::ios::binary | std::ios::trunc);
    output << "{\"schema_version\":2,\"layers\":[]}";
  }
  REQUIRE_FALSE(store.load(keep, &error));
  REQUIRE_FALSE(error.empty());
  REQUIRE(keep.layers.size() == 1);
  REQUIRE(keep.layers.front().source_key == "opaque:keep");

  kpt::gui::InspectionShareDocument invalid;
  invalid.layers.push_back(
      {kpt::gui::opaqueSourceKey("opaque"), std::filesystem::path{"/absolute.pcd"},
       Eigen::Affine3d::Identity(), {}, true});
  const auto invalid_result = store.save(invalid, true);
  REQUIRE(invalid_result.status == kpt::gui::InspectionShareSaveStatus::Failed);
  REQUIRE_FALSE(invalid_result.message.empty());

  kpt::gui::InspectionShareDocument duplicate;
  duplicate.layers.push_back(
      {kpt::gui::opaqueSourceKey("same"), std::nullopt,
       Eigen::Affine3d::Identity(), {}, true});
  duplicate.layers.push_back(duplicate.layers.front());
  REQUIRE(store.save(duplicate, true).status ==
          kpt::gui::InspectionShareSaveStatus::Failed);
}

TEST_CASE("inspection share rejects non-renderable bookmark cameras",
          "[inspection_share]") {
  TemporaryDirectory directory;
  kpt::gui::InspectionShareDocument document;
  auto invalid = snapshot();
  invalid.distance = std::numeric_limits<double>::max();
  document.bookmarks.emplace_back("bad", invalid);
  kpt::gui::InspectionShareFile store(directory.path() / "review.json");
  const auto result = store.save(document, true);
  REQUIRE(result.status == kpt::gui::InspectionShareSaveStatus::Failed);
  REQUIRE_FALSE(result.message.empty());
}

} // namespace
