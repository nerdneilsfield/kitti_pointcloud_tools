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

TEST_CASE("inspection share preserves valid UTF-8 opaque keys and rejects bad input",
          "[inspection_share]") {
  TemporaryDirectory directory;
  const auto share_path = directory.path() / "review.kpt-review.json";
  const std::string payload = std::string{"remote/"} + "\xe2\x98\x83";
  const std::string source_key = kpt::gui::opaqueSourceKey(payload);

  kpt::gui::Scene scene;
  static_cast<void>(scene.addLayer(source_key));
  kpt::gui::InspectionSettings settings;
  const auto document =
      kpt::gui::InspectionShareFile::capture(scene, settings, share_path);
  REQUIRE(document.layers.size() == 1);
  REQUIRE(document.layers.front().source_key == source_key);

  kpt::gui::InspectionShareFile store(share_path);
  REQUIRE(store.save(document, true).status ==
          kpt::gui::InspectionShareSaveStatus::Written);
  kpt::gui::InspectionShareDocument loaded;
  REQUIRE(store.load(loaded));
  REQUIRE(loaded.layers.front().source_key == source_key);

  std::ifstream input(share_path, std::ios::binary);
  input.seekg(0, std::ios::end);
  const auto length = input.tellg();
  REQUIRE(length > 0);
  std::string encoded(static_cast<std::size_t>(length), '\0');
  input.seekg(0);
  input.read(encoded.data(), static_cast<std::streamsize>(length));
  REQUIRE(input);
  const auto key_offset = encoded.find(source_key);
  REQUIRE(key_offset != std::string::npos);

  const std::string valid_encoded = encoded;
  const std::string malformed = std::string{"opaque:"} + "\xc0\x80";
  encoded.replace(key_offset, source_key.size(), malformed);
  std::ofstream output(share_path, std::ios::binary | std::ios::trunc);
  output.write(encoded.data(), static_cast<std::streamsize>(encoded.size()));
  output.close();

  kpt::gui::InspectionShareDocument preserved;
  preserved.layers.push_back(
      {kpt::gui::opaqueSourceKey("preserved"), std::nullopt,
       Eigen::Affine3d::Identity(), {}, true});
  std::string error;
  REQUIRE_FALSE(store.load(preserved, &error));
  REQUIRE_FALSE(error.empty());
  REQUIRE(preserved.layers.front().source_key == "opaque:preserved");

  encoded = valid_encoded;
  encoded.replace(key_offset, source_key.size(), "opaque:\\u0080");
  std::ofstream c1_output(share_path, std::ios::binary | std::ios::trunc);
  c1_output.write(encoded.data(), static_cast<std::streamsize>(encoded.size()));
  c1_output.close();
  error.clear();
  REQUIRE_FALSE(store.load(preserved, &error));
  REQUIRE_FALSE(error.empty());
  REQUIRE(preserved.layers.front().source_key == "opaque:preserved");
}

TEST_CASE("inspection share bounds source keys by UTF-8 bytes",
          "[inspection_share]") {
  TemporaryDirectory directory;
  const auto share_path = directory.path() / "review.kpt-review.json";
  constexpr std::string_view opaque_prefix{"opaque:"};
  constexpr std::string_view telescope{"\xf0\x9f\x94\xad"};
  const std::size_t payload_bytes =
      kpt::gui::kMaxSourceKeyBytes - opaque_prefix.size();
  const std::string ascii_key =
      kpt::gui::opaqueSourceKey(std::string(payload_bytes, 'a'));
  std::string astral_payload(payload_bytes - telescope.size(), 'a');
  astral_payload += telescope;
  const std::string astral_key = kpt::gui::opaqueSourceKey(astral_payload);

  kpt::gui::Scene scene;
  static_cast<void>(scene.addLayer(ascii_key));
  static_cast<void>(scene.addLayer(astral_key));
  kpt::gui::InspectionSettings settings;
  auto document =
      kpt::gui::InspectionShareFile::capture(scene, settings, share_path);
  REQUIRE(document.layers.size() == 2);
  REQUIRE(document.layers[0].source_key == ascii_key);
  REQUIRE(document.layers[1].source_key == astral_key);

  kpt::gui::InspectionShareFile store(share_path);
  REQUIRE(store.save(document, true).status ==
          kpt::gui::InspectionShareSaveStatus::Written);
  kpt::gui::InspectionShareDocument loaded;
  REQUIRE(store.load(loaded));
  REQUIRE(loaded.layers[0].source_key == ascii_key);
  REQUIRE(loaded.layers[1].source_key == astral_key);

  std::ifstream input(share_path, std::ios::binary);
  input.seekg(0, std::ios::end);
  const auto length = input.tellg();
  REQUIRE(length > 0);
  std::string encoded(static_cast<std::size_t>(length), '\0');
  input.seekg(0);
  input.read(encoded.data(), static_cast<std::streamsize>(length));
  REQUIRE(input);
  const auto key_offset = encoded.find(ascii_key);
  REQUIRE(key_offset != std::string::npos);
  const std::string too_long_key = ascii_key + "a";
  encoded.replace(key_offset, ascii_key.size(), too_long_key);
  std::ofstream output(share_path, std::ios::binary | std::ios::trunc);
  output.write(encoded.data(), static_cast<std::streamsize>(encoded.size()));
  output.close();

  kpt::gui::InspectionShareDocument preserved;
  preserved.layers.push_back(
      {kpt::gui::opaqueSourceKey("preserved"), std::nullopt,
       Eigen::Affine3d::Identity(), {}, true});
  std::string error;
  REQUIRE_FALSE(store.load(preserved, &error));
  REQUIRE_FALSE(error.empty());
  REQUIRE(preserved.layers.front().source_key == "opaque:preserved");

  document.layers.front().source_key = too_long_key;
  REQUIRE(store.save(document, true).status ==
          kpt::gui::InspectionShareSaveStatus::Failed);
}

TEST_CASE("inspection share v2 matches the cross-runtime semantic fixture",
          "[inspection_share]") {
  const std::filesystem::path fixture{
      "tests/data/review-share-v2-native-contract.json"};
  kpt::gui::InspectionShareDocument document;
  REQUIRE(kpt::gui::InspectionShareFile(fixture).load(document));
  REQUIRE(document.layers.size() == 5);
  REQUIRE(document.layers[0].style.color_by == kpt::ColorBy::Intensity);
  REQUIRE(document.layers[1].style.color_by == kpt::ColorBy::RGB);
  REQUIRE(document.layers[2].style.color_by == kpt::ColorBy::Z);
  REQUIRE(document.layers[3].style.color_by == kpt::ColorBy::Label);
  REQUIRE(document.layers[4].style.color_by == kpt::ColorBy::None);
  REQUIRE(document.layers[0].local_to_world.matrix()(0, 0) == Approx(-1.0));
  REQUIRE(document.layers[0].local_to_world.matrix()(0, 1) == Approx(0.25));
  REQUIRE(document.layers[3].local_to_world.matrix()(1, 1) == Approx(-1.0));
  REQUIRE(document.layers[4].style.point_size == Approx(5.0F));
  REQUIRE(document.layers[4].style.opacity == Approx(0.0F));
  REQUIRE(document.layers[4].style.scalar_min == Approx(8.0F));
  REQUIRE(document.layers[4].style.scalar_max == Approx(8.0F));
  REQUIRE(document.roi.has_value());
  REQUIRE(document.roi->contains(point(-1.0, -2.0, -3.0)));
  REQUIRE(document.measurements.size() == 1);
  REQUIRE(document.bookmarks.size() == 1);
  REQUIRE(document.bookmarks.front().name() == "Review view");
  REQUIRE(document.bookmarks.front().camera().camera_to_world(0, 2) ==
          Approx(1.0F));

  const auto base = std::filesystem::absolute(fixture).parent_path();
  REQUIRE(kpt::gui::InspectionShareFile::resolveSourcePath(
              fixture, document.layers[0]) ==
          std::optional<std::filesystem::path>{base / "clouds/intensity.xyzi"});
  REQUIRE(kpt::gui::InspectionShareFile::resolveSourcePath(
              fixture, document.layers[1]) ==
          std::optional<std::filesystem::path>{base / "clouds/rgb.xyzrgb"});
  REQUIRE(kpt::gui::InspectionShareFile::resolveSourcePath(
              fixture, document.layers[4]) ==
          std::optional<std::filesystem::path>{base / "clouds/fixed.xyzi"});

  const kpt::gui::InspectionShareLayer foreign_path{
      "path:C:/review/scan.xyz", std::filesystem::path{"clouds/windows.xyz"},
      Eigen::Affine3d::Identity(), {}, true};
  REQUIRE(kpt::gui::InspectionShareFile::resolveSourcePath(fixture, foreign_path) ==
          std::optional<std::filesystem::path>{base / "clouds/windows.xyz"});
}

TEST_CASE("inspection share v2 rejects v1 without replacing caller data",
          "[inspection_share]") {
  const std::filesystem::path v1_fixture{
      "tests/data/review-share-v1-cross-runtime.json"};
  kpt::gui::InspectionShareDocument preserved;
  preserved.layers.push_back(
      {kpt::gui::opaqueSourceKey("preserved"), std::nullopt,
       Eigen::Affine3d::Identity(), {}, true});
  std::string error;
  REQUIRE_FALSE(kpt::gui::InspectionShareFile(v1_fixture).load(preserved,
                                                                &error));
  REQUIRE(error.find("v1") != std::string::npos);
  REQUIRE(preserved.layers.front().source_key == "opaque:preserved");
}

TEST_CASE("inspection share v2 validates printable UTF-8 path keys",
          "[inspection_share]") {
  TemporaryDirectory directory;
  const auto share_path = directory.path() / "review.kpt-review.json";
  const std::string source_key =
      std::string{"path:/review/"} + "\xe7\x82\xb9\xe4\xba\x91/scan.xyz";
  kpt::gui::InspectionShareDocument document;
  document.layers.push_back(
      {source_key, std::filesystem::path{"sources/scan.xyz"},
       Eigen::Affine3d::Identity(), {}, true});
  kpt::gui::InspectionShareFile store(share_path);
  REQUIRE(store.save(document, true).status ==
          kpt::gui::InspectionShareSaveStatus::Written);

  kpt::gui::InspectionShareDocument loaded;
  REQUIRE(store.load(loaded));
  REQUIRE(loaded.layers.front().source_key == source_key);

  std::ifstream input(share_path, std::ios::binary);
  input.seekg(0, std::ios::end);
  const auto length = input.tellg();
  REQUIRE(length > 0);
  std::string encoded(static_cast<std::size_t>(length), '\0');
  input.seekg(0);
  input.read(encoded.data(), static_cast<std::streamsize>(length));
  REQUIRE(input);
  const auto source_offset = encoded.find(source_key);
  REQUIRE(source_offset != std::string::npos);
  encoded.replace(source_offset, source_key.size(),
                  "path:/review/\\u0080/scan.xyz");
  std::ofstream output(share_path, std::ios::binary | std::ios::trunc);
  output.write(encoded.data(), static_cast<std::streamsize>(encoded.size()));
  output.close();

  kpt::gui::InspectionShareDocument preserved;
  preserved.layers.push_back(
      {kpt::gui::opaqueSourceKey("preserved"), std::nullopt,
       Eigen::Affine3d::Identity(), {}, true});
  std::string error;
  REQUIRE_FALSE(store.load(preserved, &error));
  REQUIRE_FALSE(error.empty());
  REQUIRE(preserved.layers.front().source_key == "opaque:preserved");
}

TEST_CASE("inspection share v2 enforces portable layer-style bounds",
          "[inspection_share]") {
  TemporaryDirectory directory;
  kpt::gui::InspectionShareFile store(directory.path() / "review.json");
  const auto rejected = [&store](kpt::gui::LayerStyle style) {
    kpt::gui::InspectionShareDocument document;
    document.layers.push_back(
        {kpt::gui::opaqueSourceKey("style"), std::nullopt,
         Eigen::Affine3d::Identity(), std::move(style), true});
    return store.save(document, true).status ==
           kpt::gui::InspectionShareSaveStatus::Failed;
  };

  kpt::gui::LayerStyle style;
  style.point_size = 0.0F;
  REQUIRE(rejected(style));
  style.point_size = 5.01F;
  REQUIRE(rejected(style));
  style = {};
  style.opacity = -0.01F;
  REQUIRE(rejected(style));
  style = {};
  style.fixed_color.x() = 1.01F;
  REQUIRE(rejected(style));
  style = {};
  style.noise_color.y() = -0.01F;
  REQUIRE(rejected(style));
  style = {};
  style.scalar_min = 2.0F;
  style.scalar_max = 1.0F;
  REQUIRE(rejected(style));
  style = {};
  style.color_by = static_cast<kpt::ColorBy>(5);
  REQUIRE(rejected(style));
  style = {};
  style.color_map = static_cast<kpt::gui::ColorMap>(10);
  REQUIRE(rejected(style));
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
