#include "cli/legacy_player_snapshot.hpp"

#include <catch2/catch.hpp>

#include <filesystem>
#include <fstream>
#include <random>

namespace {

struct SnapshotTempDirectory {
  static std::string token() {
    static std::mt19937_64 generator(std::random_device{}());
    return std::to_string(generator());
  }

  std::filesystem::path path =
      std::filesystem::temp_directory_path() / ("kpt-snapshot-" + token());

  SnapshotTempDirectory() { std::filesystem::create_directories(path); }
  ~SnapshotTempDirectory() {
    std::error_code ignored;
    std::filesystem::remove_all(path, ignored);
  }
};

void writeXyz(const std::filesystem::path &path) {
  std::ofstream output(path);
  output << "1 2 3\n";
}

} // namespace

TEST_CASE("legacy snapshot output name preserves Unicode frame stem",
          "[cli][snapshot][unicode]") {
  const auto prefix = std::filesystem::path(u8"输出目录/前缀");
  const auto frame = std::filesystem::path(u8"输入目录/点云.pcd");

  CHECK(kpt::cli::sequenceSnapshotOutputPath(prefix, frame, "front") ==
        std::filesystem::path(u8"输出目录/前缀_点云_front.png"));
}

TEST_CASE("legacy player snapshot renders sequence and overwrites",
          "[cli][snapshot]") {
  SnapshotTempDirectory temp;
  const auto input = temp.path / std::filesystem::path(u8"输入");
  const auto prefix = temp.path / std::filesystem::path(u8"输出") /
                      std::filesystem::path(u8"快照");
  std::filesystem::create_directories(input);
  writeXyz(input / std::filesystem::path(u8"点云.xyz"));

  kpt::cli::PlayerSnapshotRequest request;
  request.sequence.input_dir = input;
  request.sequence.glob = "*.xyz";
  request.output_prefix = prefix;
  request.width = 16;
  request.height = 12;
  request.fov = 90.0F;
  request.views = {kpt::View::Front};

  CHECK(kpt::cli::runPlayerSnapshots(request) == 1);
  const auto output =
      prefix.parent_path() / std::filesystem::path(u8"快照_点云_front.png");
  REQUIRE(std::filesystem::is_regular_file(output));
  const auto first_size = std::filesystem::file_size(output);
  REQUIRE(first_size > 0);

  std::ofstream(output, std::ios::binary | std::ios::trunc) << "stale";
  REQUIRE(std::filesystem::file_size(output) == 5);
  CHECK(kpt::cli::runPlayerSnapshots(request) == 1);
  CHECK(std::filesystem::file_size(output) == first_size);
}

TEST_CASE("legacy player snapshot validates request", "[cli][snapshot]") {
  kpt::cli::PlayerSnapshotRequest request;
  request.sequence.input_dir = ".";
  request.output_prefix = "snapshot";
  request.views = {kpt::View::Front};

  request.width = 0;
  CHECK_THROWS_AS(kpt::cli::runPlayerSnapshots(request), std::invalid_argument);
  request.width = 640;
  request.views.clear();
  CHECK_THROWS_AS(kpt::cli::runPlayerSnapshots(request), std::invalid_argument);
  request.views = {kpt::View::Front};
  request.output_prefix.clear();
  CHECK_THROWS_AS(kpt::cli::runPlayerSnapshots(request), std::invalid_argument);
}
