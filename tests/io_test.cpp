#include <catch2/catch.hpp>
#include "kpt/io/io.hpp"
#include "kpt/io/format.hpp"
#include <filesystem>

namespace fs = std::filesystem;

static const fs::path data_dir = "test/data";

TEST_CASE("load bin", "[io]") {
  auto cloud = kpt::load(data_dir / "tiny.bin");
  REQUIRE(cloud->size() == 3);
  REQUIRE(cloud->points[0].x == 1.0f);
  REQUIRE(cloud->points[0].intensity == 0.5f);
  REQUIRE(cloud->points[0].r == 0);
}

TEST_CASE("load xyz auto-detect 3 col", "[io]") {
  auto cloud = kpt::load(data_dir / "tiny.xyz");
  REQUIRE(cloud->size() == 3);
  REQUIRE(cloud->points[0].x == 1.0f);
  REQUIRE(cloud->points[0].intensity == 0.0f);
}

TEST_CASE("load xyzi auto-detect 4 col", "[io]") {
  auto cloud = kpt::load(data_dir / "tiny.xyzi");
  REQUIRE(cloud->size() == 3);
  REQUIRE(cloud->points[0].intensity == 0.5f);
}

TEST_CASE("load xyzrgb auto-detect 6 col", "[io]") {
  auto cloud = kpt::load(data_dir / "tiny.xyzrgb");
  REQUIRE(cloud->size() == 3);
  REQUIRE(cloud->points[0].r == 255);
  REQUIRE(cloud->points[0].g == 0);
}

TEST_CASE("load xyzrgbi auto-detect 7 col", "[io]") {
  auto cloud = kpt::load(data_dir / "tiny.xyzrgbi");
  REQUIRE(cloud->size() == 3);
  REQUIRE(cloud->points[0].r == 255);
  REQUIRE(cloud->points[0].intensity == 0.5f);
}

TEST_CASE("detect unknown throws", "[io]") {
  REQUIRE_THROWS_AS(kpt::detect("foo.zzz"), std::runtime_error);
}

TEST_CASE("load missing file throws", "[io]") {
  REQUIRE_THROWS_AS(kpt::load("nope_xyz_does_not_exist.bin"), std::runtime_error);
}

TEST_CASE("ascii load skips bad lines, keeps good ones", "[io]") {
  auto cloud = kpt::load(data_dir / "tiny_bad.xyz");
  // 4 lines: 3 ok, 1 short (1 col), 1 odd (5 col) — wait, fixture: 3col ok / 2col skip / 5col skip / 3col ok => 2 ok
  REQUIRE(cloud->size() == 2);
  REQUIRE(cloud->points[0].x == 1.0f);
  REQUIRE(cloud->points[1].x == 1.0f);
}