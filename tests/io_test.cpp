#include "kpt/io/format.hpp"
#include "kpt/io/io.hpp"
#include <catch2/catch.hpp>
#include <filesystem>
#include <fstream>

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
  REQUIRE_THROWS_AS(kpt::load("nope_xyz_does_not_exist.bin"),
                    std::runtime_error);
}

TEST_CASE("ascii load skips bad lines, keeps good ones", "[io]") {
  auto cloud = kpt::load(data_dir / "tiny_bad.xyz");
  // 4 lines: 3 ok, 1 short (1 col), 1 odd (5 col) — wait, fixture: 3col ok /
  // 2col skip / 5col skip / 3col ok => 2 ok
  REQUIRE(cloud->size() == 2);
  REQUIRE(cloud->points[0].x == 1.0f);
  REQUIRE(cloud->points[1].x == 1.0f);
}

TEST_CASE("round-trip bin", "[io]") {
  auto cloud = kpt::load(data_dir / "tiny.bin");
  fs::path out = "build/rt.bin";
  kpt::save(out, *cloud);
  auto cloud2 = kpt::load(out);
  REQUIRE(cloud2->size() == cloud->size());
  REQUIRE(cloud2->points[1].intensity == 0.6f);
  fs::remove(out);
}

TEST_CASE("round-trip xyzrgbi explicit flavor", "[io]") {
  auto cloud = kpt::load(data_dir / "tiny.xyzrgbi");
  fs::path out = "build/rt.xyzrgbi";
  kpt::save(out, *cloud, kpt::Format::XYZRGBI);
  auto cloud2 = kpt::load(out);
  REQUIRE(cloud2->size() == cloud->size());
  REQUIRE(cloud2->points[0].x == Approx(cloud->points[0].x));
  REQUIRE(cloud2->points[0].y == Approx(cloud->points[0].y));
  REQUIRE(cloud2->points[0].z == Approx(cloud->points[0].z));
  REQUIRE(cloud2->points[0].r == cloud->points[0].r);
  REQUIRE(cloud2->points[0].g == cloud->points[0].g);
  REQUIRE(cloud2->points[0].b == cloud->points[0].b);
  REQUIRE(cloud2->points[0].intensity == Approx(cloud->points[0].intensity));
  fs::remove(out);
}

TEST_CASE("save bin drops rgb", "[io]") {
  auto cloud = kpt::load(data_dir / "tiny.xyzrgb");
  fs::path out = "build/drop.bin";
  kpt::save(out, *cloud);
  auto back = kpt::load(out);
  REQUIRE(back->points[0].r == 0); // rgb lost
  REQUIRE(back->points[0].x == 1.0f);
  fs::remove(out);
}

TEST_CASE("round-trip supports UTF-8 directories and filenames", "[io]") {
  const auto directory = fs::temp_directory_path() / "点云工具测试";
  const auto output = directory / "中文点云.xyz";
  fs::create_directories(directory);

  const auto cloud = kpt::load(data_dir / "tiny.xyz");
  kpt::save(output, *cloud);
  const auto loaded = kpt::load(output);
  REQUIRE(loaded->size() == cloud->size());

  std::error_code ignored;
  fs::remove_all(directory, ignored);
}

TEST_CASE("native Unicode paths round-trip through PCL filename APIs",
          "[io][unicode][pcl]") {
  const auto directory =
      fs::temp_directory_path() / fs::path(u8"点云工具-PCL-契约");
  fs::create_directories(directory);
  const auto cloud = kpt::load(data_dir / "tiny.xyzrgbi");

  for (const auto &filename :
       {fs::path(u8"中文点云.pcd"), fs::path(u8"中文点云.ply")}) {
    const auto output = directory / filename;
    kpt::save(output, *cloud);
    const auto loaded = kpt::load(output);
    REQUIRE(loaded->size() == cloud->size());
    REQUIRE(loaded->points[0].x == Approx(cloud->points[0].x));
    REQUIRE(loaded->points[0].intensity == Approx(cloud->points[0].intensity));
  }

  std::error_code ignored;
  fs::remove_all(directory, ignored);
}
