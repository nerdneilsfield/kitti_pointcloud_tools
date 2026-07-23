#include <catch2/catch.hpp>
#include "kpt/label/label.hpp"
#include <filesystem>
#include <memory>

namespace fs = std::filesystem;

static const fs::path data_dir = "test/data";

TEST_CASE("loadLabel reads ints", "[label]") {
  auto labels = kpt::loadLabel(data_dir / "tiny.label");
  REQUIRE(labels.size() == 3);
  REQUIRE(labels[0] == 40);
  REQUIRE(labels[1] == 70);
  REQUIRE(labels[2] == 10);
}

TEST_CASE("loadLabel missing file throws", "[label]") {
  REQUIRE_THROWS_AS(kpt::loadLabel("nope.label"), std::runtime_error);
}

TEST_CASE("rangeNetLabelMap has expected entries", "[label]") {
  auto lm = kpt::rangeNetLabelMap();
  REQUIRE(lm.at(40) == 1);   // road
  REQUIRE(lm.at(70) == 7);   // vegetation
  REQUIRE(lm.at(10) == 0);   // car -> unlabeled compact
  REQUIRE(lm.count(9999) == 0);
}

TEST_CASE("rgbLabelMap has expected entries", "[label]") {
  auto rm = kpt::rgbLabelMap();
  REQUIRE(std::get<0>(rm.at(1)) == 34);
  REQUIRE(std::get<1>(rm.at(1)) == 139);
  REQUIRE(std::get<2>(rm.at(1)) == 0);
  REQUIRE(std::get<0>(rm.at(-1)) == 227);
}

TEST_CASE("applyLabel colors points", "[label]") {
  kpt::PointCloudIRGB cloud;
  for (int i = 0; i < 3; ++i) {
    kpt::PointT pt;
    pt.x = static_cast<float>(i); pt.y = 0; pt.z = 0; pt.intensity = 0;
    cloud.push_back(pt);
  }
  auto cloud_ptr = std::make_shared<kpt::PointCloudIRGB>(cloud);
  auto labels = std::vector<int>{40, 70, 10};
  auto lm = kpt::rangeNetLabelMap();
  auto rm = kpt::rgbLabelMap();
  auto out = kpt::applyLabel(cloud_ptr, labels, lm, rm);
  REQUIRE(out->size() == 3);
  // 40 -> road -> compact 1 -> (34,139,0)
  REQUIRE(out->points[0].r == 34);
  REQUIRE(out->points[0].g == 139);
  REQUIRE(out->points[0].b == 0);
  REQUIRE(out->points[0].intensity == 1.0f);
  // 70 -> vegetation -> compact 7 -> (124,252,0)
  REQUIRE(out->points[1].r == 124);
  REQUIRE(out->points[1].g == 252);
  // 10 -> car -> compact 0 -> (0,0,0)
  REQUIRE(out->points[2].r == 0);
  REQUIRE(out->points[2].intensity == 0.0f);
}

TEST_CASE("applyLabel unknown label -> compact -1 -> red RGB", "[label]") {
  kpt::PointCloudIRGB cloud;
  kpt::PointT pt; pt.x = 0; pt.y = 0; pt.z = 0;
  cloud.push_back(pt);
  auto cloud_ptr = std::make_shared<kpt::PointCloudIRGB>(cloud);
  auto labels = std::vector<int>{9999};  // not in label_map
  auto lm = kpt::rangeNetLabelMap();
  auto rm = kpt::rgbLabelMap();
  auto out = kpt::applyLabel(cloud_ptr, labels, lm, rm);
  REQUIRE(out->size() == 1);
  REQUIRE(out->points[0].intensity == -1.0f);
  // -1 -> (227,23,13)
  REQUIRE(out->points[0].r == 227);
  REQUIRE(out->points[0].g == 23);
  REQUIRE(out->points[0].b == 13);
}

TEST_CASE("applyLabel mismatched sizes throws", "[label]") {
  kpt::PointCloudIRGB cloud;
  kpt::PointT pt; pt.x = 0; pt.y = 0; pt.z = 0;
  cloud.push_back(pt);
  auto cloud_ptr = std::make_shared<kpt::PointCloudIRGB>(cloud);
  auto labels = std::vector<int>{40, 70};  // 2 labels, 1 point
  auto lm = kpt::rangeNetLabelMap();
  auto rm = kpt::rgbLabelMap();
  REQUIRE_THROWS_AS(kpt::applyLabel(cloud_ptr, labels, lm, rm),
                    std::invalid_argument);
}

TEST_CASE("applyLabel drop_unlabeled removes -1 compact", "[label]") {
  kpt::PointCloudIRGB cloud;
  for (int i = 0; i < 2; ++i) {
    kpt::PointT pt;
    pt.x = static_cast<float>(i); pt.y = 0; pt.z = 0;
    cloud.push_back(pt);
  }
  auto cloud_ptr = std::make_shared<kpt::PointCloudIRGB>(cloud);
  // 1 -> outlier -> compact -1 ; 40 -> road -> compact 1
  auto labels = std::vector<int>{1, 40};
  auto lm = kpt::rangeNetLabelMap();
  auto rm = kpt::rgbLabelMap();
  auto out = kpt::applyLabel(cloud_ptr, labels, lm, rm, true);
  REQUIRE(out->size() == 1);
  REQUIRE(out->points[0].intensity == 1.0f);
}
