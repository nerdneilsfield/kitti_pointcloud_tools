#include <catch2/catch.hpp>
#include "kpt/render/render.hpp"
#include <memory>

TEST_CASE("renderMultiView produces images", "[render]") {
  auto cloud = std::make_shared<kpt::PointCloudIRGB>();
  for (int i = 0; i < 100; ++i) {
    kpt::PointT pt;
    pt.x = (i % 10) * 0.5f; pt.y = (i / 10) * 0.5f; pt.z = 0;
    pt.r = 255; pt.g = 0; pt.b = 0; pt.intensity = 0.5f;
    cloud->push_back(pt);
  }
  kpt::RenderOpts opts; opts.width = 64; opts.height = 64;
  auto results = kpt::renderMultiView(cloud, opts);
  REQUIRE(results.size() == opts.views.size());
  REQUIRE(results[0].image.cols == 64);
  REQUIRE(results[0].image.rows == 64);
  REQUIRE(!results[0].view_name.empty());
}

TEST_CASE("renderMultiView default opts yields 10 views", "[render]") {
  auto cloud = std::make_shared<kpt::PointCloudIRGB>();
  for (int i = 0; i < 50; ++i) {
    kpt::PointT pt;
    pt.x = static_cast<float>(i % 10);
    pt.y = static_cast<float>(i / 10);
    pt.z = static_cast<float>(i) * 0.1f;
    pt.r = 0; pt.g = 255; pt.b = 0; pt.intensity = 1.0f;
    cloud->push_back(pt);
  }
  kpt::RenderOpts opts;
  auto results = kpt::renderMultiView(cloud, opts);
  REQUIRE(results.size() == 10);
  for (const auto& r : results) {
    REQUIRE(!r.view_name.empty());
    REQUIRE(r.image.cols == opts.width);
    REQUIRE(r.image.rows == opts.height);
    REQUIRE(r.image.type() == CV_8UC3);
  }
}

TEST_CASE("renderMultiView empty cloud still returns sized results", "[render]") {
  auto cloud = std::make_shared<kpt::PointCloudIRGB>();
  kpt::RenderOpts opts; opts.width = 32; opts.height = 24;
  auto results = kpt::renderMultiView(cloud, opts);
  REQUIRE(results.size() == opts.views.size());
  // All-black images of correct size
  REQUIRE(results[0].image.cols == 32);
  REQUIRE(results[0].image.rows == 24);
}

TEST_CASE("renderMultiView zero-size cloud avoids NaN view matrix", "[render]") {
  // Single point => bounding box has zero dimensions. This used to produce
  // distance=0 and a degenerate (NaN) view matrix.
  auto cloud = std::make_shared<kpt::PointCloudIRGB>();
  kpt::PointT pt;
  pt.x = 1.0f; pt.y = 2.0f; pt.z = 3.0f;
  pt.r = 255; pt.g = 0; pt.b = 0; pt.intensity = 0.5f;
  cloud->push_back(pt);
  kpt::RenderOpts opts; opts.width = 32; opts.height = 24;
  auto results = kpt::renderMultiView(cloud, opts);
  REQUIRE(results.size() == opts.views.size());
  REQUIRE(results[0].image.cols == 32);
  REQUIRE(results[0].image.rows == 24);
}
