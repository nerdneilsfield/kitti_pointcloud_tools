#include "kpt/render/render.hpp"
#include <array>
#include <catch2/catch.hpp>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <memory>
#include <random>
#include <vector>

#define STB_IMAGE_IMPLEMENTATION
#include <stb_image.h>

namespace fs = std::filesystem;

namespace {

struct RenderTempDirectory {
  static std::string token() {
    static std::mt19937_64 generator(std::random_device{}());
    return std::to_string(generator());
  }

  fs::path path = fs::temp_directory_path() / ("kpt-render-" + token());
  RenderTempDirectory() { fs::create_directories(path); }
  ~RenderTempDirectory() {
    std::error_code ignored;
    fs::remove_all(path, ignored);
  }
};

std::vector<unsigned char> readFile(const fs::path &path) {
  std::ifstream input(path, std::ios::binary | std::ios::ate);
  REQUIRE(input);
  const auto size = input.tellg();
  REQUIRE(size > 0);
  input.seekg(0);
  std::vector<unsigned char> encoded(static_cast<std::size_t>(size));
  input.read(reinterpret_cast<char *>(encoded.data()),
             static_cast<std::streamsize>(encoded.size()));
  REQUIRE(input);
  return encoded;
}

kpt::ImageRGB8 readImageNative(const fs::path &path) {
  const auto encoded = readFile(path);
  int width = 0;
  int height = 0;
  int channels = 0;
  auto *decoded =
      stbi_load_from_memory(encoded.data(), static_cast<int>(encoded.size()),
                            &width, &height, &channels, 3);
  REQUIRE(decoded != nullptr);
  kpt::ImageRGB8 image(width, height);
  std::memcpy(image.pixels().data(), decoded, image.pixels().size());
  stbi_image_free(decoded);
  return image;
}

kpt::ImageRGB8 solidImage(int width, int height, std::uint8_t red,
                          std::uint8_t green, std::uint8_t blue) {
  kpt::ImageRGB8 image(width, height);
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      auto *pixel = image.pixel(x, y);
      pixel[0] = red;
      pixel[1] = green;
      pixel[2] = blue;
    }
  }
  return image;
}

} // namespace

TEST_CASE("renderMultiView produces images", "[render]") {
  auto cloud = std::make_shared<kpt::PointCloudIRGB>();
  for (int i = 0; i < 100; ++i) {
    kpt::PointT pt;
    pt.x = (i % 10) * 0.5f;
    pt.y = (i / 10) * 0.5f;
    pt.z = 0;
    pt.r = 255;
    pt.g = 0;
    pt.b = 0;
    pt.intensity = 0.5f;
    cloud->push_back(pt);
  }
  kpt::RenderOpts opts;
  opts.width = 64;
  opts.height = 64;
  auto results = kpt::renderMultiView(cloud, opts);
  REQUIRE(results.size() == opts.views.size());
  REQUIRE(results[0].image.width() == 64);
  REQUIRE(results[0].image.height() == 64);
  REQUIRE(results[0].view_name == "front");
}

TEST_CASE("renderMultiView default opts yields 10 views", "[render]") {
  auto cloud = std::make_shared<kpt::PointCloudIRGB>();
  for (int i = 0; i < 50; ++i) {
    kpt::PointT pt;
    pt.x = static_cast<float>(i % 10);
    pt.y = static_cast<float>(i / 10);
    pt.z = static_cast<float>(i) * 0.1f;
    pt.r = 0;
    pt.g = 255;
    pt.b = 0;
    pt.intensity = 1.0f;
    cloud->push_back(pt);
  }
  kpt::RenderOpts opts;
  auto results = kpt::renderMultiView(cloud, opts);
  REQUIRE(results.size() == 10);
  for (const auto &r : results) {
    REQUIRE(!r.view_name.empty());
    REQUIRE(r.image.width() == opts.width);
    REQUIRE(r.image.height() == opts.height);
    REQUIRE(r.image.pixels().size() ==
            static_cast<std::size_t>(opts.width * opts.height * 3));
  }
}

TEST_CASE("renderMultiView empty cloud still returns sized results",
          "[render]") {
  auto cloud = std::make_shared<kpt::PointCloudIRGB>();
  kpt::RenderOpts opts;
  opts.width = 32;
  opts.height = 24;
  auto results = kpt::renderMultiView(cloud, opts);
  REQUIRE(results.size() == opts.views.size());
  // All-black images of correct size
  REQUIRE(results[0].image.width() == 32);
  REQUIRE(results[0].image.height() == 24);
}

TEST_CASE("renderMultiView rejects a null cloud", "[render]") {
  kpt::RenderOpts opts;
  const kpt::PointCloudIRGBConstPtr cloud;
  REQUIRE_THROWS_AS(kpt::renderMultiView(cloud, opts), std::invalid_argument);
}

TEST_CASE("renderMultiView zero-size cloud avoids NaN view matrix",
          "[render]") {
  // Single point => bounding box has zero dimensions. This used to produce
  // distance=0 and a degenerate (NaN) view matrix.
  auto cloud = std::make_shared<kpt::PointCloudIRGB>();
  kpt::PointT pt;
  pt.x = 1.0f;
  pt.y = 2.0f;
  pt.z = 3.0f;
  pt.r = 255;
  pt.g = 0;
  pt.b = 0;
  pt.intensity = 0.5f;
  cloud->push_back(pt);
  kpt::RenderOpts opts;
  opts.width = 32;
  opts.height = 24;
  auto results = kpt::renderMultiView(cloud, opts);
  REQUIRE(results.size() == opts.views.size());
  REQUIRE(results[0].image.width() == 32);
  REQUIRE(results[0].image.height() == 24);
}

TEST_CASE("render view names are stable", "[render]") {
  constexpr std::array<kpt::View, 10> views = {
      kpt::View::Front,         kpt::View::Right,
      kpt::View::Back,          kpt::View::Left,
      kpt::View::Top,           kpt::View::Bottom,
      kpt::View::TopRightFront, kpt::View::TopLeftFront,
      kpt::View::BotRightFront, kpt::View::BotLeftFront};
  constexpr std::array<std::string_view, 10> names = {
      "front",         "right",       "back",          "left",
      "top",           "bottom",      "toprightfront", "topleftfront",
      "botrightfront", "botleftfront"};
  for (std::size_t index = 0; index < views.size(); ++index)
    REQUIRE(kpt::viewName(views[index]) == names[index]);
}

TEST_CASE("atomic image writing skips and overwrites", "[render]") {
  RenderTempDirectory temp;
  const auto output = temp.path / "image.png";
  const auto first = solidImage(2, 2, 1, 2, 3);
  const auto second = solidImage(2, 2, 4, 5, 6);

  REQUIRE(kpt::writeImageAtomic(output, first, false) ==
          kpt::ImageWriteStatus::Written);
  REQUIRE(kpt::writeImageAtomic(output, second, false) ==
          kpt::ImageWriteStatus::Skipped);
  const auto first_decoded = readImageNative(output);
  REQUIRE(first_decoded.pixel(0, 0)[0] == 1);
  REQUIRE(first_decoded.pixel(0, 0)[1] == 2);
  REQUIRE(first_decoded.pixel(0, 0)[2] == 3);
  REQUIRE(kpt::writeImageAtomic(output, second, true) ==
          kpt::ImageWriteStatus::Written);
  const auto second_decoded = readImageNative(output);
  REQUIRE(second_decoded.pixel(0, 0)[0] == 4);
  REQUIRE(second_decoded.pixel(0, 0)[1] == 5);
  REQUIRE(second_decoded.pixel(0, 0)[2] == 6);

  for (const auto &entry : fs::directory_iterator(temp.path)) {
    REQUIRE(entry.path().filename().string().find(".kpt-tmp-") ==
            std::string::npos);
  }
}

TEST_CASE("atomic image writing supports native Unicode paths",
          "[render][unicode]") {
  RenderTempDirectory temp;
  const auto directory = temp.path / fs::path(u8"渲染目录");
  const auto output = directory / fs::path(u8"中文图像.png");
  const auto image = solidImage(2, 2, 7, 8, 9);

  REQUIRE(kpt::writeImageAtomic(output, image, true) ==
          kpt::ImageWriteStatus::Written);
  const auto decoded = readImageNative(output);
  REQUIRE(decoded.pixel(0, 0)[0] == 7);
  REQUIRE(decoded.pixel(0, 0)[1] == 8);
  REQUIRE(decoded.pixel(0, 0)[2] == 9);
}

TEST_CASE("atomic image writing cleans a failed temporary file", "[render]") {
  RenderTempDirectory temp;
  const auto output = temp.path / "image.unsupported";
  const auto image = solidImage(1, 1, 1, 2, 3);

  REQUIRE_THROWS(kpt::writeImageAtomic(output, image, true));
  REQUIRE_FALSE(fs::exists(output));
  REQUIRE(fs::is_empty(temp.path));
}
