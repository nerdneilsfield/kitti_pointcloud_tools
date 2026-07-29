#include "kpt/cancellation.hpp"
#include "kpt/render/png_limits.hpp"
#include "kpt/render/render.hpp"
#include "platform/native_file.hpp"
#include <algorithm>
#include <array>
#include <catch2/catch.hpp>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <memory>
#include <random>
#include <vector>

#ifndef _WIN32
#include <sys/stat.h>
#endif

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

std::size_t visiblePixelCount(const kpt::ImageRGB8 &image) {
  std::size_t count = 0;
  for (std::size_t offset = 0; offset < image.pixels().size(); offset += 3) {
    if (image.pixels()[offset] != 0 || image.pixels()[offset + 1] != 0 ||
        image.pixels()[offset + 2] != 0) {
      ++count;
    }
  }
  return count;
}

bool imagesDiffer(const kpt::ImageRGB8 &lhs, const kpt::ImageRGB8 &rhs) {
  return lhs.pixels().size() != rhs.pixels().size() ||
         !std::equal(lhs.pixels().begin(), lhs.pixels().end(),
                     rhs.pixels().begin());
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
  for (const auto &result : results)
    REQUIRE(visiblePixelCount(result.image) > 0);
}

TEST_CASE("renderMultiView maps intensity-only clouds to visible pixels",
          "[render]") {
  auto cloud = std::make_shared<kpt::PointCloudIRGB>();
  for (int index = 0; index < 64; ++index) {
    kpt::PointT point;
    point.x = static_cast<float>(index % 8);
    point.y = static_cast<float>(index / 8);
    point.z = static_cast<float>(index % 3);
    point.intensity = static_cast<float>(index) / 63.0F;
    cloud->push_back(point);
  }
  kpt::RenderOpts opts;
  opts.width = 96;
  opts.height = 64;
  opts.views = {kpt::View::Front};
  const auto results = kpt::renderMultiView(cloud, opts);
  REQUIRE(results.size() == 1);
  REQUIRE(visiblePixelCount(results.front().image) > 0);
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
  REQUIRE(visiblePixelCount(results[0].image) > 0);
}

TEST_CASE("renderMultiView rejects bounds outside finite camera range",
          "[render][safety]") {
  auto cloud = std::make_shared<kpt::PointCloudIRGB>();
  kpt::PointT point{};
  point.x = std::numeric_limits<float>::max();
  point.y = std::numeric_limits<float>::max();
  point.z = std::numeric_limits<float>::min();
  point.r = 255;
  cloud->push_back(point);
  point.x = -std::numeric_limits<float>::max();
  point.y = -std::numeric_limits<float>::max();
  cloud->push_back(point);

  kpt::RenderOpts opts;
  opts.width = 16;
  opts.height = 12;
  opts.views = {kpt::View::Front};
  REQUIRE_THROWS_AS(kpt::renderMultiView(cloud, opts), std::overflow_error);
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

TEST_CASE("render color mode names are stable", "[render]") {
  CHECK(kpt::renderColorModeName(kpt::RenderColorMode::Auto) == "auto");
  CHECK(kpt::renderColorModeName(kpt::RenderColorMode::RGB) == "rgb");
  CHECK(kpt::renderColorModeName(kpt::RenderColorMode::Intensity) ==
        "intensity");
  CHECK(kpt::renderColorModeName(kpt::RenderColorMode::Z) == "z");
  CHECK(kpt::renderColorModeName(kpt::RenderColorMode::Solid) == "solid");
  CHECK(kpt::renderColorModeName(static_cast<kpt::RenderColorMode>(999)) ==
        "unknown");
}

TEST_CASE("render color modes produce visible distinct images", "[render]") {
  auto cloud = std::make_shared<kpt::PointCloudIRGB>();
  for (int index = 0; index < 64; ++index) {
    kpt::PointT point;
    point.x = static_cast<float>(index % 8);
    point.y = static_cast<float>(index / 8);
    point.z = static_cast<float>(index % 5);
    point.r = static_cast<std::uint8_t>(32 + index * 3);
    point.g = static_cast<std::uint8_t>(255 - index * 3);
    point.b = static_cast<std::uint8_t>(64 + index * 2);
    point.intensity = static_cast<float>(index) / 63.0F;
    cloud->push_back(point);
  }

  auto render = [&](kpt::RenderColorMode mode) {
    kpt::RenderOpts opts;
    opts.width = 96;
    opts.height = 64;
    opts.views = {kpt::View::TopRightFront};
    opts.color_mode = mode;
    return kpt::renderMultiView(cloud, opts).front().image;
  };

  const auto rgb = render(kpt::RenderColorMode::RGB);
  const auto intensity = render(kpt::RenderColorMode::Intensity);
  const auto z = render(kpt::RenderColorMode::Z);
  const auto solid = render(kpt::RenderColorMode::Solid);
  CHECK(visiblePixelCount(rgb) > 0);
  CHECK(visiblePixelCount(intensity) > 0);
  CHECK(visiblePixelCount(z) > 0);
  CHECK(visiblePixelCount(solid) > 0);
  CHECK(imagesDiffer(rgb, intensity));
  CHECK(imagesDiffer(intensity, z));
  CHECK(imagesDiffer(z, solid));
}

TEST_CASE("render rejects unavailable RGB and invalid color mode", "[render]") {
  auto cloud = std::make_shared<kpt::PointCloudIRGB>();
  kpt::PointT point;
  point.x = 1.0F;
  point.intensity = 0.5F;
  cloud->push_back(point);

  kpt::RenderOpts opts;
  opts.views = {kpt::View::Front};
  opts.color_mode = kpt::RenderColorMode::RGB;
  REQUIRE_THROWS_AS(kpt::renderMultiView(cloud, opts), std::invalid_argument);

  opts.color_mode = static_cast<kpt::RenderColorMode>(999);
  REQUIRE_THROWS_AS(kpt::renderMultiView(cloud, opts), std::invalid_argument);
}

TEST_CASE("PNG encoder limits reject oversized dimensions", "[render]") {
  CHECK_FALSE(kpt::pngDimensionsSupported(0, 1));
  CHECK_FALSE(kpt::pngDimensionsSupported(1, -1));
  CHECK(kpt::pngDimensionsSupported(1, 1));
  CHECK(kpt::pngDimensionsSupported(8192, 4096));
  CHECK_FALSE(kpt::pngDimensionsSupported(8193, 4096));
  CHECK_FALSE(
      kpt::pngDimensionsSupported(std::numeric_limits<int>::max() / 3 + 1, 1));
}

TEST_CASE("render validates extents and views before allocation", "[render]") {
  auto cloud = std::make_shared<kpt::PointCloudIRGB>();
  kpt::RenderOpts opts;
  opts.width = 100000;
  opts.height = 100000;
  REQUIRE_THROWS_AS(kpt::renderMultiView(cloud, opts), std::length_error);

  opts.width = 1;
  opts.height = 1;
  opts.views.clear();
  REQUIRE_THROWS_AS(kpt::renderMultiView(cloud, opts), std::invalid_argument);
  opts.views = {static_cast<kpt::View>(999)};
  REQUIRE_THROWS_AS(kpt::renderMultiView(cloud, opts), std::invalid_argument);
}

TEST_CASE("render cancellation has one typed exception contract", "[render]") {
  auto cloud = std::make_shared<kpt::PointCloudIRGB>();
  kpt::RenderOpts opts;
  std::stop_source cancellation;
  cancellation.request_stop();

  REQUIRE_THROWS_AS(kpt::renderMultiView(cloud, opts, cancellation.get_token()),
                    kpt::OperationCancelled);
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

#ifndef _WIN32
  struct stat metadata{};
  REQUIRE(::stat(output.c_str(), &metadata) == 0);
  const mode_t process_umask = ::umask(0);
  static_cast<void>(::umask(process_umask));
  // Exclusive native creation follows ordinary 0666 & process umask.
  CHECK((metadata.st_mode & 0777) == (0666 & ~process_umask));
#endif

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

TEST_CASE("atomic image writing observes cancellation before publication",
          "[render]") {
  RenderTempDirectory temp;
  const auto output = temp.path / "cancelled.png";
  const auto image = solidImage(2, 2, 1, 2, 3);
  std::stop_source cancellation;
  cancellation.request_stop();

  REQUIRE_THROWS_WITH(
      kpt::writeImageAtomic(output, image, true, cancellation.get_token()),
      Catch::Contains("operation cancelled"));
  REQUIRE_FALSE(fs::exists(output));
}

#if defined(__linux__)
TEST_CASE("native publication remains bound to opened file identity",
          "[render][platform]") {
  RenderTempDirectory temp;
  const auto temporary = temp.path / "reserved.tmp";
  const auto output = temp.path / "published.bin";
  auto opened = kpt::platform::openNativeOutputExclusively(temporary);
  REQUIRE(opened);
  auto file = std::move(opened).value();
  REQUIRE(file);
  constexpr std::array<std::uint8_t, 4> original{'k', 'p', 't', '\n'};
  REQUIRE(file->write(original));

  fs::remove(temporary);
  std::ofstream(temporary, std::ios::binary) << "attacker";
  auto published = file->publish(output, true);
  REQUIRE(published);
  REQUIRE(published.value().published);

  const auto output_data = readFile(output);
  const std::string output_bytes(output_data.begin(), output_data.end());
  REQUIRE(output_bytes == "kpt\n");
  const auto temporary_data = readFile(temporary);
  const std::string temporary_bytes(temporary_data.begin(),
                                    temporary_data.end());
  REQUIRE(temporary_bytes == "attacker");
}

TEST_CASE("anonymous native output never owns its candidate pathname",
          "[render][platform]") {
  RenderTempDirectory temp;
  const auto candidate = temp.path / "candidate.tmp";
  std::ofstream(candidate, std::ios::binary) << "existing";
  auto opened = kpt::platform::openNativeOutputExclusively(candidate);
  REQUIRE(opened);
  auto file = std::move(opened).value();
  REQUIRE(file);
  file.reset();

  const auto candidate_data = readFile(candidate);
  const std::string bytes(candidate_data.begin(), candidate_data.end());
  REQUIRE(bytes == "existing");
}
#endif

TEST_CASE("finished native output rejects later writes", "[render][platform]") {
  RenderTempDirectory temp;
  const auto candidate = temp.path / "finished.tmp";
  auto opened = kpt::platform::openNativeOutputExclusively(candidate);
  REQUIRE(opened);
  auto file = std::move(opened).value();
  REQUIRE(file);
  constexpr std::array<std::uint8_t, 1> byte{'x'};
  REQUIRE(file->write(byte));
  REQUIRE(file->finish());
  REQUIRE_FALSE(file->write(byte));
}
