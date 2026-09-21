#include <catch2/catch.hpp>

#include "cloud_bridge.hpp"

#include <bit>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <span>
#include <stop_token>
#include <string>
#include <vector>

namespace {

std::span<const std::byte> bytesOf(const std::string &value) {
  return std::as_bytes(std::span(value.data(), value.size()));
}

void appendLittleFloat(std::vector<std::byte> &bytes, float value) {
  auto bits = std::bit_cast<std::uint32_t>(value);
  if constexpr (std::endian::native == std::endian::big) {
    bits = ((bits & 0x000000FFU) << 24U) |
           ((bits & 0x0000FF00U) << 8U) |
           ((bits & 0x00FF0000U) >> 8U) |
           ((bits & 0xFF000000U) >> 24U);
  }
  const auto offset = bytes.size();
  bytes.resize(offset + sizeof(bits));
  std::memcpy(bytes.data() + offset, &bits, sizeof(bits));
}

} // namespace

TEST_CASE("Android bridge packs KITTI BIN into its stable vertex ABI",
          "[android_bridge]") {
  std::vector<std::byte> bytes;
  appendLittleFloat(bytes, 1.0F);
  appendLittleFloat(bytes, -2.0F);
  appendLittleFloat(bytes, 3.5F);
  appendLittleFloat(bytes, 0.75F);

  const auto result = kpt::android::decodePointCloud(bytes, "frame.bin");

  REQUIRE(result.ok());
  REQUIRE(result.vertices.size() == 1U);
  CHECK(result.vertices.front().x == 1.0F);
  CHECK(result.vertices.front().y == -2.0F);
  CHECK(result.vertices.front().z == 3.5F);
  CHECK(result.vertices.front().intensity == 0.75F);
  CHECK(result.metadata.has_intensity);
  CHECK_FALSE(result.metadata.has_color);
  CHECK(result.metadata.bounds[1] == -2.0F);
}

TEST_CASE("Android bridge decodes representative PCD and PLY inputs",
          "[android_bridge]") {
  SECTION("PCD ASCII intensity") {
    const std::string pcd =
        "# .PCD v0.7\nVERSION 0.7\nFIELDS x y z intensity\n"
        "SIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\n"
        "WIDTH 2\nHEIGHT 1\nPOINTS 2\nDATA ascii\n"
        "0 1 2 4\n3 4 5 8\n";
    const auto result =
        kpt::android::decodePointCloud(bytesOf(pcd), "cloud.pcd");
    REQUIRE(result.ok());
    CHECK(result.vertices.size() == 2U);
    CHECK(result.metadata.has_intensity);
    CHECK(result.metadata.intensity_min == 4.0F);
    CHECK(result.metadata.intensity_max == 8.0F);
  }

  SECTION("PLY ASCII RGB") {
    const std::string ply =
        "ply\nformat ascii 1.0\nelement vertex 1\n"
        "property float x\nproperty float y\nproperty float z\n"
        "property uchar red\nproperty uchar green\nproperty uchar blue\n"
        "end_header\n1 2 3 10 20 30\n";
    const auto result =
        kpt::android::decodePointCloud(bytesOf(ply), "cloud.ply");
    REQUIRE(result.ok());
    REQUIRE(result.vertices.size() == 1U);
    CHECK(result.metadata.has_color);
    CHECK(result.vertices.front().r == 10U);
    CHECK(result.vertices.front().g == 20U);
    CHECK(result.vertices.front().b == 30U);
  }
}

TEST_CASE("Android bridge classifies bad input and cancellation",
          "[android_bridge]") {
  SECTION("unsupported extension") {
    const std::string value = "point data";
    const auto result =
        kpt::android::decodePointCloud(bytesOf(value), "cloud.xyz");
    CHECK(result.error_code ==
          kpt::android::DecodeErrorCode::UnsupportedFormat);
  }

  SECTION("truncated BIN") {
    const std::string value = "bad";
    const auto result =
        kpt::android::decodePointCloud(bytesOf(value), "cloud.bin");
    CHECK(result.error_code == kpt::android::DecodeErrorCode::InvalidData);
  }

  SECTION("pre-cancelled request") {
    std::stop_source source;
    source.request_stop();
    const std::string value = "bad";
    const auto result = kpt::android::decodePointCloud(
        bytesOf(value), "cloud.bin", source.get_token());
    CHECK(result.error_code == kpt::android::DecodeErrorCode::Cancelled);
  }
}
