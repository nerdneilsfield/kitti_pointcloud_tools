#include <catch2/catch.hpp>

#include "platform/utf8_path.hpp"

#include <array>
#include <filesystem>
#include <initializer_list>
#include <string>
#include <string_view>

namespace {

namespace fs = std::filesystem;

std::string bytes(std::initializer_list<unsigned char> values) {
  std::string result;
  result.reserve(values.size());
  for (const auto value : values)
    result.push_back(static_cast<char>(value));
  return result;
}

} // namespace

TEST_CASE("UTF-8 paths preserve Chinese names", "[utf8]") {
  constexpr std::string_view input = "路径/点云/帧 0001.pcd";
  const auto native = kpt::platform::pathFromUtf8(input);
  REQUIRE(native);

  const auto round_trip = kpt::platform::pathToUtf8(native.value());
  REQUIRE(round_trip);
  REQUIRE(round_trip.value() == input);
}

TEST_CASE("UTF-8 path conversion rejects malformed byte sequences", "[utf8]") {
  const std::array invalid = {
      bytes({0x80U}),                       // stray continuation
      bytes({0xE2U, 0x82U}),                // truncated
      bytes({0xC0U, 0xAFU}),                // overlong two-byte
      bytes({0xE0U, 0x80U, 0xAFU}),         // overlong three-byte
      bytes({0xF0U, 0x80U, 0x80U, 0xAFU}),  // overlong four-byte
      bytes({0xEDU, 0xA0U, 0x80U}),         // UTF-16 surrogate
      bytes({0xF4U, 0x90U, 0x80U, 0x80U})}; // above U+10FFFF

  for (const auto &value : invalid) {
    INFO("invalid byte count: " << value.size());
    const auto from_utf8 = kpt::platform::pathFromUtf8(value);
    REQUIRE_FALSE(from_utf8);
    REQUIRE(from_utf8.error().code ==
            kpt::platform::PlatformErrorCode::InvalidUtf8);

#if !defined(_WIN32)
    const auto to_utf8 = kpt::platform::pathToUtf8(fs::path(value));
    REQUIRE_FALSE(to_utf8);
    REQUIRE(to_utf8.error().code ==
            kpt::platform::PlatformErrorCode::InvalidUtf8);
#endif
  }
}

#if defined(_WIN32)
TEST_CASE("UTF-8 path conversion rejects unpaired UTF-16 surrogates",
          "[utf8][windows]") {
  const std::wstring malformed(1, static_cast<wchar_t>(0xD800));
  const auto converted = kpt::platform::pathToUtf8(fs::path(malformed));
  REQUIRE_FALSE(converted);
  REQUIRE(converted.error().code ==
          kpt::platform::PlatformErrorCode::InvalidUtf8);
}
#endif
