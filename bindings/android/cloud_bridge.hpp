#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <stop_token>
#include <string>
#include <string_view>
#include <vector>

namespace kpt::android {

inline constexpr std::size_t kMaximumInputBytes = 64U * 1024U * 1024U;
inline constexpr std::size_t kMaximumPoints = 2'000'000U;

enum class DecodeErrorCode : int {
  None = 0,
  Cancelled = 1,
  UnsupportedFormat = 2,
  InvalidData = 3,
  ResourceLimit = 4,
  Io = 5,
  OutOfMemory = 6,
  Internal = 7,
};

struct PackedVertex {
  float x = 0.0F;
  float y = 0.0F;
  float z = 0.0F;
  float intensity = 0.0F;
  std::uint8_t r = 0U;
  std::uint8_t g = 0U;
  std::uint8_t b = 0U;
  std::uint8_t flags = 0U;
};

static_assert(sizeof(PackedVertex) == 20U);
static_assert(offsetof(PackedVertex, intensity) == 12U);
static_assert(offsetof(PackedVertex, r) == 16U);

struct CloudMetadata {
  std::array<float, 6> bounds{};
  float intensity_min = 0.0F;
  float intensity_max = 1.0F;
  float intensity_p05 = 0.0F;
  float intensity_p90 = 1.0F;
  std::size_t source_point_count = 0U;
  bool has_color = false;
  bool has_intensity = false;
  bool has_noise = false;
};

struct DecodeResult {
  DecodeErrorCode error_code = DecodeErrorCode::None;
  std::string error_message;
  std::vector<PackedVertex> vertices;
  CloudMetadata metadata;

  [[nodiscard]] bool ok() const noexcept {
    return error_code == DecodeErrorCode::None;
  }
};

[[nodiscard]] DecodeResult decodePointCloud(std::span<const std::byte> bytes,
                                            std::string_view source_name,
                                            std::stop_token stop = {});

} // namespace kpt::android
