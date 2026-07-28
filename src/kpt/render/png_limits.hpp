#pragma once

#include <cstdint>
#include <limits>

namespace kpt {

inline constexpr std::uint64_t kMaxPngPixels =
    std::uint64_t{32} * std::uint64_t{1024} * std::uint64_t{1024};

[[nodiscard]] constexpr bool pngDimensionsSupported(int width,
                                                    int height) noexcept {
  if (width <= 0 || height <= 0 ||
      width > std::numeric_limits<int>::max() / 3) {
    return false;
  }
  const auto pixels =
      static_cast<std::uint64_t>(width) * static_cast<std::uint64_t>(height);
  const auto filtered_row =
      static_cast<std::uint64_t>(width) * std::uint64_t{3} + std::uint64_t{1};
  const auto data_length = filtered_row * static_cast<std::uint64_t>(height);
  const auto blocks =
      (data_length + std::uint64_t{32766}) / std::uint64_t{32767};
  const auto png_length = data_length + std::uint64_t{2} +
                          blocks * std::uint64_t{5} + std::uint64_t{57};
  return pixels <= kMaxPngPixels &&
         png_length <=
             static_cast<std::uint64_t>(std::numeric_limits<int>::max());
}

} // namespace kpt
