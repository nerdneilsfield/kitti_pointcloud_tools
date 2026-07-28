#pragma once

#include <cstddef>
#include <cstdint>
#include <limits>
#include <span>
#include <stdexcept>
#include <vector>

namespace kpt {

struct ImageView {
  int width = 0;
  int height = 0;
  int stride_bytes = 0;
  std::span<const std::uint8_t> pixels;
};

class ImageRGB8 {
public:
  ImageRGB8() = default;

  ImageRGB8(int width, int height) : width_(width), height_(height) {
    if (width <= 0 || height <= 0)
      throw std::invalid_argument("image dimensions must be positive");
    if (width > std::numeric_limits<int>::max() / 3)
      throw std::length_error("image row is too large");
    stride_bytes_ = width * 3;
    const auto stride = static_cast<std::size_t>(stride_bytes_);
    const auto rows = static_cast<std::size_t>(height);
    if (rows > std::numeric_limits<std::size_t>::max() / stride)
      throw std::length_error("image is too large");
    pixels_.resize(rows * stride);
  }

  [[nodiscard]] int width() const noexcept { return width_; }
  [[nodiscard]] int height() const noexcept { return height_; }
  [[nodiscard]] int strideBytes() const noexcept { return stride_bytes_; }

  [[nodiscard]] std::span<std::uint8_t> pixels() noexcept { return pixels_; }
  [[nodiscard]] std::span<const std::uint8_t> pixels() const noexcept {
    return pixels_;
  }

  [[nodiscard]] std::uint8_t *pixel(int x, int y) {
    return pixels_.data() + pixelOffset(x, y);
  }
  [[nodiscard]] const std::uint8_t *pixel(int x, int y) const {
    return pixels_.data() + pixelOffset(x, y);
  }

  [[nodiscard]] ImageView view() const noexcept {
    return {width_, height_, stride_bytes_, pixels_};
  }

private:
  [[nodiscard]] std::size_t pixelOffset(int x, int y) const {
    if (x < 0 || x >= width_ || y < 0 || y >= height_)
      throw std::out_of_range("image pixel is out of range");
    return static_cast<std::size_t>(y) *
               static_cast<std::size_t>(stride_bytes_) +
           static_cast<std::size_t>(x) * 3U;
  }

  int width_ = 0;
  int height_ = 0;
  int stride_bytes_ = 0;
  std::vector<std::uint8_t> pixels_;
};

} // namespace kpt
