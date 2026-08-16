#include "gui/viewport/capture.hpp"

#include "kpt/cancellation.hpp"
#include "kpt/render/render.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>

namespace kpt::gui {
namespace {

[[nodiscard]] std::size_t requiredRgbaBytes(const Rgba8Image &image) {
  if (image.extent.width <= 0 || image.extent.height <= 0) {
    throw std::invalid_argument("RGBA capture dimensions must be positive");
  }
  const std::size_t width = static_cast<std::size_t>(image.extent.width);
  const std::size_t height = static_cast<std::size_t>(image.extent.height);
  if (width > (std::numeric_limits<std::size_t>::max)() / std::size_t{4}) {
    throw std::length_error("RGBA capture row size overflows");
  }
  const std::size_t packed_row = width * std::size_t{4};
  if (image.bytes_per_row < packed_row) {
    throw std::invalid_argument("RGBA capture row stride is too small");
  }
  const std::size_t rows_before_last = height - std::size_t{1};
  if (rows_before_last >
      ((std::numeric_limits<std::size_t>::max)() - packed_row) /
          image.bytes_per_row) {
    throw std::length_error("RGBA capture size overflows");
  }
  return rows_before_last * image.bytes_per_row + packed_row;
}

} // namespace

ImageRGB8 rgba8ToRgbImage(const Rgba8Image &image, std::stop_token stop) {
  const std::size_t required = requiredRgbaBytes(image);
  if (image.pixels.size() < required) {
    throw std::invalid_argument("RGBA capture pixel buffer is too small");
  }
  if (stop.stop_requested()) {
    throw OperationCancelled();
  }

  ImageRGB8 rgb{image.extent.width, image.extent.height};
  const std::size_t width = static_cast<std::size_t>(image.extent.width);
  const std::size_t height = static_cast<std::size_t>(image.extent.height);
  constexpr std::size_t rgba_channels = 4;
  constexpr std::size_t rgb_channels = 3;
  for (std::size_t row = 0; row < height; ++row) {
    if (stop.stop_requested()) {
      throw OperationCancelled();
    }
    const std::uint8_t *source =
        image.pixels.data() + row * image.bytes_per_row;
    std::uint8_t *destination =
        rgb.pixels().data() + row * width * rgb_channels;
    for (std::size_t column = 0; column < width; ++column) {
      const std::uint8_t *pixel = source + column * rgba_channels;
      std::uint8_t *output = destination + column * rgb_channels;
      output[0] = pixel[0];
      output[1] = pixel[1];
      output[2] = pixel[2];
    }
  }
  return rgb;
}

ViewportCaptureResult writeViewportCapturePng(const std::filesystem::path &output,
                                              const Rgba8Image &image,
                                              bool overwrite,
                                              std::stop_token stop) {
  try {
    const ImageRGB8 rgb = rgba8ToRgbImage(image, stop);
    const ImageWriteStatus status = writeImageAtomic(output, rgb, overwrite, stop);
    return status == ImageWriteStatus::Written
               ? ViewportCaptureResult{ViewportCaptureStatus::Written, {}}
               : ViewportCaptureResult{ViewportCaptureStatus::Skipped,
                                       "destination already exists"};
  } catch (const OperationCancelled &) {
    return {ViewportCaptureStatus::Cancelled, "operation cancelled"};
  } catch (const std::exception &error) {
    return {ViewportCaptureStatus::Failed, error.what()};
  }
}

} // namespace kpt::gui
