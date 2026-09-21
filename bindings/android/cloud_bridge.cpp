#include "cloud_bridge.hpp"

#include "kpt/cancellation.hpp"
#include "kpt/io/format.hpp"
#include "kpt/io/io.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <new>
#include <stdexcept>

namespace kpt::android {
namespace {

bool supportedFormat(std::string_view source_name) {
  try {
    const auto format = detect(std::filesystem::path(std::string(source_name)));
    return format == Format::Bin || format == Format::PCD ||
           format == Format::PLY;
  } catch (...) {
    return false;
  }
}

bool resourceLimitMessage(std::string_view message) {
  return message.find("exceeds limit") != std::string_view::npos ||
         message.find("too large") != std::string_view::npos ||
         message.find("overflow") != std::string_view::npos;
}

DecodeResult error(DecodeErrorCode code, std::string message) {
  DecodeResult result;
  result.error_code = code;
  result.error_message = std::move(message);
  return result;
}

void finishIntensityMetadata(std::vector<float> &values,
                             CloudMetadata &metadata) {
  if (values.empty())
    return;
  const auto [minimum, maximum] =
      std::minmax_element(values.begin(), values.end());
  metadata.intensity_min = *minimum;
  metadata.intensity_max = *maximum;
  const auto p05_index = static_cast<std::size_t>(
      static_cast<double>(values.size() - 1U) * 0.05);
  const auto p90_index = static_cast<std::size_t>(
      static_cast<double>(values.size() - 1U) * 0.90);
  auto p05 = values.begin() + static_cast<std::ptrdiff_t>(p05_index);
  auto p90 = values.begin() + static_cast<std::ptrdiff_t>(p90_index);
  std::nth_element(values.begin(), p05, values.end());
  metadata.intensity_p05 = *p05;
  std::nth_element(p05, p90, values.end());
  metadata.intensity_p90 = *p90;
  if (!(metadata.intensity_p05 < metadata.intensity_p90)) {
    metadata.intensity_p05 = metadata.intensity_min;
    metadata.intensity_p90 = metadata.intensity_max;
  }
}

} // namespace

DecodeResult decodePointCloud(std::span<const std::byte> bytes,
                              std::string_view source_name,
                              std::stop_token stop) {
  try {
    if (bytes.empty())
      return error(DecodeErrorCode::InvalidData, "input is empty");
    if (bytes.size() > kMaximumInputBytes)
      return error(DecodeErrorCode::ResourceLimit,
                   "input exceeds Android 64 MiB limit");
    if (source_name.empty() || source_name.size() > 1024U ||
        source_name.find('\0') != std::string_view::npos) {
      return error(DecodeErrorCode::InvalidData, "source name is invalid");
    }
    if (!supportedFormat(source_name)) {
      return error(DecodeErrorCode::UnsupportedFormat,
                   "Android spike supports BIN, PCD, and PLY only");
    }
    if (stop.stop_requested())
      throw OperationCancelled();

    auto decoded = decode(bytes, source_name, stop);
    if (!decoded.cloud || decoded.cloud->empty())
      return error(DecodeErrorCode::InvalidData, "point cloud is empty");
    if (decoded.cloud->size() > kMaximumPoints)
      return error(DecodeErrorCode::ResourceLimit,
                   "point count exceeds Android 2,000,000 limit");

    DecodeResult result;
    result.metadata.source_point_count = decoded.cloud->size();
    result.metadata.has_color = decoded.schema.has_color;
    result.metadata.has_intensity = decoded.schema.has_intensity;
    result.metadata.has_noise = decoded.schema.has_noise;
    result.vertices.reserve(decoded.cloud->size());
    std::vector<float> intensities;
    if (decoded.schema.has_intensity)
      intensities.reserve(decoded.cloud->size());

    constexpr float infinity = std::numeric_limits<float>::infinity();
    std::array<float, 3> minimum{infinity, infinity, infinity};
    std::array<float, 3> maximum{-infinity, -infinity, -infinity};
    std::size_t visited = 0U;
    for (const auto &point : *decoded.cloud) {
      if ((visited++ % 4096U) == 0U && stop.stop_requested())
        throw OperationCancelled();
      if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
          !std::isfinite(point.z))
        continue;
      const float intensity =
          std::isfinite(point.intensity) ? point.intensity : 0.0F;
      result.vertices.push_back({point.x, point.y, point.z, intensity, point.r,
                                 point.g, point.b,
                                 static_cast<std::uint8_t>(
                                     decoded.cloud->has_noise && point.noise != 0
                                         ? 1U
                                         : 0U)});
      minimum[0] = std::min(minimum[0], point.x);
      minimum[1] = std::min(minimum[1], point.y);
      minimum[2] = std::min(minimum[2], point.z);
      maximum[0] = std::max(maximum[0], point.x);
      maximum[1] = std::max(maximum[1], point.y);
      maximum[2] = std::max(maximum[2], point.z);
      if (decoded.schema.has_intensity)
        intensities.push_back(intensity);
    }
    if (result.vertices.empty())
      return error(DecodeErrorCode::InvalidData,
                   "point cloud contains no finite positions");
    result.metadata.bounds = {minimum[0], minimum[1], minimum[2],
                              maximum[0], maximum[1], maximum[2]};
    finishIntensityMetadata(intensities, result.metadata);
    return result;
  } catch (const OperationCancelled &) {
    return error(DecodeErrorCode::Cancelled, "operation cancelled");
  } catch (const std::bad_alloc &) {
    return error(DecodeErrorCode::OutOfMemory, "decoder out of memory");
  } catch (const std::length_error &exception) {
    return error(DecodeErrorCode::ResourceLimit, exception.what());
  } catch (const std::exception &exception) {
    const std::string message = exception.what();
    return error(resourceLimitMessage(message) ? DecodeErrorCode::ResourceLimit
                                               : DecodeErrorCode::InvalidData,
                 message);
  } catch (...) {
    return error(DecodeErrorCode::Internal, "unknown native decoder failure");
  }
}

} // namespace kpt::android
