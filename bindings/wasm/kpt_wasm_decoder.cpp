#include "kpt/io/io.hpp"
#include "kpt/label/label.hpp"

#include <emscripten/emscripten.h>

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <exception>
#include <fstream>
#include <limits>
#include <memory>
#include <new>
#include <stdexcept>
#include <span>
#include <string>
#include <tuple>
#include <vector>

namespace {

struct DecodeResult {
  std::vector<float> positions;
  std::vector<std::uint8_t> colors;
  std::vector<float> intensities;
  std::vector<std::uint8_t> noises;
  std::array<float, 6> bounds{};
  bool bounds_valid = false;
  bool has_color = false;
  bool has_intensity = false;
  bool has_noise = false;
  const char *fatal_error = nullptr;
  std::string error;
};

struct ConvertResult {
  std::vector<std::byte> bytes;
  const char *fatal_error = nullptr;
  std::string error;
};

void populateBuffers(const kpt::PointCloudIRGB &cloud, DecodeResult &result) {
  result.positions.resize(cloud.size() * 3U);
  if (result.has_color)
    result.colors.resize(cloud.size() * 3U);
  result.intensities.resize(cloud.size());
  if (result.has_noise)
    result.noises.resize(cloud.size());

  constexpr float infinity = std::numeric_limits<float>::infinity();
  std::array<float, 3> minimum{infinity, infinity, infinity};
  std::array<float, 3> maximum{-infinity, -infinity, -infinity};
  bool has_finite_point = false;

  for (std::size_t index = 0; index < cloud.size(); ++index) {
    const auto &point = cloud.points[index];
    const auto offset = index * 3U;
    result.positions[offset] = point.x;
    result.positions[offset + 1U] = point.y;
    result.positions[offset + 2U] = point.z;
    if (result.has_color) {
      result.colors[offset] = point.r;
      result.colors[offset + 1U] = point.g;
      result.colors[offset + 2U] = point.b;
    }
    result.intensities[index] = point.intensity;
    if (result.has_noise)
      result.noises[index] = point.noise;

    if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
        !std::isfinite(point.z)) {
      continue;
    }
    has_finite_point = true;
    minimum[0] = std::min(minimum[0], point.x);
    minimum[1] = std::min(minimum[1], point.y);
    minimum[2] = std::min(minimum[2], point.z);
    maximum[0] = std::max(maximum[0], point.x);
    maximum[1] = std::max(maximum[1], point.y);
    maximum[2] = std::max(maximum[2], point.z);
  }

  if (has_finite_point) {
    result.bounds = {minimum[0], minimum[1], minimum[2],
                     maximum[0], maximum[1], maximum[2]};
    result.bounds_valid = true;
  }
}

void applyLabels(kpt::PointCloudIRGB &cloud,
                 std::span<const std::byte> label_bytes) {
  if (label_bytes.size() % sizeof(std::uint32_t) != 0U ||
      label_bytes.size() / sizeof(std::uint32_t) != cloud.size()) {
    throw std::invalid_argument("cloud/label count mismatch");
  }
  const auto label_map = kpt::rangeNetLabelMap();
  const auto rgb_map = kpt::rgbLabelMap();
  for (std::size_t index = 0; index < cloud.size(); ++index) {
    std::uint32_t raw = 0U;
    std::memcpy(&raw, label_bytes.data() + index * sizeof(raw), sizeof(raw));
    if constexpr (std::endian::native == std::endian::big) {
      raw = ((raw & 0x000000ffU) << 24U) |
            ((raw & 0x0000ff00U) << 8U) |
            ((raw & 0x00ff0000U) >> 8U) |
            ((raw & 0xff000000U) >> 24U);
    }
    const auto semantic = static_cast<int>(raw & 0xffffU);
    const auto label = label_map.find(semantic);
    const auto compact = label == label_map.end() ? -1 : label->second;
    auto &point = cloud.points[index];
    point.intensity = static_cast<float>(compact);
    const auto color = rgb_map.find(compact);
    if (color == rgb_map.end()) {
      point.r = point.g = point.b = 0U;
    } else {
      point.r = static_cast<std::uint8_t>(std::get<0>(color->second));
      point.g = static_cast<std::uint8_t>(std::get<1>(color->second));
      point.b = static_cast<std::uint8_t>(std::get<2>(color->second));
    }
  }
}

std::uint32_t littleU32(const std::uint8_t *bytes) {
  return static_cast<std::uint32_t>(bytes[0]) |
         (static_cast<std::uint32_t>(bytes[1]) << 8U) |
         (static_cast<std::uint32_t>(bytes[2]) << 16U) |
         (static_cast<std::uint32_t>(bytes[3]) << 24U);
}

void decodeBinToBuffers(const std::uint8_t *data, std::size_t size,
                        DecodeResult &result) {
  if (size % 16U != 0U)
    throw std::invalid_argument("parse error: bin size not multiple of 16");
  const auto count = size / 16U;
  if (count > 20000000U)
    throw std::invalid_argument("parse error: bin point count exceeds limit");
  result.positions.resize(count * 3U);
  result.intensities.resize(count);
  constexpr float infinity = std::numeric_limits<float>::infinity();
  std::array<float, 3> minimum{infinity, infinity, infinity};
  std::array<float, 3> maximum{-infinity, -infinity, -infinity};
  for (std::size_t index = 0; index < count; ++index) {
    const auto *record = data + index * 16U;
    std::array<float, 4> values{};
    for (std::size_t component = 0; component < values.size(); ++component) {
      values[component] =
          std::bit_cast<float>(littleU32(record + component * 4U));
    }
    const auto offset = index * 3U;
    result.positions[offset] = values[0];
    result.positions[offset + 1U] = values[1];
    result.positions[offset + 2U] = values[2];
    result.intensities[index] = values[3];
    if (std::isfinite(values[0]) && std::isfinite(values[1]) &&
        std::isfinite(values[2])) {
      for (std::size_t axis = 0; axis < 3U; ++axis) {
        minimum[axis] = std::min(minimum[axis], values[axis]);
        maximum[axis] = std::max(maximum[axis], values[axis]);
      }
      result.bounds_valid = true;
    }
  }
  if (result.bounds_valid) {
    result.bounds = {minimum[0], minimum[1], minimum[2],
                     maximum[0], maximum[1], maximum[2]};
  }
  result.has_intensity = true;
}

void applyLabelsToBuffers(DecodeResult &result,
                          std::span<const std::byte> label_bytes) {
  if (label_bytes.size() % sizeof(std::uint32_t) != 0U ||
      label_bytes.size() / sizeof(std::uint32_t) !=
          result.intensities.size()) {
    throw std::invalid_argument("cloud/label count mismatch");
  }
  const auto label_map = kpt::rangeNetLabelMap();
  const auto rgb_map = kpt::rgbLabelMap();
  result.colors.assign(result.intensities.size() * 3U, 0U);
  for (std::size_t index = 0; index < result.intensities.size(); ++index) {
    const auto *raw = reinterpret_cast<const std::uint8_t *>(
        label_bytes.data() + index * sizeof(std::uint32_t));
    const auto semantic = static_cast<int>(littleU32(raw) & 0xffffU);
    const auto label = label_map.find(semantic);
    const auto compact = label == label_map.end() ? -1 : label->second;
    result.intensities[index] = static_cast<float>(compact);
    const auto color = rgb_map.find(compact);
    const auto offset = index * 3U;
    if (color != rgb_map.end()) {
      result.colors[offset] =
          static_cast<std::uint8_t>(std::get<0>(color->second));
      result.colors[offset + 1U] =
          static_cast<std::uint8_t>(std::get<1>(color->second));
      result.colors[offset + 2U] =
          static_cast<std::uint8_t>(std::get<2>(color->second));
    }
  }
  result.has_color = true;
}

std::unique_ptr<DecodeResult>
decodeMemory(const std::uint8_t *data, std::size_t size, const char *name,
             const std::uint8_t *labels, std::size_t label_size) {
  auto result = std::unique_ptr<DecodeResult>(new (std::nothrow) DecodeResult());
  if (!result)
    return {};
  try {
    if (data == nullptr || name == nullptr || *name == '\0') {
      result->error = "decode input is empty";
    } else {
      const std::string source(name);
      if (kpt::detect(source) == kpt::Format::Bin) {
        decodeBinToBuffers(data, size, *result);
        if (labels != nullptr) {
          applyLabelsToBuffers(
              *result,
              {reinterpret_cast<const std::byte *>(labels), label_size});
        }
        return result;
      }
      auto decoded = kpt::decode(
          {reinterpret_cast<const std::byte *>(data), size}, name);
      result->has_color = decoded.schema.has_color;
      result->has_intensity = decoded.schema.has_intensity;
      result->has_noise = decoded.schema.has_noise;
      if (labels != nullptr) {
        applyLabels(*decoded.cloud,
                    {reinterpret_cast<const std::byte *>(labels), label_size});
        result->has_color = true;
        result->has_intensity = true;
      }
      populateBuffers(*decoded.cloud, *result);
    }
  } catch (const std::bad_alloc &) {
    result->fatal_error = "decoder out of memory";
  } catch (const std::exception &error) {
    try {
      result->error = error.what();
    } catch (...) {
      result->fatal_error = "decoder error unavailable";
    }
  } catch (...) {
    result->fatal_error = "unknown decoder failure";
  }
  return result;
}

std::vector<std::uint8_t> readFileBytes(const char *path) {
  if (path == nullptr || *path == '\0')
    throw std::invalid_argument("decode path is empty");
  std::ifstream input(path, std::ios::binary | std::ios::ate);
  if (!input)
    throw std::runtime_error("file not found");
  const auto end = input.tellg();
  if (end < 0)
    throw std::runtime_error("cannot determine input size");
  std::vector<std::uint8_t> bytes(static_cast<std::size_t>(end));
  input.seekg(0);
  input.read(reinterpret_cast<char *>(bytes.data()),
             static_cast<std::streamsize>(bytes.size()));
  if (!input)
    throw std::runtime_error("truncated input");
  return bytes;
}

const DecodeResult *fromHandle(std::uintptr_t handle) {
  return reinterpret_cast<const DecodeResult *>(handle);
}

const ConvertResult *convertFromHandle(std::uintptr_t handle) {
  return reinterpret_cast<const ConvertResult *>(handle);
}

} // namespace

extern "C" {

EMSCRIPTEN_KEEPALIVE std::uint32_t kpt_decoder_abi_version() noexcept {
  return 4U;
}

EMSCRIPTEN_KEEPALIVE std::uintptr_t kpt_alloc(std::size_t size) noexcept {
  return reinterpret_cast<std::uintptr_t>(
      new (std::nothrow) std::uint8_t[size]);
}

EMSCRIPTEN_KEEPALIVE void kpt_free(std::uintptr_t pointer) noexcept {
  delete[] reinterpret_cast<std::uint8_t *>(pointer);
}

EMSCRIPTEN_KEEPALIVE std::uintptr_t
kpt_decode_memory(const std::uint8_t *data, std::size_t size,
                  const char *name) noexcept {
  return reinterpret_cast<std::uintptr_t>(
      decodeMemory(data, size, name, nullptr, 0U).release());
}

EMSCRIPTEN_KEEPALIVE std::uintptr_t
kpt_decode_labeled_memory(const std::uint8_t *data, std::size_t size,
                          const char *name, const std::uint8_t *labels,
                          std::size_t label_size) noexcept {
  return reinterpret_cast<std::uintptr_t>(
      decodeMemory(data, size, name, labels, label_size).release());
}

EMSCRIPTEN_KEEPALIVE std::uintptr_t kpt_decode_file(const char *path) noexcept {
  try {
    const auto bytes = readFileBytes(path);
    return reinterpret_cast<std::uintptr_t>(
        decodeMemory(bytes.data(), bytes.size(), path, nullptr, 0U).release());
  } catch (...) {
    auto result = std::unique_ptr<DecodeResult>(
        new (std::nothrow) DecodeResult());
    if (!result)
      return 0U;
    result->fatal_error = "cannot read decoder input";
    return reinterpret_cast<std::uintptr_t>(result.release());
  }
}

EMSCRIPTEN_KEEPALIVE std::uintptr_t
kpt_decode_labeled_file(const char *path, const char *label_path) noexcept {
  try {
    const auto bytes = readFileBytes(path);
    const auto labels = readFileBytes(label_path);
    return reinterpret_cast<std::uintptr_t>(
        decodeMemory(bytes.data(), bytes.size(), path, labels.data(),
                     labels.size()).release());
  } catch (...) {
    auto result = std::unique_ptr<DecodeResult>(
        new (std::nothrow) DecodeResult());
    if (!result)
      return 0U;
    result->fatal_error = "cannot read decoder input";
    return reinterpret_cast<std::uintptr_t>(result.release());
  }
}

EMSCRIPTEN_KEEPALIVE void
kpt_decode_result_free(std::uintptr_t handle) noexcept {
  delete fromHandle(handle);
}

EMSCRIPTEN_KEEPALIVE std::size_t
kpt_decode_result_point_count(std::uintptr_t handle) noexcept {
  const auto *result = fromHandle(handle);
  return result == nullptr ? 0U : result->intensities.size();
}

EMSCRIPTEN_KEEPALIVE std::uintptr_t
kpt_decode_result_positions(std::uintptr_t handle) noexcept {
  const auto *result = fromHandle(handle);
  return result == nullptr
             ? 0U
             : reinterpret_cast<std::uintptr_t>(result->positions.data());
}

EMSCRIPTEN_KEEPALIVE std::uintptr_t
kpt_decode_result_colors(std::uintptr_t handle) noexcept {
  const auto *result = fromHandle(handle);
  return result == nullptr
             ? 0U
             : reinterpret_cast<std::uintptr_t>(result->colors.data());
}

EMSCRIPTEN_KEEPALIVE std::uintptr_t
kpt_decode_result_intensities(std::uintptr_t handle) noexcept {
  const auto *result = fromHandle(handle);
  return result == nullptr
             ? 0U
             : reinterpret_cast<std::uintptr_t>(result->intensities.data());
}

EMSCRIPTEN_KEEPALIVE std::uintptr_t
kpt_decode_result_noises(std::uintptr_t handle) noexcept {
  const auto *result = fromHandle(handle);
  return result == nullptr
             ? 0U
             : reinterpret_cast<std::uintptr_t>(result->noises.data());
}

EMSCRIPTEN_KEEPALIVE std::uintptr_t
kpt_decode_result_bounds(std::uintptr_t handle) noexcept {
  const auto *result = fromHandle(handle);
  return result == nullptr
             ? 0U
             : reinterpret_cast<std::uintptr_t>(result->bounds.data());
}

EMSCRIPTEN_KEEPALIVE int
kpt_decode_result_bounds_valid(std::uintptr_t handle) noexcept {
  const auto *result = fromHandle(handle);
  return result != nullptr && result->bounds_valid ? 1 : 0;
}

EMSCRIPTEN_KEEPALIVE int
kpt_decode_result_has_color(std::uintptr_t handle) noexcept {
  const auto *result = fromHandle(handle);
  return result != nullptr && result->has_color ? 1 : 0;
}

EMSCRIPTEN_KEEPALIVE int
kpt_decode_result_has_intensity(std::uintptr_t handle) noexcept {
  const auto *result = fromHandle(handle);
  return result != nullptr && result->has_intensity ? 1 : 0;
}

EMSCRIPTEN_KEEPALIVE int
kpt_decode_result_has_noise(std::uintptr_t handle) noexcept {
  const auto *result = fromHandle(handle);
  return result != nullptr && result->has_noise ? 1 : 0;
}

EMSCRIPTEN_KEEPALIVE const char *
kpt_decode_result_error(std::uintptr_t handle) noexcept {
  const auto *result = fromHandle(handle);
  if (result == nullptr) {
    return "decoder result allocation failed";
  }
  return result->fatal_error == nullptr ? result->error.c_str()
                                        : result->fatal_error;
}

EMSCRIPTEN_KEEPALIVE std::uintptr_t
kpt_convert_memory(const std::uint8_t *data, std::size_t size,
                   const char *source_name, const char *target_name) noexcept {
  auto result =
      std::unique_ptr<ConvertResult>(new (std::nothrow) ConvertResult());
  if (!result)
    return 0U;
  try {
    if (data == nullptr || source_name == nullptr || *source_name == '\0' ||
        target_name == nullptr || *target_name == '\0') {
      result->error = "conversion input is empty";
    } else {
      const auto decoded = kpt::decode(
          {reinterpret_cast<const std::byte *>(data), size}, source_name);
      result->bytes = kpt::encode(*decoded.cloud, target_name);
    }
  } catch (const std::bad_alloc &) {
    result->fatal_error = "converter out of memory";
  } catch (const std::exception &error) {
    try {
      result->error = error.what();
    } catch (...) {
      result->fatal_error = "converter error unavailable";
    }
  } catch (...) {
    result->fatal_error = "unknown converter failure";
  }
  return reinterpret_cast<std::uintptr_t>(result.release());
}

EMSCRIPTEN_KEEPALIVE void
kpt_convert_result_free(std::uintptr_t handle) noexcept {
  delete convertFromHandle(handle);
}

EMSCRIPTEN_KEEPALIVE std::size_t
kpt_convert_result_size(std::uintptr_t handle) noexcept {
  const auto *result = convertFromHandle(handle);
  return result == nullptr ? 0U : result->bytes.size();
}

EMSCRIPTEN_KEEPALIVE std::uintptr_t
kpt_convert_result_data(std::uintptr_t handle) noexcept {
  const auto *result = convertFromHandle(handle);
  return result == nullptr
             ? 0U
             : reinterpret_cast<std::uintptr_t>(result->bytes.data());
}

EMSCRIPTEN_KEEPALIVE const char *
kpt_convert_result_error(std::uintptr_t handle) noexcept {
  const auto *result = convertFromHandle(handle);
  if (result == nullptr)
    return "converter result allocation failed";
  return result->fatal_error == nullptr ? result->error.c_str()
                                        : result->fatal_error;
}

} // extern "C"
