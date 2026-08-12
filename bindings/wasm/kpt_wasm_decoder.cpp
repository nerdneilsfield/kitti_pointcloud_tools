#include "kpt/io/io.hpp"
#include "kpt/label/label.hpp"

#include <emscripten/emscripten.h>
#include <emscripten/heap.h>

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <exception>
#include <limits>
#include <memory>
#include <mutex>
#include <new>
#include <span>
#include <stdexcept>
#include <string>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace {

constexpr std::size_t kMaximumInputBytes = 512U * 1024U * 1024U;
constexpr std::size_t kMaximumLabelBytes = 256U * 1024U * 1024U;
constexpr std::size_t kMaximumWorkingSetBytes = 768U * 1024U * 1024U;
constexpr std::size_t kMaximumNameBytes = 1024U;
constexpr std::size_t kMaximumPoints = 20'000'000U;

std::mutex allocation_mutex;
std::unordered_map<const std::uint8_t *, std::size_t> allocations;

bool registeredAllocation(const std::uint8_t *pointer,
                          std::size_t size) noexcept {
  try {
    std::lock_guard lock(allocation_mutex);
    const auto allocation = allocations.find(pointer);
    return allocation != allocations.end() && size <= allocation->second;
  } catch (...) {
    return false;
  }
}

std::size_t checkedMultiply(std::size_t left, std::size_t right,
                            const char *description) {
  if (right != 0U && left > std::numeric_limits<std::size_t>::max() / right)
    throw std::length_error(description);
  return left * right;
}

std::size_t checkedAdd(std::size_t left, std::size_t right,
                       const char *description) {
  if (left > std::numeric_limits<std::size_t>::max() - right)
    throw std::length_error(description);
  return left + right;
}

std::size_t decodedOutputBytes(std::size_t point_count, bool has_color,
                               bool has_noise) {
  auto bytes = checkedMultiply(point_count, 3U * sizeof(float),
                               "decoded output size overflow");
  if (has_color) {
    bytes = checkedAdd(bytes,
                       checkedMultiply(point_count, 3U * sizeof(std::uint8_t),
                                       "decoded output size overflow"),
                       "decoded output size overflow");
  }
  bytes = checkedAdd(bytes,
                     checkedMultiply(point_count, sizeof(float),
                                     "decoded output size overflow"),
                     "decoded output size overflow");
  if (has_noise)
    bytes = checkedAdd(bytes, point_count, "decoded output size overflow");
  return checkedAdd(bytes, sizeof(std::array<float, 6>),
                    "decoded output size overflow");
}

void enforceWorkingSet(std::size_t input_bytes, std::size_t label_bytes,
                       std::size_t cloud_bytes, std::size_t output_bytes) {
  auto total =
      checkedAdd(input_bytes, label_bytes, "decoder working set overflow");
  total = checkedAdd(total, cloud_bytes, "decoder working set overflow");
  total = checkedAdd(total, output_bytes, "decoder working set overflow");
  if (total > kMaximumWorkingSetBytes)
    throw std::length_error("decoder working set exceeds memory limit");
}

bool wasmHeapRange(const void *pointer, std::size_t size) noexcept {
  if (pointer == nullptr)
    return size == 0U;
  const auto address = reinterpret_cast<std::uintptr_t>(pointer);
  const auto heap_size =
      static_cast<std::uintptr_t>(emscripten_get_heap_size());
  return address <= heap_size && size <= heap_size - address;
}

std::string boundedName(const char *value, std::size_t size,
                        const char *description) {
  if (value == nullptr || size == 0U)
    throw std::invalid_argument(std::string(description) + " is empty");
  if (size > kMaximumNameBytes || !wasmHeapRange(value, size))
    throw std::invalid_argument(std::string(description) + " is invalid");
  std::string result(value, size);
  if (result.find('\0') != std::string::npos)
    throw std::invalid_argument(std::string(description) + " contains NUL");
  return result;
}

void validateInputRange(const std::uint8_t *data, std::size_t size,
                        const char *description) {
  if (data == nullptr || size == 0U)
    throw std::invalid_argument(std::string(description) + " is empty");
  if (!wasmHeapRange(data, size))
    throw std::invalid_argument(std::string(description) +
                                " is outside wasm heap");
  if (!registeredAllocation(data, size))
    throw std::invalid_argument(std::string(description) +
                                " is not an active decoder allocation");
}

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
  if (cloud.size() > kMaximumPoints)
    throw std::length_error("decoded point count exceeds limit");
  const auto coordinate_count =
      checkedMultiply(cloud.size(), 3U, "decoded coordinate count overflow");
  result.positions.resize(coordinate_count);
  if (result.has_color)
    result.colors.resize(coordinate_count);
  result.intensities.resize(cloud.size());
  if (result.has_noise)
    result.noises.resize(cloud.size());

  constexpr float infinity = std::numeric_limits<float>::infinity();
  std::array<float, 3> minimum{infinity, infinity, infinity};
  std::array<float, 3> maximum{-infinity, -infinity, -infinity};
  bool has_finite_point = false;

  for (std::size_t index = 0; index < cloud.size(); ++index) {
    const auto &point = cloud.points[index];
    const auto offset = checkedMultiply(index, 3U, "decoded offset overflow");
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
      raw = ((raw & 0x000000ffU) << 24U) | ((raw & 0x0000ff00U) << 8U) |
            ((raw & 0x00ff0000U) >> 8U) | ((raw & 0xff000000U) >> 24U);
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
                        std::size_t label_size, DecodeResult &result) {
  if (size % 16U != 0U)
    throw std::invalid_argument("parse error: bin size not multiple of 16");
  const auto count = size / 16U;
  if (count > kMaximumPoints)
    throw std::invalid_argument("parse error: bin point count exceeds limit");
  enforceWorkingSet(size, label_size, 0U,
                    decodedOutputBytes(count, label_size != 0U, false));
  const auto coordinate_count =
      checkedMultiply(count, 3U, "decoded coordinate count overflow");
  result.positions.resize(coordinate_count);
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
    if (!std::isfinite(values[0]) || !std::isfinite(values[1]) ||
        !std::isfinite(values[2]) || !std::isfinite(values[3]))
      throw std::invalid_argument("parse error: non-finite bin point");
    const auto offset = checkedMultiply(index, 3U, "decoded offset overflow");
    result.positions[offset] = values[0];
    result.positions[offset + 1U] = values[1];
    result.positions[offset + 2U] = values[2];
    result.intensities[index] = values[3];
    for (std::size_t axis = 0; axis < 3U; ++axis) {
      minimum[axis] = std::min(minimum[axis], values[axis]);
      maximum[axis] = std::max(maximum[axis], values[axis]);
    }
    result.bounds_valid = true;
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
      label_bytes.size() / sizeof(std::uint32_t) != result.intensities.size()) {
    throw std::invalid_argument("cloud/label count mismatch");
  }
  const auto label_map = kpt::rangeNetLabelMap();
  const auto rgb_map = kpt::rgbLabelMap();
  const auto coordinate_count = checkedMultiply(result.intensities.size(), 3U,
                                                "decoded color count overflow");
  result.colors.assign(coordinate_count, 0U);
  for (std::size_t index = 0; index < result.intensities.size(); ++index) {
    const auto *raw = reinterpret_cast<const std::uint8_t *>(
        label_bytes.data() + index * sizeof(std::uint32_t));
    const auto semantic = static_cast<int>(littleU32(raw) & 0xffffU);
    const auto label = label_map.find(semantic);
    const auto compact = label == label_map.end() ? -1 : label->second;
    result.intensities[index] = static_cast<float>(compact);
    const auto color = rgb_map.find(compact);
    const auto offset = checkedMultiply(index, 3U, "decoded offset overflow");
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

std::unique_ptr<DecodeResult> decodeMemory(const std::uint8_t *data,
                                           std::size_t size, const char *name,
                                           std::size_t name_size,
                                           const std::uint8_t *labels,
                                           std::size_t label_size) {
  auto result =
      std::unique_ptr<DecodeResult>(new (std::nothrow) DecodeResult());
  if (!result)
    return {};
  try {
    if (size > kMaximumInputBytes || label_size > kMaximumLabelBytes ||
        size > kMaximumWorkingSetBytes - label_size) {
      result->error = "decoder input exceeds memory limit";
    } else {
      validateInputRange(data, size, "decode input");
      if (labels != nullptr)
        validateInputRange(labels, label_size, "decode labels");
      else if (label_size != 0U)
        throw std::invalid_argument("decode labels pointer is missing");
      const auto source = boundedName(name, name_size, "decode source name");
      if (kpt::detect(source) == kpt::Format::Bin) {
        decodeBinToBuffers(data, size, label_size, *result);
        if (labels != nullptr) {
          applyLabelsToBuffers(
              *result,
              {reinterpret_cast<const std::byte *>(labels), label_size});
        }
        return result;
      }
      auto decoded = kpt::decode(
          {reinterpret_cast<const std::byte *>(data), size}, source);
      result->has_color = decoded.schema.has_color;
      result->has_intensity = decoded.schema.has_intensity;
      result->has_noise = decoded.schema.has_noise;
      if (labels != nullptr) {
        applyLabels(*decoded.cloud,
                    {reinterpret_cast<const std::byte *>(labels), label_size});
        result->has_color = true;
        result->has_intensity = true;
      }
      const auto cloud_bytes =
          checkedMultiply(decoded.cloud->points.capacity(), sizeof(kpt::PointT),
                          "decoded cloud size overflow");
      enforceWorkingSet(size, label_size, cloud_bytes,
                        decodedOutputBytes(decoded.cloud->size(),
                                           result->has_color,
                                           result->has_noise));
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

std::mutex handle_mutex;
std::unordered_set<const DecodeResult *> decode_handles;
std::unordered_set<const ConvertResult *> convert_handles;

std::uintptr_t registerHandle(DecodeResult *result) noexcept {
  if (result == nullptr)
    return 0U;
  try {
    std::lock_guard lock(handle_mutex);
    decode_handles.insert(result);
    return reinterpret_cast<std::uintptr_t>(result);
  } catch (...) {
    delete result;
    return 0U;
  }
}

std::uintptr_t registerHandle(ConvertResult *result) noexcept {
  if (result == nullptr)
    return 0U;
  try {
    std::lock_guard lock(handle_mutex);
    convert_handles.insert(result);
    return reinterpret_cast<std::uintptr_t>(result);
  } catch (...) {
    delete result;
    return 0U;
  }
}

const DecodeResult *fromHandle(std::uintptr_t handle) {
  const auto *result = reinterpret_cast<const DecodeResult *>(handle);
  std::lock_guard lock(handle_mutex);
  return decode_handles.contains(result) ? result : nullptr;
}

const ConvertResult *convertFromHandle(std::uintptr_t handle) {
  const auto *result = reinterpret_cast<const ConvertResult *>(handle);
  std::lock_guard lock(handle_mutex);
  return convert_handles.contains(result) ? result : nullptr;
}

void freeDecodeHandle(std::uintptr_t handle) {
  auto *result = reinterpret_cast<DecodeResult *>(handle);
  std::lock_guard lock(handle_mutex);
  if (decode_handles.erase(result) != 0U)
    delete result;
}

void freeConvertHandle(std::uintptr_t handle) {
  auto *result = reinterpret_cast<ConvertResult *>(handle);
  std::lock_guard lock(handle_mutex);
  if (convert_handles.erase(result) != 0U)
    delete result;
}

} // namespace

extern "C" {

EMSCRIPTEN_KEEPALIVE std::uint32_t kpt_decoder_abi_version() noexcept {
  return 5U;
}

EMSCRIPTEN_KEEPALIVE std::uintptr_t kpt_alloc(std::size_t size) noexcept {
  if (size == 0U || size > kMaximumInputBytes)
    return 0U;
  auto *allocation = new (std::nothrow) std::uint8_t[size];
  if (allocation == nullptr)
    return 0U;
  try {
    std::lock_guard lock(allocation_mutex);
    allocations.emplace(allocation, size);
  } catch (...) {
    delete[] allocation;
    return 0U;
  }
  return reinterpret_cast<std::uintptr_t>(allocation);
}

EMSCRIPTEN_KEEPALIVE void kpt_free(std::uintptr_t pointer) noexcept {
  auto *allocation = reinterpret_cast<std::uint8_t *>(pointer);
  if (allocation == nullptr)
    return;
  try {
    std::lock_guard lock(allocation_mutex);
    if (allocations.erase(allocation) != 0U)
      delete[] allocation;
  } catch (...) {
    // C ABI boundary is noexcept. A registry failure must not turn an invalid
    // free into memory corruption.
  }
}

EMSCRIPTEN_KEEPALIVE std::uintptr_t
kpt_decode_memory(const std::uint8_t *data, std::size_t size, const char *name,
                  std::size_t name_size) noexcept {
  return registerHandle(
      decodeMemory(data, size, name, name_size, nullptr, 0U).release());
}

EMSCRIPTEN_KEEPALIVE std::uintptr_t
kpt_decode_labeled_memory(const std::uint8_t *data, std::size_t size,
                          const char *name, std::size_t name_size,
                          const std::uint8_t *labels,
                          std::size_t label_size) noexcept {
  return registerHandle(
      decodeMemory(data, size, name, name_size, labels, label_size).release());
}

EMSCRIPTEN_KEEPALIVE void
kpt_decode_result_free(std::uintptr_t handle) noexcept {
  freeDecodeHandle(handle);
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
                   const char *source_name, std::size_t source_name_size,
                   const char *target_name,
                   std::size_t target_name_size) noexcept {
  auto result =
      std::unique_ptr<ConvertResult>(new (std::nothrow) ConvertResult());
  if (!result)
    return 0U;
  try {
    if (size > kMaximumInputBytes) {
      result->error = "conversion input exceeds memory limit";
    } else {
      validateInputRange(data, size, "conversion input");
      const auto source =
          boundedName(source_name, source_name_size, "conversion source name");
      const auto target =
          boundedName(target_name, target_name_size, "conversion target name");
      const auto decoded = kpt::decode(
          {reinterpret_cast<const std::byte *>(data), size}, source);
      result->bytes = kpt::encode(*decoded.cloud, target);
      if (result->bytes.size() > kMaximumInputBytes)
        throw std::length_error("conversion output exceeds memory limit");
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
  return registerHandle(result.release());
}

EMSCRIPTEN_KEEPALIVE void
kpt_convert_result_free(std::uintptr_t handle) noexcept {
  freeConvertHandle(handle);
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
