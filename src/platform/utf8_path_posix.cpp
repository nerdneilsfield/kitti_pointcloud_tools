#include "platform/utf8_path.hpp"

#include <cstddef>
#include <string>
#include <utility>

namespace kpt::platform {
namespace {

bool isContinuation(unsigned char value) {
  return value >= 0x80U && value <= 0xBFU;
}

bool isValidUtf8(std::string_view value) {
  const auto *bytes = reinterpret_cast<const unsigned char *>(value.data());
  std::size_t index = 0;
  while (index < value.size()) {
    const auto first = bytes[index];
    if (first <= 0x7FU) {
      ++index;
      continue;
    }

    if (first >= 0xC2U && first <= 0xDFU) {
      if (index + 1 >= value.size() || !isContinuation(bytes[index + 1]))
        return false;
      index += 2;
      continue;
    }

    if (first >= 0xE0U && first <= 0xEFU) {
      if (index + 2 >= value.size() || !isContinuation(bytes[index + 2]))
        return false;
      const auto second = bytes[index + 1];
      if ((first == 0xE0U && (second < 0xA0U || second > 0xBFU)) ||
          (first == 0xEDU && (second < 0x80U || second > 0x9FU)) ||
          (first != 0xE0U && first != 0xEDU && !isContinuation(second))) {
        return false;
      }
      index += 3;
      continue;
    }

    if (first >= 0xF0U && first <= 0xF4U) {
      if (index + 3 >= value.size() || !isContinuation(bytes[index + 2]) ||
          !isContinuation(bytes[index + 3])) {
        return false;
      }
      const auto second = bytes[index + 1];
      if ((first == 0xF0U && (second < 0x90U || second > 0xBFU)) ||
          (first == 0xF4U && (second < 0x80U || second > 0x8FU)) ||
          (first != 0xF0U && first != 0xF4U && !isContinuation(second))) {
        return false;
      }
      index += 4;
      continue;
    }

    return false;
  }
  return true;
}

PlatformError invalidUtf8(std::string message) {
  return {PlatformErrorCode::InvalidUtf8, std::move(message), {}};
}

} // namespace

PlatformResult<std::filesystem::path> pathFromUtf8(std::string_view value) {
  if (!isValidUtf8(value))
    return invalidUtf8("UI path is not valid UTF-8");
  return std::filesystem::path(std::string(value));
}

PlatformResult<std::string> pathToUtf8(const std::filesystem::path &value) {
  const auto &native = value.native();
  if (!isValidUtf8(native))
    return invalidUtf8("Native path cannot be represented as UTF-8");
  return native;
}

PlatformResult<std::string>
pathToUtf8ForNarrowApi(const std::filesystem::path &value) {
  return pathToUtf8(value);
}

} // namespace kpt::platform
