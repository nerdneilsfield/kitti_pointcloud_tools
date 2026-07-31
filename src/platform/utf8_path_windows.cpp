#ifndef NOMINMAX
#define NOMINMAX
#endif
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif

#include "platform/utf8_path.hpp"

#include <windows.h>

#include <limits>
#include <string>
#include <utility>

namespace kpt::platform {
namespace {

PlatformError conversionError(std::string message,
                              std::error_code system_error = {}) {
  return {PlatformErrorCode::InvalidUtf8, std::move(message), system_error};
}

std::error_code lastWindowsError() {
  return {static_cast<int>(GetLastError()), std::system_category()};
}

bool fitsWindowsLength(std::size_t size) {
  return size <= static_cast<std::size_t>((std::numeric_limits<int>::max)());
}

} // namespace

PlatformResult<std::filesystem::path> pathFromUtf8(std::string_view value) {
  if (value.empty())
    return std::filesystem::path{};
  if (!fitsWindowsLength(value.size()))
    return conversionError("UTF-8 path exceeds the Windows API length limit");

  const int input_size = static_cast<int>(value.size());
  const int required = MultiByteToWideChar(
      CP_UTF8, MB_ERR_INVALID_CHARS, value.data(), input_size, nullptr, 0);
  if (required == 0)
    return conversionError("UI path is not valid UTF-8", lastWindowsError());

  std::wstring native(static_cast<std::size_t>(required), L'\0');
  if (MultiByteToWideChar(CP_UTF8, MB_ERR_INVALID_CHARS, value.data(),
                          input_size, native.data(), required) == 0) {
    return conversionError("cannot convert UTF-8 path to UTF-16",
                           lastWindowsError());
  }
  return std::filesystem::path(std::move(native));
}

PlatformResult<std::string> pathToUtf8(const std::filesystem::path &value) {
  const auto &native = value.native();
  if (native.empty())
    return std::string{};
  if (!fitsWindowsLength(native.size()))
    return conversionError("native path exceeds the Windows API length limit");

  const int input_size = static_cast<int>(native.size());
  const int required =
      WideCharToMultiByte(CP_UTF8, WC_ERR_INVALID_CHARS, native.data(),
                          input_size, nullptr, 0, nullptr, nullptr);
  if (required == 0)
    return conversionError("native path is not valid UTF-16",
                           lastWindowsError());

  std::string utf8(static_cast<std::size_t>(required), '\0');
  if (WideCharToMultiByte(CP_UTF8, WC_ERR_INVALID_CHARS, native.data(),
                          input_size, utf8.data(), required, nullptr,
                          nullptr) == 0) {
    return conversionError("cannot convert native path to UTF-8",
                           lastWindowsError());
  }
  return utf8;
}

} // namespace kpt::platform
