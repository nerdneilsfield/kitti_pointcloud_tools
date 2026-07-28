#pragma once

#include "common/result.hpp"

#include <string>
#include <system_error>

namespace kpt::platform {

enum class PlatformErrorCode {
  ConfigurationDirectoryUnavailable,
  EnvironmentDecodeFailed,
  FontFileUnavailable,
  InvalidUtf8,
  NativeFileIoFailed,
  PlatformInitializationFailed,
  SettingsIoFailed
};

struct PlatformError {
  PlatformErrorCode code;
  std::string message;
  std::error_code system_error;
};

template <class T> using PlatformResult = Result<T, PlatformError>;

} // namespace kpt::platform
