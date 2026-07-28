#include "platform/native_file.hpp"

#include <utility>

namespace kpt::platform {

PlatformResult<void>
replaceFileAtomically(const std::filesystem::path &source,
                      const std::filesystem::path &destination) {
  std::error_code error;
  std::filesystem::rename(source, destination, error);
  if (error) {
    return PlatformError{PlatformErrorCode::NativeFileIoFailed,
                         "cannot atomically replace file", error};
  }
  return {};
}

} // namespace kpt::platform
