#include "platform/detail/atomic_replace.hpp"

#define WIN32_LEAN_AND_MEAN
#include <windows.h>

#include <memory>

namespace kpt::platform::detail {
namespace {

PlatformError replaceError() {
  return {PlatformErrorCode::SettingsIoFailed,
          "cannot atomically replace settings file",
          {static_cast<int>(GetLastError()), std::system_category()}};
}

class WindowsAtomicReplace final : public AtomicReplace {
public:
  PlatformResult<void>
  replace(const std::filesystem::path &source,
          const std::filesystem::path &destination) override {
    if (ReplaceFileW(destination.c_str(), source.c_str(), nullptr,
                     REPLACEFILE_WRITE_THROUGH, nullptr, nullptr) != FALSE) {
      return {};
    }

    const DWORD replace_error = GetLastError();
    if (replace_error != ERROR_FILE_NOT_FOUND &&
        replace_error != ERROR_PATH_NOT_FOUND) {
      SetLastError(replace_error);
      return replaceError();
    }

    if (MoveFileExW(source.c_str(), destination.c_str(),
                    MOVEFILE_REPLACE_EXISTING | MOVEFILE_WRITE_THROUGH) ==
        FALSE) {
      return replaceError();
    }
    return {};
  }
};

} // namespace

std::unique_ptr<AtomicReplace> createAtomicReplace() {
  return std::make_unique<WindowsAtomicReplace>();
}

} // namespace kpt::platform::detail
