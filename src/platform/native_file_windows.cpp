#include "platform/native_file.hpp"

#define WIN32_LEAN_AND_MEAN
#include <windows.h>

namespace kpt::platform {
namespace {

PlatformError replaceError(const char *message) {
  return {PlatformErrorCode::NativeFileIoFailed,
          message,
          {static_cast<int>(GetLastError()), std::system_category()}};
}

} // namespace

PlatformResult<void>
replaceFileAtomically(const std::filesystem::path &source,
                      const std::filesystem::path &destination) {
  if (ReplaceFileW(destination.c_str(), source.c_str(), nullptr, 0, nullptr,
                   nullptr) != FALSE) {
    return {};
  }
  const DWORD error = GetLastError();
  if (error != ERROR_FILE_NOT_FOUND && error != ERROR_PATH_NOT_FOUND) {
    SetLastError(error);
    return replaceError("cannot replace file");
  }
  if (MoveFileExW(source.c_str(), destination.c_str(),
                  MOVEFILE_REPLACE_EXISTING | MOVEFILE_WRITE_THROUGH) ==
      FALSE) {
    return replaceError("cannot move file");
  }
  return {};
}

PlatformResult<bool>
moveFileAtomicallyIfAbsent(const std::filesystem::path &source,
                           const std::filesystem::path &destination) {
  if (MoveFileExW(source.c_str(), destination.c_str(),
                  MOVEFILE_WRITE_THROUGH) != FALSE) {
    return true;
  }
  const DWORD error = GetLastError();
  if (error == ERROR_FILE_EXISTS || error == ERROR_ALREADY_EXISTS)
    return false;
  SetLastError(error);
  return replaceError("cannot atomically publish file");
}

PlatformResult<bool> createFileExclusively(const std::filesystem::path &path) {
  const HANDLE file = CreateFileW(path.c_str(), GENERIC_WRITE, 0, nullptr,
                                  CREATE_NEW, FILE_ATTRIBUTE_NORMAL, nullptr);
  if (file == INVALID_HANDLE_VALUE) {
    const DWORD error = GetLastError();
    if (error == ERROR_FILE_EXISTS || error == ERROR_ALREADY_EXISTS)
      return false;
    SetLastError(error);
    return replaceError("cannot reserve temporary output");
  }
  if (CloseHandle(file) == FALSE)
    return replaceError("cannot close temporary output");
  return true;
}

} // namespace kpt::platform
