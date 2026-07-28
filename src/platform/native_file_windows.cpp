#include "platform/native_file.hpp"

#define WIN32_LEAN_AND_MEAN
#include <windows.h>

#include <algorithm>
#include <cstdint>
#include <limits>
#include <memory>
#include <span>
#include <utility>

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

namespace {

class WindowsOutputFile final : public NativeOutputFile {
public:
  explicit WindowsOutputFile(HANDLE file) : file_(file) {}
  ~WindowsOutputFile() override {
    if (file_ != INVALID_HANDLE_VALUE)
      static_cast<void>(CloseHandle(file_));
  }

  PlatformResult<void> write(std::span<const std::uint8_t> bytes) override {
    std::size_t offset = 0;
    while (offset < bytes.size()) {
      const auto chunk = static_cast<DWORD>(std::min<std::size_t>(
          bytes.size() - offset, std::numeric_limits<DWORD>::max()));
      DWORD written = 0;
      if (WriteFile(file_, bytes.data() + offset, chunk, &written, nullptr) ==
          FALSE) {
        return replaceError("cannot write temporary output");
      }
      if (written == 0) {
        SetLastError(ERROR_WRITE_FAULT);
        return replaceError("cannot write temporary output");
      }
      offset += written;
    }
    return {};
  }

  PlatformResult<void> finish() override {
    if (file_ == INVALID_HANDLE_VALUE) {
      SetLastError(ERROR_INVALID_HANDLE);
      return replaceError("temporary output is already closed");
    }
    if (FlushFileBuffers(file_) == FALSE)
      return replaceError("cannot flush temporary output");
    const HANDLE file = std::exchange(file_, INVALID_HANDLE_VALUE);
    if (CloseHandle(file) == FALSE)
      return replaceError("cannot close temporary output");
    return {};
  }

private:
  HANDLE file_ = INVALID_HANDLE_VALUE;
};

} // namespace

PlatformResult<std::unique_ptr<NativeOutputFile>>
openNativeOutputExclusively(const std::filesystem::path &path) {
  const HANDLE file = CreateFileW(path.c_str(), GENERIC_WRITE, 0, nullptr,
                                  CREATE_NEW, FILE_ATTRIBUTE_NORMAL, nullptr);
  if (file == INVALID_HANDLE_VALUE) {
    const DWORD error = GetLastError();
    if (error == ERROR_FILE_EXISTS || error == ERROR_ALREADY_EXISTS)
      return std::unique_ptr<NativeOutputFile>{};
    SetLastError(error);
    return replaceError("cannot reserve temporary output");
  }
  return std::unique_ptr<NativeOutputFile>(
      std::make_unique<WindowsOutputFile>(file));
}

} // namespace kpt::platform
