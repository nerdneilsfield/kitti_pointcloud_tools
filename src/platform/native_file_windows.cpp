#ifndef NOMINMAX
#define NOMINMAX
#endif
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif

#include "platform/native_file.hpp"

#include <windows.h>

#include <algorithm>
#include <cstdint>
#include <filesystem>
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

void discardOpenFile(HANDLE file) {
  FILE_DISPOSITION_INFO disposition{};
  disposition.DeleteFile = TRUE;
  static_cast<void>(SetFileInformationByHandle(
      file, FileDispositionInfo, &disposition, sizeof(disposition)));
}

} // namespace

PlatformResult<NativeFileCommit>
replaceFileAtomically(const std::filesystem::path &source,
                      const std::filesystem::path &destination) {
  if (ReplaceFileW(destination.c_str(), source.c_str(), nullptr, 0, nullptr,
                   nullptr) != FALSE) {
    return NativeFileCommit{};
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
  return NativeFileCommit{};
}

PlatformResult<NativeFileCommit>
moveFileAtomicallyIfAbsent(const std::filesystem::path &source,
                           const std::filesystem::path &destination) {
  if (MoveFileExW(source.c_str(), destination.c_str(),
                  MOVEFILE_WRITE_THROUGH) != FALSE) {
    return NativeFileCommit{};
  }
  const DWORD error = GetLastError();
  if (error == ERROR_FILE_EXISTS || error == ERROR_ALREADY_EXISTS)
    return NativeFileCommit{false, false, {}};
  SetLastError(error);
  return replaceError("cannot atomically publish file");
}

namespace {

class WindowsOutputFile final : public NativeOutputFile {
public:
  WindowsOutputFile(HANDLE file, std::filesystem::path path)
      : file_(file), path_(std::move(path)) {}
  ~WindowsOutputFile() override {
    if (file_ != INVALID_HANDLE_VALUE) {
      if (!published_)
        discardOpenFile(file_);
      static_cast<void>(CloseHandle(file_));
    } else if (!published_) {
      static_cast<void>(DeleteFileW(path_.c_str()));
    }
  }

  PlatformResult<void> write(std::span<const std::uint8_t> bytes) override {
    if (finished_) {
      SetLastError(ERROR_INVALID_HANDLE);
      return replaceError("temporary output is already finished");
    }
    std::size_t offset = 0;
    while (offset < bytes.size()) {
      const auto chunk = static_cast<DWORD>(std::min<std::size_t>(
          bytes.size() - offset, (std::numeric_limits<DWORD>::max)()));
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
    if (finished_)
      return {};
    if (FlushFileBuffers(file_) == FALSE)
      return replaceError("cannot flush temporary output");
    finished_ = true;
    return {};
  }

  PlatformResult<NativeFileCommit>
  publish(const std::filesystem::path &destination, bool overwrite) override {
    auto finished = finish();
    if (!finished)
      return finished.error();
    if (CloseHandle(file_) == FALSE)
      return replaceError("cannot close temporary output before publication");
    file_ = INVALID_HANDLE_VALUE;

    // Path-based Windows rename APIs preserve the no-replace guarantee under
    // concurrent writers. Closing first also makes the published file
    // immediately readable by callers using the default exclusive share mode.
    auto commit = overwrite ? replaceFileAtomically(path_, destination)
                            : moveFileAtomicallyIfAbsent(path_, destination);
    if (commit)
      published_ = commit.value().published;
    return commit;
  }

private:
  HANDLE file_ = INVALID_HANDLE_VALUE;
  std::filesystem::path path_;
  bool finished_ = false;
  bool published_ = false;
};

} // namespace

PlatformResult<std::unique_ptr<NativeOutputFile>>
openNativeOutputExclusively(const std::filesystem::path &path) {
  const HANDLE file =
      CreateFileW(path.c_str(), GENERIC_WRITE | DELETE, 0, nullptr, CREATE_NEW,
                  FILE_ATTRIBUTE_NORMAL | FILE_FLAG_WRITE_THROUGH, nullptr);
  if (file == INVALID_HANDLE_VALUE) {
    const DWORD error = GetLastError();
    if (error == ERROR_FILE_EXISTS || error == ERROR_ALREADY_EXISTS)
      return std::unique_ptr<NativeOutputFile>{};
    SetLastError(error);
    return replaceError("cannot reserve temporary output");
  }
  try {
    return std::unique_ptr<NativeOutputFile>(
        std::make_unique<WindowsOutputFile>(file, path));
  } catch (...) {
    discardOpenFile(file);
    static_cast<void>(CloseHandle(file));
    throw;
  }
}

} // namespace kpt::platform
