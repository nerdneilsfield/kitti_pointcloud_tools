#include "platform/native_file.hpp"

#define WIN32_LEAN_AND_MEAN
#include <windows.h>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <limits>
#include <memory>
#include <span>
#include <utility>
#include <vector>

namespace kpt::platform {
namespace {

PlatformError replaceError(const char *message) {
  return {PlatformErrorCode::NativeFileIoFailed,
          message,
          {static_cast<int>(GetLastError()), std::system_category()}};
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
    return NativeFileCommit{false, std::nullopt};
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

    std::error_code absolute_error;
    const auto absolute =
        std::filesystem::absolute(destination, absolute_error);
    if (absolute_error) {
      return PlatformError{PlatformErrorCode::NativeFileIoFailed,
                           "cannot resolve output path", absolute_error};
    }
    const auto &name = absolute.native();
    const auto name_bytes = name.size() * sizeof(wchar_t);
    const auto buffer_size = offsetof(FILE_RENAME_INFO, FileName) + name_bytes;
    std::vector<std::byte> buffer(buffer_size);
    auto *rename = reinterpret_cast<FILE_RENAME_INFO *>(buffer.data());
    rename->ReplaceIfExists = overwrite ? TRUE : FALSE;
    rename->RootDirectory = nullptr;
    rename->FileNameLength = static_cast<DWORD>(name_bytes);
    std::copy(name.begin(), name.end(), rename->FileName);
    if (SetFileInformationByHandle(file_, FileRenameInfo, rename,
                                   static_cast<DWORD>(buffer.size())) ==
        FALSE) {
      const DWORD error = GetLastError();
      if (!overwrite &&
          (error == ERROR_FILE_EXISTS || error == ERROR_ALREADY_EXISTS)) {
        return NativeFileCommit{false, std::nullopt};
      }
      SetLastError(error);
      return replaceError("cannot atomically publish output handle");
    }
    return NativeFileCommit{};
  }

private:
  HANDLE file_ = INVALID_HANDLE_VALUE;
  bool finished_ = false;
};

} // namespace

PlatformResult<std::unique_ptr<NativeOutputFile>>
openNativeOutputExclusively(const std::filesystem::path &path) {
  const HANDLE file =
      CreateFileW(path.c_str(), GENERIC_WRITE | DELETE, 0, nullptr, CREATE_NEW,
                  FILE_ATTRIBUTE_NORMAL, nullptr);
  if (file == INVALID_HANDLE_VALUE) {
    const DWORD error = GetLastError();
    if (error == ERROR_FILE_EXISTS || error == ERROR_ALREADY_EXISTS)
      return std::unique_ptr<NativeOutputFile>{};
    SetLastError(error);
    return replaceError("cannot reserve temporary output");
  }
  try {
    return std::unique_ptr<NativeOutputFile>(
        std::make_unique<WindowsOutputFile>(file));
  } catch (...) {
    static_cast<void>(CloseHandle(file));
    static_cast<void>(DeleteFileW(path.c_str()));
    throw;
  }
}

} // namespace kpt::platform
