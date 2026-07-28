#include "platform/native_file.hpp"

#include <cerrno>
#include <fcntl.h>
#include <system_error>
#include <unistd.h>
#include <utility>

namespace kpt::platform {
namespace {

PlatformError ioError(const char *message, int error) {
  return {PlatformErrorCode::NativeFileIoFailed,
          message,
          {error, std::generic_category()}};
}

PlatformResult<void> syncPath(const std::filesystem::path &path,
                              const char *message,
                              bool allow_unsupported = false) {
  const int descriptor = ::open(path.c_str(), O_RDONLY);
  if (descriptor < 0)
    return ioError(message, errno);
  if (::fsync(descriptor) != 0) {
    const int error = errno;
    static_cast<void>(::close(descriptor));
    if (allow_unsupported && (error == EINVAL || error == ENOTSUP))
      return {};
    return ioError(message, error);
  }
  if (::close(descriptor) != 0)
    return ioError(message, errno);
  return {};
}

} // namespace

PlatformResult<void>
replaceFileAtomically(const std::filesystem::path &source,
                      const std::filesystem::path &destination) {
  auto synced = syncPath(source, "cannot flush temporary output");
  if (!synced)
    return synced.error();

  std::error_code error;
  std::filesystem::rename(source, destination, error);
  if (error) {
    return PlatformError{PlatformErrorCode::NativeFileIoFailed,
                         "cannot atomically replace file", error};
  }
  auto parent = destination.parent_path();
  if (parent.empty())
    parent = ".";
  synced = syncPath(parent, "cannot flush output directory", true);
  if (!synced)
    return synced.error();
  return {};
}

PlatformResult<bool>
moveFileAtomicallyIfAbsent(const std::filesystem::path &source,
                           const std::filesystem::path &destination) {
  auto synced = syncPath(source, "cannot flush temporary output");
  if (!synced)
    return synced.error();

  if (::link(source.c_str(), destination.c_str()) != 0) {
    if (errno == EEXIST)
      return false;
    return ioError("cannot atomically publish file", errno);
  }
  if (::unlink(source.c_str()) != 0) {
    const int error = errno;
    static_cast<void>(::unlink(destination.c_str()));
    return ioError("cannot remove temporary output after publish", error);
  }

  auto parent = destination.parent_path();
  if (parent.empty())
    parent = ".";
  synced = syncPath(parent, "cannot flush output directory", true);
  if (!synced)
    return synced.error();
  return true;
}

PlatformResult<bool> createFileExclusively(const std::filesystem::path &path) {
  const int descriptor =
      ::open(path.c_str(), O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC, 0600);
  if (descriptor < 0) {
    if (errno == EEXIST)
      return false;
    return ioError("cannot reserve temporary output", errno);
  }
  if (::close(descriptor) != 0)
    return ioError("cannot close temporary output", errno);
  return true;
}

} // namespace kpt::platform
