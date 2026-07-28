#include "platform/native_file.hpp"

#include <cerrno>
#include <cstdint>
#include <fcntl.h>
#include <memory>
#include <span>
#include <system_error>
#include <unistd.h>
#include <utility>

#if defined(__linux__)
#include <linux/fs.h>
#include <sys/syscall.h>
#elif defined(__APPLE__)
#include <stdio.h>
#endif

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
  // Rename is already visible and cannot be rolled back. Directory durability
  // is best-effort so callers never receive "not written" after commit.
  static_cast<void>(syncPath(parent, "cannot flush output directory", true));
  return {};
}

PlatformResult<bool>
moveFileAtomicallyIfAbsent(const std::filesystem::path &source,
                           const std::filesystem::path &destination) {
  auto synced = syncPath(source, "cannot flush temporary output");
  if (!synced)
    return synced.error();

  bool moved = false;
#if defined(__linux__)
  if (::syscall(SYS_renameat2, AT_FDCWD, source.c_str(), AT_FDCWD,
                destination.c_str(), RENAME_NOREPLACE) == 0) {
    moved = true;
  } else if (errno == EEXIST) {
    return false;
  } else if (errno != ENOSYS && errno != EINVAL && errno != EOPNOTSUPP) {
    return ioError("cannot atomically publish file", errno);
  }
#elif defined(__APPLE__)
  if (::renamex_np(source.c_str(), destination.c_str(), RENAME_EXCL) == 0) {
    moved = true;
  } else if (errno == EEXIST) {
    return false;
  } else if (errno != ENOTSUP && errno != EINVAL) {
    return ioError("cannot atomically publish file", errno);
  }
#endif

  // Portable fallback for older kernels/filesystems. Same-directory temporary
  // files avoid EXDEV; platforms with exclusive rename use it above, including
  // Linux/macOS filesystems that do not support hard links.
  if (!moved && ::link(source.c_str(), destination.c_str()) != 0) {
    if (errno == EEXIST)
      return false;
    return ioError("cannot atomically publish file", errno);
  }
  if (!moved) {
    // Destination is committed once link succeeds. Never roll it back by path:
    // another process could replace that entry first. A rare failed temporary
    // unlink leaves only an extra hard link for later cleanup.
    static_cast<void>(::unlink(source.c_str()));
  }

  auto parent = destination.parent_path();
  if (parent.empty())
    parent = ".";
  // Publication cannot be rolled back, so directory durability is
  // best-effort rather than a misleading "not written" result after commit.
  static_cast<void>(syncPath(parent, "cannot flush output directory", true));
  return true;
}

namespace {

class PosixOutputFile final : public NativeOutputFile {
public:
  explicit PosixOutputFile(int descriptor) : descriptor_(descriptor) {}
  ~PosixOutputFile() override {
    if (descriptor_ >= 0)
      static_cast<void>(::close(descriptor_));
  }

  PlatformResult<void> write(std::span<const std::uint8_t> bytes) override {
    std::size_t offset = 0;
    while (offset < bytes.size()) {
      const auto written =
          ::write(descriptor_, bytes.data() + offset, bytes.size() - offset);
      if (written < 0) {
        if (errno == EINTR)
          continue;
        return ioError("cannot write temporary output", errno);
      }
      if (written == 0)
        return ioError("cannot write temporary output", EIO);
      offset += static_cast<std::size_t>(written);
    }
    return {};
  }

  PlatformResult<void> finish() override {
    if (descriptor_ < 0)
      return ioError("temporary output is already closed", EBADF);
    if (::fsync(descriptor_) != 0)
      return ioError("cannot flush temporary output", errno);
    const int descriptor = std::exchange(descriptor_, -1);
    if (::close(descriptor) != 0)
      return ioError("cannot close temporary output", errno);
    return {};
  }

private:
  int descriptor_ = -1;
};

} // namespace

PlatformResult<std::unique_ptr<NativeOutputFile>>
openNativeOutputExclusively(const std::filesystem::path &path) {
  const int descriptor =
      ::open(path.c_str(), O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC, 0666);
  if (descriptor < 0) {
    if (errno == EEXIST)
      return std::unique_ptr<NativeOutputFile>{};
    return ioError("cannot reserve temporary output", errno);
  }
  return std::unique_ptr<NativeOutputFile>(
      std::make_unique<PosixOutputFile>(descriptor));
}

} // namespace kpt::platform
