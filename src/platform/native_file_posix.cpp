#include "platform/native_file.hpp"

#include <atomic>
#include <cerrno>
#include <cstdint>
#include <fcntl.h>
#include <memory>
#include <span>
#include <string>
#include <sys/stat.h>
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

PlatformResult<NativeFileCommit>
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
  NativeFileCommit commit;
  auto directory_sync = syncPath(parent, "cannot flush output directory", true);
  if (!directory_sync)
    commit.durability_warning = std::move(directory_sync).error();
  return commit;
}

PlatformResult<NativeFileCommit>
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
    return NativeFileCommit{false, std::nullopt};
  } else if (errno != ENOSYS && errno != EINVAL && errno != EOPNOTSUPP) {
    return ioError("cannot atomically publish file", errno);
  }
#elif defined(__APPLE__)
  if (::renamex_np(source.c_str(), destination.c_str(), RENAME_EXCL) == 0) {
    moved = true;
  } else if (errno == EEXIST) {
    return NativeFileCommit{false, std::nullopt};
  } else if (errno != ENOTSUP && errno != EINVAL) {
    return ioError("cannot atomically publish file", errno);
  }
#endif

  // Portable fallback for older kernels/filesystems. Same-directory temporary
  // files avoid EXDEV; platforms with exclusive rename use it above, including
  // Linux/macOS filesystems that do not support hard links.
  if (!moved && ::link(source.c_str(), destination.c_str()) != 0) {
    if (errno == EEXIST)
      return NativeFileCommit{false, std::nullopt};
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
  NativeFileCommit commit;
  auto directory_sync = syncPath(parent, "cannot flush output directory", true);
  if (!directory_sync)
    commit.durability_warning = std::move(directory_sync).error();
  return commit;
}

namespace {

bool sameFile(const struct stat &left, const struct stat &right) {
  return left.st_dev == right.st_dev && left.st_ino == right.st_ino;
}

PlatformResult<void> verifyOpenPath(int descriptor,
                                    const std::filesystem::path &path) {
  struct stat opened{};
  struct stat named{};
  if (::fstat(descriptor, &opened) != 0)
    return ioError("cannot inspect temporary output handle", errno);
  if (::lstat(path.c_str(), &named) != 0)
    return ioError("temporary output path was replaced", errno);
  if (!S_ISREG(named.st_mode) || !sameFile(opened, named))
    return ioError("temporary output path was replaced", ESTALE);
  return {};
}

std::optional<PlatformError> unlinkIfOwned(int descriptor,
                                           const std::filesystem::path &path) {
  struct stat opened{};
  struct stat named{};
  if (::fstat(descriptor, &opened) == 0 && ::lstat(path.c_str(), &named) == 0 &&
      sameFile(opened, named)) {
    if (::unlink(path.c_str()) != 0)
      return ioError("cannot remove committed temporary output", errno);
  }
  return std::nullopt;
}

#if defined(__linux__)
int linkOpenFile(int descriptor, bool anonymous,
                 const std::filesystem::path &destination) {
  if (anonymous)
    return ::linkat(descriptor, "", AT_FDCWD, destination.c_str(),
                    AT_EMPTY_PATH);
  const auto descriptor_path =
      std::string("/proc/self/fd/") + std::to_string(descriptor);
  return ::linkat(AT_FDCWD, descriptor_path.c_str(), AT_FDCWD,
                  destination.c_str(), AT_SYMLINK_FOLLOW);
}
#endif

class PosixOutputFile final : public NativeOutputFile {
public:
  PosixOutputFile(int descriptor, std::filesystem::path path, bool anonymous)
      : descriptor_(descriptor), path_(std::move(path)), anonymous_(anonymous) {
  }
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
    if (finished_)
      return {};
    if (::fsync(descriptor_) != 0)
      return ioError("cannot flush temporary output", errno);
    finished_ = true;
    return {};
  }

  PlatformResult<NativeFileCommit>
  publish(const std::filesystem::path &destination, bool overwrite) override {
    auto finished = finish();
    if (!finished)
      return finished.error();

    bool published = false;
#if defined(__linux__)
    if (!overwrite) {
      if (linkOpenFile(descriptor_, anonymous_, destination) == 0) {
        published = true;
      } else if (errno == EEXIST) {
        return NativeFileCommit{false, std::nullopt};
      } else if (errno != EPERM && errno != EOPNOTSUPP && errno != ENOTSUP &&
                 errno != ENOENT && errno != EXDEV) {
        return ioError("cannot atomically publish output handle", errno);
      }
    } else {
      static std::atomic<std::uint64_t> counter{0};
      auto parent = destination.parent_path();
      if (parent.empty())
        parent = ".";
      for (int attempt = 0; attempt < 64 && !published; ++attempt) {
        auto name = destination.filename();
        name += ".kpt-publish-" + std::to_string(::getpid()) + "-" +
                std::to_string(counter.fetch_add(1));
        const auto staging = parent / name;
        if (linkOpenFile(descriptor_, anonymous_, staging) == 0) {
          if (::rename(staging.c_str(), destination.c_str()) != 0) {
            const int error = errno;
            static_cast<void>(::unlink(staging.c_str()));
            return ioError("cannot atomically replace output", error);
          }
          published = true;
        } else if (errno != EEXIST && errno != EPERM && errno != EOPNOTSUPP &&
                   errno != ENOTSUP && errno != ENOENT && errno != EXDEV) {
          return ioError("cannot stage output handle", errno);
        } else if (errno != EEXIST) {
          break;
        }
      }
      if (!published && errno == EEXIST)
        return ioError("cannot reserve output publication name", EEXIST);
    }
#endif

    if (!published) {
      if (anonymous_)
        return ioError("cannot publish anonymous output handle", errno);
      // macOS and filesystems without hard-link support lack a portable
      // descriptor-based rename. Refuse a visibly replaced source before the
      // atomic rename; callers must use a directory not writable by attackers.
      auto verified = verifyOpenPath(descriptor_, path_);
      if (!verified)
        return verified.error();
      if (overwrite) {
        if (::rename(path_.c_str(), destination.c_str()) != 0)
          return ioError("cannot atomically replace output", errno);
      } else {
#if defined(__APPLE__)
        if (::renamex_np(path_.c_str(), destination.c_str(), RENAME_EXCL) !=
            0) {
          if (errno == EEXIST)
            return NativeFileCommit{false, std::nullopt};
          return ioError("cannot atomically publish output", errno);
        }
#else
        if (::link(path_.c_str(), destination.c_str()) != 0) {
          if (errno == EEXIST)
            return NativeFileCommit{false, std::nullopt};
          return ioError("cannot atomically publish output", errno);
        }
#endif
      }
      published = true;
    }

    auto cleanup_warning = anonymous_ ? std::optional<PlatformError>{}
                                      : unlinkIfOwned(descriptor_, path_);
    auto parent = destination.parent_path();
    if (parent.empty())
      parent = ".";
    NativeFileCommit commit;
    commit.durability_warning = std::move(cleanup_warning);
    auto directory_sync =
        syncPath(parent, "cannot flush output directory", true);
    if (!directory_sync && !commit.durability_warning)
      commit.durability_warning = std::move(directory_sync).error();
    return commit;
  }

private:
  int descriptor_ = -1;
  std::filesystem::path path_;
  bool anonymous_ = false;
  bool finished_ = false;
};

} // namespace

PlatformResult<std::unique_ptr<NativeOutputFile>>
openNativeOutputExclusively(const std::filesystem::path &path) {
#if defined(__linux__)
  auto parent = path.parent_path();
  if (parent.empty())
    parent = ".";
  const int anonymous =
      ::open(parent.c_str(), O_TMPFILE | O_WRONLY | O_CLOEXEC, 0666);
  if (anonymous >= 0) {
    try {
      return std::unique_ptr<NativeOutputFile>(
          std::make_unique<PosixOutputFile>(anonymous, path, true));
    } catch (...) {
      static_cast<void>(::close(anonymous));
      throw;
    }
  }
  if (errno != EOPNOTSUPP && errno != ENOTSUP && errno != EINVAL &&
      errno != EISDIR) {
    return ioError("cannot reserve anonymous temporary output", errno);
  }
#endif
  const int descriptor =
      ::open(path.c_str(), O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC, 0666);
  if (descriptor < 0) {
    if (errno == EEXIST)
      return std::unique_ptr<NativeOutputFile>{};
    return ioError("cannot reserve temporary output", errno);
  }
  try {
    return std::unique_ptr<NativeOutputFile>(
        std::make_unique<PosixOutputFile>(descriptor, path, false));
  } catch (...) {
    static_cast<void>(::close(descriptor));
    static_cast<void>(::unlink(path.c_str()));
    throw;
  }
}

} // namespace kpt::platform
