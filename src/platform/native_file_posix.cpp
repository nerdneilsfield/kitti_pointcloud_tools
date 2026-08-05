#include "platform/native_file.hpp"

#include <atomic>
#include <cerrno>
#include <cstdint>
#include <fcntl.h>
#include <memory>
#include <span>
#include <string>
#include <sys/stat.h>
#include <sys/types.h>
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
    commit.post_commit_warnings.push_back(std::move(directory_sync).error());
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
    return NativeFileCommit{false, false, {}};
  } else if (errno != ENOSYS && errno != EINVAL && errno != EOPNOTSUPP) {
    return ioError("cannot atomically publish file", errno);
  }
#elif defined(__APPLE__)
  if (::renamex_np(source.c_str(), destination.c_str(), RENAME_EXCL) == 0) {
    moved = true;
  } else if (errno == EEXIST) {
    return NativeFileCommit{false, false, {}};
  } else if (errno != ENOTSUP && errno != EINVAL) {
    return ioError("cannot atomically publish file", errno);
  }
#endif

  if (!moved)
    return ioError("cannot safely publish file without no-replace rename",
                   ENOTSUP);

  auto parent = destination.parent_path();
  if (parent.empty())
    parent = ".";
  NativeFileCommit commit;
  auto directory_sync = syncPath(parent, "cannot flush output directory", true);
  if (!directory_sync)
    commit.post_commit_warnings.push_back(std::move(directory_sync).error());
  return commit;
}

namespace {

bool sameFile(const struct stat &left, const struct stat &right) {
  return left.st_dev == right.st_dev && left.st_ino == right.st_ino;
}

bool privateDirectory(const std::filesystem::path &path) {
  struct stat directory{};
  if (::lstat(path.c_str(), &directory) != 0 ||
      !S_ISDIR(directory.st_mode) || directory.st_uid != ::geteuid())
    return false;
  return (directory.st_mode & (S_IWGRP | S_IWOTH)) == 0;
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
  if (::linkat(descriptor, "", AT_FDCWD, destination.c_str(),
               AT_EMPTY_PATH) == 0) {
    return 0;
  }
  const auto descriptor_path =
      std::string("/proc/self/fd/") + std::to_string(descriptor);
  const int result = ::linkat(AT_FDCWD, descriptor_path.c_str(), AT_FDCWD,
                              destination.c_str(), AT_SYMLINK_FOLLOW);
  if (result != 0 && errno == 0)
    errno = anonymous ? EOPNOTSUPP : EPERM;
  return result;
}
#endif

class PosixOutputFile final : public NativeOutputFile {
public:
  PosixOutputFile(int descriptor, std::filesystem::path path, bool anonymous)
      : descriptor_(descriptor), path_(std::move(path)), anonymous_(anonymous) {
  }
  ~PosixOutputFile() override {
    if (descriptor_ >= 0) {
      if (!anonymous_)
        static_cast<void>(unlinkIfOwned(descriptor_, path_));
      static_cast<void>(::close(descriptor_));
    }
  }

  PlatformResult<void> write(std::span<const std::uint8_t> bytes) override {
    if (finished_)
      return ioError("temporary output is already finished", EBADF);
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
    bool identity_bound = false;
#if defined(__linux__)
    if (!overwrite) {
      if (linkOpenFile(descriptor_, anonymous_, destination) == 0) {
        published = true;
        identity_bound = true;
      } else if (errno == EEXIST) {
        return NativeFileCommit{false, true, {}};
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
        auto name =
            std::filesystem::path(".kpt-publish-" + std::to_string(::getpid()) +
                                  "-" + std::to_string(counter.fetch_add(1)));
        const auto staging = parent / name;
        if (linkOpenFile(descriptor_, anonymous_, staging) == 0) {
          if (::rename(staging.c_str(), destination.c_str()) != 0) {
            const int error = errno;
            static_cast<void>(unlinkIfOwned(descriptor_, staging));
            return ioError("cannot atomically replace output", error);
          }
          published = true;
          identity_bound = true;
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
      if (!overwrite)
        return ioError("cannot safely publish output without no-replace rename",
                       ENOTSUP);
      if (anonymous_)
        return ioError("cannot publish anonymous output handle", errno);
      auto parent = path_.parent_path();
      if (parent.empty())
        parent = ".";
      if (!privateDirectory(parent))
        return ioError("cannot safely publish output in a shared directory",
                       EACCES);
      // macOS and filesystems without descriptor-based rename lack a portable
      // way to bind a path operation to this open handle. Only permit overwrite
      // fallback in an owner-only directory, then verify inode immediately
      // before atomic directory operation.
      auto verified = verifyOpenPath(descriptor_, path_);
      if (!verified)
        return verified.error();
      if (::rename(path_.c_str(), destination.c_str()) != 0)
        return ioError("cannot atomically replace output", errno);
      identity_bound = true;
      published = true;
    }

    auto cleanup_warning = anonymous_ ? std::optional<PlatformError>{}
                                      : unlinkIfOwned(descriptor_, path_);
    auto parent = destination.parent_path();
    if (parent.empty())
      parent = ".";
    NativeFileCommit commit;
    commit.source_identity_bound = identity_bound;
    if (cleanup_warning)
      commit.post_commit_warnings.push_back(std::move(*cleanup_warning));
    auto directory_sync =
        syncPath(parent, "cannot flush output directory", true);
    if (!directory_sync)
      commit.post_commit_warnings.push_back(std::move(directory_sync).error());
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
      ::open(parent.c_str(), O_TMPFILE | O_WRONLY | O_CLOEXEC, 0600);
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
      errno != EISDIR && errno != ENOENT) {
    return ioError("cannot reserve anonymous temporary output", errno);
  }
#endif
  const int descriptor =
      ::open(path.c_str(), O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC, 0600);
  if (descriptor < 0) {
    if (errno == EEXIST)
      return std::unique_ptr<NativeOutputFile>{};
    return ioError("cannot reserve temporary output", errno);
  }
  try {
    return std::unique_ptr<NativeOutputFile>(
        std::make_unique<PosixOutputFile>(descriptor, path, false));
  } catch (...) {
    static_cast<void>(unlinkIfOwned(descriptor, path));
    static_cast<void>(::close(descriptor));
    throw;
  }
}

} // namespace kpt::platform
