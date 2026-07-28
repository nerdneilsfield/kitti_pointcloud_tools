#include "platform/detail/atomic_replace.hpp"

#include <filesystem>
#include <fcntl.h>
#include <unistd.h>

#include <utility>

namespace kpt::platform::detail {
namespace {

class PosixAtomicReplace final : public AtomicReplace {
public:
  PlatformResult<void>
  replace(const std::filesystem::path &source,
          const std::filesystem::path &destination) override {
    std::error_code error;
    std::filesystem::rename(source, destination, error);
    if (error) {
      return PlatformError{PlatformErrorCode::SettingsIoFailed,
                           "cannot atomically replace settings file", error};
    }
    fsyncDirectoryContaining(destination);
    return {};
  }

private:
  // fsync of the file data alone does not guarantee that the directory entry
  // renaming is durable after power loss. Best-effort fsync the parent
  // directory; a failure here is not fatal because rename itself succeeded
  // and some filesystems (tmpfs, certain network FS) reject directory fsync.
  static void fsyncDirectoryContaining(const std::filesystem::path &path) {
    const std::filesystem::path parent = path.parent_path();
    if (parent.empty())
      return;
    const int dir = ::open(parent.c_str(), O_RDONLY | O_CLOEXEC);
    if (dir < 0)
      return;
    (void)::fsync(dir);
    (void)::close(dir);
  }
};

} // namespace

std::unique_ptr<AtomicReplace> createAtomicReplace() {
  return std::make_unique<PosixAtomicReplace>();
}

} // namespace kpt::platform::detail
