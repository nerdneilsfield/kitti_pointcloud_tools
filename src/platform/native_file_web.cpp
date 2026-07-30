#include "platform/native_file.hpp"

#include <memory>
#include <string>

namespace kpt::platform {
namespace {

template <class T> PlatformResult<T> unsupported(std::string operation) {
  return PlatformError{PlatformErrorCode::NativeFileIoFailed,
                       std::move(operation) +
                           " is unavailable in the browser build",
                       {}};
}

} // namespace

PlatformResult<NativeFileCommit>
replaceFileAtomically(const std::filesystem::path &,
                      const std::filesystem::path &) {
  return unsupported<NativeFileCommit>("atomic file replacement");
}

PlatformResult<NativeFileCommit>
moveFileAtomicallyIfAbsent(const std::filesystem::path &,
                           const std::filesystem::path &) {
  return unsupported<NativeFileCommit>("atomic file publication");
}

PlatformResult<std::unique_ptr<NativeOutputFile>>
openNativeOutputExclusively(const std::filesystem::path &) {
  return unsupported<std::unique_ptr<NativeOutputFile>>("native output");
}

} // namespace kpt::platform
