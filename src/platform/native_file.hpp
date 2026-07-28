#pragma once

#include "platform/error.hpp"

#include <cstdint>
#include <filesystem>
#include <memory>
#include <optional>
#include <span>
#include <vector>

namespace kpt::platform {

struct NativeFileCommit {
  bool published = true;
  // True only when publication names the file held by the open native handle,
  // rather than a verified pathname fallback.
  bool source_identity_bound = false;
  std::vector<PlatformError> post_commit_warnings;
};

[[nodiscard]] PlatformResult<NativeFileCommit>
replaceFileAtomically(const std::filesystem::path &source,
                      const std::filesystem::path &destination);
// Path-based publication is intended for caller-owned temporary directories.
// NativeOutputFile::publish is preferred when hostile directory writers are
// in scope because it binds publication to the opened file identity.

// Publishes source only when destination does not already exist. The existence
// check and publish are one filesystem operation, so another process cannot be
// overwritten between a probe and the commit.
[[nodiscard]] PlatformResult<NativeFileCommit>
moveFileAtomicallyIfAbsent(const std::filesystem::path &source,
                           const std::filesystem::path &destination);

class NativeOutputFile {
public:
  virtual ~NativeOutputFile() = default;
  [[nodiscard]] virtual PlatformResult<void>
  write(std::span<const std::uint8_t> bytes) = 0;
  // Seals writes and flushes file data. The native handle remains owned only
  // so publish() can rename that exact file identity.
  [[nodiscard]] virtual PlatformResult<void> finish() = 0;
  // Linux and Windows publish the exact file represented by this handle.
  // Platforms without handle-based rename report source_identity_bound=false
  // after their verified-path fallback; such use requires a trusted directory.
  [[nodiscard]] virtual PlatformResult<NativeFileCommit>
  publish(const std::filesystem::path &destination, bool overwrite) = 0;
};

// POSIX may reserve an anonymous file, in which case candidate need not become
// visible. On named-file fallbacks, a null successful value means candidate
// already exists. The returned handle owns safe cleanup until publish or
// destruction; callers must never remove candidate themselves.
[[nodiscard]] PlatformResult<std::unique_ptr<NativeOutputFile>>
openNativeOutputExclusively(const std::filesystem::path &path);

} // namespace kpt::platform
