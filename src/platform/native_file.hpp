#pragma once

#include "platform/error.hpp"

#include <cstdint>
#include <filesystem>
#include <memory>
#include <span>

namespace kpt::platform {

[[nodiscard]] PlatformResult<void>
replaceFileAtomically(const std::filesystem::path &source,
                      const std::filesystem::path &destination);

// Publishes source only when destination does not already exist. The existence
// check and publish are one filesystem operation, so another process cannot be
// overwritten between a probe and the commit.
[[nodiscard]] PlatformResult<bool>
moveFileAtomicallyIfAbsent(const std::filesystem::path &source,
                           const std::filesystem::path &destination);

class NativeOutputFile {
public:
  virtual ~NativeOutputFile() = default;
  [[nodiscard]] virtual PlatformResult<void>
  write(std::span<const std::uint8_t> bytes) = 0;
  [[nodiscard]] virtual PlatformResult<void> finish() = 0;
};

// A null successful value means the candidate already exists. The returned
// native handle stays open until finish/destruction; callers must not reopen
// the path between reservation and writing.
[[nodiscard]] PlatformResult<std::unique_ptr<NativeOutputFile>>
openNativeOutputExclusively(const std::filesystem::path &path);

} // namespace kpt::platform
