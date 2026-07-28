#pragma once

#include "platform/error.hpp"

#include <filesystem>

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

// Reserves a new native path without truncating an existing file. Returns
// false when the candidate already exists.
[[nodiscard]] PlatformResult<bool>
createFileExclusively(const std::filesystem::path &path);

} // namespace kpt::platform
