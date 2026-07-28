#pragma once

#include "platform/error.hpp"

#include <filesystem>

namespace kpt::platform {

[[nodiscard]] PlatformResult<void>
replaceFileAtomically(const std::filesystem::path &source,
                      const std::filesystem::path &destination);

} // namespace kpt::platform
