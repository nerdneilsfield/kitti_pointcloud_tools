#pragma once

#include "platform/error.hpp"

#include <filesystem>
#include <string>
#include <string_view>

namespace kpt::platform {

PlatformResult<std::filesystem::path> pathFromUtf8(std::string_view value);
PlatformResult<std::string> pathToUtf8(const std::filesystem::path &value);
PlatformResult<std::string>
pathToUtf8ForNarrowApi(const std::filesystem::path &value);

} // namespace kpt::platform
