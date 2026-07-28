#pragma once

#include "platform/error.hpp"

#include <filesystem>
#include <map>
#include <string>
#include <string_view>

namespace kpt::gui {

platform::PlatformResult<std::filesystem::path>
dialogInitialDirectory(std::string_view current, bool directory);
platform::PlatformResult<std::filesystem::path>
normalizeDialogPath(std::string_view value, std::string_view current_directory);
platform::PlatformResult<std::filesystem::path>
selectedDialogDirectory(const std::map<std::string, std::string> &selection,
                        std::string_view current_path);

} // namespace kpt::gui
