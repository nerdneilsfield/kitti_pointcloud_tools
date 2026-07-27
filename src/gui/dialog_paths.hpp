#pragma once

#include <filesystem>
#include <map>
#include <string>

namespace kpt::gui {

std::filesystem::path dialogInitialDirectory(const std::string &current,
                                             bool directory);
std::filesystem::path normalizeDialogPath(const std::string &value);
std::filesystem::path
selectedDialogDirectory(const std::map<std::string, std::string> &selection,
                        const std::string &current_path);

} // namespace kpt::gui
