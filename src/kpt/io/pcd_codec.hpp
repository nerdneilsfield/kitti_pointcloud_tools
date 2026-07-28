#pragma once

#include "kpt/types.hpp"

#include <filesystem>

namespace kpt::io_detail {

void loadPcd(const std::filesystem::path &path, PointCloudIRGB &cloud);
void savePcd(const std::filesystem::path &path,
             const PointCloudIRGB &cloud);

} // namespace kpt::io_detail
