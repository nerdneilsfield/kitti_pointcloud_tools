#pragma once

#include "kpt/types.hpp"

#include <filesystem>

namespace kpt::io_detail {

void loadPly(const std::filesystem::path &path, PointCloudIRGB &cloud);
void savePly(const std::filesystem::path &path, const PointCloudIRGB &cloud);

} // namespace kpt::io_detail
