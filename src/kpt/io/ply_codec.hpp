#pragma once

#include "kpt/types.hpp"

#include <filesystem>
#include <ostream>
#include <stop_token>

namespace kpt::io_detail {

void loadPly(const std::filesystem::path &path, PointCloudIRGB &cloud,
             std::stop_token stop = std::stop_token{});
void savePly(const std::filesystem::path &path, const PointCloudIRGB &cloud);
void savePly(std::ostream &output, const std::filesystem::path &display_path,
             const PointCloudIRGB &cloud,
             std::stop_token stop = std::stop_token{});

} // namespace kpt::io_detail
