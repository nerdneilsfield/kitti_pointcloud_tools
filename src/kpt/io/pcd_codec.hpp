#pragma once

#include "kpt/types.hpp"

#include <filesystem>
#include <istream>
#include <ostream>
#include <stop_token>

namespace kpt::io_detail {

void loadPcd(const std::filesystem::path &path, PointCloudIRGB &cloud,
             std::stop_token stop = std::stop_token{});
void loadPcd(std::istream &input, const std::filesystem::path &display_path,
             PointCloudIRGB &cloud, bool &has_color, bool &has_intensity,
             std::stop_token stop = std::stop_token{});
void savePcd(const std::filesystem::path &path, const PointCloudIRGB &cloud);
void savePcd(std::ostream &output, const std::filesystem::path &display_path,
             const PointCloudIRGB &cloud,
             std::stop_token stop = std::stop_token{});

} // namespace kpt::io_detail
