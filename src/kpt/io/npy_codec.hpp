#pragma once
#include "kpt/types.hpp"
#include <filesystem>
#include <istream>
#include <stop_token>

namespace kpt::io_detail {

void loadNpy(std::istream &input, const std::filesystem::path &path,
             PointCloudIRGB &cloud, bool &has_color, bool &has_intensity,
             std::stop_token stop);

void saveNpy(std::ostream &output, const std::filesystem::path &path,
             const PointCloudIRGB &cloud, std::stop_token stop);

} // namespace kpt::io_detail
