#pragma once
#include "kpt/types.hpp"
#include <filesystem>
#include <span>
#include <stop_token>
#include <string_view>

namespace kpt::io_detail {

void loadLas(const std::filesystem::path &path, PointCloudIRGB &cloud,
             bool &has_color, bool &has_intensity, std::stop_token stop);

void loadLas(std::istream &input, const std::filesystem::path &path,
             PointCloudIRGB &cloud, bool &has_color, bool &has_intensity,
             std::stop_token stop);

void saveLas(std::ostream &output, const std::filesystem::path &path,
             const PointCloudIRGB &cloud, std::stop_token stop);

} // namespace kpt::io_detail
