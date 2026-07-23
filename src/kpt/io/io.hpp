#pragma once
#include "kpt/types.hpp"
#include "kpt/io/format.hpp"
#include <filesystem>
#include <optional>

namespace kpt {

PointCloudIRGBPtr load(const std::filesystem::path& p);

void save(const std::filesystem::path& p, const PointCloudIRGB& cloud,
          std::optional<Format> ascii_flavor = std::nullopt);

}  // namespace kpt