#pragma once
#include "kpt/io/format.hpp"
#include "kpt/types.hpp"
#include <filesystem>
#include <optional>
#include <stop_token>

namespace kpt {

PointCloudIRGBPtr load(const std::filesystem::path &p,
                       std::stop_token stop = std::stop_token{});

enum class CloudWriteStatus { Written, Skipped };

CloudWriteStatus saveAtomic(const std::filesystem::path &p,
                            const PointCloudIRGB &cloud, bool overwrite,
                            std::optional<Format> ascii_flavor = std::nullopt,
                            std::stop_token stop = {});

void save(const std::filesystem::path &p, const PointCloudIRGB &cloud,
          std::optional<Format> ascii_flavor = std::nullopt);

} // namespace kpt
