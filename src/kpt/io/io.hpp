#pragma once
#include "kpt/io/format.hpp"
#include "kpt/types.hpp"
#include <filesystem>
#include <optional>
#include <span>
#include <stop_token>
#include <string_view>

namespace kpt {

struct CloudSchema {
  bool has_color = false;
  bool has_intensity = false;
};

struct DecodedCloud {
  PointCloudIRGBPtr cloud;
  CloudSchema schema;
};

PointCloudIRGBPtr load(const std::filesystem::path &p,
                       std::stop_token stop = std::stop_token{});
DecodedCloud decode(std::span<const std::byte> bytes,
                    std::string_view source_name,
                    std::stop_token stop = std::stop_token{});

enum class CloudWriteStatus { Written, Skipped };

CloudWriteStatus saveAtomic(const std::filesystem::path &p,
                            const PointCloudIRGB &cloud, bool overwrite,
                            std::optional<Format> ascii_flavor = std::nullopt,
                            std::stop_token stop = {});

void save(const std::filesystem::path &p, const PointCloudIRGB &cloud,
          std::optional<Format> ascii_flavor = std::nullopt);

} // namespace kpt
