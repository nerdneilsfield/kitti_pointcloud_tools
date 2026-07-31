#pragma once

#include <filesystem>
#include <functional>
#include <optional>
#include <string>
#include <vector>

namespace kpt::gui::web {

class AssetStager {
public:
  using Completion = std::function<void(std::optional<std::string>)>;

  virtual ~AssetStager() = default;
  // On error, the stager has rolled back every acquisition. On success, the
  // caller owns one acquisition per path and must release it exactly once.
  virtual void stage(std::vector<std::filesystem::path> paths,
                     Completion completion) = 0;
  virtual void release(const std::vector<std::filesystem::path> &paths) = 0;
};

} // namespace kpt::gui::web
