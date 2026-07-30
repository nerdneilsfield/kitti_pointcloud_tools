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
  virtual void stage(std::vector<std::filesystem::path> paths,
                     Completion completion) = 0;
  virtual void release(const std::vector<std::filesystem::path> &paths) = 0;
};

} // namespace kpt::gui::web
