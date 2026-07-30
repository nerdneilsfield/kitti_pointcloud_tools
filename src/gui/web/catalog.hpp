#pragma once

#include <filesystem>
#include <string>
#include <vector>

namespace kpt::gui::web {

struct CatalogValidation {
  std::vector<std::filesystem::path> clouds;
  std::string error;

  explicit operator bool() const noexcept { return error.empty(); }
};

CatalogValidation
validateCatalog(std::vector<std::filesystem::path> clouds,
                const std::vector<std::filesystem::path> &labels);

} // namespace kpt::gui::web
