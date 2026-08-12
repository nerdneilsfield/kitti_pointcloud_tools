#include "gui/web/catalog.hpp"

#include "kpt/io/format.hpp"
#include "kpt/workflow/sequence_order.hpp"

#include <algorithm>
#include <set>
#include <string>
#include <utility>

namespace kpt::gui::web {
namespace {

std::string name(const std::filesystem::path &path) {
  return path.filename().string();
}

std::string stem(const std::filesystem::path &path) {
  return path.stem().string();
}

} // namespace

CatalogValidation
validateCatalog(std::vector<std::filesystem::path> clouds,
                const std::vector<std::filesystem::path> &labels) {
  if (clouds.empty())
    return {{}, "Select at least one point-cloud frame"};
  workflow::sortSequencePaths(clouds);

  std::set<std::string> cloud_names;
  std::set<std::string> cloud_stems;
  for (const auto &cloud : clouds) {
    try {
      static_cast<void>(detect(cloud));
    } catch (...) {
      return {{}, "Unsupported point-cloud format: " + name(cloud)};
    }
    if (!cloud_names.insert(name(cloud)).second)
      return {{}, "Duplicate point-cloud filename: " + name(cloud)};
    if (!cloud_stems.insert(stem(cloud)).second)
      return {{}, "Duplicate point-cloud frame stem: " + stem(cloud)};
  }

  if (!labels.empty()) {
    std::set<std::string> label_names;
    std::set<std::string> label_stems;
    for (const auto &label : labels) {
      if (label.extension() != ".label")
        return {{}, "Unsupported label format: " + name(label)};
      if (!label_names.insert(name(label)).second)
        return {{}, "Duplicate label filename: " + name(label)};
      label_stems.insert(stem(label));
    }
    for (const auto &cloud_stem : cloud_stems) {
      if (!label_stems.contains(cloud_stem))
        return {{}, "Missing semantic label for frame: " + cloud_stem};
    }
    for (const auto &label_stem : label_stems) {
      if (!cloud_stems.contains(label_stem))
        return {{}, "Label has no matching point-cloud frame: " + label_stem};
    }
  }
  return {std::move(clouds), {}};
}

} // namespace kpt::gui::web
