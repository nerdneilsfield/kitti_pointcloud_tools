#include "gui/dialog_paths.hpp"

#include <system_error>

namespace kpt::gui {
namespace {

namespace fs = std::filesystem;

fs::path absoluteNormalized(fs::path path) {
  std::error_code error;
  if (path.empty())
    path = fs::current_path(error);
  if (error)
    return path.lexically_normal();

  if (path.is_relative()) {
    path = fs::absolute(path, error);
    if (error)
      return path.lexically_normal();
  }

  const auto canonical = fs::weakly_canonical(path, error);
  return error ? path.lexically_normal() : canonical;
}

} // namespace

fs::path dialogInitialDirectory(const std::string &current, bool directory) {
  fs::path candidate;
  if (!current.empty()) {
    const fs::path supplied(current);
    candidate = directory ? supplied : supplied.parent_path();
  }
  candidate = absoluteNormalized(std::move(candidate));

  std::error_code error;
  while (!candidate.empty() && !fs::is_directory(candidate, error)) {
    error.clear();
    const auto parent = candidate.parent_path();
    if (parent == candidate)
      break;
    candidate = parent;
  }
  if (!candidate.empty() && fs::is_directory(candidate, error))
    return candidate;
  return absoluteNormalized({});
}

fs::path normalizeDialogPath(const std::string &value) {
  return absoluteNormalized(fs::path(value));
}

fs::path
selectedDialogDirectory(const std::map<std::string, std::string> &selection,
                        const std::string &current_path) {
  const auto &value =
      selection.empty() ? current_path : selection.begin()->second;
  return normalizeDialogPath(value);
}

} // namespace kpt::gui
