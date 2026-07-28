#include "gui/dialog_paths.hpp"

#include "platform/utf8_path.hpp"

#include <system_error>

namespace kpt::gui {
namespace {

namespace fs = std::filesystem;

fs::path absoluteNormalized(fs::path path, const fs::path &base = {}) {
  std::error_code error;
  if (path.is_relative()) {
    auto anchor = base;
    if (anchor.empty())
      anchor = fs::current_path(error);
    else if (anchor.is_relative())
      anchor = fs::absolute(anchor, error);
    if (error)
      return path.lexically_normal();
    path = anchor / path;
  }

  path = path.lexically_normal();
  const auto canonical = fs::weakly_canonical(path, error);
  return error ? path.lexically_normal() : canonical;
}

} // namespace

platform::PlatformResult<fs::path>
dialogInitialDirectory(std::string_view current, bool directory) {
  fs::path candidate;
  if (!current.empty()) {
    auto supplied = platform::pathFromUtf8(current);
    if (!supplied)
      return supplied.error();
    const auto &supplied_path = supplied.value();
    candidate = directory ? supplied_path : supplied_path.parent_path();
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

platform::PlatformResult<fs::path>
normalizeDialogPath(std::string_view value,
                    std::string_view current_directory) {
  auto path = platform::pathFromUtf8(value);
  if (!path)
    return path.error();

  fs::path base;
  if (!current_directory.empty()) {
    auto current = platform::pathFromUtf8(current_directory);
    if (!current)
      return current.error();
    base = std::move(current).value();
  }
  return absoluteNormalized(std::move(path).value(), base);
}

platform::PlatformResult<fs::path>
selectedDialogDirectory(const std::map<std::string, std::string> &selection,
                        std::string_view current_path) {
  const auto &value =
      selection.empty() ? current_path : selection.begin()->second;
  return normalizeDialogPath(value, current_path);
}

} // namespace kpt::gui
