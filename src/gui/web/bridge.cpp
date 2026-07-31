#include "gui/web/bridge.hpp"

#include "gui/web/catalog.hpp"
#include "platform/utf8_path.hpp"

#include <emscripten.h>

#include <algorithm>
#include <atomic>
#include <filesystem>
#include <functional>
#include <map>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace kpt::gui::web {
namespace {

struct State {
  std::mutex mutex;
  std::optional<std::filesystem::path> viewer;
  std::vector<std::filesystem::path> clouds;
  std::vector<std::filesystem::path> labels;
  std::optional<std::filesystem::path> poses;
  std::optional<std::filesystem::path> poses2;
  std::string error;
  std::atomic<unsigned> next_request{1};
  std::map<unsigned, AssetStager::Completion> completions;
};

State &state() {
  static State value;
  return value;
}

void recordErrorNoThrow(std::string_view message) noexcept {
  try {
    auto &shared = state();
    std::lock_guard lock(shared.mutex);
    shared.error = message;
  } catch (...) {
  }
}

constexpr std::size_t kMaximumSelectionPayloadBytes = 1024 * 1024;
constexpr std::size_t kMaximumSelectionPaths = 20'000;
constexpr std::size_t kMaximumSelectionPathBytes = 64 * 1024;

std::string boundedString(const char *value, std::string_view description) {
  if (value == nullptr)
    return {};
  std::size_t length = 0;
  while (length <= kMaximumSelectionPayloadBytes && value[length] != '\0')
    ++length;
  if (length > kMaximumSelectionPayloadBytes)
    throw std::runtime_error(std::string(description) + " exceeds 1 MiB limit");
  return std::string(value, length);
}

std::optional<PickerKind> decodePickerKind(int kind) {
  switch (kind) {
  case static_cast<int>(PickerKind::Viewer):
    return PickerKind::Viewer;
  case static_cast<int>(PickerKind::Clouds):
    return PickerKind::Clouds;
  case static_cast<int>(PickerKind::Labels):
    return PickerKind::Labels;
  case static_cast<int>(PickerKind::Poses):
    return PickerKind::Poses;
  case static_cast<int>(PickerKind::Poses2):
    return PickerKind::Poses2;
  default:
    return std::nullopt;
  }
}

std::vector<std::filesystem::path> decodePaths(const char *payload) {
  std::vector<std::filesystem::path> result;
  if (payload == nullptr)
    return result;
  std::istringstream input(boundedString(payload, "selection payload"));
  std::string line;
  while (std::getline(input, line)) {
    if (line.empty())
      continue;
    if (line.size() > kMaximumSelectionPathBytes)
      throw std::runtime_error("selection path exceeds 64 KiB limit");
    if (result.size() >= kMaximumSelectionPaths)
      throw std::runtime_error("selection path count exceeds 20000 limit");
    auto decoded = platform::pathFromUtf8(line);
    if (!decoded)
      throw std::runtime_error(decoded.error().message);
    result.push_back(std::move(decoded).value());
  }
  return result;
}

std::string encodePaths(const std::vector<std::filesystem::path> &paths) {
  std::string result;
  for (const auto &path : paths) {
    auto encoded = platform::pathToUtf8(path);
    if (!encoded)
      continue;
    if (!result.empty())
      result.push_back('\n');
    result += encoded.value();
  }
  return result;
}

EM_JS(void, openBrowserPicker, (int kind), {
  globalThis.KptWeb.pick(kind);
});

EM_JS(void, stageBrowserAssets,
      (const char *paths, unsigned request_id), {
        globalThis.KptWeb.stage(UTF8ToString(paths), request_id);
      });

EM_JS(void, releaseBrowserAssets, (const char *paths), {
  globalThis.KptWeb.release(UTF8ToString(paths));
});

class BrowserAssetStager final : public AssetStager {
public:
  void stage(std::vector<std::filesystem::path> paths,
             Completion completion) override {
    if (paths.empty()) {
      completion(std::nullopt);
      return;
    }
    auto &shared = state();
    const unsigned request = shared.next_request.fetch_add(1);
    {
      std::lock_guard lock(shared.mutex);
      shared.completions.emplace(request, std::move(completion));
    }
    const std::string encoded = encodePaths(paths);
    stageBrowserAssets(encoded.c_str(), request);
  }

  void release(const std::vector<std::filesystem::path> &paths) override {
    const std::string encoded = encodePaths(paths);
    releaseBrowserAssets(encoded.c_str());
  }
};

} // namespace

void openPicker(PickerKind kind) {
  openBrowserPicker(static_cast<int>(kind));
}

SelectionSnapshot selectionSnapshot() {
  auto &shared = state();
  std::lock_guard lock(shared.mutex);
  return {shared.viewer,
          shared.clouds.size(),
          shared.labels.size(),
          shared.poses.has_value(),
          shared.poses2.has_value(),
          shared.error};
}

SequenceBuild buildSequence() {
  auto &shared = state();
  std::lock_guard lock(shared.mutex);
  if (!shared.error.empty())
    return {{}, shared.error};
  auto catalog = validateCatalog(shared.clouds, shared.labels);
  if (!catalog)
    return {{}, std::move(catalog.error)};

  workflow::SequenceOptions options;
  options.input_dir = "/kpt-import/clouds";
  options.glob = "*";
  if (!shared.labels.empty())
    options.label_dir = "/kpt-import/labels";
  options.poses = shared.poses;
  options.poses2 = shared.poses2;
  return {std::make_shared<workflow::SequenceSource>(
              std::move(options), std::move(catalog.clouds)),
          {}};
}

std::shared_ptr<AssetStager> createAssetStager() {
  return std::make_shared<BrowserAssetStager>();
}

extern "C" {

EMSCRIPTEN_KEEPALIVE void kpt_web_selection_changed(int kind,
                                                    const char *payload,
                                                    const char *error) {
  try {
    auto &shared = state();
    std::lock_guard lock(shared.mutex);
    try {
      const auto picker = decodePickerKind(kind);
      if (!picker)
        throw std::runtime_error("invalid picker kind");
      shared.error = boundedString(error, "selection error");
      if (!shared.error.empty())
        return;
      auto paths = decodePaths(payload);
      switch (*picker) {
      case PickerKind::Viewer:
        shared.viewer = paths.empty() ? std::optional<std::filesystem::path>{}
                                      : std::optional(paths.front());
        break;
      case PickerKind::Clouds:
        shared.clouds = std::move(paths);
        break;
      case PickerKind::Labels:
        shared.labels = std::move(paths);
        break;
      case PickerKind::Poses:
        shared.poses = paths.empty() ? std::optional<std::filesystem::path>{}
                                     : std::optional(paths.front());
        break;
      case PickerKind::Poses2:
        shared.poses2 = paths.empty() ? std::optional<std::filesystem::path>{}
                                      : std::optional(paths.front());
        break;
      }
    } catch (const std::exception &exception) {
      shared.error = exception.what();
    } catch (...) {
      shared.error = "unknown selection error";
    }
  } catch (...) {
    recordErrorNoThrow("unable to update browser selection");
  }
}

EMSCRIPTEN_KEEPALIVE const char *kpt_web_selection_error() {
  static thread_local std::string snapshot;
  try {
    auto &shared = state();
    std::lock_guard lock(shared.mutex);
    snapshot = shared.error;
    return snapshot.c_str();
  } catch (...) {
    return "unable to read selection error";
  }
}

EMSCRIPTEN_KEEPALIVE void kpt_web_stage_complete(unsigned request_id,
                                                 const char *error) {
  try {
    std::optional<std::string> stage_error;
    try {
      auto decoded = boundedString(error, "staging error");
      if (!decoded.empty())
        stage_error = std::move(decoded);
    } catch (const std::exception &exception) {
      stage_error = exception.what();
    } catch (...) {
      stage_error = "unknown staging error";
    }

    AssetStager::Completion completion;
    {
      auto &shared = state();
      std::lock_guard lock(shared.mutex);
      const auto found = shared.completions.find(request_id);
      if (found == shared.completions.end())
        return;
      completion = std::move(found->second);
      shared.completions.erase(found);
    }
    try {
      completion(std::move(stage_error));
    } catch (...) {
      recordErrorNoThrow("asset staging completion failed");
    }
  } catch (...) {
    recordErrorNoThrow("unable to complete asset staging");
  }
}

} // extern "C"

} // namespace kpt::gui::web
