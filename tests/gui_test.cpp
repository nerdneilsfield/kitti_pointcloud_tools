#include <catch2/catch.hpp>

#include "gui/dialog_paths.hpp"
#include "gui/job_system.hpp"
#include "gui/point_renderer.hpp"
#include "platform/utf8_path.hpp"

#include "ImGuiFileDialog.h"

#include <atomic>
#include <chrono>
#include <fstream>
#include <limits>
#include <map>
#include <string>
#include <thread>

namespace fs = std::filesystem;

using namespace std::chrono_literals;

namespace {

class TemporaryTree {
public:
  TemporaryTree()
      : root(fs::temp_directory_path() /
             ("kpt-dialog-" + std::to_string(std::chrono::steady_clock::now()
                                                 .time_since_epoch()
                                                 .count()))) {
    fs::create_directories(root);
  }

  ~TemporaryTree() {
    std::error_code ignored;
    fs::remove_all(root, ignored);
  }

  fs::path root;
};

class CurrentPathGuard {
public:
  explicit CurrentPathGuard(const fs::path &path)
      : original_(fs::current_path()) {
    fs::current_path(path);
  }

  ~CurrentPathGuard() {
    std::error_code ignored;
    fs::current_path(original_, ignored);
  }

private:
  fs::path original_;
};

std::string asUtf8(const fs::path &path) {
  auto result = kpt::platform::pathToUtf8(path);
  REQUIRE(result);
  return std::move(result).value();
}

class InspectableFileDialog : public IGFD::FileDialog {
public:
  void enumerate() {
    m_FileDialogInternal.fileManager.OpenCurrentPath(m_FileDialogInternal);
  }

  bool select(std::string_view filename) {
    auto &manager = m_FileDialogInternal.fileManager;
    for (std::size_t index = 0; index < manager.GetFullFileListSize();
         ++index) {
      auto file = manager.GetFullFileAt(index);
      if (file && file->fileNameExt == filename) {
        manager.SelectFileName(file);
        return true;
      }
    }
    return false;
  }
};

} // namespace

TEST_CASE("file dialog anchors ancestor sibling and UTF-8 selections",
          "[gui][dialog]") {
  TemporaryTree tree;
  const auto a = tree.root / "A";
  const auto b = a / "B";
  const auto c = b / "C";
  const auto sibling = b / "Sibling";
  const auto chinese = a / "中文目录";
  fs::create_directories(c);
  fs::create_directories(sibling);
  fs::create_directories(chinese);
  CurrentPathGuard process_directory(tree.root);

  const auto fallback = kpt::gui::dialogInitialDirectory("", false);
  REQUIRE(fallback);
  REQUIRE(fallback.value() == tree.root);

  const auto initial =
      kpt::gui::dialogInitialDirectory(asUtf8(c / "cloud.xyz"), false);
  REQUIRE(initial);
  REQUIRE(initial.value() == c);

  for (const auto &selected :
       {c / "c.pcd", b / "b.pcd", a / "a.pcd", sibling / "sibling.pcd",
        chinese / "点云 帧.pcd"}) {
    const auto normalized =
        kpt::gui::normalizeDialogPath(asUtf8(selected), asUtf8(c));
    REQUIRE(normalized);
    REQUIRE(normalized.value() == selected);
  }

  const auto relative = kpt::gui::normalizeDialogPath("../上级.pcd", asUtf8(c));
  REQUIRE(relative);
  REQUIRE(relative.value() == b / "上级.pcd");

  const std::map<std::string, std::string> parent_selection = {{"..", ".."}};
  const auto parent =
      kpt::gui::selectedDialogDirectory(parent_selection, asUtf8(c));
  REQUIRE(parent);
  REQUIRE(parent.value() == b);

  const auto current = kpt::gui::selectedDialogDirectory({}, asUtf8(chinese));
  REQUIRE(current);
  REQUIRE(current.value() == chinese);
}

TEST_CASE("ImGuiFileDialog enumerates and returns UTF-8 paths",
          "[gui][dialog][utf8]") {
  TemporaryTree tree;
  const auto directory = tree.root / "中文目录";
  const std::string filename = "点云 帧.pcd";
  fs::create_directories(directory);
  std::ofstream(directory / filename) << "fixture";

  IGFD::FileDialogConfig config;
  config.path = asUtf8(directory);
  config.fileName = filename;

  InspectableFileDialog dialog;
  dialog.OpenDialog("utf8-contract", "UTF-8 contract", ".pcd", config);
  dialog.enumerate();
  REQUIRE(dialog.select(filename));

  REQUIRE(dialog.GetCurrentPath() == asUtf8(directory));
  REQUIRE(dialog.GetFilePathName() == asUtf8(directory / filename));

  const auto selection = dialog.GetSelection();
  REQUIRE(selection.count(filename) == 1);
  REQUIRE(selection.at(filename) == asUtf8(directory / filename));
}

TEST_CASE("GUI bounds ignore non-finite points and track scalar ranges",
          "[gui]") {
  kpt::PointCloudIRGB cloud;
  kpt::PointT first{};
  first.x = -2.0F;
  first.y = 1.0F;
  first.z = 3.0F;
  first.intensity = 0.25F;
  cloud.push_back(first);

  kpt::PointT second{};
  second.x = 4.0F;
  second.y = 5.0F;
  second.z = -1.0F;
  second.intensity = 2.0F;
  cloud.push_back(second);

  kpt::PointT invalid{};
  invalid.x = std::numeric_limits<float>::quiet_NaN();
  cloud.push_back(invalid);

  const auto bounds = kpt::gui::calculateBounds(cloud);
  REQUIRE(bounds.finite_points == 2);
  REQUIRE(bounds.minimum.x() == -2.0F);
  REQUIRE(bounds.maximum.y() == 5.0F);
  REQUIRE(bounds.z_min == -1.0F);
  REQUIRE(bounds.z_max == 3.0F);
  REQUIRE(bounds.intensity_min == 0.25F);
  REQUIRE(bounds.intensity_max == 2.0F);
  REQUIRE(bounds.radius > 0.0F);
}

TEST_CASE("GUI bounds are benign for empty cloud", "[gui]") {
  const auto bounds = kpt::gui::calculateBounds({});
  REQUIRE(bounds.finite_points == 0);
  REQUIRE(bounds.center.isZero());
  REQUIRE(bounds.radius == 1.0F);
}

TEST_CASE("job system reports completion and cancellation", "[gui]") {
  kpt::gui::JobSystem jobs;
  jobs.setWorkerLimit(1);
  std::atomic<bool> release{false};
  const auto blocker =
      jobs.submit("blocker", kpt::gui::JobPriority::High,
                  [&release](std::stop_token stop,
                             const kpt::gui::JobSystem::Reporter &report) {
                    while (!release.load() && !stop.stop_requested()) {
                      std::this_thread::sleep_for(1ms);
                    }
                    report(1.0F, "released");
                  });
  const auto cancelled = jobs.submit(
      "cancelled", kpt::gui::JobPriority::Low,
      [](std::stop_token, const kpt::gui::JobSystem::Reporter &) {});
  jobs.cancel(cancelled);
  release.store(true);

  const auto deadline = std::chrono::steady_clock::now() + 2s;
  bool blocker_done = false;
  bool queued_cancelled = false;
  while (std::chrono::steady_clock::now() < deadline) {
    for (const auto &snapshot : jobs.snapshots()) {
      if (snapshot.id == blocker &&
          snapshot.state == kpt::gui::JobState::Succeeded) {
        blocker_done = true;
      }
      if (snapshot.id == cancelled &&
          snapshot.state == kpt::gui::JobState::Cancelled) {
        queued_cancelled = true;
      }
    }
    if (blocker_done && queued_cancelled)
      break;
    std::this_thread::sleep_for(2ms);
  }
  REQUIRE(blocker_done);
  REQUIRE(queued_cancelled);
}

TEST_CASE("job system reserves one active-player worker for high priority",
          "[gui]") {
  kpt::gui::JobSystem jobs;
  if (jobs.maxWorkers() < 2) {
    SUCCEED("single-core host has no reservable worker");
    return;
  }

  jobs.setWorkerLimit(2);
  jobs.setPlayerActive(true);
  std::atomic<bool> normal_started{false};
  std::atomic<bool> release_normal{false};
  std::atomic<bool> low_started{false};
  std::atomic<bool> high_started{false};

  jobs.submit("normal blocker", kpt::gui::JobPriority::Normal,
              [&](std::stop_token stop, const kpt::gui::JobSystem::Reporter &) {
                normal_started.store(true);
                while (!release_normal.load() && !stop.stop_requested())
                  std::this_thread::sleep_for(1ms);
              });

  const auto wait_for = [](const std::atomic<bool> &flag) {
    const auto deadline = std::chrono::steady_clock::now() + 2s;
    while (!flag.load() && std::chrono::steady_clock::now() < deadline)
      std::this_thread::sleep_for(1ms);
    return flag.load();
  };
  REQUIRE(wait_for(normal_started));

  jobs.submit("low", kpt::gui::JobPriority::Low,
              [&](std::stop_token, const kpt::gui::JobSystem::Reporter &) {
                low_started.store(true);
              });
  std::this_thread::sleep_for(30ms);
  REQUIRE_FALSE(low_started.load());

  jobs.submit("high", kpt::gui::JobPriority::High,
              [&](std::stop_token, const kpt::gui::JobSystem::Reporter &) {
                high_started.store(true);
              });
  REQUIRE(wait_for(high_started));
  REQUIRE_FALSE(low_started.load());

  release_normal.store(true);
  REQUIRE(wait_for(low_started));
}
