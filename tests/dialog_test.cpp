#include <catch2/catch.hpp>

#include "gui/dialog_paths.hpp"
#include "platform/utf8_path.hpp"

#include "ImGuiFileDialog.h"

#include <chrono>
#include <filesystem>
#include <fstream>
#include <map>
#include <string>
#include <string_view>

namespace {

namespace fs = std::filesystem;

fs::path nativePath(std::string_view value) {
  auto result = kpt::platform::pathFromUtf8(value);
  REQUIRE(result);
  return std::move(result).value();
}

std::string asUtf8(const fs::path &path) {
  auto result = kpt::platform::pathToUtf8(path);
  REQUIRE(result);
  return std::move(result).value();
}

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
          "[dialog][utf8]") {
  TemporaryTree tree;
  const auto a = tree.root / "A";
  const auto b = a / "B";
  const auto c = b / "C";
  const auto sibling = b / "Sibling";
  const auto chinese = a / nativePath("中文目录");
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
        chinese / nativePath("点云 帧.pcd")}) {
    const auto normalized =
        kpt::gui::normalizeDialogPath(asUtf8(selected), asUtf8(c));
    REQUIRE(normalized);
    REQUIRE(normalized.value() == selected);
  }

  const auto relative = kpt::gui::normalizeDialogPath("../上级.pcd", asUtf8(c));
  REQUIRE(relative);
  REQUIRE(relative.value() == b / nativePath("上级.pcd"));

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
          "[dialog][utf8]") {
  TemporaryTree tree;
  const auto directory = tree.root / nativePath("中文目录");
  const std::string filename = "点云 帧.pcd";
  const auto native_filename = nativePath(filename);
  fs::create_directories(directory);
  std::ofstream(directory / native_filename) << "fixture";

  IGFD::FileDialogConfig config;
  config.path = asUtf8(directory);
  config.fileName = filename;

  InspectableFileDialog dialog;
  dialog.OpenDialog("utf8-contract", "UTF-8 contract", ".pcd", config);
  dialog.enumerate();
  REQUIRE(dialog.select(filename));

  REQUIRE(dialog.GetCurrentPath() == asUtf8(directory));
  REQUIRE(dialog.GetFilePathName() == asUtf8(directory / native_filename));

  const auto selection = dialog.GetSelection();
  REQUIRE(selection.count(filename) == 1);
  REQUIRE(selection.at(filename) == asUtf8(directory / native_filename));
}
