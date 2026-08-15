#pragma once

#include "gui/scene/scene.hpp"

#include <filesystem>
#include <string>

namespace kpt::gui {

// Persistent application inspection state. This is deliberately separate from
// platform::SettingsStore, which owns only the ImGui layout INI file.
class InspectionSettingsFile {
public:
  static constexpr int kSchemaVersion = 1;

  explicit InspectionSettingsFile(std::filesystem::path file);

  [[nodiscard]] const std::filesystem::path &path() const noexcept;

  // A missing file is a valid empty settings state. Malformed or unsupported
  // files return false and leave `settings` unchanged.
  [[nodiscard]] bool load(InspectionSettings &settings,
                          std::string *error = nullptr) const;

  // Writes a versioned JSON document through a same-directory temporary file
  // then atomically replaces the destination.
  [[nodiscard]] bool save(const InspectionSettings &settings,
                          std::string *error = nullptr) const;

private:
  std::filesystem::path file_;
};

} // namespace kpt::gui
