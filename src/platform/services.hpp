#pragma once

#include "platform/error.hpp"

#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <string_view>

namespace kpt::platform {

struct FontFace {
  std::filesystem::path file;
  int face_index = 0;
};

class Paths {
public:
  virtual ~Paths() = default;

  [[nodiscard]] virtual PlatformResult<std::filesystem::path>
  configDirectory() const = 0;
};

class Fonts {
public:
  virtual ~Fonts() = default;

  [[nodiscard]] virtual PlatformResult<std::optional<FontFace>>
  matchUiFont(std::u32string_view required_characters) const = 0;
};

class SettingsStore {
public:
  virtual ~SettingsStore() = default;

  [[nodiscard]] virtual PlatformResult<std::optional<std::string>>
  loadIni() const = 0;
  [[nodiscard]] virtual PlatformResult<void>
  saveIniAtomically(std::string_view contents) = 0;
};

struct Services {
  std::unique_ptr<Paths> paths;
  std::unique_ptr<Fonts> fonts;
  std::unique_ptr<SettingsStore> settings;
};

[[nodiscard]] Services createServices();

} // namespace kpt::platform
