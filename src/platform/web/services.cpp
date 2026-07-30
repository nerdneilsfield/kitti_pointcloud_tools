#include "platform/services.hpp"

#include <emscripten.h>

#include <cstdlib>
#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <string_view>

namespace kpt::platform {
namespace {

PlatformError webError(PlatformErrorCode code, std::string message) {
  return {code, std::move(message), {}};
}

EM_JS(char *, loadWebIni, (), {
  try {
    const value = globalThis.localStorage.getItem("kpt.imgui.ini");
    if (value === null)
      return 0;
    const size = lengthBytesUTF8(value) + 1;
    const result = _malloc(size);
    stringToUTF8(value, result, size);
    return result;
  } catch (_) {
    return 0;
  }
});

EM_JS(int, saveWebIni, (const char *contents, std::size_t size), {
  try {
    globalThis.localStorage.setItem(
        "kpt.imgui.ini", UTF8ToString(contents, size));
    return 1;
  } catch (_) {
    return 0;
  }
});

class WebLifetime final : public PlatformLifetime {};

class WebPaths final : public Paths {
public:
  PlatformResult<std::filesystem::path> configDirectory() const override {
    return std::filesystem::path("/kpt-config");
  }
};

class WebFonts final : public Fonts {
public:
  PlatformResult<std::optional<FontFace>>
  matchUiFont(std::u32string_view) const override {
    return std::optional<FontFace>(
        FontFace{"/assets/NotoSansSC-VF.ttf", 0});
  }
};

class WebSettingsStore final : public SettingsStore {
public:
  PlatformResult<std::optional<std::string>> loadIni() const override {
    char *contents = loadWebIni();
    if (contents == nullptr)
      return std::optional<std::string>{};
    std::string value(contents);
    std::free(contents);
    return std::optional<std::string>(std::move(value));
  }

  PlatformResult<void>
  saveIniAtomically(std::string_view contents) override {
    if (saveWebIni(contents.data(), contents.size()) == 0) {
      return webError(PlatformErrorCode::SettingsIoFailed,
                      "browser localStorage rejected ImGui settings");
    }
    return {};
  }
};

} // namespace

PlatformResult<Services> createServices() {
  Services services;
  services.platform_lifetime = std::make_unique<WebLifetime>();
  services.paths = std::make_unique<WebPaths>();
  services.fonts = std::make_unique<WebFonts>();
  services.settings = std::make_unique<WebSettingsStore>();
  return services;
}

} // namespace kpt::platform
