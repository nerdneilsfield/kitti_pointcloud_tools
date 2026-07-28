#include <catch2/catch.hpp>

#include "font_fixture_data.hpp"
#include "platform/detail/atomic_replace.hpp"
#include "platform/services.hpp"
#include "platform/settings_store.hpp"
#include "platform/utf8_path.hpp"

#if defined(__linux__) || defined(__APPLE__)
#include <ft2build.h>
#include FT_FREETYPE_H
#endif

#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace {

namespace fs = std::filesystem;

#if defined(__linux__) || defined(__APPLE__)
class EnvironmentGuard {
public:
  explicit EnvironmentGuard(const char *name) : name_(name) {
    if (const char *value = std::getenv(name); value != nullptr)
      original_ = value;
  }

  ~EnvironmentGuard() {
    if (original_)
      setenv(name_.c_str(), original_->c_str(), 1);
    else
      unsetenv(name_.c_str());
  }

private:
  std::string name_;
  std::optional<std::string> original_;
};
#elif defined(_WIN32)
class WideEnvironmentGuard {
public:
  explicit WideEnvironmentGuard(const wchar_t *name) : name_(name) {
    if (const wchar_t *value = _wgetenv(name); value != nullptr)
      original_ = value;
  }

  ~WideEnvironmentGuard() {
    _wputenv_s(name_.c_str(), original_ ? original_->c_str() : L"");
  }

private:
  std::wstring name_;
  std::optional<std::wstring> original_;
};
#endif

class TemporaryDirectory {
public:
  TemporaryDirectory() {
    const auto serial =
        std::chrono::steady_clock::now().time_since_epoch().count();
    path_ =
        fs::temp_directory_path() / ("kpt-platform-" + std::to_string(serial));
    fs::create_directories(path_);
  }

  ~TemporaryDirectory() {
    std::error_code ignored;
    fs::remove_all(path_, ignored);
  }

  const fs::path &path() const { return path_; }

private:
  fs::path path_;
};

class RecordingAtomicReplace final
    : public kpt::platform::detail::AtomicReplace {
public:
  kpt::platform::PlatformResult<void>
  replace(const fs::path &source, const fs::path &destination) override {
    called = true;
    same_directory = source.parent_path() == destination.parent_path();
    temporary_existed = fs::is_regular_file(source);
    std::error_code error;
    fs::rename(source, destination, error);
    if (error) {
      return kpt::platform::PlatformError{
          kpt::platform::PlatformErrorCode::SettingsIoFailed,
          "test replacement failed", error};
    }
    return {};
  }

  bool called = false;
  bool same_directory = false;
  bool temporary_existed = false;
};

std::string utf8(const fs::path &path) {
  auto result = kpt::platform::pathToUtf8(path);
  REQUIRE(result);
  return result.value();
}

kpt::platform::Services createServices() {
  auto created = kpt::platform::createServices();
  REQUIRE(created);
  auto services = std::move(created).value();
  REQUIRE(services.platform_lifetime);
  REQUIRE(services.paths);
  REQUIRE(services.fonts);
  REQUIRE(services.settings);
  return services;
}

fs::path nativePath(std::string_view value) {
  auto decoded = kpt::platform::pathFromUtf8(value);
  REQUIRE(decoded);
  return std::move(decoded).value();
}

std::vector<std::uint8_t> decodeBase64(std::string_view encoded) {
  constexpr std::string_view alphabet =
      "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  std::vector<std::uint8_t> bytes;
  unsigned accumulator = 0;
  int bits = 0;
  for (const char character : encoded) {
    if (character == '=')
      break;
    const auto index = alphabet.find(character);
    REQUIRE(index != std::string_view::npos);
    accumulator = (accumulator << 6U) | static_cast<unsigned>(index);
    bits += 6;
    if (bits >= 8) {
      bits -= 8;
      bytes.push_back(static_cast<std::uint8_t>((accumulator >> bits) & 0xffU));
    }
  }
  return bytes;
}

std::uint16_t readBe16(const std::vector<std::uint8_t> &bytes,
                       std::size_t offset) {
  REQUIRE(offset + 2 <= bytes.size());
  return static_cast<std::uint16_t>(
      (static_cast<unsigned>(bytes[offset]) << 8U) | bytes[offset + 1]);
}

std::uint32_t readBe32(const std::vector<std::uint8_t> &bytes,
                       std::size_t offset) {
  REQUIRE(offset + 4 <= bytes.size());
  return (static_cast<std::uint32_t>(bytes[offset]) << 24U) |
         (static_cast<std::uint32_t>(bytes[offset + 1]) << 16U) |
         (static_cast<std::uint32_t>(bytes[offset + 2]) << 8U) |
         bytes[offset + 3];
}

void writeBe32(std::vector<std::uint8_t> &bytes, std::size_t offset,
               std::uint32_t value) {
  REQUIRE(offset + 4 <= bytes.size());
  bytes[offset] = static_cast<std::uint8_t>(value >> 24U);
  bytes[offset + 1] = static_cast<std::uint8_t>(value >> 16U);
  bytes[offset + 2] = static_cast<std::uint8_t>(value >> 8U);
  bytes[offset + 3] = static_cast<std::uint8_t>(value);
}

void appendBe32(std::vector<std::uint8_t> &bytes, std::uint32_t value) {
  const auto offset = bytes.size();
  bytes.resize(offset + 4);
  writeBe32(bytes, offset, value);
}

std::size_t appendTtcFace(std::vector<std::uint8_t> &collection,
                          std::vector<std::uint8_t> font) {
  while (collection.size() % 4 != 0)
    collection.push_back(0);
  const auto base = collection.size();
  const auto table_count = readBe16(font, 4);
  REQUIRE(12U + static_cast<std::size_t>(table_count) * 16U <= font.size());
  for (std::size_t index = 0; index < table_count; ++index) {
    const auto record = 12U + index * 16U;
    const auto old_offset = readBe32(font, record + 8U);
    REQUIRE(old_offset < font.size());
    writeBe32(font, record + 8U, static_cast<std::uint32_t>(base + old_offset));
  }
  collection.insert(collection.end(), font.begin(), font.end());
  return base;
}

fs::path writeDeterministicTtc(const fs::path &directory) {
  // TTC face 0 lacks U+10280. Face 1 contains it, so the returned face index
  // cannot accidentally pass by selecting the collection's first face.
  std::vector<std::uint8_t> collection{'t', 't', 'c', 'f'};
  appendBe32(collection, 0x00010000U);
  appendBe32(collection, 2U);
  appendBe32(collection, 0U);
  appendBe32(collection, 0U);
  const auto first =
      appendTtcFace(collection, decodeBase64(kpt::test::kNotoSansLydianBase64));
  const auto second =
      appendTtcFace(collection, decodeBase64(kpt::test::kNotoSansLycianBase64));
  writeBe32(collection, 12U, static_cast<std::uint32_t>(first));
  writeBe32(collection, 16U, static_cast<std::uint32_t>(second));

  const auto output = directory / nativePath("确定性字体集合.ttc");
  std::ofstream stream(output, std::ios::binary | std::ios::trunc);
  REQUIRE(stream);
  stream.write(reinterpret_cast<const char *>(collection.data()),
               static_cast<std::streamsize>(collection.size()));
  stream.close();
  REQUIRE(stream);
  return output;
}

} // namespace

#if defined(__linux__)
TEST_CASE("Linux config directory follows absolute XDG path",
          "[platform][paths]") {
  EnvironmentGuard xdg_guard("XDG_CONFIG_HOME");
  EnvironmentGuard home_guard("HOME");
  TemporaryDirectory temporary;
  const auto xdg = temporary.path() / nativePath("配置");
  setenv("XDG_CONFIG_HOME", utf8(xdg).c_str(), 1);
  setenv("HOME", "/must/not/win", 1);

  auto services = createServices();
  const auto directory = services.paths->configDirectory();
  REQUIRE(directory);
  REQUIRE(directory.value() == xdg / "kpt");
  REQUIRE(directory.value().is_absolute());
  REQUIRE(fs::is_directory(directory.value()));
}

TEST_CASE("relative XDG config is ignored in favor of HOME",
          "[platform][paths]") {
  EnvironmentGuard xdg_guard("XDG_CONFIG_HOME");
  EnvironmentGuard home_guard("HOME");
  TemporaryDirectory temporary;
  setenv("XDG_CONFIG_HOME", "relative-config", 1);
  setenv("HOME", utf8(temporary.path()).c_str(), 1);

  auto services = createServices();
  const auto directory = services.paths->configDirectory();
  REQUIRE(directory);
  REQUIRE(directory.value() == temporary.path() / ".config" / "kpt");
}

TEST_CASE("invalid UTF-8 environment is a structured error",
          "[platform][paths][utf8]") {
  EnvironmentGuard xdg_guard("XDG_CONFIG_HOME");
  const std::string malformed(1, static_cast<char>(0x80));
  setenv("XDG_CONFIG_HOME", malformed.c_str(), 1);

  auto created = kpt::platform::createServices();
  REQUIRE(created);
  auto services = std::move(created).value();
  const auto directory = services.paths->configDirectory();
  REQUIRE_FALSE(directory);
  REQUIRE(directory.error().code ==
          kpt::platform::PlatformErrorCode::EnvironmentDecodeFailed);

  const auto settings = services.settings->loadIni();
  REQUIRE_FALSE(settings);
  REQUIRE(settings.error().code ==
          kpt::platform::PlatformErrorCode::EnvironmentDecodeFailed);
}
#elif defined(__APPLE__)
TEST_CASE("macOS config directory is native absolute and writable",
          "[platform][paths][macos]") {
  auto services = createServices();
  const auto directory = services.paths->configDirectory();
  REQUIRE(directory);
  REQUIRE(directory.value().is_absolute());
  REQUIRE(fs::is_directory(directory.value()));

  const auto serial =
      std::chrono::steady_clock::now().time_since_epoch().count();
  const auto fixture =
      directory.value() / nativePath("中文契约-" + std::to_string(serial));
  std::error_code error;
  fs::create_directories(fixture, error);
  REQUIRE_FALSE(error);
  REQUIRE(fs::is_directory(fixture));
  fs::remove(fixture, error);
  REQUIRE_FALSE(error);
}
#elif defined(_WIN32)
TEST_CASE("Windows config directory is native absolute and writable",
          "[platform][paths][windows]") {
  auto services = createServices();
  const auto directory = services.paths->configDirectory();
  REQUIRE(directory);
  REQUIRE(directory.value().is_absolute());

  const auto serial =
      std::chrono::steady_clock::now().time_since_epoch().count();
  const auto fixture =
      directory.value() / nativePath("中文契约-" + std::to_string(serial));
  std::error_code error;
  fs::create_directories(fixture, error);
  REQUIRE_FALSE(error);
  REQUIRE(fs::is_directory(fixture));
  fs::remove(fixture, error);
  REQUIRE_FALSE(error);
}
#endif

TEST_CASE("settings distinguish absent empty and read failure",
          "[platform][settings]") {
  TemporaryDirectory temporary;
  const auto ini = temporary.path() / nativePath("设置") / "imgui.ini";
  auto store = kpt::platform::makeSettingsStore(
      ini, kpt::platform::detail::createAtomicReplace());

  auto absent = store->loadIni();
  REQUIRE(absent);
  REQUIRE_FALSE(absent.value());

  fs::create_directories(ini.parent_path());
  std::ofstream(ini, std::ios::binary);
  auto empty = store->loadIni();
  REQUIRE(empty);
  REQUIRE(empty.value());
  REQUIRE(empty.value()->empty());

  fs::remove(ini);
  fs::create_directory(ini);
  auto failed = store->loadIni();
  REQUIRE_FALSE(failed);
  REQUIRE(failed.error().code ==
          kpt::platform::PlatformErrorCode::SettingsIoFailed);
}

TEST_CASE("settings save writes sibling temp then atomically replaces",
          "[platform][settings]") {
  TemporaryDirectory temporary;
  const auto ini = temporary.path() / "nested" / "imgui.ini";
  auto replacement = std::make_unique<RecordingAtomicReplace>();
  auto *recording = replacement.get();
  auto store = kpt::platform::makeSettingsStore(ini, std::move(replacement));

  REQUIRE(store->saveIniAtomically("[Window][中文]\nPos=1,2\n"));
  REQUIRE(recording->called);
  REQUIRE(recording->same_directory);
  REQUIRE(recording->temporary_existed);

  auto loaded = store->loadIni();
  REQUIRE(loaded);
  REQUIRE(loaded.value() == "[Window][中文]\nPos=1,2\n");

  REQUIRE(store->saveIniAtomically("replacement"));
  loaded = store->loadIni();
  REQUIRE(loaded);
  REQUIRE(loaded.value() == "replacement");

  const auto native_ini = temporary.path() / "native" / "imgui.ini";
  auto native_store = kpt::platform::makeSettingsStore(
      native_ini, kpt::platform::detail::createAtomicReplace());
  REQUIRE(native_store->saveIniAtomically("first"));
  REQUIRE(native_store->saveIniAtomically("second"));
  const auto native_loaded = native_store->loadIni();
  REQUIRE(native_loaded);
  REQUIRE(native_loaded.value() == "second");
}

TEST_CASE("settings temporary files are process-unique and leave no residue",
          "[platform][settings][concurrency]") {
  TemporaryDirectory temporary;
  const auto ini = temporary.path() / "shared" / "imgui.ini";
  auto first = kpt::platform::makeSettingsStore(
      ini, kpt::platform::detail::createAtomicReplace());
  auto second = kpt::platform::makeSettingsStore(
      ini, kpt::platform::detail::createAtomicReplace());

  bool first_ok = false;
  bool second_ok = false;
  std::thread left(
      [&] { first_ok = static_cast<bool>(first->saveIniAtomically("left")); });
  std::thread right([&] {
    second_ok = static_cast<bool>(second->saveIniAtomically("right"));
  });
  left.join();
  right.join();
  REQUIRE(first_ok);
  REQUIRE(second_ok);

  const auto loaded = first->loadIni();
  REQUIRE(loaded);
  REQUIRE(loaded.value());
  REQUIRE((*loaded.value() == "left" || *loaded.value() == "right"));
  for (const auto &entry : fs::directory_iterator(ini.parent_path())) {
    const auto name = utf8(entry.path().filename());
    REQUIRE(name.find(".tmp.") == std::string::npos);
  }
}

TEST_CASE("font override selects exact face in deterministic TTC",
          "[platform][fonts][ttc]") {
  TemporaryDirectory temporary;
  const auto collection = writeDeterministicTtc(temporary.path());
#if defined(__linux__) || defined(__APPLE__)
  EnvironmentGuard override_guard("KPT_CJK_FONT");
  setenv("KPT_CJK_FONT", utf8(collection).c_str(), 1);

  FT_Library library = nullptr;
  REQUIRE(FT_Init_FreeType(&library) == 0);
  FT_Face first = nullptr;
  FT_Face second = nullptr;
  REQUIRE(FT_New_Face(library, collection.c_str(), 0, &first) == 0);
  REQUIRE(FT_New_Face(library, collection.c_str(), 1, &second) == 0);
  REQUIRE(FT_Get_Char_Index(first, static_cast<FT_ULong>(U'\U00010280')) == 0);
  REQUIRE(FT_Get_Char_Index(second, static_cast<FT_ULong>(U'\U00010280')) != 0);
  FT_Done_Face(first);
  FT_Done_Face(second);
  FT_Done_FreeType(library);
#elif defined(_WIN32)
  WideEnvironmentGuard override_guard(L"KPT_CJK_FONT");
  REQUIRE(_wputenv_s(L"KPT_CJK_FONT", collection.c_str()) == 0);
#endif

  auto services = createServices();
  const auto matched = services.fonts->matchUiFont(U"\U00010280");
  const std::string match_error = matched ? "" : matched.error().message;
  INFO(match_error);
  REQUIRE(matched);
  REQUIRE(matched.value());
  REQUIRE(matched.value()->file == collection);
  REQUIRE(matched.value()->face_index == 1);
}

TEST_CASE("font override is authoritative and invalid values do not fallback",
          "[platform][fonts]") {
#if defined(__linux__) || defined(__APPLE__)
  EnvironmentGuard override_guard("KPT_CJK_FONT");
  TemporaryDirectory temporary;
  setenv("KPT_CJK_FONT", utf8(temporary.path() / "missing-font.ttc").c_str(),
         1);

  auto services = createServices();
  const auto matched = services.fonts->matchUiFont(U"中文");
  REQUIRE_FALSE(matched);
  REQUIRE(matched.error().code ==
          kpt::platform::PlatformErrorCode::FontFileUnavailable);

  const std::string malformed(1, static_cast<char>(0x80));
  setenv("KPT_CJK_FONT", malformed.c_str(), 1);
  const auto undecodable = services.fonts->matchUiFont(U"中文");
  REQUIRE_FALSE(undecodable);
  REQUIRE(undecodable.error().code ==
          kpt::platform::PlatformErrorCode::EnvironmentDecodeFailed);
#elif defined(_WIN32)
  WideEnvironmentGuard override_guard(L"KPT_CJK_FONT");
  TemporaryDirectory temporary;
  const auto missing = temporary.path() / nativePath("缺失字体.ttc");
  REQUIRE(_wputenv_s(L"KPT_CJK_FONT", missing.c_str()) == 0);

  auto services = createServices();
  const auto matched = services.fonts->matchUiFont(U"中文");
  REQUIRE_FALSE(matched);
  REQUIRE(matched.error().code ==
          kpt::platform::PlatformErrorCode::FontFileUnavailable);
#endif
}

TEST_CASE("platform font result has a readable face containing required glyphs",
          "[platform][fonts]") {
#if defined(__linux__) || defined(__APPLE__)
  EnvironmentGuard override_guard("KPT_CJK_FONT");
  unsetenv("KPT_CJK_FONT");
  auto services = createServices();
  const auto matched = services.fonts->matchUiFont(U"中文");
  REQUIRE(matched);

  if (!matched.value()) {
    SUCCEED("host has no CJK font; no-match is non-fatal");
    return;
  }

  REQUIRE(fs::is_regular_file(matched.value()->file));
  FT_Library library = nullptr;
  REQUIRE(FT_Init_FreeType(&library) == 0);
  FT_Face collection = nullptr;
  REQUIRE(FT_New_Face(library, matched.value()->file.c_str(), -1,
                      &collection) == 0);
  const auto face_count = collection->num_faces;
  FT_Done_Face(collection);
  if (face_count > 1) {
    REQUIRE(matched.value()->face_index >= 0);
    REQUIRE(matched.value()->face_index < face_count);
  } else {
    WARN("TTC contract not exercised: host selected a single-face font");
  }
  FT_Face face = nullptr;
  REQUIRE(FT_New_Face(library, matched.value()->file.c_str(),
                      matched.value()->face_index, &face) == 0);
  REQUIRE(FT_Get_Char_Index(face, static_cast<FT_ULong>(U'中')) != 0);
  REQUIRE(FT_Get_Char_Index(face, static_cast<FT_ULong>(U'文')) != 0);
  FT_Done_Face(face);
  FT_Done_FreeType(library);

  setenv("KPT_CJK_FONT", utf8(matched.value()->file).c_str(), 1);
  const auto overridden = services.fonts->matchUiFont(U"中文");
  REQUIRE(overridden);
  REQUIRE(overridden.value());
  REQUIRE(overridden.value()->file == matched.value()->file);
#elif defined(_WIN32)
  WideEnvironmentGuard override_guard(L"KPT_CJK_FONT");
  REQUIRE(_wputenv_s(L"KPT_CJK_FONT", L"") == 0);
  auto services = createServices();
  const auto matched = services.fonts->matchUiFont(U"中文");
  REQUIRE(matched);

  if (!matched.value()) {
    SUCCEED("host has no local CJK font; no-match is non-fatal");
    return;
  }

  REQUIRE(fs::is_regular_file(matched.value()->file));
  TemporaryDirectory temporary;
  const auto directory = temporary.path() / nativePath("字体目录");
  fs::create_directories(directory);
  const auto copied = directory / (nativePath("中文字体").native() +
                                   matched.value()->file.extension().native());
  fs::copy_file(matched.value()->file, copied);
  REQUIRE(_wputenv_s(L"KPT_CJK_FONT", copied.c_str()) == 0);

  const auto overridden = services.fonts->matchUiFont(U"中文");
  REQUIRE(overridden);
  REQUIRE(overridden.value());
  REQUIRE(overridden.value()->file == copied);
#endif
}

TEST_CASE("font no-match remains non-fatal", "[platform][fonts]") {
#if defined(__linux__) || defined(__APPLE__)
  EnvironmentGuard override_guard("KPT_CJK_FONT");
  unsetenv("KPT_CJK_FONT");
#elif defined(_WIN32)
  WideEnvironmentGuard override_guard(L"KPT_CJK_FONT");
  REQUIRE(_wputenv_s(L"KPT_CJK_FONT", L"") == 0);
#endif
  auto services = createServices();
  const auto matched =
      services.fonts->matchUiFont(std::u32string_view{U"\U0010FFFF", 1});
  REQUIRE(matched);
  REQUIRE_FALSE(matched.value());
}
