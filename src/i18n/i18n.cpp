#include "i18n/i18n.hpp"

#include <array>
#include <cstdlib>
#include <cstring>
#include <string>
#include <unordered_map>

#if defined(_WIN32)
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#elif defined(__APPLE__)
#include <clocale>
#elif defined(__linux__) || defined(__EMSCRIPTEN__)
#include <clocale>
#endif

namespace kpt::i18n {
namespace detail {

const std::unordered_map<std::string_view, std::string_view> &translations_en();
const std::unordered_map<std::string_view, std::string_view> &translations_zh_cn();

struct LanguageEntry {
  std::string_view code;
  std::string_view display;
  bool cjk;
  const std::unordered_map<std::string_view, std::string_view> &(*table)();
};

const std::array<LanguageEntry, 2> &languages() {
  static const std::array<LanguageEntry, 2> entries{{
      {"en", "English", false, translations_en},
      {"zh-CN", "简体中文", true, translations_zh_cn},
  }};
  return entries;
}

const LanguageEntry *findLanguage(std::string_view code) {
  for (const auto &entry : languages()) {
    if (entry.code == code)
      return &entry;
  }
  for (const auto &entry : languages()) {
    if (code.starts_with(entry.code.substr(0, 2)))
      return &entry;
  }
  return nullptr;
}

} // namespace detail

namespace {

const detail::LanguageEntry *current_ = nullptr;

std::string detectSystemLanguage() {
#if defined(_WIN32)
  wchar_t locale_name[LOCALE_NAME_MAX_LENGTH] = {};
  if (GetUserDefaultLocaleName(locale_name,
                               static_cast<int>(std::size(locale_name)))) {
    std::wstring wide(locale_name);
    std::string narrow(wide.begin(), wide.end());
    if (narrow.starts_with("zh"))
      return "zh-CN";
  }
#elif defined(__APPLE__) || defined(__linux__) || defined(__EMSCRIPTEN__)
  const char *env = nullptr;
  for (const char *var : {"LC_ALL", "LC_MESSAGES", "LANG"}) {
    env = std::getenv(var);
    if (env && *env)
      break;
  }
  if (env) {
    std::string_view sv(env);
    if (sv.starts_with("zh"))
      return "zh-CN";
  }
#endif
  return "en";
}

} // namespace

void initialize(std::string_view lang) {
  if (lang.empty())
    lang = detectSystemLanguage();
  const auto *entry = detail::findLanguage(lang);
  current_ = entry ? entry : &detail::languages().front();
}

void setLanguage(std::string_view lang) {
  const auto *entry = detail::findLanguage(lang);
  if (entry)
    current_ = entry;
}

std::string_view currentLanguage() {
  return current_ ? current_->code : "en";
}

std::vector<std::string_view> availableLanguages() {
  std::vector<std::string_view> result;
  for (const auto &entry : detail::languages())
    result.push_back(entry.code);
  return result;
}

bool needsCJK() {
  return current_ && current_->cjk;
}

const char *tr(std::string_view key) {
  if (current_) {
    const auto &table = current_->table();
    const auto it = table.find(key);
    if (it != table.end())
      return it->second.data();
  }
  const auto &fallback = detail::translations_en();
  const auto it = fallback.find(key);
  if (it != fallback.end())
    return it->second.data();
  return key.data();
}

} // namespace kpt::i18n
