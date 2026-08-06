#pragma once

#include <string_view>
#include <vector>

namespace kpt::i18n {

void initialize(std::string_view lang = "");

void setLanguage(std::string_view lang);

[[nodiscard]] std::string_view currentLanguage();

[[nodiscard]] std::vector<std::string_view> availableLanguages();

[[nodiscard]] bool needsCJK();

const char *tr(std::string_view key);

} // namespace kpt::i18n
