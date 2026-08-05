#pragma once

#include <charconv>
#include <locale>
#include <sstream>
#include <string>
#include <string_view>
#include <system_error>

namespace kpt::io_detail {

struct FloatingParseResult {
  const char *ptr = nullptr;
  std::errc ec = std::errc::invalid_argument;
};

template <typename T>
FloatingParseResult parseAsciiFloating(std::string_view text, T &value) {
#if defined(__APPLE__)
  // Apple libc++ in Xcode 16.4 has no floating-point from_chars overload.
  // Classic locale preserves the locale-independent decimal grammar used by
  // the from_chars general format on other standard libraries.
  std::istringstream input{std::string(text)};
  input.imbue(std::locale::classic());
  input >> value;
  if (!input || input.peek() != std::char_traits<char>::eof())
    return {text.data(), std::errc::invalid_argument};
  return {text.data() + text.size(), std::errc{}};
#else
  const auto result = std::from_chars(
      text.data(), text.data() + text.size(), value,
      std::chars_format::general);
  return {result.ptr, result.ec};
#endif
}

} // namespace kpt::io_detail
