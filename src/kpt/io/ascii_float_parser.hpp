#pragma once

#include <charconv>
#include <cstddef>
#include <iomanip>
#include <iterator>
#include <locale>
#include <limits>
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

inline bool isJsonNumber(std::string_view text) {
  std::size_t position = 0;
  if (position < text.size() && text[position] == '-') ++position;
  if (position == text.size()) return false;

  if (text[position] == '0') {
    ++position;
  } else if (text[position] >= '1' && text[position] <= '9') {
    while (position < text.size() && text[position] >= '0' &&
           text[position] <= '9') {
      ++position;
    }
  } else {
    return false;
  }

  if (position < text.size() && text[position] == '.') {
    ++position;
    const auto fraction_begin = position;
    while (position < text.size() && text[position] >= '0' &&
           text[position] <= '9') {
      ++position;
    }
    if (position == fraction_begin) return false;
  }

  if (position < text.size() && (text[position] == 'e' || text[position] == 'E')) {
    ++position;
    if (position < text.size() &&
        (text[position] == '+' || text[position] == '-')) {
      ++position;
    }
    const auto exponent_begin = position;
    while (position < text.size() && text[position] >= '0' &&
           text[position] <= '9') {
      ++position;
    }
    if (position == exponent_begin) return false;
  }

  return position == text.size();
}

template <typename T>
FloatingParseResult parseJsonFloatingPrefix(std::string_view text, T &value) {
#if defined(__APPLE__)
  constexpr std::string_view kNumberCharacters = "0123456789+-.eE";
  const auto token_size = text.find_first_not_of(kNumberCharacters);
  const auto token = text.substr(0, token_size);
  if (!isJsonNumber(token)) return {text.data(), std::errc::invalid_argument};
  const auto result = parseAsciiFloating(token, value);
  const auto consumed = result.ptr - token.data();
  return {text.data() + consumed, result.ec};
#else
  const auto result = std::from_chars(
      text.data(), text.data() + text.size(), value,
      std::chars_format::general);
  return {result.ptr, result.ec};
#endif
}

template <typename T>
bool appendAsciiFloating(std::string &output, T value) {
#if defined(__APPLE__)
  // Apple libc++ does not expose floating-point to_chars for the macOS 13
  // deployment target. max_digits10 keeps the stream representation lossless.
  std::ostringstream stream;
  stream.imbue(std::locale::classic());
  stream << std::setprecision(std::numeric_limits<T>::max_digits10) << value;
  if (!stream) return false;
  output += stream.str();
  return true;
#else
  char buffer[64];
  const auto [end, error] = std::to_chars(
      std::begin(buffer), std::end(buffer), value,
      std::chars_format::general);
  if (error != std::errc{}) return false;
  output.append(buffer, end);
  return true;
#endif
}

} // namespace kpt::io_detail
