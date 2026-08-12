#pragma once

#include "platform/utf8_path.hpp"

#include <algorithm>
#include <charconv>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <set>
#include <string>
#include <string_view>
#include <vector>

namespace kpt::workflow {
namespace detail {

enum class SequenceTokenKind { Text, Number };

struct SequenceToken {
  SequenceTokenKind kind;
  std::string_view value;
};

// Filename sequence grammar is deliberately small: text and numeric fields,
// with an optional fractional part.  Keeping the scanner stateful makes mixed
// names such as scan_2_1224.09.bin sort without first guessing one global
// naming convention for the whole directory.
class SequenceNameScanner {
public:
  explicit SequenceNameScanner(std::string_view name) : remaining_(name) {}

  [[nodiscard]] bool empty() const noexcept { return remaining_.empty(); }

  SequenceToken next() noexcept {
    const bool number = isDigit(remaining_.front());
    std::size_t length = 0;
    if (number) {
      while (length < remaining_.size() && isDigit(remaining_[length]))
        ++length;
      if (length + 1 < remaining_.size() && remaining_[length] == '.' &&
          isDigit(remaining_[length + 1])) {
        ++length;
        while (length < remaining_.size() && isDigit(remaining_[length]))
          ++length;
      }
    } else {
      while (length < remaining_.size() && !isDigit(remaining_[length]))
        ++length;
    }
    const auto value = remaining_.substr(0, length);
    remaining_.remove_prefix(length);
    return {number ? SequenceTokenKind::Number : SequenceTokenKind::Text,
            value};
  }

private:
  static bool isDigit(char value) noexcept {
    return value >= '0' && value <= '9';
  }

  std::string_view remaining_;
};

inline std::string_view trimLeadingZeroes(std::string_view value) noexcept {
  const auto first = value.find_first_not_of('0');
  return first == std::string_view::npos ? std::string_view{} : value.substr(first);
}

inline std::string_view trimTrailingZeroes(std::string_view value) noexcept {
  const auto last = value.find_last_not_of('0');
  return last == std::string_view::npos ? std::string_view{} : value.substr(0, last + 1);
}

inline int compareNumericToken(std::string_view left,
                               std::string_view right) noexcept {
  const auto left_dot = left.find('.');
  const auto right_dot = right.find('.');
  const auto left_integer = trimLeadingZeroes(left.substr(0, left_dot));
  const auto right_integer = trimLeadingZeroes(right.substr(0, right_dot));
  if (left_integer.size() != right_integer.size())
    return left_integer.size() < right_integer.size() ? -1 : 1;
  if (const auto order = left_integer.compare(right_integer); order != 0)
    return order < 0 ? -1 : 1;

  const auto left_fraction = trimTrailingZeroes(
      left_dot == std::string_view::npos ? std::string_view{}
                                        : left.substr(left_dot + 1));
  const auto right_fraction = trimTrailingZeroes(
      right_dot == std::string_view::npos ? std::string_view{}
                                          : right.substr(right_dot + 1));
  const auto digits = std::max(left_fraction.size(), right_fraction.size());
  for (std::size_t index = 0; index < digits; ++index) {
    const char left_digit =
        index < left_fraction.size() ? left_fraction[index] : '0';
    const char right_digit =
        index < right_fraction.size() ? right_fraction[index] : '0';
    if (left_digit != right_digit)
      return left_digit < right_digit ? -1 : 1;
  }
  return 0;
}

struct SequenceNumericFields {
  std::vector<std::string_view> sequence_numbers;
  std::vector<std::string_view> timestamps;
};

struct CatalogNumericField {
  std::string value;
  bool timestamp;
};

inline std::vector<CatalogNumericField> catalogNumericFields(
    std::string_view name) {
  std::vector<CatalogNumericField> fields;
  SequenceNameScanner scanner(name);
  while (!scanner.empty()) {
    const auto token = scanner.next();
    if (token.kind == SequenceTokenKind::Number)
      fields.push_back({std::string(token.value),
                        token.value.find('.') != std::string_view::npos});
  }
  return fields;
}

inline SequenceNumericFields numericFields(std::string_view name) {
  SequenceNumericFields fields;
  SequenceNameScanner scanner(name);
  while (!scanner.empty()) {
    const auto token = scanner.next();
    if (token.kind != SequenceTokenKind::Number)
      continue;
    if (token.value.find('.') == std::string_view::npos)
      fields.sequence_numbers.push_back(token.value);
    else
      fields.timestamps.push_back(token.value);
  }
  return fields;
}

inline int compareNumericFields(
    const std::vector<std::string_view> &left,
    const std::vector<std::string_view> &right) noexcept {
  const auto shared = std::min(left.size(), right.size());
  for (std::size_t index = 0; index < shared; ++index) {
    if (const auto order = compareNumericToken(left[index], right[index]);
        order != 0)
      return order;
  }
  if (left.size() != right.size())
    return left.size() < right.size() ? -1 : 1;
  return 0;
}

inline bool sequenceNameLess(std::string_view left,
                             std::string_view right) {
  const auto left_fields = numericFields(left);
  const auto right_fields = numericFields(right);
  // Integer fields denote explicit frame/sequence numbers.  They are the
  // primary key even when a decimal timestamp occurs earlier in the name.
  if (!left_fields.sequence_numbers.empty() &&
      !right_fields.sequence_numbers.empty()) {
    if (const auto order = compareNumericFields(left_fields.sequence_numbers,
                                                right_fields.sequence_numbers);
        order != 0)
      return order < 0;
  }
  if (!left_fields.timestamps.empty() && !right_fields.timestamps.empty()) {
    if (const auto order = compareNumericFields(left_fields.timestamps,
                                                right_fields.timestamps);
        order != 0)
      return order < 0;
  }

  SequenceNameScanner left_scanner(left);
  SequenceNameScanner right_scanner(right);
  while (!left_scanner.empty() && !right_scanner.empty()) {
    const auto left_token = left_scanner.next();
    const auto right_token = right_scanner.next();
    if (left_token.kind == right_token.kind) {
      const int order = left_token.kind == SequenceTokenKind::Number
                            ? compareNumericToken(left_token.value,
                                                  right_token.value)
                            : left_token.value.compare(right_token.value);
      if (order != 0)
        return order < 0;
    } else if (left_token.value != right_token.value) {
      return left_token.value < right_token.value;
    }
  }
  if (left_scanner.empty() != right_scanner.empty())
    return left_scanner.empty();
  // Numerically equivalent spellings (1, 01, 1.0) still need deterministic
  // ordering so std::sort observes a strict weak ordering on all platforms.
  return left < right;
}

struct CatalogFieldScore {
  std::size_t index = 0;
  std::size_t distinct = 0;
  bool all_unique = false;
  bool integer = false;
  long double density = 0.0L;
};

inline std::optional<std::uint64_t> unsignedValue(
    std::string_view value) noexcept {
  if (value.find('.') != std::string_view::npos)
    return std::nullopt;
  std::uint64_t result = 0;
  const auto parsed =
      std::from_chars(value.data(), value.data() + value.size(), result);
  if (parsed.ec != std::errc{} || parsed.ptr != value.data() + value.size())
    return std::nullopt;
  return result;
}

inline bool betterFieldScore(const CatalogFieldScore &left,
                             const CatalogFieldScore &right) noexcept {
  if (left.all_unique != right.all_unique)
    return left.all_unique;
  if (left.integer != right.integer)
    return left.integer;
  if (left.density != right.density)
    return left.density > right.density;
  if (left.distinct != right.distinct)
    return left.distinct > right.distinct;
  // When evidence is otherwise equal, frame indices are more commonly the
  // last changing integer field than an embedded sensor/date identifier.
  return left.index > right.index;
}

inline std::optional<std::size_t> inferSequenceField(
    const std::vector<std::vector<CatalogNumericField>> &catalog) {
  if (catalog.size() < 3)
    return std::nullopt;
  std::size_t field_count = catalog.front().size();
  for (const auto &fields : catalog)
    field_count = std::min(field_count, fields.size());

  std::optional<CatalogFieldScore> best;
  for (std::size_t index = 0; index < field_count; ++index) {
    const bool timestamp = catalog.front()[index].timestamp;
    if (std::ranges::any_of(catalog, [index, timestamp](const auto &fields) {
          return fields[index].timestamp != timestamp;
        }))
      continue;

    std::vector<std::string_view> values;
    values.reserve(catalog.size());
    for (const auto &fields : catalog)
      values.push_back(fields[index].value);
    std::sort(values.begin(), values.end(), [](auto left, auto right) {
      const auto order = compareNumericToken(left, right);
      return order != 0 ? order < 0 : left < right;
    });
    const auto unique_end = std::unique(
        values.begin(), values.end(), [](auto left, auto right) {
          return compareNumericToken(left, right) == 0;
        });
    const auto distinct =
        static_cast<std::size_t>(std::distance(values.begin(), unique_end));
    if (distinct < 2)
      continue;

    CatalogFieldScore score{index, distinct, distinct == catalog.size(),
                            !timestamp, 0.0L};
    if (!timestamp) {
      std::optional<std::uint64_t> minimum;
      std::optional<std::uint64_t> maximum;
      for (const auto &fields : catalog) {
        const auto value = unsignedValue(fields[index].value);
        if (!value) {
          minimum.reset();
          break;
        }
        minimum = minimum ? std::min(*minimum, *value) : *value;
        maximum = maximum ? std::max(*maximum, *value) : *value;
      }
      if (minimum && maximum && *maximum - *minimum != UINT64_MAX) {
        const auto span = *maximum - *minimum + 1U;
        score.density = static_cast<long double>(distinct) /
                        static_cast<long double>(span);
      }
    } else {
      score.density = static_cast<long double>(distinct) /
                      static_cast<long double>(catalog.size());
    }
    if (!best || betterFieldScore(score, *best))
      best = score;
  }
  return best ? std::optional<std::size_t>{best->index} : std::nullopt;
}

} // namespace detail

inline void sortSequencePaths(std::vector<std::filesystem::path> &paths) {
  struct Entry {
    std::filesystem::path path;
    std::string name;
    std::vector<detail::CatalogNumericField> fields;
  };
  std::vector<Entry> entries;
  entries.reserve(paths.size());
  for (auto &path : paths) {
    auto converted = platform::pathToUtf8(path.filename());
    auto name = converted ? std::move(converted).value()
                          : path.filename().string();
    entries.push_back(
        {std::move(path), std::move(name), {}});
    entries.back().fields = detail::catalogNumericFields(entries.back().name);
  }
  std::vector<std::vector<detail::CatalogNumericField>> catalog;
  catalog.reserve(entries.size());
  for (const auto &entry : entries)
    catalog.push_back(entry.fields);
  const auto primary = detail::inferSequenceField(catalog);
  std::sort(entries.begin(), entries.end(), [primary](const auto &left,
                                                       const auto &right) {
    if (primary && *primary < left.fields.size() &&
        *primary < right.fields.size()) {
      if (const auto order = detail::compareNumericToken(
              left.fields[*primary].value, right.fields[*primary].value);
          order != 0)
        return order < 0;
    }
    if (detail::sequenceNameLess(left.name, right.name))
      return true;
    if (detail::sequenceNameLess(right.name, left.name))
      return false;
    return left.path < right.path;
  });
  for (std::size_t index = 0; index < entries.size(); ++index)
    paths[index] = std::move(entries[index].path);
}

} // namespace kpt::workflow
