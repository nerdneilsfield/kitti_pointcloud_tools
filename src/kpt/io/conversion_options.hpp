#pragma once

#include "kpt/io/format.hpp"

#include <filesystem>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>

namespace kpt::io {

inline bool isAsciiFormat(Format format) noexcept {
  return format == Format::XYZ || format == Format::XYZI ||
         format == Format::XYZRGB || format == Format::XYZRGBI;
}

inline std::optional<Format> parseAsciiFlavor(std::string_view value) {
  if (value.empty())
    return std::nullopt;
  if (value == "xyz")
    return Format::XYZ;
  if (value == "xyzi")
    return Format::XYZI;
  if (value == "xyzrgb")
    return Format::XYZRGB;
  if (value == "xyzrgbi")
    return Format::XYZRGBI;
  throw std::invalid_argument("unknown ascii-flavor: " + std::string(value));
}

inline void validateAsciiFlavor(Format output,
                                const std::optional<Format> &flavor) {
  if (!flavor)
    return;
  if (!isAsciiFormat(output) || *flavor != output) {
    throw std::invalid_argument(
        "--ascii-flavor must match the output format (" + toString(output) +
        ")");
  }
}

inline void validateAsciiFlavor(const std::filesystem::path &output,
                                const std::optional<Format> &flavor) {
  if (!flavor)
    return;
  try {
    validateAsciiFlavor(detect(output), flavor);
  } catch (const std::runtime_error &) {
    throw std::invalid_argument(
        "--ascii-flavor requires a matching supported output extension");
  }
}

inline void validateLogLevel(int level) {
  if (level < 0 || level > 3)
    throw std::invalid_argument("log-level must be in [0,3]");
}

} // namespace kpt::io
