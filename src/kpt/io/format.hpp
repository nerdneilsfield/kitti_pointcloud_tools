#pragma once
#include "kpt/types.hpp"
#include <filesystem>
#include <stdexcept>
#include <string>

namespace kpt {

inline Format detect(const std::filesystem::path &p) {
  auto ext = p.extension().native();
  for (auto &c : ext) {
    if (c >= 'A' && c <= 'Z')
      c += 'a' - 'A';
  }
  if (ext == std::filesystem::path(".bin").native())
    return Format::Bin;
  if (ext == std::filesystem::path(".pcd").native())
    return Format::PCD;
  if (ext == std::filesystem::path(".ply").native())
    return Format::PLY;
  if (ext == std::filesystem::path(".las").native())
    return Format::LAS;
  if (ext == std::filesystem::path(".xyz").native())
    return Format::XYZ;
  if (ext == std::filesystem::path(".xyzi").native())
    return Format::XYZI;
  if (ext == std::filesystem::path(".xyzrgb").native())
    return Format::XYZRGB;
  if (ext == std::filesystem::path(".xyzrgbi").native())
    return Format::XYZRGBI;
  throw std::runtime_error("unknown format");
}

inline std::string toString(Format f) {
  switch (f) {
  case Format::Bin:
    return "bin";
  case Format::PCD:
    return "pcd";
  case Format::PLY:
    return "ply";
  case Format::LAS:
    return "las";
  case Format::XYZ:
    return "xyz";
  case Format::XYZI:
    return "xyzi";
  case Format::XYZRGB:
    return "xyzrgb";
  case Format::XYZRGBI:
    return "xyzrgbi";
  }
  return "?";
}

} // namespace kpt
