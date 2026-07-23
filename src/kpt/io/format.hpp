#pragma once
#include "kpt/types.hpp"
#include <filesystem>
#include <string>
#include <stdexcept>

namespace kpt {

inline Format detect(const std::filesystem::path& p) {
  std::string ext = p.extension().string();
  // lowercase
  for (auto& c : ext) c = static_cast<char>(std::tolower(c));
  if (ext == ".bin") return Format::Bin;
  if (ext == ".pcd") return Format::PCD;
  if (ext == ".ply") return Format::PLY;
  if (ext == ".xyz")    return Format::XYZ;
  if (ext == ".xyzi")   return Format::XYZI;
  if (ext == ".xyzrgb") return Format::XYZRGB;
  if (ext == ".xyzrgbi")return Format::XYZRGBI;
  throw std::runtime_error("unknown format: " + ext);
}

inline std::string toString(Format f) {
  switch (f) {
    case Format::Bin: return "bin";
    case Format::PCD: return "pcd";
    case Format::PLY: return "ply";
    case Format::XYZ: return "xyz";
    case Format::XYZI: return "xyzi";
    case Format::XYZRGB: return "xyzrgb";
    case Format::XYZRGBI: return "xyzrgbi";
  }
  return "?";
}

}  // namespace kpt