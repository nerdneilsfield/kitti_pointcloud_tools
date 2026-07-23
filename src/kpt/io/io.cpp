#include "kpt/io/io.hpp"

#include <spdlog/spdlog.h>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

#include <fstream>
#include <sstream>
#include <vector>

namespace kpt {

namespace {

PointCloudIRGBPtr makeCloud() { return std::make_shared<PointCloudIRGB>(); }

void loadBin(const std::filesystem::path& p, const PointCloudIRGBPtr& cloud) {
  std::ifstream ifs(p, std::ios::binary | std::ios::ate);
  if (!ifs) throw std::runtime_error("file not found: " + p.string());
  const auto size = ifs.tellg();
  ifs.seekg(0, std::ios::beg);
  constexpr size_t rec = 4 * sizeof(float);
  if (static_cast<size_t>(size) % rec != 0)
    throw std::runtime_error("parse error: bin size not multiple of 16: " + p.string());
  const size_t n = static_cast<size_t>(size) / rec;
  for (size_t i = 0; i < n; ++i) {
    float buf[4];
    ifs.read(reinterpret_cast<char*>(buf), rec);
    PointT pt;
    pt.x = buf[0];
    pt.y = buf[1];
    pt.z = buf[2];
    pt.intensity = buf[3];
    pt.r = pt.g = pt.b = 0;
    cloud->push_back(pt);
  }
}

void loadPCL(const std::filesystem::path& p, const PointCloudIRGBPtr& cloud) {
  const auto fmt = detect(p);
  if (fmt == Format::PCD) {
    if (pcl::io::loadPCDFile<PointT>(p.string(), *cloud) < 0)
      throw std::runtime_error("parse error: pcd load failed: " + p.string());
  } else if (fmt == Format::PLY) {
    if (pcl::io::loadPLYFile<PointT>(p.string(), *cloud) < 0)
      throw std::runtime_error("parse error: ply load failed: " + p.string());
  }
}

void loadAscii(const std::filesystem::path& p, const PointCloudIRGBPtr& cloud) {
  std::ifstream ifs(p);
  if (!ifs) throw std::runtime_error("file not found: " + p.string());
  std::string line;
  int warn_count = 0;
  while (std::getline(ifs, line)) {
    if (line.empty()) continue;
    std::istringstream ss(line);
    std::vector<float> v;
    float f;
    while (ss >> f) v.push_back(f);
    if (v.size() < 3) {
      if (++warn_count <= 50) spdlog::warn("skip short line: {}", line);
      continue;
    }
    PointT pt;
    pt.x = v[0];
    pt.y = v[1];
    pt.z = v[2];
    pt.r = pt.g = pt.b = 0;
    pt.intensity = 0.0f;
    if (v.size() == 3) {
      // xyz only
    } else if (v.size() == 4) {
      pt.intensity = v[3];
    } else if (v.size() == 6) {
      pt.r = static_cast<uint8_t>(v[3]);
      pt.g = static_cast<uint8_t>(v[4]);
      pt.b = static_cast<uint8_t>(v[5]);
    } else if (v.size() == 7) {
      pt.r = static_cast<uint8_t>(v[3]);
      pt.g = static_cast<uint8_t>(v[4]);
      pt.b = static_cast<uint8_t>(v[5]);
      pt.intensity = v[6];
    } else {
      if (++warn_count <= 50) spdlog::warn("skip line with {} cols: {}", v.size(), line);
      continue;
    }
    cloud->push_back(pt);
  }
  if (warn_count > 50) spdlog::warn("... {} more skipped lines", warn_count - 50);
}

}  // namespace

PointCloudIRGBPtr load(const std::filesystem::path& p) {
  if (!std::filesystem::exists(p))
    throw std::runtime_error("file not found: " + p.string());
  auto cloud = makeCloud();
  auto fmt = detect(p);
  switch (fmt) {
    case Format::Bin:
      loadBin(p, cloud);
      break;
    case Format::PCD:
    case Format::PLY:
      loadPCL(p, cloud);
      break;
    default:
      loadAscii(p, cloud);
      break;
  }
  return cloud;
}

void save(const std::filesystem::path& /*p*/, const PointCloudIRGB& /*cloud*/,
          std::optional<Format> /*ascii_flavor*/) {
  throw std::runtime_error("save: not implemented yet");
}

}  // namespace kpt