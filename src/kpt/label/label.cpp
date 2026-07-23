#include "kpt/label/label.hpp"
#include <spdlog/spdlog.h>
#include <fstream>
#include <cassert>

namespace kpt {

std::vector<int> loadLabel(const std::filesystem::path& p) {
  std::ifstream ifs(p, std::ios::binary);
  if (!ifs) throw std::runtime_error("file not found: " + p.string());
  std::vector<int> result;
  int label;
  while (ifs.read(reinterpret_cast<char*>(&label), sizeof(int))) {
    result.push_back(label);
  }
  return result;
}

std::map<int,int> rangeNetLabelMap() {
  return {
    {0,0},{1,-1},{10,0},{11,-1},{13,0},{15,-1},{16,-1},{18,0},{20,0},
    {30,-1},{31,-1},{32,-1},{40,1},{44,1},{48,2},{49,3},{50,4},{51,5},
    {52,0},{60,6},{70,7},{71,8},{72,9},{80,10},{81,11},{99,-1},
    {252,-1},{253,-1},{254,-1},{255,-1},{256,-1},{257,-1},{258,-1},{259,-1},
  };
}

std::map<int,std::tuple<int,int,int>> rgbLabelMap() {
  return {
    {0,{0,0,0}},{1,{34,139,0}},{2,{0,255,127}},{3,{8,46,84}},
    {4,{106,90,205}},{5,{65,105,225}},{6,{240,255,255}},{7,{124,252,0}},
    {8,{176,48,96}},{9,{160,32,240}},{10,{218,112,214}},{11,{221,160,221}},
    {-1,{227,23,13}},
  };
}

PointCloudIRGBPtr applyLabel(const PointCloudIRGBConstPtr& cloud,
                             const std::vector<int>& labels,
                             const std::map<int,int>& label_map,
                             const std::map<int,std::tuple<int,int,int>>& rgb_map,
                             bool drop_unlabeled) {
  auto out = std::make_shared<PointCloudIRGB>();
  assert(cloud->size() == labels.size());
  for (size_t i = 0; i < cloud->size(); ++i) {
    auto pt = cloud->points[i];
    int compact = -1;  // default for unknown labels
    auto lit = label_map.find(labels[i]);
    if (lit != label_map.end()) compact = lit->second;
    pt.intensity = static_cast<float>(compact);
    if (drop_unlabeled && compact == -1) continue;
    auto rit = rgb_map.find(compact);
    if (rit != rgb_map.end()) {
      pt.r = static_cast<uint8_t>(std::get<0>(rit->second));
      pt.g = static_cast<uint8_t>(std::get<1>(rit->second));
      pt.b = static_cast<uint8_t>(std::get<2>(rit->second));
    } else {
      pt.r = pt.g = pt.b = 0;  // unknown compact id -> black
    }
    out->push_back(pt);
  }
  return out;
}

}  // namespace kpt
