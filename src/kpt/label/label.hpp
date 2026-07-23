#pragma once
#include "kpt/types.hpp"
#include <filesystem>
#include <map>
#include <tuple>
#include <vector>

namespace kpt {

std::vector<int> loadLabel(const std::filesystem::path& p);
std::map<int,int> rangeNetLabelMap();
std::map<int,std::tuple<int,int,int>> rgbLabelMap();

PointCloudIRGBPtr applyLabel(const PointCloudIRGBConstPtr& cloud,
                             const std::vector<int>& labels,
                             const std::map<int,int>& label_map,
                             const std::map<int,std::tuple<int,int,int>>& rgb_map,
                             bool drop_unlabeled = false);

}  // namespace kpt
