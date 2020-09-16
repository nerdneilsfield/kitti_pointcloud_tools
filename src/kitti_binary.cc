/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   kitti_binary.cc                                    :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: dengqi <dengqi935@outl>                    +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2020/09/07 10:42:18 by dengqi            #+#    #+#             */
/*   Updated: 2020/09/07 11:14:59 by dengqi           ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include "kitti_binary.h"

#include <spdlog/spdlog.h>

namespace kitti_binary_tools {

KittiBinary::KittiBinary() {}

KittiBinary::KittiBinary(const std::string& filename)
     {
  spdlog::debug("Call [KittiBinary] Constructor with  File {}", filename);
  LoadFromFile(filename);
}

void KittiBinary::LoadFromFile(const std::string& filename) {
  spdlog::debug("Call [KittiBinary] LoadFrom File {}", filename);
  std::ifstream ifs(filename.c_str(), std::ios::binary | std::ios::ate);
  ifs.seekg(0, std::ios::beg);
  for (size_t i = 0; ifs.good() && !ifs.eof(); i++) {
    SimplePoint point;
    PointType pcl_point;
    ifs.read(reinterpret_cast<char*>(&point), sizeof(SimplePoint));
    pcl_point.x = point.x;
    pcl_point.y = point.y;
    pcl_point.z = point.z;
    pcl_point.intensity = point.intensity;
    cloud_.push_back(pcl_point);
    size_++;
  }
  spdlog::debug("[KittiBinary] LoadFrom File: Load {} points", size_);
}

void KittiBinary::SaveAsPCD(const std::string& out_file_name) {
  spdlog::debug("[KittiBinary] SaveAsPCD {} with size of {}", out_file_name, cloud_.size());
  pcl::io::savePCDFileBinary(out_file_name, cloud_);
}

void KittiBinary::SaveAsPLY(const std::string& out_file_name) {
  spdlog::debug("[KittiBinary] SaveAsPLY {} with size of {}", out_file_name, cloud_.size());
  pcl::io::savePLYFileBinary(out_file_name, cloud_);
}


}  // namespace kitti_binary_tools