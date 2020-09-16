/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   kitti_binary.cc                                    :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: dengqi <dengqi935@outl>                    +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2020/09/07 10:42:18 by dengqi            #+#    #+#             */
/*   Updated: 2020/09/16 22:12:30 by dengqi           ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include "kitti_binary.h"

#include <spdlog/spdlog.h>

namespace kitti_binary_tools {

KittiBinary::KittiBinary() {}

KittiBinary::KittiBinary(const std::string& filename) {
  spdlog::debug("Call [KittiBinary] Constructor with  File {}", filename);
  LoadFromFile(filename);
}

void static KittiBinary::DumpToFile(const PointCloud& point_cloud,
                             const std::string& file_name) {
  size_t size = point_cloud.size();
  SimplePoint points[size];
  for (size_t i = 0; i < size; i++) {
    points[i].x = point_cloud.points[i].x;
    points[i].y = point_cloud.points[i].y;
    points[i].z = point_cloud.points[i].z;
    points[i].intensity = point_cloud.points[i].intensity;
  }
  char* buffer = reinterpret_cast<char*>(points);
  std::ofstream ofs(file_name.c_str(), std::ios::binary);
  ofs << buffer;
  ofs.close();
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
  spdlog::debug("[KittiBinary] SaveAsPCD {} with size of {}", out_file_name,
                cloud_.size());
  pcl::io::savePCDFileBinary(out_file_name, cloud_);
}

void KittiBinary::SaveAsPLY(const std::string& out_file_name) {
  spdlog::debug("[KittiBinary] SaveAsPLY {} with size of {}", out_file_name,
                cloud_.size());
  pcl::io::savePLYFileBinary(out_file_name, cloud_);
}

}  // namespace kitti_binary_tools
