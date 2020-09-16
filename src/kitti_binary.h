/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   kitti_binary.h                                     :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: dengqi <dengqi935@outl>                    +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2020/09/07 10:29:11 by dengqi            #+#    #+#             */
/*   Updated: 2020/09/16 21:45:30 by dengqi           ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#ifndef __KITTI_TOOLS_KITTI_BINARY_H_
#define __KITTI_TOOLS_KITTI_BINARY_H_
#pragma once

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>

#include <cstdint>
#include <fstream>
#include <string>

namespace kitti_binary_tools {

using PointType = pcl::PointXYZI;
using PointCloud = pcl::PointCloud<PointType>;
using PointCloudPtr = pcl::PointCloud<PointType>::Ptr;

struct SimplePoint {
  float x;
  float y;
  float z;
  float intensity;
};

class KittiBinary {
 private:
  int32_t size_ = 0;
  PointCloud cloud_;

  std::string file_name_;

 public:
  KittiBinary();

  KittiBinary(const std::string& filename);

  void DumpToFile(const PointCloud& point_cloud, const std::string& file_name);

  void LoadFromFile(const std::string& filename);

  void SaveAsPCD(const std::string& out_file_name);

  void SaveAsPLY(const std::string& out_file_name);
};

}  // namespace kitti_binary_tools

#endif  // !__KITTI_TOOLS_KITTI_BINARY_H_
