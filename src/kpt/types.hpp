#pragma once

#include "kpt/core_types.hpp"

#include <pcl/impl/point_types.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace kpt {

struct EIGEN_ALIGN16 PointXYZRGBI {
  PCL_ADD_POINT4D;
  PCL_ADD_RGB;
  float intensity;
  PCL_MAKE_ALIGNED_OPERATOR_NEW
};

using PointT = PointXYZRGBI;
using PointCloudIRGB = pcl::PointCloud<PointT>;
using PointCloudIRGBPtr = PointCloudIRGB::Ptr;
using PointCloudIRGBConstPtr = PointCloudIRGB::ConstPtr;

} // namespace kpt

POINT_CLOUD_REGISTER_POINT_STRUCT(
    kpt::PointXYZRGBI,
    (float, x, x)(float, y, y)(float, z, z)(float, rgb, rgb)(float, intensity,
                                                             intensity))
