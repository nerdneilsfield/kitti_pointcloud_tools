#pragma once
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/impl/point_types.hpp>
#include <optional>
#include <string>
#include <filesystem>

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

enum class Format { Bin, PCD, PLY, XYZ, XYZI, XYZRGB, XYZRGBI };
enum class ColorBy { Intensity, RGB, Z, Label, None };
enum class View { Front, Right, Back, Left, Top, Bottom,
                  TopRightFront, TopLeftFront, BotRightFront, BotLeftFront };

}  // namespace kpt

POINT_CLOUD_REGISTER_POINT_STRUCT(
    kpt::PointXYZRGBI,
    (float, x, x)(float, y, y)(float, z, z)
    (float, rgb, rgb)
    (float, intensity, intensity))