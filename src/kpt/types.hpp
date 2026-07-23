#pragma once
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <optional>
#include <string>
#include <filesystem>

namespace kpt {

using PointT = pcl::PointXYZRGBI;
using PointCloudIRGB = pcl::PointCloud<PointT>;
using PointCloudIRGBPtr = PointCloudIRGB::Ptr;
using PointCloudIRGBConstPtr = PointCloudIRGB::ConstPtr;

enum class Format { Bin, PCD, PLY, XYZ, XYZI, XYZRGB, XYZRGBI };
enum class ColorBy { Intensity, RGB, Z, Label, None };
enum class View { Front, Right, Back, Left, Top, Bottom,
                  TopRightFront, TopLeftFront, BotRightFront, BotLeftFront };

}  // namespace kpt