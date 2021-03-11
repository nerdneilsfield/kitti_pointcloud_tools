#ifndef __TYPES_H_
#define __TYPES_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

typedef pcl::PointXYZI PointT;
typedef pcl::PointCloud<pcl::PointXYZI> PointCloudT;
typedef pcl::PointCloud<pcl::PointXYZI>::Ptr PointCloudPtrT;
typedef pcl::PointCloud<pcl::PointXYZI>::ConstPtr PointCloudConstPtrT;
typedef pcl::PointXYZRGB PointRGBT;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointRGBCloudT;
typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointRGBCloudPtrT;
typedef pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr PointRGBCloudConstPtrT;

#endif  // __TYPES_H_
