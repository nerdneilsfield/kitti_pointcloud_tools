#include "cloud2images_soft.hpp"

#include <iostream>

#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

using namespace kitti_pointcloud_tools;

int main(int argc, char **argv) {
  if (argc != 3) {
    std::cerr << "Usage: " << argv[0] << " <pcd_file> <output_prefix>"
              << std::endl;
    return 1;
  }

  std::string pcd_file = argv[1];
  std::string output_prefix = argv[2];

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZI>);
  pcl::io::loadPCDFile(pcd_file, *cloud);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_xyz(
      new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::copyPointCloud(*cloud, *cloud_xyz);

  for (int i = 0; i < cloud_xyz->size(); i++) {
    cloud_xyz->points[i].r = 0;
    cloud_xyz->points[i].g = 0;
    cloud_xyz->points[i].b = 255;
  }

  generateMultiViewImages(cloud_xyz, output_prefix, 640, 480, 120.0f);

  std::cout << "Done." << std::endl;
  return 0;
}
