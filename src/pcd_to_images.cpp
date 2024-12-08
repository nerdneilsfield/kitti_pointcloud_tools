#include "cloud2images.hpp"

#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <opencv2/opencv.hpp>

using namespace kitti_pointcloud_tools;

int main(int argc, char **argv) {
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <pcd_file>" << std::endl;
    return 1;
  }

  std::string pcd_file = argv[1];

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::io::loadPCDFile(pcd_file, *cloud);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::copyPointCloud(*cloud, *cloud_xyz);



  Cloud2Images cloud2images(640, 480);
  auto images = cloud2images.convert(cloud_xyz);

  for (size_t i = 0; i < images.size(); ++i) {
    std::string filename = pcd_file + "_" + CLOUD2IMAGES_VIEW_NAMES.at(i) + ".png";
    cv::imwrite(filename, images[i]);
    std::cout << "Saved " << filename << std::endl;
  }

  std::cout << "Done." << std::endl;
  return 0;
}
