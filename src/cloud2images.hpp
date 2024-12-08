#pragma once

#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

namespace kitti_pointcloud_tools {

const std::unordered_map<int, std::string> CLOUD2IMAGES_VIEW_NAMES = {
    {0, "正面视图"}, {1, "右侧视图"}, {2, "背面视图"}, {3, "左侧视图"},
    {4, "俯视图"},   {5, "仰视图"}};

class Cloud2Images {
public:
  Cloud2Images(uint32_t width, uint32_t height)
      : width_(width), height_(height) {
    viewer_ = pcl::visualization::PCLVisualizer::Ptr(
        new pcl::visualization::PCLVisualizer("Cloud2Images"));
    viewer_->setBackgroundColor(0, 0, 0);
  }
  ~Cloud2Images() = default;

  auto convert(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
      -> std::vector<cv::Mat>;

private:
  uint32_t width_;
  uint32_t height_;

  pcl::visualization::PCLVisualizer::Ptr viewer_;
};

} // namespace kitti_pointcloud_tools