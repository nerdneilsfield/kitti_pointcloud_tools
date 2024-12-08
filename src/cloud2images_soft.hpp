#pragma once

#include <string>

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace kitti_pointcloud_tools {
class SimpleRenderer {
private:
  int width;
  int height;
  float fx, fy; // 焦距
  float cx, cy; // 主点

public:
  SimpleRenderer(int w, int h, float fov_degree = 60.0f);

  cv::Mat render(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                 const Eigen::Matrix4f &view_matrix,
                 bool with_z_buffer = true);
};

// 计算点云的边界信息
struct CloudBoundingBox {
    Eigen::Vector3f min_pt;
    Eigen::Vector3f max_pt;
    Eigen::Vector3f center;
    Eigen::Vector3f dimensions;
    float max_dimension;
    
    CloudBoundingBox(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
    // 根据视角计算合适的相机距离
    float getOptimalDistance(float theta, float phi, float fov_degree) const;
};


// 生成视图矩阵
Eigen::Matrix4f createViewMatrix(const Eigen::Vector3f& center, float theta, float phi, float distance);   
/**
 * @brief Generate multiple view images from a point cloud.
 *
 * @param cloud the input point cloud
 * @param output_prefix the prefix of output image files
 * @param image_width the width of the output images
 * @param image_height the height of the output images
 *
 * The function generates 6 images from 6 different views: front, right, back,
 * left, top and bottom. The image files are named as
 * "output_prefix_view_0.png", "output_prefix_view_1.png", ...,
 * "output_prefix_view_5.png".
 */
void generateMultiViewImages(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                           const std::string& output_prefix,
                           int image_width,
                           int image_height,  
                           float fov_degree);

} // namespace kitti_pointcloud_tools