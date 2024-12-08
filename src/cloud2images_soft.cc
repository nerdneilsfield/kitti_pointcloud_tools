#include "cloud2images_soft.hpp"

#include <limits>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>  // 已有
#include <pcl/common/transforms.h>
#include <pcl/common/centroid.h>

namespace kitti_pointcloud_tools {

SimpleRenderer::SimpleRenderer(int w, int h, float fov_degree)
    : width(w), height(h) {
  // 根据FOV计算焦距
  float fov = fov_degree * M_PI / 180.0f;
  fx = width / (2.0f * tan(fov / 2.0f));
  fy = fx;
  cx = width / 2.0f;
  cy = height / 2.0f;
}

cv::Mat SimpleRenderer::render(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                               const Eigen::Matrix4f &view_matrix,
                               bool with_z_buffer) {
  cv::Mat image = cv::Mat::zeros(height, width, CV_8UC3);
  cv::Mat z_buffer = cv::Mat::ones(height, width, CV_32F) *
                     std::numeric_limits<float>::infinity();

  // 计算点的大小（根据点云密度自适应）
  float point_size = 1.0f;
  int total_points = cloud->points.size();
  if (total_points < 100000) { // 如果点云稀疏，增加点的大小
    point_size = 2.0f;
  }

  for (const auto &pt : cloud->points) {
    if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z))
      continue;

    Eigen::Vector4f p(pt.x, pt.y, pt.z, 1.0f);
    Eigen::Vector4f p_cam = view_matrix * p;

    if (p_cam[2] <= 0)
      continue;

    float x = (p_cam[0] * fx) / p_cam[2] + cx;
    float y = (p_cam[1] * fy) / p_cam[2] + cy;

    // 使用点大小进行渲染
    for (float dy = -point_size; dy <= point_size; dy++) {
      for (float dx = -point_size; dx <= point_size; dx++) {
        int pixel_x = static_cast<int>(x + dx);
        int pixel_y = static_cast<int>(y + dy);

        if (pixel_x < 0 || pixel_x >= width || pixel_y < 0 || pixel_y >= height)
          continue;

        if (with_z_buffer) {
          if (p_cam[2] >= z_buffer.at<float>(pixel_y, pixel_x))
            continue;
          z_buffer.at<float>(pixel_y, pixel_x) = p_cam[2];
        }

        image.at<cv::Vec3b>(pixel_y, pixel_x) = cv::Vec3b(pt.b, pt.g, pt.r);
      }
    }
  }

  return image;
}

CloudBoundingBox::CloudBoundingBox(
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {
  pcl::PointXYZRGB min_point, max_point;
  pcl::getMinMax3D(*cloud, min_point, max_point);

  min_pt = Eigen::Vector3f(min_point.x, min_point.y, min_point.z);
  max_pt = Eigen::Vector3f(max_point.x, max_point.y, max_point.z);

  // 计算中心点
  center = (min_pt + max_pt) / 2.0f;

  // 计算各个维度的尺寸
  dimensions = max_pt - min_pt;

  // 计算最大维度
  max_dimension = std::max({dimensions.x(), dimensions.y(), dimensions.z()});
}

float CloudBoundingBox::getOptimalDistance(float theta, float phi,
                                           float fov_degree) const {
  float fov = fov_degree * M_PI / 180.0f;

  // 计算在当前视角下的投影尺寸
  float projected_width, projected_height;

  if (std::abs(std::cos(theta)) > std::abs(std::sin(theta))) {
    // 正面/背面视图
    projected_width = dimensions.y();
    projected_height = dimensions.z();
  } else {
    // 侧面视图
    projected_width = dimensions.x();
    projected_height = dimensions.z();
  }

  if (std::abs(std::sin(phi)) > 0.7) {
    // 俯视/仰视
    projected_width = dimensions.x();
    projected_height = dimensions.y();
  }

  // 根据FOV计算需要的最小距离，确保整个物体都在视野内
  float distance_for_width = projected_width / (2.0f * std::tan(fov / 2.0f));
  float distance_for_height = projected_height / (2.0f * std::tan(fov / 2.0f));

  // 添加一些边距（这里用1.5作为安全系数）
  return 1.5f * std::max(distance_for_width, distance_for_height);
}

Eigen::Matrix4f createViewMatrix(const Eigen::Vector3f &center, float theta,
                                 float phi, float distance) {
  Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

  // 计算相机位置（相对于点云中心）
  float x = distance * cos(phi) * cos(theta);
  float y = distance * cos(phi) * sin(theta);
  float z = distance * sin(phi);

  // 相机位置加上点云中心的偏移
  Eigen::Vector3f eye(x + center.x(), y + center.y(), z + center.z());
  Eigen::Vector3f look_at = center;
  Eigen::Vector3f up(0, 0, 1);

  Eigen::Vector3f f = (look_at - eye).normalized();
  Eigen::Vector3f s = f.cross(up).normalized();
  Eigen::Vector3f u = s.cross(f);

  view << s.x(), s.y(), s.z(), -eye.dot(s), u.x(), u.y(), u.z(), -eye.dot(u),
      -f.x(), -f.y(), -f.z(), eye.dot(f), 0, 0, 0, 1;

  return view;
}

void generateMultiViewImages(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                             const std::string &output_prefix, int image_width,
                             int image_height, float fov_degree) {
  SimpleRenderer renderer(image_width, image_height, fov_degree);

  // 计算点云的边界信息
  CloudBoundingBox bbox(cloud);

  // 定义不同的视角 (theta, phi) in radians
  // 修改视角定义：减小俯仰角度，添加更多视角
  std::vector<std::pair<float, float>> views = {
      {0, 0},                // 正面
      {M_PI / 2, 0},         // 右侧
      {M_PI, 0},             // 背面
      {-M_PI / 2, 0},        // 左侧
      {0, M_PI / 4},         // 俯视45度
      {0, -M_PI / 4},        // 仰视45度
      {M_PI / 4, M_PI / 4},  // 右前上45度
      {-M_PI / 4, M_PI / 4}, // 左前上45度
      {M_PI / 4, -M_PI / 4}, // 右前下45度
      {-M_PI / 4, -M_PI / 4} // 左前下45度
  };

  std::unordered_map<size_t, std::string> view_names = {
      {0, "正面"}, {1, "右侧"},   {2, "背面"},   {3, "左侧"},   {4, "俯视"},
      {5, "仰视"}, {6, "右前上"}, {7, "左前上"}, {8, "右前下"}, {9, "左前下"}};

  // 渲染每个视角
  for (size_t i = 0; i < views.size(); i++) {
    float theta = views[i].first;
    float phi = views[i].second;

    // 获取当前视角的最优相机距离
    float optimal_distance = bbox.getOptimalDistance(theta, phi, fov_degree);

    // 生成视图矩阵
    Eigen::Matrix4f view_matrix =
        createViewMatrix(bbox.center, theta, phi, optimal_distance);

    // 渲染图像
    cv::Mat image = renderer.render(cloud, view_matrix);

    // 保存图片
    std::string filename = output_prefix + "_" + view_names[i] + ".png";
    cv::imwrite(filename, image);
  }
}

} // namespace kitti_pointcloud_tools