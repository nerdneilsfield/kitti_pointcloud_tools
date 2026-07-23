#include "kpt/render/render.hpp"

#include <limits>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>

namespace kpt {

namespace {

class SimpleRenderer {
  int width;
  int height;
  float fx, fy;
  float cx, cy;

 public:
  SimpleRenderer(int w, int h, float fov_degree) : width(w), height(h) {
    float fov = fov_degree * static_cast<float>(M_PI) / 180.0f;
    fx = width / (2.0f * std::tan(fov / 2.0f));
    fy = fx;
    cx = width / 2.0f;
    cy = height / 2.0f;
  }

  cv::Mat render(const PointCloudIRGBConstPtr& cloud,
                 const Eigen::Matrix4f& view_matrix,
                 bool with_z_buffer = true) {
    cv::Mat image = cv::Mat::zeros(height, width, CV_8UC3);
    cv::Mat z_buffer = cv::Mat::ones(height, width, CV_32F) *
                       std::numeric_limits<float>::infinity();

    float point_size = 1.0f;
    if (cloud->size() < 100000) point_size = 2.0f;

    for (const auto& pt : cloud->points) {
      if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z))
        continue;

      Eigen::Vector4f p(pt.x, pt.y, pt.z, 1.0f);
      Eigen::Vector4f p_cam = view_matrix * p;

      if (p_cam[2] <= 0) continue;

      float x = (p_cam[0] * fx) / p_cam[2] + cx;
      float y = (p_cam[1] * fy) / p_cam[2] + cy;

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
};

struct CloudBoundingBox {
  Eigen::Vector3f min_pt = Eigen::Vector3f::Zero();
  Eigen::Vector3f max_pt = Eigen::Vector3f::Zero();
  Eigen::Vector3f center = Eigen::Vector3f::Zero();
  Eigen::Vector3f dimensions = Eigen::Vector3f::Zero();
  float max_dimension = 0.0f;

  CloudBoundingBox() = default;
  explicit CloudBoundingBox(const PointCloudIRGBConstPtr& cloud) {
    PointT min_point, max_point;
    pcl::getMinMax3D(*cloud, min_point, max_point);

    min_pt = Eigen::Vector3f(min_point.x, min_point.y, min_point.z);
    max_pt = Eigen::Vector3f(max_point.x, max_point.y, max_point.z);
    center = (min_pt + max_pt) / 2.0f;
    dimensions = max_pt - min_pt;
    max_dimension = std::max({dimensions.x(), dimensions.y(), dimensions.z()});
  }

  float getOptimalDistance(float theta, float phi, float fov_degree) const {
    float fov = fov_degree * static_cast<float>(M_PI) / 180.0f;

    float projected_width, projected_height;

    if (std::abs(std::cos(theta)) > std::abs(std::sin(theta))) {
      projected_width = dimensions.y();
      projected_height = dimensions.z();
    } else {
      projected_width = dimensions.x();
      projected_height = dimensions.z();
    }

    if (std::abs(std::sin(phi)) > 0.7f) {
      projected_width = dimensions.x();
      projected_height = dimensions.y();
    }

    float distance_for_width = projected_width / (2.0f * std::tan(fov / 2.0f));
    float distance_for_height = projected_height / (2.0f * std::tan(fov / 2.0f));

    return 1.5f * std::max(distance_for_width, distance_for_height);
  }
};

Eigen::Matrix4f createViewMatrix(const Eigen::Vector3f& center, float theta,
                                float phi, float distance) {
  Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

  float x = distance * std::cos(phi) * std::cos(theta);
  float y = distance * std::cos(phi) * std::sin(theta);
  float z = distance * std::sin(phi);

  Eigen::Vector3f eye(x + center.x(), y + center.y(), z + center.z());
  Eigen::Vector3f look_at = center;
  Eigen::Vector3f up(0, 0, 1);

  Eigen::Vector3f f = (look_at - eye).normalized();
  Eigen::Vector3f s = f.cross(up).normalized();
  Eigen::Vector3f u = s.cross(f);

  view << s.x(), s.y(), s.z(), -eye.dot(s),
          u.x(), u.y(), u.z(), -eye.dot(u),
          -f.x(), -f.y(), -f.z(), eye.dot(f),
          0, 0, 0, 1;

  return view;
}

std::pair<float, float> viewAngles(View v) {
  switch (v) {
    case View::Front:          return {0.0f, 0.0f};
    case View::Right:          return {static_cast<float>(M_PI) / 2, 0.0f};
    case View::Back:           return {static_cast<float>(M_PI), 0.0f};
    case View::Left:           return {-static_cast<float>(M_PI) / 2, 0.0f};
    case View::Top:            return {0.0f, static_cast<float>(M_PI) / 4};
    case View::Bottom:         return {0.0f, -static_cast<float>(M_PI) / 4};
    case View::TopRightFront:  return {static_cast<float>(M_PI) / 4, static_cast<float>(M_PI) / 4};
    case View::TopLeftFront:   return {-static_cast<float>(M_PI) / 4, static_cast<float>(M_PI) / 4};
    case View::BotRightFront:  return {static_cast<float>(M_PI) / 4, -static_cast<float>(M_PI) / 4};
    case View::BotLeftFront:   return {-static_cast<float>(M_PI) / 4, -static_cast<float>(M_PI) / 4};
  }
  return {0.0f, 0.0f};
}

std::string viewName(View v) {
  switch (v) {
    case View::Front:          return "正面";
    case View::Right:          return "右侧";
    case View::Back:           return "背面";
    case View::Left:           return "左侧";
    case View::Top:            return "俯视";
    case View::Bottom:         return "仰视";
    case View::TopRightFront:  return "右前上";
    case View::TopLeftFront:   return "左前上";
    case View::BotRightFront:  return "右前下";
    case View::BotLeftFront:   return "左前下";
  }
  return "unknown";
}

}  // namespace

std::vector<RenderResult> renderMultiView(const PointCloudIRGBConstPtr& cloud,
                                          const RenderOpts& opts) {
  SimpleRenderer renderer(opts.width, opts.height, opts.fov);

  // Degenerate (empty) cloud: still produce correctly-sized black frames so
  // callers can rely on result count == opts.views.size().
  CloudBoundingBox bbox;
  if (!cloud->empty()) bbox = CloudBoundingBox(cloud);
  Eigen::Vector3f center = bbox.center;

  std::vector<RenderResult> results;
  results.reserve(opts.views.size());

  for (const auto& v : opts.views) {
    auto [theta, phi] = viewAngles(v);
    float optimal_distance = 0.0f;
    if (!cloud->empty() && bbox.max_dimension > 0.0f) {
      optimal_distance = bbox.getOptimalDistance(theta, phi, opts.fov);
    } else if (cloud->empty()) {
      // Pick a benign distance so the view matrix is still well-formed.
      optimal_distance = 1.0f;
    }

    Eigen::Matrix4f view_matrix =
        createViewMatrix(center, theta, phi, optimal_distance);

    cv::Mat image = renderer.render(cloud, view_matrix);

    results.push_back({viewName(v), std::move(image)});
  }

  return results;
}

}  // namespace kpt
