#include "kpt/render/render.hpp"
#include "platform/native_file.hpp"
#include "platform/utf8_path.hpp"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <mutex>
#include <numbers>
#include <random>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

#include <limits>
namespace kpt {

namespace {

std::mutex image_commit_mutex;

std::string displayPath(const std::filesystem::path &path) {
  auto converted = platform::pathToUtf8(path);
  return converted ? std::move(converted).value() : "<invalid-native-path>";
}

void replaceImageFile(const std::filesystem::path &source,
                      const std::filesystem::path &destination) {
  auto replaced = platform::replaceFileAtomically(source, destination);
  if (!replaced)
    throw std::system_error(replaced.error().system_error,
                            replaced.error().message);
}

std::filesystem::path imageTemporaryPath(const std::filesystem::path &output) {
  static thread_local std::mt19937_64 generator(std::random_device{}());
  auto name = output.stem();
  name += ".kpt-tmp-" + std::to_string(generator());
  name += output.extension().native();
  return output.parent_path() / name;
}

class SimpleRenderer {
  int width;
  int height;
  float fx, fy;
  float cx, cy;

public:
  SimpleRenderer(int w, int h, float fov_degree) : width(w), height(h) {
    float fov = fov_degree * std::numbers::pi_v<float> / 180.0f;
    fx = width / (2.0f * std::tan(fov / 2.0f));
    fy = fx;
    cx = width / 2.0f;
    cy = height / 2.0f;
  }

  cv::Mat render(const PointCloudIRGBConstPtr &cloud,
                 const Eigen::Matrix4f &view_matrix,
                 bool with_z_buffer = true) {
    cv::Mat image = cv::Mat::zeros(height, width, CV_8UC3);
    cv::Mat z_buffer =
        cv::Mat::ones(height, width, CV_32F) *
        static_cast<double>(std::numeric_limits<float>::infinity());

    float point_size = 1.0f;
    if (cloud->size() < 100000)
      point_size = 2.0f;

    for (const auto &pt : cloud->points) {
      if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z))
        continue;

      Eigen::Vector4f p(pt.x, pt.y, pt.z, 1.0f);
      Eigen::Vector4f p_cam = view_matrix * p;

      if (p_cam[2] <= 0)
        continue;

      float x = (p_cam[0] * fx) / p_cam[2] + cx;
      float y = (p_cam[1] * fy) / p_cam[2] + cy;

      for (float dy = -point_size; dy <= point_size; dy++) {
        for (float dx = -point_size; dx <= point_size; dx++) {
          int pixel_x = static_cast<int>(x + dx);
          int pixel_y = static_cast<int>(y + dy);

          if (pixel_x < 0 || pixel_x >= width || pixel_y < 0 ||
              pixel_y >= height)
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
  explicit CloudBoundingBox(const PointCloudIRGBConstPtr &cloud) {
    if (!cloud || cloud->empty())
      return;
    min_pt = Eigen::Vector3f::Constant(std::numeric_limits<float>::infinity());
    max_pt = Eigen::Vector3f::Constant(-std::numeric_limits<float>::infinity());
    bool has_finite_point = false;
    for (const auto &point : cloud->points) {
      const Eigen::Vector3f position(point.x, point.y, point.z);
      if (!position.allFinite())
        continue;
      min_pt = min_pt.cwiseMin(position);
      max_pt = max_pt.cwiseMax(position);
      has_finite_point = true;
    }
    if (!has_finite_point) {
      min_pt.setZero();
      max_pt.setZero();
      return;
    }
    center = (min_pt + max_pt) / 2.0f;
    dimensions = max_pt - min_pt;
    max_dimension = std::max({dimensions.x(), dimensions.y(), dimensions.z()});
  }

  float getOptimalDistance(float theta, float phi, float fov_degree) const {
    float fov = fov_degree * std::numbers::pi_v<float> / 180.0f;

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
    float distance_for_height =
        projected_height / (2.0f * std::tan(fov / 2.0f));

    return 1.5f * std::max(distance_for_width, distance_for_height);
  }
};

Eigen::Matrix4f createViewMatrix(const Eigen::Vector3f &center, float theta,
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

  view << s.x(), s.y(), s.z(), -eye.dot(s), u.x(), u.y(), u.z(), -eye.dot(u),
      -f.x(), -f.y(), -f.z(), eye.dot(f), 0, 0, 0, 1;

  return view;
}

std::pair<float, float> viewAngles(View v) {
  switch (v) {
  case View::Front:
    return {0.0f, 0.0f};
  case View::Right:
    return {std::numbers::pi_v<float> / 2, 0.0f};
  case View::Back:
    return {std::numbers::pi_v<float>, 0.0f};
  case View::Left:
    return {-std::numbers::pi_v<float> / 2, 0.0f};
  case View::Top:
    return {0.0f, std::numbers::pi_v<float> / 4};
  case View::Bottom:
    return {0.0f, -std::numbers::pi_v<float> / 4};
  case View::TopRightFront:
    return {std::numbers::pi_v<float> / 4, std::numbers::pi_v<float> / 4};
  case View::TopLeftFront:
    return {-std::numbers::pi_v<float> / 4, std::numbers::pi_v<float> / 4};
  case View::BotRightFront:
    return {std::numbers::pi_v<float> / 4, -std::numbers::pi_v<float> / 4};
  case View::BotLeftFront:
    return {-std::numbers::pi_v<float> / 4, -std::numbers::pi_v<float> / 4};
  }
  return {0.0f, 0.0f};
}

} // namespace

std::string_view viewName(View v) {
  switch (v) {
  case View::Front:
    return "front";
  case View::Right:
    return "right";
  case View::Back:
    return "back";
  case View::Left:
    return "left";
  case View::Top:
    return "top";
  case View::Bottom:
    return "bottom";
  case View::TopRightFront:
    return "toprightfront";
  case View::TopLeftFront:
    return "topleftfront";
  case View::BotRightFront:
    return "botrightfront";
  case View::BotLeftFront:
    return "botleftfront";
  }
  return "unknown";
}

ImageWriteStatus writeImageAtomic(const std::filesystem::path &output,
                                  const cv::Mat &image, bool overwrite) {
  if (std::filesystem::exists(output) && !overwrite)
    return ImageWriteStatus::Skipped;
  if (!output.parent_path().empty())
    std::filesystem::create_directories(output.parent_path());

  const auto temporary = imageTemporaryPath(output);
  try {
    std::vector<unsigned char> encoded;
    auto extension = platform::pathToUtf8(output.extension());
    if (!extension)
      throw std::runtime_error("image extension is not valid UTF-8");
    if (!cv::imencode(extension.value(), image, encoded))
      throw std::runtime_error("failed to encode image: " +
                               displayPath(output));
    std::ofstream stream(temporary, std::ios::binary | std::ios::trunc);
    if (!stream)
      throw std::runtime_error("failed to open image: " + displayPath(output));
    stream.write(reinterpret_cast<const char *>(encoded.data()),
                 static_cast<std::streamsize>(encoded.size()));
    stream.flush();
    if (!stream)
      throw std::runtime_error("failed to write image: " + displayPath(output));
    stream.close();
    {
      std::lock_guard commit_lock(image_commit_mutex);
      if (std::filesystem::exists(output) && !overwrite) {
        std::error_code ignored;
        std::filesystem::remove(temporary, ignored);
        return ImageWriteStatus::Skipped;
      }
      replaceImageFile(temporary, output);
    }
  } catch (...) {
    std::error_code ignored;
    std::filesystem::remove(temporary, ignored);
    throw;
  }
  return ImageWriteStatus::Written;
}

std::vector<RenderResult> renderMultiView(const PointCloudIRGBConstPtr &cloud,
                                          const RenderOpts &opts) {
  if (!cloud)
    throw std::invalid_argument("renderMultiView requires a cloud");
  SimpleRenderer renderer(opts.width, opts.height, opts.fov);

  // Degenerate (empty) cloud: still produce correctly-sized black frames so
  // callers can rely on result count == opts.views.size().
  CloudBoundingBox bbox;
  if (!cloud->empty())
    bbox = CloudBoundingBox(cloud);
  Eigen::Vector3f center = bbox.center;

  std::vector<RenderResult> results;
  results.reserve(opts.views.size());

  for (const auto &v : opts.views) {
    auto [theta, phi] = viewAngles(v);
    float optimal_distance = 0.0f;
    if (!cloud->empty() && bbox.max_dimension > 0.0f) {
      optimal_distance = bbox.getOptimalDistance(theta, phi, opts.fov);
    }
    if (optimal_distance <= 0.0f || !std::isfinite(optimal_distance)) {
      // Empty cloud or zero-size cloud: pick a benign distance so the view
      // matrix stays well-formed (avoids normalizing a zero look vector).
      optimal_distance = 1.0f;
    }

    Eigen::Matrix4f view_matrix =
        createViewMatrix(center, theta, phi, optimal_distance);

    cv::Mat image = renderer.render(cloud, view_matrix);

    results.push_back({std::string(viewName(v)), std::move(image)});
  }

  return results;
}

} // namespace kpt
