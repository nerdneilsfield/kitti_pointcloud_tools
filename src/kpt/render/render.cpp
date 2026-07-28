#include "kpt/render/render.hpp"
#include "platform/native_file.hpp"
#include "platform/utf8_path.hpp"

#define STB_IMAGE_WRITE_STATIC
#define STBI_WRITE_NO_STDIO
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include <stb_image_write.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <limits>
#include <memory>
#include <mutex>
#include <numbers>
#include <optional>
#include <random>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

#include <Eigen/Dense>

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

struct TemporaryImageFile {
  std::filesystem::path path;
  std::unique_ptr<platform::NativeOutputFile> output;
};

TemporaryImageFile openImageTemporaryFile(const std::filesystem::path &output) {
  static thread_local std::mt19937_64 generator(std::random_device{}());
  for (int attempt = 0; attempt < 64; ++attempt) {
    auto name = output.stem();
    name += ".kpt-tmp-" + std::to_string(generator());
    name += output.extension().native();
    auto candidate = output.parent_path() / name;
    auto opened = platform::openNativeOutputExclusively(candidate);
    if (!opened)
      throw std::system_error(opened.error().system_error,
                              opened.error().message);
    auto output_file = std::move(opened).value();
    if (output_file)
      return {std::move(candidate), std::move(output_file)};
  }
  throw std::runtime_error("cannot reserve unique temporary image: " +
                           displayPath(output));
}

class SimpleRenderer {
  int width;
  int height;
  float fx, fy;
  float cx, cy;

public:
  SimpleRenderer(int w, int h, float fov_degree) : width(w), height(h) {
    if (width <= 0 || height <= 0)
      throw std::invalid_argument("render dimensions must be positive");
    if (!std::isfinite(fov_degree) || fov_degree <= 0.0F ||
        fov_degree >= 180.0F)
      throw std::invalid_argument("render FOV must be in (0, 180)");
    float fov = fov_degree * std::numbers::pi_v<float> / 180.0f;
    fx = width / (2.0f * std::tan(fov / 2.0f));
    fy = fx;
    cx = width / 2.0f;
    cy = height / 2.0f;
  }

  ImageRGB8 render(const PointCloudIRGBConstPtr &cloud,
                   const Eigen::Matrix4f &view_matrix,
                   bool with_z_buffer = true) {
    ImageRGB8 image(width, height);
    const auto pixel_count =
        static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
    std::vector<float> z_buffer(pixel_count,
                                std::numeric_limits<float>::infinity());

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
      if (!std::isfinite(x) || !std::isfinite(y))
        continue;

      for (float dy = -point_size; dy <= point_size; dy++) {
        for (float dx = -point_size; dx <= point_size; dx++) {
          const float projected_x = x + dx;
          const float projected_y = y + dy;
          if (projected_x < 0.0F || projected_x >= static_cast<float>(width) ||
              projected_y < 0.0F || projected_y >= static_cast<float>(height)) {
            continue;
          }
          const int pixel_x = static_cast<int>(projected_x);
          const int pixel_y = static_cast<int>(projected_y);

          if (with_z_buffer) {
            const auto pixel_index = static_cast<std::size_t>(pixel_y) *
                                         static_cast<std::size_t>(width) +
                                     static_cast<std::size_t>(pixel_x);
            if (p_cam[2] >= z_buffer[pixel_index])
              continue;
            z_buffer[pixel_index] = p_cam[2];
          }

          auto *pixel = image.pixel(pixel_x, pixel_y);
          pixel[0] = pt.r;
          pixel[1] = pt.g;
          pixel[2] = pt.b;
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
    Eigen::Vector3d center64;
    Eigen::Vector3d dimensions64;
    for (Eigen::Index axis = 0; axis < 3; ++axis) {
      const double minimum = static_cast<double>(min_pt[axis]);
      const double maximum = static_cast<double>(max_pt[axis]);
      dimensions64[axis] = maximum - minimum;
      center64[axis] = minimum + dimensions64[axis] / 2.0;
    }
    if (!center64.allFinite() || !dimensions64.allFinite() ||
        (dimensions64.array() >
         static_cast<double>(std::numeric_limits<float>::max()))
            .any()) {
      throw std::overflow_error("cloud bounds exceed renderer range");
    }
    center = center64.cast<float>();
    dimensions = dimensions64.cast<float>();
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

struct PngSink {
  platform::NativeOutputFile *output = nullptr;
  std::optional<platform::PlatformError> error;
};

void writePngChunk(void *context, void *data, int size) {
  auto &sink = *static_cast<PngSink *>(context);
  if (sink.error || size < 0)
    return;
  auto written = sink.output->write({static_cast<const std::uint8_t *>(data),
                                     static_cast<std::size_t>(size)});
  if (!written)
    sink.error = std::move(written).error();
}

void validateImageView(ImageView image) {
  if (image.width <= 0 || image.height <= 0 || image.stride_bytes <= 0)
    throw std::invalid_argument("image dimensions and stride must be positive");
  if (image.width > std::numeric_limits<int>::max() / 3 ||
      image.stride_bytes < image.width * 3)
    throw std::invalid_argument("image stride is too small");
  const auto stride = static_cast<std::size_t>(image.stride_bytes);
  const auto rows_before_last = static_cast<std::size_t>(image.height - 1);
  if (rows_before_last > (std::numeric_limits<std::size_t>::max() -
                          static_cast<std::size_t>(image.width) * 3U) /
                             stride)
    throw std::length_error("image view is too large");
  const auto required =
      rows_before_last * stride + static_cast<std::size_t>(image.width) * 3U;
  if (image.pixels.size() < required)
    throw std::invalid_argument("image view pixel buffer is too small");

  // stb_image_write buffers filtered pixels, zlib output and the final PNG,
  // all with int lengths. Bound its exact worst-case arithmetic and total
  // working set to avoid signed overflow or hostile snapshot OOM.
  constexpr std::uint64_t kMaxPngPixels =
      std::uint64_t{32} * std::uint64_t{1024} * std::uint64_t{1024};
  const auto pixels = static_cast<std::uint64_t>(image.width) *
                      static_cast<std::uint64_t>(image.height);
  const auto filtered_row =
      static_cast<std::uint64_t>(image.width) * std::uint64_t{3} +
      std::uint64_t{1};
  const auto data_length =
      filtered_row * static_cast<std::uint64_t>(image.height);
  const auto blocks =
      (data_length + std::uint64_t{32766}) / std::uint64_t{32767};
  const auto zlib_length =
      data_length + std::uint64_t{2} + blocks * std::uint64_t{5};
  const auto png_length = zlib_length + std::uint64_t{57};
  if (pixels > kMaxPngPixels ||
      png_length > static_cast<std::uint64_t>(std::numeric_limits<int>::max()))
    throw std::length_error("PNG image is too large for stb_image_write");
}

bool hasPngExtension(const std::filesystem::path &path) {
  auto extension = path.extension().native();
  for (auto &character : extension) {
    if (character >= 'A' && character <= 'Z') {
      character += 'a' - 'A';
    }
  }
  return extension == std::filesystem::path(".png").native();
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
                                  ImageView image, bool overwrite) {
  validateImageView(image);
  if (!hasPngExtension(output))
    throw std::invalid_argument("only PNG image output is supported: " +
                                displayPath(output));
  if (std::filesystem::exists(output) && !overwrite)
    return ImageWriteStatus::Skipped;
  if (!output.parent_path().empty())
    std::filesystem::create_directories(output.parent_path());

  auto temporary = openImageTemporaryFile(output);
  try {
    PngSink sink{temporary.output.get(), std::nullopt};
    const int encoded =
        stbi_write_png_to_func(writePngChunk, &sink, image.width, image.height,
                               3, image.pixels.data(), image.stride_bytes);
    if (encoded == 0 || sink.error)
      throw std::runtime_error("failed to encode image: " +
                               displayPath(output));
    auto finished = temporary.output->finish();
    if (!finished)
      throw std::system_error(finished.error().system_error,
                              finished.error().message);
    temporary.output.reset();
    {
      std::lock_guard commit_lock(image_commit_mutex);
      if (!overwrite) {
        auto published =
            platform::moveFileAtomicallyIfAbsent(temporary.path, output);
        if (!published)
          throw std::system_error(published.error().system_error,
                                  published.error().message);
        if (!published.value()) {
          std::error_code ignored;
          std::filesystem::remove(temporary.path, ignored);
          return ImageWriteStatus::Skipped;
        }
      } else {
        replaceImageFile(temporary.path, output);
      }
    }
  } catch (...) {
    temporary.output.reset();
    std::error_code ignored;
    std::filesystem::remove(temporary.path, ignored);
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
    if (!view_matrix.allFinite())
      throw std::overflow_error("cloud camera exceeds renderer range");

    ImageRGB8 image = renderer.render(cloud, view_matrix);

    results.push_back({std::string(viewName(v)), std::move(image)});
  }

  return results;
}

} // namespace kpt
