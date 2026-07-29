#include "kpt/render/render.hpp"
#include "kpt/cancellation.hpp"
#include "kpt/render/detail/stb_png.hpp"
#include "kpt/render/png_limits.hpp"
#include "platform/native_file.hpp"
#include "platform/utf8_path.hpp"

#include <algorithm>
#include <array>
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
#include <spdlog/spdlog.h>

namespace kpt {

namespace {

std::string displayPath(const std::filesystem::path &path) {
  auto converted = platform::pathToUtf8(path);
  return converted ? std::move(converted).value() : "<invalid-native-path>";
}

struct TemporaryImageFile {
  std::unique_ptr<platform::NativeOutputFile> output;
};

struct RenderColorMapping {
  RenderColorMode mode = RenderColorMode::Solid;
  float intensity_min = 0.0F;
  float intensity_max = 1.0F;
  float z_min = 0.0F;
  float z_max = 1.0F;

  [[nodiscard]] static std::uint8_t grayscale(float value, float minimum,
                                              float maximum) {
    const float range = maximum - minimum;
    const float normalized =
        range > std::numeric_limits<float>::epsilon()
            ? std::clamp((value - minimum) / range, 0.0F, 1.0F)
            : 0.8F;
    return static_cast<std::uint8_t>(32.0F + normalized * 223.0F);
  }

  [[nodiscard]] static std::array<std::uint8_t, 3>
  heightColor(float value, float minimum, float maximum) {
    const float range = maximum - minimum;
    const float normalized =
        range > std::numeric_limits<float>::epsilon()
            ? std::clamp((value - minimum) / range, 0.0F, 1.0F)
            : 0.5F;
    if (normalized < 0.5F) {
      const float blend = normalized * 2.0F;
      return {
          static_cast<std::uint8_t>(32.0F * (1.0F - blend) + 64.0F * blend),
          static_cast<std::uint8_t>(80.0F * (1.0F - blend) + 255.0F * blend),
          static_cast<std::uint8_t>(255.0F * (1.0F - blend) + 96.0F * blend)};
    }
    const float blend = (normalized - 0.5F) * 2.0F;
    return {static_cast<std::uint8_t>(64.0F * (1.0F - blend) + 255.0F * blend),
            static_cast<std::uint8_t>(255.0F * (1.0F - blend) + 64.0F * blend),
            static_cast<std::uint8_t>(96.0F * (1.0F - blend) + 32.0F * blend)};
  }

  [[nodiscard]] std::array<std::uint8_t, 3> color(const PointT &point) const {
    switch (mode) {
    case RenderColorMode::RGB:
      return {point.r, point.g, point.b};
    case RenderColorMode::Intensity: {
      if (!std::isfinite(point.intensity))
        return {220, 220, 220};
      const auto value =
          grayscale(point.intensity, intensity_min, intensity_max);
      return {value, value, value};
    }
    case RenderColorMode::Z:
      return heightColor(point.z, z_min, z_max);
    case RenderColorMode::Solid:
      return {220, 220, 220};
    case RenderColorMode::Auto:
      break;
    }
    return {220, 220, 220};
  }
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
      return {std::move(output_file)};
  }
  throw std::runtime_error("cannot reserve unique temporary image: " +
                           displayPath(output));
}

class SimpleRenderer {
  int width;
  int height;
  float cx, cy;

public:
  SimpleRenderer(int w, int h) : width(w), height(h) {
    if (width <= 0 || height <= 0)
      throw std::invalid_argument("render dimensions must be positive");
    if (!pngDimensionsSupported(width, height))
      throw std::length_error("render dimensions exceed the 32 Mi pixel limit");
    cx = width / 2.0f;
    cy = height / 2.0f;
  }

  template <typename IncludePoint, typename ProjectPoint>
  ImageRGB8
  render(const PointCloudIRGBConstPtr &cloud, std::size_t retained_points,
         const RenderColorMapping &color_mapping, IncludePoint include_point,
         ProjectPoint project_point, std::stop_token stop) {
    if (stop.stop_requested())
      throw OperationCancelled();
    ImageRGB8 image(width, height);
    if (stop.stop_requested())
      throw OperationCancelled();
    const auto pixel_count =
        static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
    std::vector<float> z_buffer(pixel_count,
                                std::numeric_limits<float>::infinity());

    const float point_size = retained_points < 100000U ? 2.0F : 1.0F;

    std::size_t point_index = 0;
    for (const auto &pt : cloud->points) {
      if ((point_index++ % 4096U) == 0U && stop.stop_requested())
        throw OperationCancelled();
      if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z))
        continue;
      if (!include_point(pt))
        continue;

      float x = 0.0F;
      float y = 0.0F;
      float depth = 0.0F;
      if (!project_point(pt, x, y, depth) || !std::isfinite(x) ||
          !std::isfinite(y) || !std::isfinite(depth) || depth <= 0.0F)
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

          const auto pixel_index = static_cast<std::size_t>(pixel_y) *
                                       static_cast<std::size_t>(width) +
                                   static_cast<std::size_t>(pixel_x);
          if (depth >= z_buffer[pixel_index])
            continue;
          z_buffer[pixel_index] = depth;

          const auto color = color_mapping.color(pt);
          auto *pixel = image.pixel(pixel_x, pixel_y);
          pixel[0] = color[0];
          pixel[1] = color[1];
          pixel[2] = color[2];
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
  bool has_visible_rgb = false;
  bool has_finite_intensity = false;
  float intensity_min = std::numeric_limits<float>::infinity();
  float intensity_max = -std::numeric_limits<float>::infinity();
  std::size_t points = 0;

  void include(const PointT &point, bool include_color_statistics) {
    const Eigen::Vector3f position(point.x, point.y, point.z);
    if (points == 0U) {
      min_pt = position;
      max_pt = position;
    } else {
      min_pt = min_pt.cwiseMin(position);
      max_pt = max_pt.cwiseMax(position);
    }
    ++points;
    if (include_color_statistics) {
      has_visible_rgb =
          has_visible_rgb || point.r != 0 || point.g != 0 || point.b != 0;
      if (std::isfinite(point.intensity)) {
        intensity_min = std::min(intensity_min, point.intensity);
        intensity_max = std::max(intensity_max, point.intensity);
        has_finite_intensity = true;
      }
    }
  }

  void finish() {
    if (points == 0U) {
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

  [[nodiscard]] RenderColorMapping
  colorMapping(RenderColorMode requested) const {
    RenderColorMode resolved = requested;
    if (resolved == RenderColorMode::Auto) {
      resolved = has_visible_rgb
                     ? RenderColorMode::RGB
                     : (has_finite_intensity ? RenderColorMode::Intensity
                                             : RenderColorMode::Solid);
    }
    return {resolved, intensity_min, intensity_max, min_pt.z(), max_pt.z()};
  }
};

struct CloudAnalysis {
  CloudBoundingBox input;
  CloudBoundingBox framed;
  Eigen::Vector3f trim_min = Eigen::Vector3f::Zero();
  Eigen::Vector3f trim_max = Eigen::Vector3f::Zero();

  [[nodiscard]] bool contains(const PointT &point) const {
    const Eigen::Vector3f position(point.x, point.y, point.z);
    return position.allFinite() &&
           (position.array() >= trim_min.array()).all() &&
           (position.array() <= trim_max.array()).all();
  }

  [[nodiscard]] RenderCloudStats stats() const {
    return {
        {input.dimensions.x(), input.dimensions.y(), input.dimensions.z()},
        {framed.dimensions.x(), framed.dimensions.y(), framed.dimensions.z()},
        input.points,
        framed.points};
  }
};

CloudAnalysis analyzeCloud(const PointCloudIRGBConstPtr &cloud,
                           float trim_percent, std::stop_token stop) {
  CloudAnalysis analysis;
  std::array<std::vector<float>, 3> coordinates;
  for (auto &axis : coordinates)
    axis.reserve(cloud->size());

  std::size_t point_index = 0;
  for (const auto &point : cloud->points) {
    if ((point_index++ % 4096U) == 0U && stop.stop_requested())
      throw OperationCancelled();
    const Eigen::Vector3f position(point.x, point.y, point.z);
    if (!position.allFinite())
      continue;
    analysis.input.include(point, false);
    coordinates[0].push_back(point.x);
    coordinates[1].push_back(point.y);
    coordinates[2].push_back(point.z);
  }
  analysis.input.finish();
  if (analysis.input.points == 0U)
    return analysis;

  const auto trim_count = static_cast<std::size_t>(
      std::floor(static_cast<double>(analysis.input.points) *
                 static_cast<double>(trim_percent) / 100.0));
  for (Eigen::Index axis = 0; axis < 3; ++axis) {
    if (stop.stop_requested())
      throw OperationCancelled();
    auto &values = coordinates[static_cast<std::size_t>(axis)];
    std::sort(values.begin(), values.end());
    analysis.trim_min[axis] = values[trim_count];
    analysis.trim_max[axis] = values[values.size() - 1U - trim_count];
  }

  point_index = 0;
  for (const auto &point : cloud->points) {
    if ((point_index++ % 4096U) == 0U && stop.stop_requested())
      throw OperationCancelled();
    if (analysis.contains(point))
      analysis.framed.include(point, true);
  }

  if (analysis.framed.points == 0U) {
    analysis.trim_min = analysis.input.min_pt;
    analysis.trim_max = analysis.input.max_pt;
    point_index = 0;
    for (const auto &point : cloud->points) {
      if ((point_index++ % 4096U) == 0U && stop.stop_requested())
        throw OperationCancelled();
      if (analysis.contains(point))
        analysis.framed.include(point, true);
    }
  }
  analysis.framed.finish();
  return analysis;
}

struct CameraBasis {
  Eigen::Vector3f right;
  Eigen::Vector3f up;
  Eigen::Vector3f back;
};

CameraBasis cameraBasis(float theta, float phi) {
  const Eigen::Vector3f back(std::cos(phi) * std::cos(theta),
                             std::cos(phi) * std::sin(theta), std::sin(phi));
  const Eigen::Vector3f forward = -back;
  const Eigen::Vector3f up_hint =
      std::abs(forward.dot(Eigen::Vector3f::UnitZ())) > 0.99F
          ? Eigen::Vector3f::UnitY()
          : Eigen::Vector3f::UnitZ();
  const Eigen::Vector3f right = forward.cross(up_hint).normalized();
  return {right, right.cross(forward).normalized(), back};
}

Eigen::Matrix4f createViewMatrix(const Eigen::Vector3f &center,
                                 const CameraBasis &basis, float distance) {
  const Eigen::Vector3f eye = center + basis.back * distance;
  Eigen::Matrix4f view = Eigen::Matrix4f::Identity();
  view.block<1, 3>(0, 0) = basis.right.transpose();
  view.block<1, 3>(1, 0) = basis.up.transpose();
  view.block<1, 3>(2, 0) = basis.back.transpose();
  view(0, 3) = -eye.dot(basis.right);
  view(1, 3) = -eye.dot(basis.up);
  view(2, 3) = -eye.dot(basis.back);

  return view;
}

float optimalDistance(const CloudBoundingBox &bbox, const CameraBasis &basis,
                      int width, int height, float fov_degree) {
  const float tangent_horizontal =
      std::tan(fov_degree * std::numbers::pi_v<float> / 360.0F);
  const float tangent_vertical = tangent_horizontal *
                                 static_cast<float>(height) /
                                 static_cast<float>(width);
  float required = 0.0F;
  for (int mask = 0; mask < 8; ++mask) {
    const Eigen::Vector3f corner{
        (mask & 1) != 0 ? bbox.max_pt.x() : bbox.min_pt.x(),
        (mask & 2) != 0 ? bbox.max_pt.y() : bbox.min_pt.y(),
        (mask & 4) != 0 ? bbox.max_pt.z() : bbox.min_pt.z()};
    const Eigen::Vector3f offset = corner - bbox.center;
    const float camera_x = basis.right.dot(offset);
    const float camera_y = basis.up.dot(offset);
    const float camera_z = basis.back.dot(offset);
    required =
        std::max(required, camera_z + std::abs(camera_x) / tangent_horizontal);
    required =
        std::max(required, camera_z + std::abs(camera_y) / tangent_vertical);
  }
  return required * 1.05F;
}

struct OrthographicFrame {
  float scale = 1.0F;
  float projected_center_x = 0.0F;
  float projected_center_y = 0.0F;
  float nearest_back = 0.0F;
};

OrthographicFrame orthographicFrame(const CloudBoundingBox &bbox,
                                    const CameraBasis &basis, int width,
                                    int height) {
  float minimum_x = std::numeric_limits<float>::infinity();
  float maximum_x = -std::numeric_limits<float>::infinity();
  float minimum_y = std::numeric_limits<float>::infinity();
  float maximum_y = -std::numeric_limits<float>::infinity();
  float maximum_back = -std::numeric_limits<float>::infinity();
  for (int mask = 0; mask < 8; ++mask) {
    const Eigen::Vector3f corner{
        (mask & 1) != 0 ? bbox.max_pt.x() : bbox.min_pt.x(),
        (mask & 2) != 0 ? bbox.max_pt.y() : bbox.min_pt.y(),
        (mask & 4) != 0 ? bbox.max_pt.z() : bbox.min_pt.z()};
    const Eigen::Vector3f offset = corner - bbox.center;
    const float projected_x = basis.right.dot(offset);
    const float projected_y = basis.up.dot(offset);
    minimum_x = std::min(minimum_x, projected_x);
    maximum_x = std::max(maximum_x, projected_x);
    minimum_y = std::min(minimum_y, projected_y);
    maximum_y = std::max(maximum_y, projected_y);
    maximum_back = std::max(maximum_back, basis.back.dot(offset));
  }

  const float extent_x = maximum_x - minimum_x;
  const float extent_y = maximum_y - minimum_y;
  constexpr float usable_fraction = 0.9F;
  float scale = std::numeric_limits<float>::infinity();
  if (extent_x > std::numeric_limits<float>::epsilon()) {
    scale = usable_fraction * static_cast<float>(width) / extent_x;
  }
  if (extent_y > std::numeric_limits<float>::epsilon()) {
    scale = std::min(scale,
                     usable_fraction * static_cast<float>(height) / extent_y);
  }
  if (!std::isfinite(scale))
    scale = 1.0F;
  if (scale <= 0.0F)
    throw std::overflow_error(
        "cloud orthographic scale exceeds renderer range");

  return {scale, (minimum_x + maximum_x) * 0.5F, (minimum_y + maximum_y) * 0.5F,
          maximum_back};
}

std::pair<float, float> viewAngles(View v) {
  const float isometric_elevation = std::atan(1.0F / std::sqrt(2.0F));
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
    return {0.0f, std::numbers::pi_v<float> / 2};
  case View::Bottom:
    return {0.0f, -std::numbers::pi_v<float> / 2};
  case View::TopRightFront:
    return {std::numbers::pi_v<float> / 4, isometric_elevation};
  case View::TopLeftFront:
    return {-std::numbers::pi_v<float> / 4, isometric_elevation};
  case View::BotRightFront:
    return {std::numbers::pi_v<float> / 4, -isometric_elevation};
  case View::BotLeftFront:
    return {-std::numbers::pi_v<float> / 4, -isometric_elevation};
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
  if (!pngDimensionsSupported(image.width, image.height))
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

std::string_view renderColorModeName(RenderColorMode mode) {
  switch (mode) {
  case RenderColorMode::Auto:
    return "auto";
  case RenderColorMode::RGB:
    return "rgb";
  case RenderColorMode::Intensity:
    return "intensity";
  case RenderColorMode::Z:
    return "z";
  case RenderColorMode::Solid:
    return "solid";
  }
  return "unknown";
}

std::string_view renderProjectionName(RenderProjection projection) {
  switch (projection) {
  case RenderProjection::Orthographic:
    return "orthographic";
  case RenderProjection::Perspective:
    return "perspective";
  }
  return "unknown";
}

ImageWriteStatus writeImageAtomic(const std::filesystem::path &output,
                                  ImageView image, bool overwrite,
                                  std::stop_token stop) {
  if (stop.stop_requested())
    throw OperationCancelled();
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
        render_detail::writePng(writePngChunk, &sink, image.width, image.height,
                                3, image.pixels.data(), image.stride_bytes);
    if (sink.error)
      throw std::system_error(sink.error->system_error, sink.error->message);
    if (encoded == 0)
      throw std::runtime_error("failed to encode image: " +
                               displayPath(output));
    if (stop.stop_requested())
      throw OperationCancelled();
    auto published = temporary.output->publish(output, overwrite);
    if (!published)
      throw std::system_error(published.error().system_error,
                              published.error().message);
    if (!published.value().published) {
      temporary.output.reset();
      return ImageWriteStatus::Skipped;
    }
    for (const auto &warning : published.value().post_commit_warnings) {
      spdlog::warn("{}: {}", warning.message, warning.system_error.message());
    }
    temporary.output.reset();
  } catch (...) {
    temporary.output.reset();
    throw;
  }
  return ImageWriteStatus::Written;
}

std::vector<RenderResult> renderMultiView(const PointCloudIRGBConstPtr &cloud,
                                          const RenderOpts &opts,
                                          std::stop_token stop) {
  if (!cloud)
    throw std::invalid_argument("renderMultiView requires a cloud");
  if (opts.views.empty())
    throw std::invalid_argument("renderMultiView requires at least one view");
  for (const auto view : opts.views) {
    if (viewName(view) == "unknown")
      throw std::invalid_argument("renderMultiView received an invalid view");
  }
  if (renderColorModeName(opts.color_mode) == "unknown")
    throw std::invalid_argument(
        "renderMultiView received an invalid color mode");
  if (renderProjectionName(opts.projection) == "unknown")
    throw std::invalid_argument(
        "renderMultiView received an invalid projection");
  if (!std::isfinite(opts.trim_percent) || opts.trim_percent < 0.0F ||
      opts.trim_percent >= 50.0F) {
    throw std::invalid_argument("render trim percent must be in [0, 50)");
  }
  if (opts.projection == RenderProjection::Perspective &&
      (!std::isfinite(opts.fov) || opts.fov <= 0.0F || opts.fov >= 180.0F)) {
    throw std::invalid_argument("render FOV must be in (0, 180)");
  }
  if (stop.stop_requested())
    throw OperationCancelled();
  SimpleRenderer renderer(opts.width, opts.height);

  // Degenerate (empty) cloud: still produce correctly-sized black frames so
  // callers can rely on result count == opts.views.size().
  const CloudAnalysis analysis = analyzeCloud(cloud, opts.trim_percent, stop);
  const CloudBoundingBox &bbox = analysis.framed;
  if (bbox.points > 0U && opts.color_mode == RenderColorMode::RGB &&
      !bbox.has_visible_rgb) {
    throw std::invalid_argument(
        "RGB coloring requested but cloud has no visible RGB values; use "
        "auto, intensity, z, or solid");
  }
  const RenderColorMapping color_mapping = bbox.colorMapping(opts.color_mode);
  const RenderCloudStats cloud_stats = analysis.stats();

  std::vector<RenderResult> results;
  results.reserve(opts.views.size());

  for (const auto &v : opts.views) {
    if (stop.stop_requested())
      throw OperationCancelled();
    auto [theta, phi] = viewAngles(v);
    const CameraBasis camera = cameraBasis(theta, phi);
    ImageRGB8 image;
    const auto include_point = [&analysis](const PointT &point) {
      return analysis.contains(point);
    };
    if (opts.projection == RenderProjection::Orthographic) {
      const OrthographicFrame frame =
          orthographicFrame(bbox, camera, opts.width, opts.height);
      const auto project_point = [&bbox, &camera, frame,
                                  &opts](const PointT &point, float &x,
                                         float &y, float &depth) {
        const Eigen::Vector3f offset =
            Eigen::Vector3f(point.x, point.y, point.z) - bbox.center;
        x = static_cast<float>(opts.width) * 0.5F +
            (camera.right.dot(offset) - frame.projected_center_x) * frame.scale;
        y = static_cast<float>(opts.height) * 0.5F -
            (camera.up.dot(offset) - frame.projected_center_y) * frame.scale;
        depth = 1.0F + frame.nearest_back - camera.back.dot(offset);
        return true;
      };
      image = renderer.render(cloud, bbox.points, color_mapping, include_point,
                              project_point, stop);
    } else {
      float optimal_distance = 0.0F;
      if (bbox.max_dimension > 0.0F) {
        optimal_distance =
            optimalDistance(bbox, camera, opts.width, opts.height, opts.fov);
      }
      if (optimal_distance <= 0.0F || !std::isfinite(optimal_distance))
        optimal_distance = 1.0F;
      const Eigen::Matrix4f view_matrix =
          createViewMatrix(bbox.center, camera, optimal_distance);
      if (!view_matrix.allFinite())
        throw std::overflow_error("cloud camera exceeds renderer range");
      const float focal_length =
          static_cast<float>(opts.width) /
          (2.0F * std::tan(opts.fov * std::numbers::pi_v<float> / 360.0F));
      const auto project_point = [&view_matrix, focal_length,
                                  &opts](const PointT &point, float &x,
                                         float &y, float &depth) {
        const Eigen::Vector4f camera_point =
            view_matrix * Eigen::Vector4f(point.x, point.y, point.z, 1.0F);
        depth = -camera_point.z();
        if (depth <= 0.0F)
          return false;
        x = camera_point.x() * focal_length / depth +
            static_cast<float>(opts.width) * 0.5F;
        y = static_cast<float>(opts.height) * 0.5F -
            camera_point.y() * focal_length / depth;
        return true;
      };
      image = renderer.render(cloud, bbox.points, color_mapping, include_point,
                              project_point, stop);
    }
    if (stop.stop_requested())
      throw OperationCancelled();

    results.push_back(
        {std::string(viewName(v)), std::move(image), cloud_stats});
  }

  return results;
}

} // namespace kpt
