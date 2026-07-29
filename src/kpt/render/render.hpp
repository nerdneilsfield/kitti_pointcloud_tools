#pragma once
#include "kpt/render/image.hpp"
#include "kpt/types.hpp"
#include <array>
#include <cstddef>
#include <filesystem>
#include <stop_token>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace kpt {

enum class RenderColorMode { Auto, RGB, Intensity, Z, Solid };
enum class RenderProjection { Orthographic, Perspective };

struct RenderOpts {
  int width = 640;
  int height = 480;
  float fov = 120.0f;
  RenderProjection projection = RenderProjection::Orthographic;
  float trim_percent = 1.0F;
  RenderColorMode color_mode = RenderColorMode::Auto;
  std::vector<View> views = {View::Front,         View::Right,
                             View::Back,          View::Left,
                             View::Top,           View::Bottom,
                             View::TopRightFront, View::TopLeftFront,
                             View::BotRightFront, View::BotLeftFront};
};

struct RenderCloudStats {
  std::array<float, 3> input_dimensions{};
  std::array<float, 3> framed_dimensions{};
  std::size_t finite_points = 0;
  std::size_t retained_points = 0;
};

struct RenderResult {
  std::string view_name;
  ImageRGB8 image;
  RenderCloudStats cloud_stats;
};

enum class ImageWriteStatus { Written, Skipped };

std::string_view viewName(View view);
std::string_view renderColorModeName(RenderColorMode mode);
std::string_view renderProjectionName(RenderProjection projection);

ImageWriteStatus writeImageAtomic(const std::filesystem::path &output,
                                  ImageView image, bool overwrite,
                                  std::stop_token stop = {});

inline ImageWriteStatus writeImageAtomic(const std::filesystem::path &output,
                                         const ImageRGB8 &image, bool overwrite,
                                         std::stop_token stop = {}) {
  return writeImageAtomic(output, image.view(), overwrite, stop);
}

std::vector<RenderResult>
renderMultiView(const PointCloudIRGBConstPtr &cloud, const RenderOpts &opts,
                std::stop_token stop = std::stop_token{});

} // namespace kpt
