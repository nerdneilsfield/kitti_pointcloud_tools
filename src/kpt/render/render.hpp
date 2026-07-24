#pragma once
#include "kpt/types.hpp"
#include <opencv2/opencv.hpp>
#include <filesystem>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace kpt {

struct RenderOpts {
  int width = 640;
  int height = 480;
  float fov = 120.0f;
  std::vector<View> views = {View::Front, View::Right, View::Back, View::Left,
                             View::Top, View::Bottom, View::TopRightFront,
                             View::TopLeftFront, View::BotRightFront, View::BotLeftFront};
};

struct RenderResult {
  std::string view_name;
  cv::Mat image;
};

enum class ImageWriteStatus { Written, Skipped };

std::string_view viewName(View view);

ImageWriteStatus writeImageAtomic(const std::filesystem::path &output,
                                  const cv::Mat &image, bool overwrite);

std::vector<RenderResult> renderMultiView(const PointCloudIRGBConstPtr& cloud,
                                          const RenderOpts& opts);

}  // namespace kpt
