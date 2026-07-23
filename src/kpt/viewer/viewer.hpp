#pragma once
#include "kpt/types.hpp"
#include <pcl/visualization/pcl_visualizer.h>
#include <Eigen/Core>
#include <string>

namespace kpt {

struct ViewerOpts {
  ColorBy colorby = ColorBy::Intensity;
  int point_size = 3;
  Eigen::Vector3f bg = {0, 0, 0};
};

class InteractiveViewer {
 public:
  explicit InteractiveViewer(const ViewerOpts& opts);
  void show(const PointCloudIRGBConstPtr& cloud, const std::string& title);
  void spin();  // blocks until window closed
  pcl::visualization::PCLVisualizer::Ptr raw() { return viewer_; }

 private:
  ViewerOpts opts_;
  pcl::visualization::PCLVisualizer::Ptr viewer_;
};

}  // namespace kpt
