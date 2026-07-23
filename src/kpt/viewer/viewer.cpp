#include "kpt/viewer/viewer.hpp"

#include <chrono>
#include <thread>

// PCL's addPointCloud<PointT> template relies on
// PointCloudGeometryHandlerXYZ<PointT>, whose implementation lives in the
// impl headers below. For PCL's built-in point types PCL ships explicit
// instantiations in its libraries; for our custom PointXYZRGBI we must
// include the impl so the template gets instantiated in this TU.
#include <pcl/visualization/impl/pcl_visualizer.hpp>
#include <pcl/visualization/impl/point_cloud_geometry_handlers.hpp>

namespace kpt {

InteractiveViewer::InteractiveViewer(const ViewerOpts& opts)
    : opts_(opts),
      viewer_(new pcl::visualization::PCLVisualizer("KPT Viewer")) {
  viewer_->setBackgroundColor(
      static_cast<double>(opts_.bg.x()),
      static_cast<double>(opts_.bg.y()),
      static_cast<double>(opts_.bg.z()));
  viewer_->addCoordinateSystem(1.0);
  viewer_->initCameraParameters();
}

void InteractiveViewer::show(const PointCloudIRGBConstPtr& cloud,
                            const std::string& title) {
  viewer_->removeAllPointClouds();
  if (opts_.colorby == ColorBy::RGB) {
    pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(cloud);
    viewer_->addPointCloud<PointT>(cloud, rgb, "pc");
  } else if (opts_.colorby == ColorBy::Intensity) {
    pcl::visualization::PointCloudColorHandlerGenericField<PointT> field(
        cloud, "intensity");
    viewer_->addPointCloud<PointT>(cloud, field, "pc");
  } else if (opts_.colorby == ColorBy::Z) {
    pcl::visualization::PointCloudColorHandlerGenericField<PointT> field(
        cloud, "z");
    viewer_->addPointCloud<PointT>(cloud, field, "pc");
  } else {
    pcl::visualization::PointCloudColorHandlerCustom<PointT> single(
        cloud, 255, 255, 255);
    viewer_->addPointCloud<PointT>(cloud, single, "pc");
  }
  viewer_->setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_POINT_SIZE, opts_.point_size, "pc");
  viewer_->addText(title, 20, 20, 0, 0, 0, "title");
}

void InteractiveViewer::spin() {
  while (!viewer_->wasStopped()) {
    viewer_->spinOnce(100);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}

}  // namespace kpt
