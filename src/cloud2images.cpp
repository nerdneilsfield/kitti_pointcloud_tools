#include "cloud2images.hpp"

#include <opencv2/opencv.hpp>
#include <vtkImageData.h>
#include <vtkWindowToImageFilter.h>

namespace kitti_pointcloud_tools {

auto Cloud2Images::convert(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
    -> std::vector<cv::Mat> {

  viewer_->removeAllPointClouds();
  viewer_->addPointCloud(cloud, "cloud");

  viewer_->setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");

  std::vector<std::pair<double, double>> views = {
      {0, 0},   // 正面视图
      {90, 0},  // 右侧视图
      {180, 0}, // 背面视图
      {270, 0}, // 左侧视图
      {0, 90},  // 俯视图
      {0, -90}  // 仰视图
  };

  std::vector<cv::Mat> images;
  // 对每个视角生成图片
  for (size_t i = 0; i < views.size(); i++) {
    // 设置相机参数
    double x = cos(views[i].first * M_PI / 180) * 3;
    double y = sin(views[i].first * M_PI / 180) * 3;
    double z = sin(views[i].second * M_PI / 180) * 3;

    // 更新setCameraPosition调用，提供完整的参数
    viewer_->setCameraPosition(x, y, z, // 相机位置
                               0, 0, 0, // 焦点位置（场景中心）
                               0, 0, 1  // 上方向向量
    );

    viewer_->spinOnce();

    // 获取视窗图像
    vtkSmartPointer<vtkRenderWindow> renderWindow = viewer_->getRenderWindow();
    vtkSmartPointer<vtkWindowToImageFilter> windowToImageFilter =
        vtkSmartPointer<vtkWindowToImageFilter>::New();
    windowToImageFilter->SetInput(renderWindow);
    windowToImageFilter->Update();

    // 将VTK图像转换为OpenCV格式
    vtkSmartPointer<vtkImageData> image = windowToImageFilter->GetOutput();
    int dims[3];
    image->GetDimensions(dims);
    cv::Mat cvImage(dims[1], dims[0], CV_8UC3);

    unsigned char *pixels = (unsigned char *)image->GetScalarPointer();
    for (int y = 0; y < dims[1]; y++) {
      for (int x = 0; x < dims[0]; x++) {
        int idx = (y * dims[0] + x) * 3;
        cvImage.at<cv::Vec3b>(y, x) =
            cv::Vec3b(pixels[idx + 2], pixels[idx + 1], pixels[idx]);
      }
    }

    images.push_back(cvImage);
  }

  return images;
}

} // namespace kitti_pointcloud_tools