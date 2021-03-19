//
// Created by dengqi on 2021/3/19.
//
#include "se_helper.h"
#include "types.h"

#include <cstdlib>
#include <iostream>
#include <thread>
#include <vector>

#include <pcl/common/common_headers.h>
#include <pcl/console/parse.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "rapidcsv.h"
#include <spdlog/spdlog.h>

using namespace std::chrono_literals;

std::vector<std::vector<double>> readPoses(const std::string &pose_file_path) {
  std::vector<std::vector<double>> res;
  rapidcsv::Document doc(pose_file_path, rapidcsv::LabelParams(-1, -1));
  auto length = doc.GetColumn<double>(1).size();
  spdlog::info("read {} poses from {}", length, pose_file_path);
  for (int i = 0; i < length; i++) {
    res.emplace_back(doc.GetRow<double>(i));
  }
  return res;
}

pcl::visualization::PCLVisualizer::Ptr
rgbVis(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud, unsigned int id) {
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  pcl::visualization::PCLVisualizer::Ptr viewer(
      new pcl::visualization::PCLVisualizer("3D Viewer"));
  viewer->setBackgroundColor(1, 1, 1);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(
      cloud);
  viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb, "pc");
  viewer->setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "pc");
  viewer->addCoordinateSystem(1.0);
  char id_text[128];
  sprintf(id_text, "Frame: %d", id);
  viewer->addText(id_text, 20, 20, 0, 0, 0, "id");
  viewer->initCameraParameters();
  viewer->setCameraPosition(-4, 0, 2.5, 0, 0, 2.5);
  //    viewer->setCameraFieldOfView(0.523599);
  //    viewer->setCameraClipDistances(0.00522511, 50);
  return (viewer);
}

void UpdateViewer(pcl::visualization::PCLVisualizer::Ptr viewer,
                  PointRGBCloudConstPtrT cloud, unsigned int id) {
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(
      cloud);
  viewer->updatePointCloud<pcl::PointXYZRGB>(cloud, rgb, "pc");
  char id_text[128];
  sprintf(id_text, "Frame: %d", id);
  viewer->updateText(id_text, 20, 20, 0, 0, 0, "id");
  viewer->spinOnce(100);
  std::this_thread::sleep_for(100ms);
}

int main(int argc, char **argv) {
  if (argc < 6) {
    std::cout << "hello" << std::endl;
    return 0;
  }

  std::string pcd_dir{argv[1]};
  std::string label_dir1{argv[2]};
  //  std::string label_dir2{argv[3]};

  if (pcd_dir[pcd_dir.size() - 1] != '/') {
    pcd_dir = pcd_dir.append("/");
  }
  if (label_dir1[label_dir1.size() - 1] != '/') {
    label_dir1 = label_dir1.append("/");
  }

  spdlog::info("pcd_dir: {}", pcd_dir);
  spdlog::info("label_dir1: {}", label_dir1);

  auto start_index = static_cast<unsigned int>(std::atoi(argv[5]));
  auto end_index = static_cast<unsigned int>(std::atoi(argv[6]));

  if (end_index < start_index) {
    spdlog::error("end_index is smaller than start_index: {} < {}", end_index,
                  start_index);
    return 0;
  }

  PointCloudPtrT raw_in_cloud(new PointCloudT);
  PointCloudPtrT in_cloud1(new PointCloudT);
  PointRGBCloudPtrT rgb_cloud1(new PointRGBCloudT);

  int id = start_index;

  char pcd_path[512];
  sprintf(pcd_path, "%s%06d.pcd", pcd_dir.c_str(), id);
  char label_path1[512];
  sprintf(label_path1, "%s%06d.label", label_dir1.c_str(), id);
  char label_path2[512];

  auto labels1 = cpu::loadLabel(label_path1);
  std::map<int, int> label_map = cpu::getRangeNetLabelMap();
  std::map<int, std::tuple<int, int, int>> rgb_map = cpu::getRGBLabelMap();

  in_cloud1 = cpu::bindLabel(raw_in_cloud, labels1, label_map, 0);
  rgb_cloud1 = cpu::bindRGBLabel(in_cloud1, rgb_map);

  auto poses1 = readPoses(argv[3]);
  auto poses2 = readPoses(argv[4]);

  PointRGBCloudPtrT rgb_cloud2(new PointRGBCloudT);

  PointRGBT p1, p2;
  for (int idx = 0; idx <= start_index; idx++) {
    p1.x = poses1[idx][0];
    p1.y = poses1[idx][1];
    p1.z = 0;
    p2.x = poses2[idx][0];
    p2.y = poses2[idx][1];
    p1.z = 0;
    p1.r = 255;
    p1.g = 0;
    p1.b = 0;
    p2.r = 0;
    p2.g = 255;
    p2.b = 0;

    rgb_cloud2->points.emplace_back(p1);
    rgb_cloud2->points.emplace_back(p2);
  }

  auto viewer1 = rgbVis(rgb_cloud1, id);
  auto viewer2 = rgbVis(rgb_cloud2, id);

  viewer1->spinOnce(100);
  viewer2->spinOnce(100);

  for (id = start_index + 1;
       id <= end_index && id <= std::min(poses1.size(), poses2.size()); id++) {
    sprintf(pcd_path, "%s%06d.pcd", pcd_dir.c_str(), id);
    sprintf(label_path1, "%s%06d.label", label_dir1.c_str(), id);
    pcl::io::loadPCDFile(pcd_path, *raw_in_cloud);
    labels1 = cpu::loadLabel(label_path1);
    in_cloud1 = cpu::bindLabel(raw_in_cloud, labels1, label_map, 0);
    rgb_cloud1 = cpu::bindRGBLabel(in_cloud1, rgb_map);
    p1.x = poses1[id][0];
    p1.y = poses1[id][1];
    p1.z = 0;
    p2.x = poses2[id][0];
    p2.y = poses2[id][1];
    p1.z = 0;
    p1.r = 255;
    p1.g = 0;
    p1.b = 0;
    p2.r = 0;
    p2.g = 255;
    p2.b = 0;

    rgb_cloud2->points.emplace_back(p1);
    rgb_cloud2->points.emplace_back(p2);

    UpdateViewer(viewer1, rgb_cloud1, id);
    UpdateViewer(viewer2, rgb_cloud2, id);
  }

  while (!viewer1->wasStopped() && !viewer2->wasStopped()) {
    viewer1->spinOnce(100);
    viewer2->spinOnce(100);
    std::this_thread::sleep_for(100ms);
  }
}
