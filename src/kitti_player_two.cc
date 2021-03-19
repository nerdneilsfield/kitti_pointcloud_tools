#include "types.h"
#include "se_helper.h"

#include <cstdlib>
#include <iostream>
#include <thread>

#include <pcl/common/common_headers.h>
#include <pcl/console/parse.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <spdlog/spdlog.h>

using namespace std::chrono_literals;

pcl::visualization::PCLVisualizer::Ptr
rgbVis(PointRGBCloudPtrT cloud1, PointRGBCloudConstPtrT cloud2, unsigned int id, int &v1, int &v2) {
    // --------------------------------------------
    // -----Open 3D viewer and add point cloud-----
    // --------------------------------------------
    pcl::visualization::PCLVisualizer::Ptr viewer(
            new pcl::visualization::PCLVisualizer("3D Viewer"));
    char id_text[128];
    sprintf(id_text, "Frame: %d", id);
    viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
    viewer->setBackgroundColor(1, 1, 1, v1);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(
            cloud1);
    viewer->addPointCloud<pcl::PointXYZRGB>(cloud1, rgb, "pc1", v1);
    viewer->setPointCloudRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "pc1");
    viewer->addText(id_text, 20, 20, 0, 0, 0, "id1", v1);

    viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
    viewer->setBackgroundColor(1, 1, 1, v2);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb2(
            cloud2);
    viewer->addPointCloud<pcl::PointXYZRGB>(cloud2, rgb2, "pc2", v2);
    viewer->setPointCloudRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "pc2");
    viewer->addText(id_text, 20, 20, 0, 0, 0, "id2", v2);

    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();
    viewer->setCameraPosition(-4, 0, 2.5, 0, 0, 2.5, v1);
    viewer->setCameraPosition(-4, 0, 2.5, 0, 0, 2.5, v2);
//    viewer->setCameraFieldOfView(0.523599);
//    viewer->setCameraClipDistances(0.00522511, 50);
    return (viewer);
}

void UpdateViewer(pcl::visualization::PCLVisualizer::Ptr viewer, PointRGBCloudConstPtrT cloud1, PointRGBCloudConstPtrT cloud2, unsigned int id) {
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb1(
            cloud1);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb2(
            cloud2);
    viewer->updatePointCloud<pcl::PointXYZRGB>(cloud1, rgb1, "pc1");
    viewer->updatePointCloud<pcl::PointXYZRGB>(cloud2, rgb2, "pc2");
    char id_text[128];
    sprintf(id_text, "Frame: %d", id);
    viewer->updateText(id_text, 20, 20, 0, 0, 0, "id1");
    viewer->updateText(id_text, 20, 20, 0, 0, 0, "id2");
    viewer->spinOnce(100);
    std::this_thread::sleep_for(100ms);
}

int main(int argc, char **argv) {
    if (argc != 6) {
        std::cout << "hello" << std::endl;
        return 0;
    }

    std::string pcd_dir{argv[1]};
    std::string label_dir1{argv[2]};
    std::string label_dir2{argv[3]};

    if (pcd_dir[pcd_dir.size() - 1] != '/') {
        pcd_dir = pcd_dir.append("/");
    }
    if (label_dir2[label_dir2.size() - 1] != '/') {
        label_dir2 = label_dir2.append("/");
    }

    spdlog::info("pcd_dir: {}", pcd_dir);
    spdlog::info("label_dir1: {}", label_dir1);
    spdlog::info("label_dir2: {}", label_dir2);

    auto start_index = static_cast<unsigned int>(std::atoi(argv[4]));
    auto end_index = static_cast<unsigned int>(std::atoi(argv[5]));

    if (end_index < start_index) {
        spdlog::error("end_index is smaller than start_index: {} < {}", end_index, start_index);
        return 0;
    }

    PointCloudPtrT raw_in_cloud(new PointCloudT);
    PointCloudPtrT in_cloud1(new PointCloudT);
    PointRGBCloudPtrT rgb_cloud1(new PointRGBCloudT);
    PointCloudPtrT in_cloud2(new PointCloudT);
    PointRGBCloudPtrT rgb_cloud2(new PointRGBCloudT);

    int id = start_index;

    char pcd_path[512];
    sprintf(pcd_path, "%s%06d.pcd", pcd_dir.c_str(), id);
    char label_path1[512];
    sprintf(label_path1, "%s%06d.label", label_dir1.c_str(), id);
    char label_path2[512];
    sprintf(label_path2, "%s%06d.label", label_dir2.c_str(), id);
    pcl::io::loadPCDFile(pcd_path, *raw_in_cloud);
    auto labels1 = cpu::loadLabel(label_path1);
    auto labels2 = cpu::loadLabel(label_path2);
    std::map<int, int> label_map = cpu::getRangeNetLabelMap();
    std::map<int, std::tuple<int, int, int>> rgb_map = cpu::getRGBLabelMap();

    in_cloud1 = cpu::bindLabel(raw_in_cloud, labels1,
                              label_map, 0);
    in_cloud2 = cpu::bindLabel(raw_in_cloud, labels2,
                               label_map, 0);
    rgb_cloud1 = cpu::bindRGBLabel(in_cloud1, rgb_map);
    rgb_cloud2 = cpu::bindRGBLabel(in_cloud2, rgb_map);

    int v1(0);
    int v2(0);

    auto viewer = rgbVis(rgb_cloud1, rgb_cloud2, id, v1, v2);

    viewer->spinOnce(100);

    for (id = start_index + 1; id <= end_index; id++) {
        sprintf(pcd_path, "%s%06d.pcd", pcd_dir.c_str(), id);
        sprintf(label_path1, "%s%06d.label", label_dir1.c_str(), id);
        sprintf(label_path2, "%s%06d.label", label_dir2.c_str(), id);
        pcl::io::loadPCDFile(pcd_path, *raw_in_cloud);
        labels1 = cpu::loadLabel(label_path1);
        labels2 = cpu::loadLabel(label_path2);
        in_cloud1 = cpu::bindLabel(raw_in_cloud, labels1,
                                   label_map, 0);
        in_cloud2 = cpu::bindLabel(raw_in_cloud, labels2,
                                   label_map, 0);
        rgb_cloud1 = cpu::bindRGBLabel(in_cloud1, rgb_map);
        rgb_cloud2 = cpu::bindRGBLabel(in_cloud2, rgb_map);
        UpdateViewer(viewer, rgb_cloud1, rgb_cloud2, id);
    }


    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
        std::this_thread::sleep_for(100ms);
    }
}
