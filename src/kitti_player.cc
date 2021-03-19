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

// --------------
// -----Help-----
// --------------
void printUsage(const char *progName) {
    std::cout << "\n\nUsage: " << progName << " [options]\n\n"
              << "Options:\n"
              << "-------------------------------------------\n"
              << "-h           this help\n"
              << "-s           Simple visualisation example\n"
              << "-r           RGB colour visualisation example\n"
              << "-c           Custom colour visualisation example\n"
              << "-n           Normals visualisation example\n"
              << "-a           Shapes visualisation example\n"
              << "-v           Viewports example\n"
              << "-i           Interaction Customization example\n"
              << "\n\n";
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
    viewer->setCameraPosition(-4, 0, 2.5,0, 0, 2.5);
//    viewer->setCameraFieldOfView(0.523599);
//    viewer->setCameraClipDistances(0.00522511, 50);
    return (viewer);
}

void UpdateViewer(pcl::visualization::PCLVisualizer::Ptr viewer, PointRGBCloudConstPtrT cloud, unsigned int id) {
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
    if (argc != 5) {
        std::cout << "hello" << std::endl;
        return 0;
    }

    std::string pcd_dir{argv[1]};
    std::string label_dir{argv[2]};

    if(pcd_dir[pcd_dir.size() - 1] !='/') {
        pcd_dir = pcd_dir.append("/");
    }
    if(label_dir[label_dir.size() - 1] !='/') {
        label_dir = label_dir.append("/");
    }

    spdlog::info("pcd_dir: {}", pcd_dir);
    spdlog::info("label_dir: {}", label_dir);

    auto start_index = static_cast<unsigned int>(std::atoi(argv[3]));
    auto end_index = static_cast<unsigned int>(std::atoi(argv[4]));

    if(end_index < start_index){
        spdlog::error("end_index is smaller than start_index: {} < {}", end_index, start_index);
        return 0;
    }

    PointCloudPtrT raw_in_cloud(new PointCloudT);
    PointCloudPtrT in_cloud(new PointCloudT);
    PointRGBCloudPtrT rgb_cloud(new PointRGBCloudT);

    int id = start_index;

    char pcd_path[512];
    sprintf(pcd_path, "%s%06d.pcd", pcd_dir.c_str(), id);
    char label_path[512];
    sprintf(label_path, "%s%06d.label", label_dir.c_str(), id);
    pcl::io::loadPCDFile(pcd_path, *raw_in_cloud);
    auto labels = cpu::loadLabel(label_path);
    std::map<int, int> label_map = cpu::getRangeNetLabelMap();
    std::map<int, std::tuple<int, int, int>> rgb_map = cpu::getRGBLabelMap();

    in_cloud = cpu::bindLabel(raw_in_cloud, labels,
                                             label_map, 0);
    rgb_cloud = cpu::bindRGBLabel(in_cloud, rgb_map);

    auto viewer = rgbVis(rgb_cloud, id);

    viewer->spinOnce(100);

    for(unsigned int id = start_index + 1;  id <= end_index; id++){
        sprintf(pcd_path,"%s%06d.pcd", pcd_dir.c_str(), id);
        sprintf(label_path, "%s%06d.label", label_dir.c_str(), id);
        pcl::io::loadPCDFile(pcd_path, *raw_in_cloud);
        labels = cpu::loadLabel(label_path);
        in_cloud = cpu::bindLabel(raw_in_cloud, labels,
                                  label_map, 0);
        rgb_cloud = cpu::bindRGBLabel(in_cloud, rgb_map);
        UpdateViewer(viewer, rgb_cloud, id);
    }


    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
        std::this_thread::sleep_for(100ms);
    }
}
