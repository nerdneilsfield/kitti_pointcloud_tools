#include "kitti_binary.h"

#include <spdlog/spdlog.h>

using namespace kitti_binary_tools;
int main(int argc, char *argv[])
{
    spdlog::set_level(spdlog::level::debug);

    if (argc != 3)
    {
        std::cout << "Usage: convert_pcd_to_binary.cc [pcd file] [binary file]" << std::endl;
        return 0;
    }
    PointCloud cloud;
    pcl::io::loadPCDFile(argv[1], cloud);

    spdlog::debug("load {} points for {}", cloud.points.size(), argv[1]);

    KittiBinary kitti{};
    kitti.DumpToFile(cloud, argv[2]);
}
