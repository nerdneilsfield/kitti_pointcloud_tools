#include "kitti_binary.h"


using namespace kitti_binary_tools;
int main(int argc, char *argv[]){
    if(argc != 3){
        std::cout << "Usage: convert_pcd_to_binary.cc [pcd file] [binary file]" << std::endl;
        return 0; 
    }
    PointCloud cloud;
    pcl::io::loadPCDFile(argv[1], cloud);
   
	KittiBinary kitti(argv[1]);
	kitti.DumpToFile(cloud, argv[2]);
}
