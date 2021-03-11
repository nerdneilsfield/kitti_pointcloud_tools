#ifndef NDT_CPU_SE_HELPER
#define NDT_CPU_SE_HELPER

#include <types.h>

#include <map>
#include <string>
#include <unordered_map>
#include <tuple>
#include <vector>

namespace cpu {

    constexpr int RANGENET_NUM_LABEL = 12;

/// \brief Segement the cloud with label(which store in intensity)
/// \param cloud_in the input point cloud
/// \param number_inputs the number of label types
    std::vector<PointCloudPtrT> getSegmentFast(PointCloudConstPtrT cloud_in,
                                               int number_inputs);

/// \brief get the range net label map
    std::map<int, int> getRangeNetLabelMap();

    std::map<int, std::tuple<int, int, int>> getRGBLabelMap();

/// \breif read the label file
    std::vector<int> loadLabel(const std::string &label_file);

/// \brief Bind the label with pointcloud with label file
    PointCloudPtrT bindLabelWithFile(PointCloudPtrT cloud_in,
                                     const std::string &label_file,
                                     std::map<int, int> &label_map);

/// \brief Bind the label with pointcloud with give labels
    PointCloudPtrT bindLabel(PointCloudPtrT cloud_in,
                             const std::vector<int> &labels,
                             std::map<int, int> &label_map, const int &s = 0);

    PointRGBCloudPtrT
    bindRGBLabel(PointCloudPtrT cloud_in,
                 std::map<int, std::tuple<int, int, int>>& rgb_map);

}  // namespace cpu

#endif  // NDT_CPU_SE_HELPER
