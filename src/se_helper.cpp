#undef __ARM_NEON__
#undef __ARM_NEON

#include "se_helper.h"

#include <cassert>
#include <fstream>
#include <string>

#include <spdlog/spdlog.h>

#define __ARM_NEON__
#define __ARM_NEON

namespace cpu {

    std::vector<PointCloudPtrT> getSegmentFast(PointCloudConstPtrT cloud_in,
                                               int number_inputs) {
        std::vector<PointCloudPtrT> segmented_cloud;
        for (int i = 0; i < number_inputs; i++) {
            segmented_cloud.emplace_back(new PointCloudT);
        }
        spdlog::debug("get segment fast: " + std::to_string(number_inputs));

        int count = 0;
        int size = cloud_in->points.size();

        // insert point to segmented cloud by their label
        for (int i = 0; i < size; i++) {
            PointT point = cloud_in->points[i];
            int addr = round(point.intensity);
            if (addr >= 0) {
                count++;
                segmented_cloud[addr]->points.push_back(point);
            }
        }

        spdlog::debug("the effective point is {} of {} with ratio {}", count, size,
                      static_cast<double>(count) / static_cast<double>(size));
        return segmented_cloud;
    }


    std::map<int, int> getRangeNetLabelMap() {
        std::map<int, int> lm;
        lm[0] = 0; // "unlabeled"
        lm[1] = -1; // "outlier" mapped to "unlabeled" --------------------------mapped
        lm[10] = 0; // "car"
        lm[11] = -1; // "bicycle"
        lm[13] =
                0; // "bus" mapped to "other-vehicle" --------------------------mapped
        lm[15] = -1; // "motorcycle"
        lm[16] =
                -1; // "on-rails" mapped to "other-vehicle" ---------------------mapped
        lm[18] = 0; // "truck"
        lm[20] = 0; // "other-vehicle"
        lm[30] = -1; // "person"
        lm[31] = -1; // "bicyclist"
        lm[32] = -1; // "motorcyclist"
        lm[40] = 1;  // "road"
        lm[44] = 1;  // "parking"
        lm[48] = 2;  // "sidewalk"
        lm[49] = 3;  // "other-ground"
        lm[50] = 4;  // "building"
        lm[51] = 5;  // "fence"
        lm[52] =
                0; // "other-structure" mapped to "unlabeled" ------------------mapped
        lm[60] =
                6; // "lane-marking" to "road" ---------------------------------mapped
        lm[70] = 7;  // "vegetation"
        lm[71] = 8;  // "trunk"
        lm[72] = 9;  // "terrain"
        lm[80] = 10; // "pole"
        lm[81] = 11; // "traffic-sign"
        lm[99] =
                -1; // "other-object" to "unlabeled" ----------------------------mapped
        lm[252] = -1; // "moving-car"
        lm[253] = -1; // "moving-bicyclist"
        lm[254] = -1; // "moving-person"
        lm[255] = -1; // "moving-motorcyclist"
        lm[256] =
                -1; // "moving-on-rails" mapped to "moving-other-vehicle" ------mapped
        lm[257] =
                -1; // "moving-bus" mapped to "moving-other-vehicle" -----------mapped
        lm[258] = -1; // "moving-truck"
        lm[259] = -1; // "moving-other-vehicle"
        return lm;
    }

    std::map<int, std::tuple<int, int, int>> getRGBLabelMap() {
        std::map<int, std::tuple<int, int, int>> rgb_label_map;
        rgb_label_map[0] = std::make_tuple(0, 0, 0);
        rgb_label_map[1] = std::make_tuple(34, 139, 0);
        rgb_label_map[2] = std::make_tuple(0, 255, 127);
        rgb_label_map[3] = std::make_tuple(8, 46, 84);
        rgb_label_map[4] = std::make_tuple(106, 90, 205);
        rgb_label_map[5] = std::make_tuple(65, 105, 225);
        rgb_label_map[6] = std::make_tuple(240, 255, 255);
        rgb_label_map[7] = std::make_tuple(124, 252, 0);
        rgb_label_map[8] = std::make_tuple(176, 48, 96);
        rgb_label_map[9] = std::make_tuple(160, 32, 240);
        rgb_label_map[10] = std::make_tuple(218, 112, 214);
        rgb_label_map[11] = std::make_tuple(221, 160, 221);
        rgb_label_map[-1] = std::make_tuple(227, 23, 13);
        return rgb_label_map;
    }

    std::vector<int> loadLabel(const std::string &label_file) {

        std::ifstream label_file_in(label_file, std::ios::in | std::ios::binary);

        std::vector<int> result;
        int label = 0;
        while (!label_file_in.eof()) {
            label_file_in.read((char *) &label, sizeof(int));
            result.push_back(label);
        }
        return result;
    }

    PointCloudPtrT bindLabelWithFile(PointCloudPtrT cloud_in,
                                     const std::string &label_file,
                                     std::map<int, int> &label_map) {
        PointCloudPtrT binded_map(new PointCloudT);
        std::ifstream label_file_in(label_file, std::ios::in | std::ios::binary);

        int count = 0;
        int label = 0;
        while (!label_file_in.eof()) {
            PointT point = cloud_in->points[count];
            label_file_in.read((char *) &label, sizeof(int));
            point.intensity = label_map[label];
            binded_map->points.push_back(point);
            count++;
        }
        return binded_map;
    }

    PointCloudPtrT bindLabel(PointCloudPtrT cloud_in,
                             const std::vector<int> &labels,
                             std::map<int, int> &label_map, const int &s) {
        PointCloudPtrT binded_map(new PointCloudT);
        spdlog::debug("call bind label with input: {} points, label: {}",
                      cloud_in->points.size(), labels.size());
        assert(cloud_in->points.size() == labels.size());
        for (auto i = 0; i < cloud_in->points.size(); i++) {
            auto point = cloud_in->points[i];
            point.intensity = label_map[labels[i]];
            if (s == 1) {
                if (point.intensity != -1) {
                    binded_map->points.push_back(point);
                }
            } else {
                binded_map->points.push_back(point);
            }
        }
        return binded_map;
    }

    PointRGBCloudPtrT
    bindRGBLabel(PointCloudPtrT cloud_in,
                 std::map<int, std::tuple<int, int, int>> &rgb_map) {

        spdlog::debug("call rgb label with {} points", cloud_in->points.size());
        PointRGBCloudPtrT binded_map(new PointRGBCloudT);

        for (auto i = 0; i < cloud_in->points.size(); i++) {
            auto point = cloud_in->points[i];
            PointRGBT new_point;
            int r, g, b;
            std::tie(r, g, b
            ) = rgb_map[point.intensity];
            new_point.
                    x = point.x;
            new_point.
                    y = point.y;
            new_point.
                    z = point.z;
            new_point.
                    r = r;
            new_point.
                    g = g;
            new_point.
                    b = b;
            binded_map->points.
                    push_back(new_point);
        }
        return
                binded_map;
    }
} // namespace cpu