#include "kpt/label/label.hpp"
#include "kpt/cancellation.hpp"
#include "platform/utf8_path.hpp"
#include <fstream>
#include <spdlog/spdlog.h>
#include <stdexcept>
#include <system_error>
#include <unordered_map>

namespace kpt {

namespace {
constexpr std::uintmax_t kMaxLabelBytes =
    std::uintmax_t{20'000'000U} * sizeof(int);
}

std::vector<int> loadLabel(const std::filesystem::path &p,
                           std::stop_token stop) {
  if (stop.stop_requested())
    throw OperationCancelled();
  std::error_code error;
  const auto status = std::filesystem::symlink_status(p, error);
  if (error || std::filesystem::is_symlink(status) ||
      !std::filesystem::is_regular_file(status))
    throw std::runtime_error("label path is not a regular file");
  const auto bytes = std::filesystem::file_size(p, error);
  if (error || bytes > kMaxLabelBytes || bytes % sizeof(int) != 0)
    throw std::runtime_error("label file exceeds limit or is truncated");
  std::ifstream ifs(p, std::ios::binary);
  if (!ifs) {
    auto display = platform::pathToUtf8(p);
    throw std::runtime_error(
        "file not found: " +
        (display ? std::move(display).value() : "<invalid-native-path>"));
  }
  std::vector<int> result;
  const auto expected_count = bytes / sizeof(int);
  result.reserve(expected_count);
  for (std::size_t count = 0; count < expected_count; ++count) {
    if ((count % 4096U) == 0U && stop.stop_requested())
      throw OperationCancelled();
    int label = 0;
    if (!ifs.read(reinterpret_cast<char *>(&label), sizeof(int)))
      throw std::runtime_error("label file changed or was truncated while "
                               "reading");
    result.push_back(label);
  }
  char trailing = '\0';
  if (ifs.read(&trailing, 1))
    throw std::runtime_error("label file changed or exceeds limit while "
                             "reading");
  if (ifs.bad())
    throw std::runtime_error("label file read failed");
  return result;
}

std::map<int, int> rangeNetLabelMap() {
  return {
      {0, 0},    {1, -1},   {10, 0},   {11, -1},  {13, 0},   {15, -1},
      {16, -1},  {18, 0},   {20, 0},   {30, -1},  {31, -1},  {32, -1},
      {40, 1},   {44, 1},   {48, 2},   {49, 3},   {50, 4},   {51, 5},
      {52, 0},   {60, 6},   {70, 7},   {71, 8},   {72, 9},   {80, 10},
      {81, 11},  {99, -1},  {252, -1}, {253, -1}, {254, -1}, {255, -1},
      {256, -1}, {257, -1}, {258, -1}, {259, -1},
  };
}

std::map<int, std::tuple<int, int, int>> rgbLabelMap() {
  return {
      {0, {0, 0, 0}},       {1, {34, 139, 0}},     {2, {0, 255, 127}},
      {3, {8, 46, 84}},     {4, {106, 90, 205}},   {5, {65, 105, 225}},
      {6, {240, 255, 255}}, {7, {124, 252, 0}},    {8, {176, 48, 96}},
      {9, {160, 32, 240}},  {10, {218, 112, 214}}, {11, {221, 160, 221}},
      {-1, {227, 23, 13}},
  };
}

PointCloudIRGBPtr
applyLabel(const PointCloudIRGBConstPtr &cloud, const std::vector<int> &labels,
           const std::map<int, int> &label_map,
           const std::map<int, std::tuple<int, int, int>> &rgb_map,
           bool drop_unlabeled, std::stop_token stop) {
  if (stop.stop_requested())
    throw OperationCancelled();
  if (!cloud)
    throw std::invalid_argument("cloud must not be null");
  auto out = std::make_shared<PointCloudIRGB>();
  out->has_noise = cloud->has_noise;
  if (cloud->size() != labels.size())
    throw std::invalid_argument(
        "cloud/label count mismatch: " + std::to_string(cloud->size()) +
        " vs " + std::to_string(labels.size()));
  std::unordered_map<int, int> compact_lookup;
  compact_lookup.reserve(label_map.size());
  for (const auto &[label, compact] : label_map)
    compact_lookup.emplace(label, compact);
  std::unordered_map<int, std::tuple<int, int, int>> color_lookup;
  color_lookup.reserve(rgb_map.size());
  for (const auto &[compact, color] : rgb_map)
    color_lookup.emplace(compact, color);
  for (size_t i = 0; i < cloud->size(); ++i) {
    if ((i % 4096U) == 0U && stop.stop_requested())
      throw OperationCancelled();
    auto pt = cloud->points[i];
    int compact = -1; // default for unknown labels
    auto lit = compact_lookup.find(labels[i]);
    if (lit != compact_lookup.end())
      compact = lit->second;
    pt.intensity = static_cast<float>(compact);
    if (drop_unlabeled && compact == -1)
      continue;
    auto rit = color_lookup.find(compact);
    if (rit != color_lookup.end()) {
      pt.r = static_cast<uint8_t>(std::get<0>(rit->second));
      pt.g = static_cast<uint8_t>(std::get<1>(rit->second));
      pt.b = static_cast<uint8_t>(std::get<2>(rit->second));
    } else {
      pt.r = pt.g = pt.b = 0; // unknown compact id -> black
    }
    out->push_back(pt);
  }
  return out;
}

} // namespace kpt
