#pragma once

#include "kpt/core_types.hpp"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <utility>
#include <vector>

namespace kpt {

struct PointXYZRGBI {
  float x = 0.0F;
  float y = 0.0F;
  float z = 0.0F;
  std::uint8_t r = 0;
  std::uint8_t g = 0;
  std::uint8_t b = 0;
  float intensity = 0.0F;
};

using PointT = PointXYZRGBI;

class PointCloudIRGB {
public:
  using value_type = PointT;
  using container_type = std::vector<PointT>;
  using iterator = container_type::iterator;
  using const_iterator = container_type::const_iterator;

  container_type points;

  [[nodiscard]] bool empty() const noexcept { return points.empty(); }
  [[nodiscard]] std::size_t size() const noexcept { return points.size(); }
  void clear() noexcept { points.clear(); }
  void reserve(std::size_t count) { points.reserve(count); }
  void push_back(const PointT &point) { points.push_back(point); }
  void push_back(PointT &&point) { points.push_back(std::move(point)); }

  iterator begin() noexcept { return points.begin(); }
  const_iterator begin() const noexcept { return points.begin(); }
  const_iterator cbegin() const noexcept { return points.cbegin(); }
  iterator end() noexcept { return points.end(); }
  const_iterator end() const noexcept { return points.end(); }
  const_iterator cend() const noexcept { return points.cend(); }

  PointCloudIRGB &operator+=(const PointCloudIRGB &other) {
    points.insert(points.end(), other.points.begin(), other.points.end());
    return *this;
  }
};

using PointCloudIRGBPtr = std::shared_ptr<PointCloudIRGB>;
using PointCloudIRGBConstPtr = std::shared_ptr<const PointCloudIRGB>;

} // namespace kpt
