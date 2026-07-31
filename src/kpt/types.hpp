#pragma once

#include "kpt/core_types.hpp"

#include <array>
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
  std::uint8_t noise = 0;
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
  std::size_t width = 0;
  std::size_t height = 1;
  std::array<float, 7> viewpoint{0.0F, 0.0F, 0.0F, 1.0F,
                                 0.0F, 0.0F, 0.0F};
  bool has_noise = false;

  [[nodiscard]] bool empty() const noexcept { return points.empty(); }
  [[nodiscard]] std::size_t size() const noexcept { return points.size(); }
  void clear() noexcept {
    points.clear();
    resetMetadata();
  }
  void reserve(std::size_t count) { points.reserve(count); }
  void push_back(const PointT &point) {
    points.push_back(point);
    setUnorganizedShape();
  }
  void push_back(PointT &&point) {
    points.push_back(std::move(point));
    setUnorganizedShape();
  }

  iterator begin() noexcept { return points.begin(); }
  const_iterator begin() const noexcept { return points.begin(); }
  const_iterator cbegin() const noexcept { return points.cbegin(); }
  iterator end() noexcept { return points.end(); }
  const_iterator end() const noexcept { return points.end(); }
  const_iterator cend() const noexcept { return points.cend(); }

  PointCloudIRGB &operator+=(const PointCloudIRGB &other) {
    if (this == &other) {
      const auto copy = points;
      points.insert(points.end(), copy.begin(), copy.end());
    } else {
      points.insert(points.end(), other.points.begin(), other.points.end());
      has_noise = has_noise || other.has_noise;
    }
    setUnorganizedShape();
    return *this;
  }

private:
  void setUnorganizedShape() noexcept {
    width = points.size();
    height = 1;
  }

  void resetMetadata() noexcept {
    width = 0;
    height = 1;
    viewpoint = {0.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F};
    has_noise = false;
  }
};

using PointCloudIRGBPtr = std::shared_ptr<PointCloudIRGB>;
using PointCloudIRGBConstPtr = std::shared_ptr<const PointCloudIRGB>;

} // namespace kpt
