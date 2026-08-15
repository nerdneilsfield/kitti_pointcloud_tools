#pragma once

#include "kpt/types.hpp"

#include <Eigen/Geometry>

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace kpt::gui {

using LayerId = std::uint64_t;

// A closed, world-space box.  Points on every face belong to the ROI.
class RoiBox {
public:
  RoiBox(Eigen::Vector3d minimum, Eigen::Vector3d maximum);

  [[nodiscard]] const Eigen::Vector3d &minimum() const noexcept;
  [[nodiscard]] const Eigen::Vector3d &maximum() const noexcept;
  [[nodiscard]] bool contains(const Eigen::Vector3d &world_point) const noexcept;

private:
  Eigen::Vector3d minimum_;
  Eigen::Vector3d maximum_;
};

struct CloudLayer {
  LayerId id = 0;
  // Stable across a share-file round trip.  Runtime LayerId values are not.
  std::string source_key;
  std::shared_ptr<const PointCloudIRGB> cloud;
  Eigen::Affine3d local_to_world = Eigen::Affine3d::Identity();
  bool visible = true;
};

class Scene {
public:
  [[nodiscard]] LayerId addLayer(
      std::string source_key,
      std::shared_ptr<const PointCloudIRGB> cloud = {});
  [[nodiscard]] bool removeLayer(LayerId id);

  [[nodiscard]] CloudLayer *findLayer(LayerId id) noexcept;
  [[nodiscard]] const CloudLayer *findLayer(LayerId id) const noexcept;
  [[nodiscard]] CloudLayer *findLayerBySourceKey(const std::string &source_key) noexcept;
  [[nodiscard]] const CloudLayer *
  findLayerBySourceKey(const std::string &source_key) const noexcept;
  [[nodiscard]] const std::vector<CloudLayer> &layers() const noexcept;

  void setRoi(std::optional<RoiBox> roi);
  [[nodiscard]] const std::optional<RoiBox> &roi() const noexcept;

private:
  std::vector<CloudLayer> layers_;
  std::optional<RoiBox> roi_;
  LayerId next_layer_id_ = 1;
};

} // namespace kpt::gui
