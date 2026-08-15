#include "gui/scene/scene.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace kpt::gui {
namespace {

[[nodiscard]] bool finite(const Eigen::Vector3d &value) noexcept {
  return value.allFinite();
}

} // namespace

RoiBox::RoiBox(Eigen::Vector3d minimum, Eigen::Vector3d maximum)
    : minimum_(std::move(minimum)), maximum_(std::move(maximum)) {
  if (!finite(minimum_) || !finite(maximum_) ||
      (minimum_.array() > maximum_.array()).any()) {
    throw std::invalid_argument("ROI bounds must be finite and ordered");
  }
}

const Eigen::Vector3d &RoiBox::minimum() const noexcept { return minimum_; }

const Eigen::Vector3d &RoiBox::maximum() const noexcept { return maximum_; }

bool RoiBox::contains(const Eigen::Vector3d &world_point) const noexcept {
  return finite(world_point) &&
         (world_point.array() >= minimum_.array()).all() &&
         (world_point.array() <= maximum_.array()).all();
}

LayerId Scene::addLayer(std::string source_key,
                        std::shared_ptr<const PointCloudIRGB> cloud) {
  if (source_key.empty()) {
    throw std::invalid_argument("layer source key must not be empty");
  }
  if (findLayerBySourceKey(source_key) != nullptr) {
    throw std::invalid_argument("layer source key must be unique");
  }
  if (next_layer_id_ == std::numeric_limits<LayerId>::max()) {
    throw std::overflow_error("layer ID space exhausted");
  }

  const LayerId id = next_layer_id_++;
  layers_.push_back({id, std::move(source_key), std::move(cloud)});
  return id;
}

bool Scene::removeLayer(LayerId id) {
  const auto iterator = std::find_if(layers_.begin(), layers_.end(),
                                     [id](const CloudLayer &layer) {
                                       return layer.id == id;
                                     });
  if (iterator == layers_.end()) {
    return false;
  }
  layers_.erase(iterator);
  return true;
}

CloudLayer *Scene::findLayer(LayerId id) noexcept {
  const auto iterator = std::find_if(layers_.begin(), layers_.end(),
                                     [id](const CloudLayer &layer) {
                                       return layer.id == id;
                                     });
  return iterator == layers_.end() ? nullptr : &*iterator;
}

const CloudLayer *Scene::findLayer(LayerId id) const noexcept {
  return const_cast<Scene *>(this)->findLayer(id);
}

CloudLayer *Scene::findLayerBySourceKey(const std::string &source_key) noexcept {
  const auto iterator = std::find_if(
      layers_.begin(), layers_.end(), [&source_key](const CloudLayer &layer) {
        return layer.source_key == source_key;
      });
  return iterator == layers_.end() ? nullptr : &*iterator;
}

const CloudLayer *
Scene::findLayerBySourceKey(const std::string &source_key) const noexcept {
  return const_cast<Scene *>(this)->findLayerBySourceKey(source_key);
}

const std::vector<CloudLayer> &Scene::layers() const noexcept { return layers_; }

void Scene::setRoi(std::optional<RoiBox> roi) { roi_ = std::move(roi); }

const std::optional<RoiBox> &Scene::roi() const noexcept { return roi_; }

} // namespace kpt::gui
