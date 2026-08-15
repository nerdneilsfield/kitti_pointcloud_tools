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

CloudLayer::CloudLayer(LayerId id, std::string source_key,
                       std::shared_ptr<const PointCloudIRGB> cloud)
    : id_(id), source_key_(std::move(source_key)), cloud_(std::move(cloud)) {}

LayerId CloudLayer::id() const noexcept { return id_; }

const std::string &CloudLayer::sourceKey() const noexcept { return source_key_; }

const std::shared_ptr<const PointCloudIRGB> &CloudLayer::cloud() const noexcept {
  return cloud_;
}

const Eigen::Affine3d &CloudLayer::localToWorld() const noexcept {
  return local_to_world_;
}

bool CloudLayer::visible() const noexcept { return visible_; }

void CloudLayer::setLocalToWorld(Eigen::Affine3d transform) {
  if (!transform.matrix().allFinite()) {
    throw std::invalid_argument("layer transform must be finite");
  }
  local_to_world_ = std::move(transform);
}

void CloudLayer::setVisible(bool visible) noexcept { visible_ = visible; }

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

Measurement::Measurement(MeasurementId id, std::string source_key,
                         Eigen::Vector3d first_world,
                         std::optional<Eigen::Vector3d> second_world)
    : id_(id), source_key_(std::move(source_key)),
      first_world_(std::move(first_world)),
      second_world_(std::move(second_world)) {
  if (id_ == 0 || source_key_.empty() || !finite(first_world_) ||
      (second_world_.has_value() && !finite(*second_world_))) {
    throw std::invalid_argument("measurement must have finite world points and source key");
  }
}

MeasurementId Measurement::id() const noexcept { return id_; }

const std::string &Measurement::sourceKey() const noexcept { return source_key_; }

const Eigen::Vector3d &Measurement::firstWorld() const noexcept {
  return first_world_;
}

const std::optional<Eigen::Vector3d> &Measurement::secondWorld() const noexcept {
  return second_world_;
}

std::optional<double> Measurement::distance() const noexcept {
  if (!second_world_.has_value()) {
    return std::nullopt;
  }
  return (*second_world_ - first_world_).norm();
}

UndoStack::UndoStack() {
  undo_.reserve(kCapacity);
  redo_.reserve(kCapacity);
}

void UndoStack::execute(Command command) {
  if (!command.undo || !command.redo) {
    throw std::invalid_argument("undo command needs both undo and redo actions");
  }
  command.redo();
  // Both histories have fixed capacity, so all mutations below are noexcept.
  // In particular, a successful redo can never be followed by an allocation
  // failure that would leave the external state ahead of the history.
  if (undo_.size() == kCapacity) {
    undo_.erase(undo_.begin());
  }
  undo_.push_back(std::move(command));
  redo_.clear();
}

bool UndoStack::undo() {
  if (undo_.empty()) {
    return false;
  }
  Command &command = undo_.back();
  command.undo();
  redo_.push_back(std::move(command));
  undo_.pop_back();
  return true;
}

bool UndoStack::redo() {
  if (redo_.empty()) {
    return false;
  }
  Command &command = redo_.back();
  command.redo();
  undo_.push_back(std::move(command));
  redo_.pop_back();
  return true;
}

void UndoStack::clear() noexcept {
  undo_.clear();
  redo_.clear();
}

std::size_t UndoStack::undoCount() const noexcept { return undo_.size(); }

std::size_t UndoStack::redoCount() const noexcept { return redo_.size(); }

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
  layers_.push_back(CloudLayer{id, std::move(source_key), std::move(cloud)});
  return id;
}

bool Scene::removeLayer(LayerId id) {
  const auto iterator = std::find_if(layers_.begin(), layers_.end(),
                                     [id](const CloudLayer &layer) {
                                       return layer.id() == id;
                                     });
  if (iterator == layers_.end()) {
    return false;
  }
  layers_.erase(iterator);
  return true;
}

const CloudLayer *Scene::findLayer(LayerId id) const noexcept {
  const auto iterator = std::find_if(layers_.begin(), layers_.end(),
                                     [id](const CloudLayer &layer) {
                                       return layer.id() == id;
                                     });
  return iterator == layers_.end() ? nullptr : &*iterator;
}

const CloudLayer *Scene::findLayerBySourceKey(const std::string &source_key) const noexcept {
  const auto iterator = std::find_if(
      layers_.begin(), layers_.end(), [&source_key](const CloudLayer &layer) {
        return layer.sourceKey() == source_key;
      });
  return iterator == layers_.end() ? nullptr : &*iterator;
}

const std::vector<CloudLayer> &Scene::layers() const noexcept { return layers_; }

bool Scene::setLayerTransform(LayerId id, Eigen::Affine3d transform) {
  const auto iterator = std::find_if(layers_.begin(), layers_.end(),
                                     [id](const CloudLayer &layer) {
                                       return layer.id() == id;
                                     });
  if (iterator == layers_.end()) {
    return false;
  }
  iterator->setLocalToWorld(std::move(transform));
  return true;
}

bool Scene::setLayerVisible(LayerId id, bool visible) noexcept {
  const auto iterator = std::find_if(layers_.begin(), layers_.end(),
                                     [id](const CloudLayer &layer) {
                                       return layer.id() == id;
                                     });
  if (iterator == layers_.end()) {
    return false;
  }
  iterator->setVisible(visible);
  return true;
}

MeasurementId Scene::addMeasurement(
    std::string source_key, Eigen::Vector3d first_world,
    std::optional<Eigen::Vector3d> second_world) {
  if (next_measurement_id_ == std::numeric_limits<MeasurementId>::max()) {
    throw std::overflow_error("measurement ID space exhausted");
  }
  const MeasurementId id = next_measurement_id_++;
  measurements_.emplace_back(id, std::move(source_key), std::move(first_world),
                             std::move(second_world));
  return id;
}

const std::vector<Measurement> &Scene::measurements() const noexcept {
  return measurements_;
}

bool Scene::measurementDetached(const Measurement &measurement) const noexcept {
  return findLayerBySourceKey(measurement.sourceKey()) == nullptr;
}

void Scene::setRoi(std::optional<RoiBox> roi) { roi_ = std::move(roi); }

const std::optional<RoiBox> &Scene::roi() const noexcept { return roi_; }

} // namespace kpt::gui
