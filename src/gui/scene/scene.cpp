#include "gui/scene/scene.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace kpt::gui {
namespace {

constexpr std::string_view kPathSourcePrefix = "path:";
constexpr std::string_view kOpaqueSourcePrefix = "opaque:";

[[nodiscard]] bool hasPrefix(std::string_view value,
                             std::string_view prefix) noexcept {
  return value.size() >= prefix.size() &&
         value.substr(0, prefix.size()) == prefix;
}

[[nodiscard]] bool isCanonicalPathPayload(std::string_view payload) {
  if (payload.empty()) {
    return false;
  }
  const std::filesystem::path path{std::string{payload}};
  return path.is_absolute() && path.lexically_normal().generic_string() == payload;
}

[[nodiscard]] std::string normalizeSourceKey(std::string source_key) {
  if (source_key.empty()) {
    throw std::invalid_argument("source key must not be empty");
  }
  if (isCanonicalSourceKey(source_key)) {
    return source_key;
  }
  if (hasPrefix(source_key, kPathSourcePrefix) ||
      hasPrefix(source_key, kOpaqueSourcePrefix)) {
    throw std::invalid_argument("source key must be canonical");
  }
  // Existing callers supplied plain strings. Preserve that API, but store
  // every new value in the unambiguous opaque namespace.
  return opaqueSourceKey(source_key);
}

[[nodiscard]] bool finite(const Eigen::Vector3d &value) noexcept {
  return value.allFinite();
}

[[nodiscard]] bool finiteAffine(const Eigen::Affine3d &transform) noexcept {
  const Eigen::Vector4d expected_bottom_row{0.0, 0.0, 0.0, 1.0};
  return transform.matrix().allFinite() &&
         transform.matrix().row(3).isApprox(expected_bottom_row.transpose());
}

} // namespace

std::string pathSourceKey(const std::filesystem::path &path,
                          const std::filesystem::path &base_directory) {
  if (path.empty()) {
    throw std::invalid_argument("source path must not be empty");
  }

  std::filesystem::path absolute_path;
  if (path.is_absolute()) {
    absolute_path = path;
  } else {
    if (base_directory.empty() || !base_directory.is_absolute()) {
      throw std::invalid_argument(
          "relative source path requires an absolute base directory");
    }
    absolute_path = base_directory / path;
  }

  const auto normalized = absolute_path.lexically_normal().generic_string();
  if (!isCanonicalPathPayload(normalized)) {
    throw std::invalid_argument("source path must normalize to an absolute path");
  }
  return std::string{kPathSourcePrefix} + normalized;
}

std::string opaqueSourceKey(std::string_view payload) {
  if (payload.empty()) {
    throw std::invalid_argument("opaque source payload must not be empty");
  }
  return std::string{kOpaqueSourcePrefix} + std::string{payload};
}

bool isCanonicalSourceKey(std::string_view source_key) {
  if (hasPrefix(source_key, kPathSourcePrefix)) {
    return isCanonicalPathPayload(source_key.substr(kPathSourcePrefix.size()));
  }
  return hasPrefix(source_key, kOpaqueSourcePrefix) &&
         source_key.size() > kOpaqueSourcePrefix.size();
}

std::optional<Eigen::Vector3d>
transformLocalToWorld(const Eigen::Vector3d &local_point,
                      const Eigen::Affine3d &local_to_world) noexcept {
  if (!finite(local_point) || !finiteAffine(local_to_world)) {
    return std::nullopt;
  }
  const Eigen::Vector3d world_point = local_to_world * local_point;
  return finite(world_point) ? std::optional<Eigen::Vector3d>{world_point}
                             : std::nullopt;
}

CameraBookmark::CameraBookmark(std::string name, CameraSnapshot camera)
    : name_(std::move(name)), camera_(std::move(camera)) {
  if (name_.empty()) {
    throw std::invalid_argument("bookmark name must not be empty");
  }
}

const std::string &CameraBookmark::name() const noexcept { return name_; }

const CameraSnapshot &CameraBookmark::camera() const noexcept { return camera_; }

void InspectionSettings::saveBookmark(CameraBookmark bookmark) {
  const auto iterator = std::find_if(
      bookmarks_.begin(), bookmarks_.end(), [&bookmark](const CameraBookmark &item) {
        return item.name() == bookmark.name();
      });
  if (iterator == bookmarks_.end()) {
    bookmarks_.push_back(std::move(bookmark));
  } else {
    *iterator = std::move(bookmark);
  }
}

bool InspectionSettings::removeBookmark(const std::string &name) noexcept {
  const auto iterator = std::find_if(
      bookmarks_.begin(), bookmarks_.end(), [&name](const CameraBookmark &item) {
        return item.name() == name;
      });
  if (iterator == bookmarks_.end()) {
    return false;
  }
  bookmarks_.erase(iterator);
  return true;
}

const CameraBookmark *
InspectionSettings::findBookmark(const std::string &name) const noexcept {
  const auto iterator = std::find_if(
      bookmarks_.begin(), bookmarks_.end(), [&name](const CameraBookmark &item) {
        return item.name() == name;
      });
  return iterator == bookmarks_.end() ? nullptr : &*iterator;
}

const std::vector<CameraBookmark> &InspectionSettings::bookmarks() const noexcept {
  return bookmarks_;
}

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
  if (!finiteAffine(transform)) {
    throw std::invalid_argument("layer transform must be finite affine");
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

bool RoiBox::containsTransformedLocal(
    const Eigen::Vector3d &local_point,
    const Eigen::Affine3d &local_to_world) const noexcept {
  const auto world_point = transformLocalToWorld(local_point, local_to_world);
  return world_point.has_value() && contains(*world_point);
}

Measurement::Measurement(MeasurementId id, std::string source_key,
                         Eigen::Vector3d first_world,
                         std::optional<Eigen::Vector3d> second_world)
    : id_(id), source_key_(normalizeSourceKey(std::move(source_key))),
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
  source_key = normalizeSourceKey(std::move(source_key));
  if (findLayerBySourceKey(source_key) != nullptr) {
    throw std::invalid_argument("layer source key must be unique");
  }
  if (next_layer_id_ == std::numeric_limits<LayerId>::max()) {
    throw std::overflow_error("layer ID space exhausted");
  }

  const LayerId id = next_layer_id_++;
  layers_.push_back(CloudLayer{id, std::move(source_key), std::move(cloud)});
  if (!active_layer_id_.has_value()) {
    active_layer_id_ = id;
  }
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
  if (active_layer_id_ == id) {
    active_layer_id_ = layers_.empty() ? std::nullopt
                                       : std::optional<LayerId>{layers_.front().id()};
  }
  return true;
}

void Scene::clearLayers() noexcept {
  layers_.clear();
  active_layer_id_.reset();
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
        if (layer.sourceKey() == source_key) {
          return true;
        }
        // Compatibility lookup for pre-namespaced callers. Do not allocate in
        // this noexcept query; stored legacy values are always opaque keys.
        return !hasPrefix(source_key, kPathSourcePrefix) &&
               !hasPrefix(source_key, kOpaqueSourcePrefix) &&
               hasPrefix(layer.sourceKey(), kOpaqueSourcePrefix) &&
               layer.sourceKey().substr(kOpaqueSourcePrefix.size()) == source_key;
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

std::optional<LayerId> Scene::activeLayer() const noexcept {
  return active_layer_id_;
}

bool Scene::setActiveLayer(std::optional<LayerId> id) noexcept {
  if (id.has_value() && findLayer(*id) == nullptr) {
    return false;
  }
  active_layer_id_ = id;
  return true;
}

MeasurementId Scene::addMeasurement(
    std::string source_key, Eigen::Vector3d first_world,
    std::optional<Eigen::Vector3d> second_world) {
  if (!second_world.has_value()) {
    return beginMeasurement(std::move(source_key), std::move(first_world));
  }
  if (next_measurement_id_ == std::numeric_limits<MeasurementId>::max()) {
    throw std::overflow_error("measurement ID space exhausted");
  }
  const MeasurementId id = next_measurement_id_;
  Measurement measurement{id, std::move(source_key), std::move(first_world),
                          std::move(second_world)};
  auto after = measurements_;
  after.push_back(std::move(measurement));
  commitMeasurements(std::move(after));
  ++next_measurement_id_;
  return id;
}

MeasurementId Scene::beginMeasurement(std::string source_key,
                                      Eigen::Vector3d first_world) {
  if (next_measurement_id_ == std::numeric_limits<MeasurementId>::max()) {
    throw std::overflow_error("measurement ID space exhausted");
  }
  const MeasurementId id = next_measurement_id_;
  Measurement measurement{id, std::move(source_key), std::move(first_world)};
  auto after = measurements_;
  after.push_back(std::move(measurement));
  commitMeasurements(std::move(after));
  ++next_measurement_id_;
  return id;
}

bool Scene::completeMeasurement(MeasurementId id, Eigen::Vector3d second_world) {
  const auto iterator = std::find_if(
      measurements_.begin(), measurements_.end(), [id](const Measurement &item) {
        return item.id() == id;
      });
  if (iterator == measurements_.end() || iterator->secondWorld().has_value()) {
    return false;
  }

  Measurement completed{id, iterator->sourceKey(), iterator->firstWorld(),
                        std::move(second_world)};
  auto after = measurements_;
  after[static_cast<std::size_t>(iterator - measurements_.begin())] =
      std::move(completed);
  commitMeasurements(std::move(after));
  return true;
}

bool Scene::clearMeasurements() {
  if (measurements_.empty()) {
    return false;
  }
  commitMeasurements({});
  return true;
}

const std::vector<Measurement> &Scene::measurements() const noexcept {
  return measurements_;
}

bool Scene::measurementDetached(const Measurement &measurement) const noexcept {
  return findLayerBySourceKey(measurement.sourceKey()) == nullptr;
}

bool Scene::undo() { return undo_stack_.undo(); }

bool Scene::redo() { return undo_stack_.redo(); }

void Scene::setRoi(std::optional<RoiBox> roi) { roi_ = std::move(roi); }

const std::optional<RoiBox> &Scene::roi() const noexcept { return roi_; }

void Scene::applyMeasurements(
    const std::shared_ptr<const std::vector<Measurement>> &snapshot) {
  // Copy before swap: allocation failure leaves current scene state intact.
  auto replacement = *snapshot;
  measurements_.swap(replacement);
}

void Scene::commitMeasurements(std::vector<Measurement> after) {
  const auto before_snapshot =
      std::make_shared<const std::vector<Measurement>>(measurements_);
  const auto after_snapshot =
      std::make_shared<const std::vector<Measurement>>(std::move(after));
  undo_stack_.execute({
      [this, before_snapshot] { applyMeasurements(before_snapshot); },
      [this, after_snapshot] { applyMeasurements(after_snapshot); },
  });
}

} // namespace kpt::gui
