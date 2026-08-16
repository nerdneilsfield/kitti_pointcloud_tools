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
constexpr std::string_view kSha256SourcePrefix = "sha256:";

[[nodiscard]] bool hasPrefix(std::string_view value,
                             std::string_view prefix) noexcept {
  return value.size() >= prefix.size() &&
         value.substr(0, prefix.size()) == prefix;
}

[[nodiscard]] bool isCanonicalPathPayload(std::string_view payload) {
  if (payload.empty() || payload.find('\\') != std::string_view::npos ||
      payload.find("//") != std::string_view::npos) {
    return false;
  }
  for (const char value : payload) {
    const auto character = static_cast<unsigned char>(value);
    if (character <= 0x1fU || character == 0x7fU) {
      return false;
    }
  }

  std::size_t component_start = 0;
  if (payload.front() == '/') {
    component_start = 1;
  } else if (payload.size() >= 3U &&
             ((payload[0] >= 'A' && payload[0] <= 'Z') ||
              (payload[0] >= 'a' && payload[0] <= 'z')) &&
             payload[1] == ':' && payload[2] == '/') {
    component_start = 3;
  } else {
    return false;
  }

  while (component_start < payload.size()) {
    const std::size_t component_end = payload.find('/', component_start);
    const std::string_view component = payload.substr(
        component_start, component_end == std::string_view::npos
                             ? std::string_view::npos
                             : component_end - component_start);
    if (component.empty() || component == "." || component == "..") {
      return false;
    }
    if (component_end == std::string_view::npos) {
      break;
    }
    component_start = component_end + 1U;
  }
  return true;
}

[[nodiscard]] bool decodeUtf8Scalar(std::string_view text, std::size_t &offset,
                                    char32_t &scalar) noexcept {
  if (offset >= text.size()) {
    return false;
  }
  const auto first = static_cast<unsigned char>(text[offset]);
  if (first <= 0x7fU) {
    scalar = first;
    ++offset;
    return true;
  }

  std::size_t continuation_count = 0;
  char32_t value = 0;
  char32_t minimum = 0;
  if (first >= 0xc2U && first <= 0xdfU) {
    continuation_count = 1;
    value = first & 0x1fU;
    minimum = 0x80U;
  } else if (first >= 0xe0U && first <= 0xefU) {
    continuation_count = 2;
    value = first & 0x0fU;
    minimum = 0x800U;
  } else if (first >= 0xf0U && first <= 0xf4U) {
    continuation_count = 3;
    value = first & 0x07U;
    minimum = 0x10000U;
  } else {
    return false;
  }
  if (continuation_count >= text.size() - offset) {
    return false;
  }
  for (std::size_t index = 1; index <= continuation_count; ++index) {
    const auto continuation =
        static_cast<unsigned char>(text[offset + index]);
    if ((continuation & 0xc0U) != 0x80U) {
      return false;
    }
    value = static_cast<char32_t>((value << 6U) | (continuation & 0x3fU));
  }
  if (value < minimum || value > 0x10ffffU ||
      (value >= 0xd800U && value <= 0xdfffU)) {
    return false;
  }
  scalar = value;
  offset += continuation_count + 1U;
  return true;
}

[[nodiscard]] bool isCanonicalOpaquePayload(std::string_view payload) noexcept {
  if (payload.empty()) {
    return false;
  }
  std::size_t offset = 0;
  while (offset < payload.size()) {
    char32_t scalar = 0;
    if (!decodeUtf8Scalar(payload, offset, scalar) || scalar <= 0x1fU ||
        scalar == 0x7fU || (scalar >= 0x80U && scalar <= 0x9fU)) {
      return false;
    }
  }
  return true;
}

[[nodiscard]] bool isLowerHexDigest(std::string_view payload) noexcept {
  if (payload.size() != 64U) {
    return false;
  }
  return std::all_of(payload.begin(), payload.end(), [](const char value) {
    return (value >= '0' && value <= '9') ||
           (value >= 'a' && value <= 'f');
  });
}

[[nodiscard]] std::string normalizeSourceKey(std::string source_key) {
  if (source_key.empty()) {
    throw std::invalid_argument("source key must not be empty");
  }
  if (isCanonicalSourceKey(source_key)) {
    return source_key;
  }
  if (hasPrefix(source_key, kPathSourcePrefix) ||
      hasPrefix(source_key, kOpaqueSourcePrefix) ||
      hasPrefix(source_key, kSha256SourcePrefix)) {
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

bool isValidLayerStyle(const LayerStyle &style) noexcept {
  return std::isfinite(style.point_size) && style.point_size > 0.0F &&
         std::isfinite(style.opacity) && style.opacity >= 0.0F &&
         style.opacity <= 1.0F && std::isfinite(style.scalar_min) &&
         std::isfinite(style.scalar_max) &&
         style.scalar_min <= style.scalar_max && style.fixed_color.allFinite() &&
         style.noise_color.allFinite();
}

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
  if (!isCanonicalOpaquePayload(payload)) {
    throw std::invalid_argument(
        "opaque source payload must be valid UTF-8 non-control text");
  }
  return std::string{kOpaqueSourcePrefix} + std::string{payload};
}

bool isCanonicalSourceKey(std::string_view source_key) {
  if (hasPrefix(source_key, kPathSourcePrefix)) {
    return isCanonicalPathPayload(source_key.substr(kPathSourcePrefix.size()));
  }
  if (hasPrefix(source_key, kSha256SourcePrefix)) {
    return isLowerHexDigest(source_key.substr(kSha256SourcePrefix.size()));
  }
  return hasPrefix(source_key, kOpaqueSourcePrefix) && isCanonicalOpaquePayload(
      source_key.substr(kOpaqueSourcePrefix.size()));
}

struct Scene::ReviewState {
  std::vector<CloudLayer> layers;
  std::optional<RoiBox> roi;
  std::optional<LayerId> active_layer_id;
};

struct InspectionSettings::State {
  std::vector<CameraBookmark> bookmarks;
};

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

InspectionSettings::InspectionSettings() : state_(std::make_shared<State>()) {}

bool InspectionSettings::equalBookmark(const CameraBookmark &left,
                                       const CameraBookmark &right) noexcept {
  const CameraSnapshot &left_camera = left.camera();
  const CameraSnapshot &right_camera = right.camera();
  return left.name() == right.name() &&
         (left_camera.target.array() == right_camera.target.array()).all() &&
         (left_camera.rotation_center.array() ==
          right_camera.rotation_center.array()).all() &&
         (left_camera.camera_to_world.array() ==
          right_camera.camera_to_world.array()).all() &&
         left_camera.distance == right_camera.distance &&
         left_camera.fov_y_degrees == right_camera.fov_y_degrees;
}

void InspectionSettings::applyBookmarks(
    const std::shared_ptr<State> &state,
    const std::shared_ptr<const std::vector<CameraBookmark>> &bookmarks) {
  if (!state || !bookmarks) {
    throw std::invalid_argument("bookmark state must not be null");
  }
  // Make every potentially throwing element copy before publishing it. This
  // keeps UndoStack's retryable callback contract intact if allocation fails.
  auto copy = *bookmarks;
  state->bookmarks.swap(copy);
}

void InspectionSettings::commitBookmarks(std::vector<CameraBookmark> after) {
  const auto before =
      std::make_shared<const std::vector<CameraBookmark>>(state_->bookmarks);
  const auto next =
      std::make_shared<const std::vector<CameraBookmark>>(std::move(after));
  if (before->size() == next->size() &&
      std::equal(before->begin(), before->end(), next->begin(),
                 [](const CameraBookmark &left, const CameraBookmark &right) {
                   return equalBookmark(left, right);
                 })) {
    return;
  }
  const std::shared_ptr<State> state = state_;
  undo_stack_.execute({
      [state, before] { applyBookmarks(state, before); },
      [state, next] { applyBookmarks(state, next); },
  });
}

void InspectionSettings::saveBookmark(CameraBookmark bookmark) {
  auto after = state_->bookmarks;
  const auto iterator = std::find_if(
      after.begin(), after.end(), [&bookmark](const CameraBookmark &item) {
        return item.name() == bookmark.name();
      });
  if (iterator == after.end()) {
    after.push_back(std::move(bookmark));
  } else if (!equalBookmark(*iterator, bookmark)) {
    *iterator = std::move(bookmark);
  } else {
    return;
  }
  commitBookmarks(std::move(after));
}

bool InspectionSettings::removeBookmark(const std::string &name) {
  const auto iterator = std::find_if(
      state_->bookmarks.begin(), state_->bookmarks.end(),
      [&name](const CameraBookmark &item) {
        return item.name() == name;
      });
  if (iterator == state_->bookmarks.end()) {
    return false;
  }
  auto after = state_->bookmarks;
  after.erase(after.begin() + (iterator - state_->bookmarks.begin()));
  commitBookmarks(std::move(after));
  return true;
}

const CameraBookmark *
InspectionSettings::findBookmark(const std::string &name) const noexcept {
  const auto iterator = std::find_if(
      state_->bookmarks.begin(), state_->bookmarks.end(),
      [&name](const CameraBookmark &item) {
        return item.name() == name;
      });
  return iterator == state_->bookmarks.end() ? nullptr : &*iterator;
}

const std::vector<CameraBookmark> &InspectionSettings::bookmarks() const noexcept {
  return state_->bookmarks;
}

bool InspectionSettings::undo() { return undo_stack_.undo(); }

bool InspectionSettings::redo() { return undo_stack_.redo(); }

bool InspectionSettings::canUndo() const noexcept {
  return undo_stack_.undoCount() != 0;
}

bool InspectionSettings::canRedo() const noexcept {
  return undo_stack_.redoCount() != 0;
}

void InspectionSettings::clearHistory() noexcept { undo_stack_.clear(); }

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

const LayerStyle &CloudLayer::style() const noexcept { return style_; }

bool CloudLayer::visible() const noexcept { return visible_; }

void CloudLayer::setLocalToWorld(Eigen::Affine3d transform) {
  if (!finiteAffine(transform)) {
    throw std::invalid_argument("layer transform must be finite affine");
  }
  local_to_world_ = std::move(transform);
}

void CloudLayer::setStyle(LayerStyle style) {
  if (!isValidLayerStyle(style)) {
    throw std::invalid_argument("layer style must be finite and in range");
  }
  style_ = std::move(style);
}

void CloudLayer::setCloud(std::shared_ptr<const PointCloudIRGB> cloud) noexcept {
  cloud_ = std::move(cloud);
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
    : id_(id), first_source_key_(normalizeSourceKey(std::move(source_key))),
      first_world_(std::move(first_world)), second_world_(std::move(second_world)) {
  if (second_world_.has_value()) {
    second_source_key_ = first_source_key_;
  }
  if (id_ == 0 || first_source_key_.empty() || !finite(first_world_) ||
      (second_world_.has_value() && !finite(*second_world_))) {
    throw std::invalid_argument("measurement must have finite world points and source key");
  }
}

Measurement::Measurement(MeasurementId id, std::string first_source_key,
                         Eigen::Vector3d first_world,
                         std::string second_source_key,
                         Eigen::Vector3d second_world)
    : id_(id),
      first_source_key_(normalizeSourceKey(std::move(first_source_key))),
      second_source_key_(normalizeSourceKey(std::move(second_source_key))),
      first_world_(std::move(first_world)), second_world_(std::move(second_world)) {
  if (id_ == 0 || first_source_key_.empty() || !second_source_key_.has_value() ||
      second_source_key_->empty() || !finite(first_world_) ||
      !second_world_.has_value() || !finite(*second_world_)) {
    throw std::invalid_argument("measurement must have finite world points and source keys");
  }
}

MeasurementId Measurement::id() const noexcept { return id_; }

const std::string &Measurement::sourceKey() const noexcept {
  return first_source_key_;
}

const std::string &Measurement::firstSourceKey() const noexcept {
  return first_source_key_;
}

const std::optional<std::string> &Measurement::secondSourceKey() const noexcept {
  return second_source_key_;
}

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

std::shared_ptr<const Scene::ReviewState> Scene::reviewState() const {
  return std::make_shared<const ReviewState>(
      ReviewState{layers_, roi_, active_layer_id_});
}

bool Scene::reviewStatesEqual(const ReviewState &left,
                              const ReviewState &right) noexcept {
  if (left.active_layer_id != right.active_layer_id ||
      left.roi.has_value() != right.roi.has_value() ||
      left.layers.size() != right.layers.size()) {
    return false;
  }
  if (left.roi &&
      ((left.roi->minimum().array() != right.roi->minimum().array()).any() ||
       (left.roi->maximum().array() != right.roi->maximum().array()).any())) {
    return false;
  }
  for (std::size_t index = 0; index < left.layers.size(); ++index) {
    const CloudLayer &a = left.layers[index];
    const CloudLayer &b = right.layers[index];
    const LayerStyle &as = a.style();
    const LayerStyle &bs = b.style();
    if (a.id() != b.id() || a.sourceKey() != b.sourceKey() ||
        a.cloud() != b.cloud() || a.visible() != b.visible() ||
        (a.localToWorld().matrix().array() != b.localToWorld().matrix().array())
            .any() ||
        as.color_by != bs.color_by || as.color_map != bs.color_map ||
        as.point_size != bs.point_size || as.opacity != bs.opacity ||
        as.scalar_min != bs.scalar_min || as.scalar_max != bs.scalar_max ||
        (as.fixed_color.array() != bs.fixed_color.array()).any() ||
        (as.noise_color.array() != bs.noise_color.array()).any() ||
        as.highlight_noise != bs.highlight_noise ||
        as.intensity_equalize != bs.intensity_equalize) {
      return false;
    }
  }
  return true;
}

void Scene::applyReviewState(const std::shared_ptr<const ReviewState> &state) {
  if (!state) {
    throw std::invalid_argument("review state must not be null");
  }
  // Make every potentially throwing copy before replacing any live state.
  auto layers = state->layers;
  auto roi = state->roi;
  const auto active_layer_id = state->active_layer_id;
  layers_.swap(layers);
  roi_.swap(roi);
  active_layer_id_ = active_layer_id;
}

void Scene::commitReviewState(std::shared_ptr<const ReviewState> before,
                              std::shared_ptr<const ReviewState> after) {
  if (!before || !after || reviewStatesEqual(*before, *after)) {
    return;
  }
  undo_stack_.execute({
      [this, before] { applyReviewState(before); },
      [this, after] { applyReviewState(after); },
  });
}

void Scene::applyOrCommitReviewState(std::shared_ptr<const ReviewState> before,
                                     std::shared_ptr<const ReviewState> after) {
  if (!before || !after || reviewStatesEqual(*before, *after)) {
    return;
  }
  if (transaction_before_) {
    applyReviewState(after);
    return;
  }
  commitReviewState(std::move(before), std::move(after));
}

LayerId Scene::addLayer(std::string source_key,
                        std::shared_ptr<const PointCloudIRGB> cloud) {
  source_key = normalizeSourceKey(std::move(source_key));
  if (findLayerBySourceKey(source_key) != nullptr) {
    throw std::invalid_argument("layer source key must be unique");
  }
  if (next_layer_id_ == std::numeric_limits<LayerId>::max()) {
    throw std::overflow_error("layer ID space exhausted");
  }

  const LayerId id = next_layer_id_;
  const auto before = reviewState();
  auto after = std::make_shared<ReviewState>(*before);
  after->layers.push_back(CloudLayer{id, std::move(source_key), std::move(cloud)});
  if (!after->active_layer_id.has_value()) {
    after->active_layer_id = id;
  }
  applyOrCommitReviewState(before, after);
  ++next_layer_id_;
  return id;
}

bool Scene::removeLayer(LayerId id) {
  const auto before = reviewState();
  auto after = std::make_shared<ReviewState>(*before);
  const auto iterator = std::find_if(after->layers.begin(), after->layers.end(),
                                     [id](const CloudLayer &layer) {
                                       return layer.id() == id;
                                     });
  if (iterator == after->layers.end()) {
    return false;
  }
  after->layers.erase(iterator);
  if (after->active_layer_id == id) {
    after->active_layer_id = after->layers.empty()
                                  ? std::nullopt
                                  : std::optional<LayerId>{after->layers.front().id()};
  }
  applyOrCommitReviewState(before, after);
  return true;
}

void Scene::clearLayers() noexcept {
  layers_.clear();
  active_layer_id_.reset();
  transaction_before_.reset();
  undo_stack_.clear();
}

void Scene::resetForImport() noexcept {
  layers_.clear();
  measurements_.clear();
  roi_.reset();
  active_layer_id_.reset();
  transaction_before_.reset();
  undo_stack_.clear();
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
               !hasPrefix(source_key, kSha256SourcePrefix) &&
               hasPrefix(layer.sourceKey(), kOpaqueSourcePrefix) &&
               layer.sourceKey().substr(kOpaqueSourcePrefix.size()) == source_key;
      });
  return iterator == layers_.end() ? nullptr : &*iterator;
}

const std::vector<CloudLayer> &Scene::layers() const noexcept { return layers_; }

bool Scene::setLayerCloud(LayerId id,
                          std::shared_ptr<const PointCloudIRGB> cloud) {
  const auto before = reviewState();
  auto after = std::make_shared<ReviewState>(*before);
  const auto iterator = std::find_if(after->layers.begin(), after->layers.end(),
                                     [id](const CloudLayer &layer) {
                                       return layer.id() == id;
                                     });
  if (iterator == after->layers.end()) {
    return false;
  }
  if (iterator->cloud() == cloud) {
    return true;
  }
  iterator->setCloud(std::move(cloud));
  applyOrCommitReviewState(before, after);
  return true;
}

bool Scene::hydrateLayerCloud(
    LayerId id, std::shared_ptr<const PointCloudIRGB> cloud) noexcept {
  const auto iterator = std::find_if(layers_.begin(), layers_.end(),
                                     [id](const CloudLayer &layer) {
                                       return layer.id() == id;
                                     });
  if (iterator == layers_.end()) {
    return false;
  }
  iterator->setCloud(std::move(cloud));
  return true;
}

bool Scene::setLayerTransform(LayerId id, Eigen::Affine3d transform) {
  const auto before = reviewState();
  auto after = std::make_shared<ReviewState>(*before);
  const auto iterator = std::find_if(after->layers.begin(), after->layers.end(),
                                     [id](const CloudLayer &layer) {
                                       return layer.id() == id;
                                     });
  if (iterator == after->layers.end()) {
    return false;
  }
  iterator->setLocalToWorld(std::move(transform));
  applyOrCommitReviewState(before, after);
  return true;
}

bool Scene::setLayerStyle(LayerId id, LayerStyle style) {
  const auto before = reviewState();
  auto after = std::make_shared<ReviewState>(*before);
  const auto iterator = std::find_if(after->layers.begin(), after->layers.end(),
                                     [id](const CloudLayer &layer) {
                                       return layer.id() == id;
                                     });
  if (iterator == after->layers.end()) {
    return false;
  }
  iterator->setStyle(std::move(style));
  applyOrCommitReviewState(before, after);
  return true;
}

bool Scene::setLayerVisible(LayerId id, bool visible) {
  const auto before = reviewState();
  auto after = std::make_shared<ReviewState>(*before);
  const auto iterator = std::find_if(after->layers.begin(), after->layers.end(),
                                     [id](const CloudLayer &layer) {
                                       return layer.id() == id;
                                     });
  if (iterator == after->layers.end()) {
    return false;
  }
  iterator->setVisible(visible);
  applyOrCommitReviewState(before, after);
  return true;
}

std::optional<LayerId> Scene::activeLayer() const noexcept {
  return active_layer_id_;
}

bool Scene::setActiveLayer(std::optional<LayerId> id) {
  if (id.has_value() && findLayer(*id) == nullptr) {
    return false;
  }
  const auto before = reviewState();
  auto after = std::make_shared<ReviewState>(*before);
  after->active_layer_id = id;
  applyOrCommitReviewState(before, after);
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

MeasurementId Scene::addMeasurement(std::string first_source_key,
                                    Eigen::Vector3d first_world,
                                    std::string second_source_key,
                                    Eigen::Vector3d second_world) {
  if (next_measurement_id_ == std::numeric_limits<MeasurementId>::max()) {
    throw std::overflow_error("measurement ID space exhausted");
  }
  const MeasurementId id = next_measurement_id_;
  Measurement measurement{id, std::move(first_source_key),
                          std::move(first_world), std::move(second_source_key),
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
  if (iterator == measurements_.end()) {
    return false;
  }
  return completeMeasurement(id, iterator->firstSourceKey(),
                             std::move(second_world));
}

bool Scene::completeMeasurement(MeasurementId id, std::string second_source_key,
                                Eigen::Vector3d second_world) {
  const auto iterator = std::find_if(
      measurements_.begin(), measurements_.end(), [id](const Measurement &item) {
        return item.id() == id;
      });
  if (iterator == measurements_.end() || iterator->secondWorld().has_value()) {
    return false;
  }

  Measurement completed{id, iterator->firstSourceKey(), iterator->firstWorld(),
                        std::move(second_source_key), std::move(second_world)};
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
  if (findLayerBySourceKey(measurement.firstSourceKey()) == nullptr) {
    return true;
  }
  return measurement.secondSourceKey().has_value() &&
         findLayerBySourceKey(*measurement.secondSourceKey()) == nullptr;
}

bool Scene::undo() { return undo_stack_.undo(); }

bool Scene::redo() { return undo_stack_.redo(); }

void Scene::clearHistory() noexcept {
  transaction_before_.reset();
  undo_stack_.clear();
}

bool Scene::beginTransaction() {
  if (transaction_before_) {
    return false;
  }
  transaction_before_ = reviewState();
  return true;
}

bool Scene::commitTransaction() {
  if (!transaction_before_) {
    return false;
  }
  const auto before = std::move(transaction_before_);
  const auto after = reviewState();
  commitReviewState(before, after);
  return true;
}

bool Scene::cancelTransaction() {
  if (!transaction_before_) {
    return false;
  }
  const auto before = std::move(transaction_before_);
  applyReviewState(before);
  return true;
}

bool Scene::transactionActive() const noexcept {
  return transaction_before_ != nullptr;
}

void Scene::setRoi(std::optional<RoiBox> roi) {
  const auto before = reviewState();
  auto after = std::make_shared<ReviewState>(*before);
  after->roi = std::move(roi);
  applyOrCommitReviewState(before, after);
}

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
