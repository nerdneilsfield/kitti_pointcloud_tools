#pragma once

#include "kpt/types.hpp"
#include "gui/viewport/model.hpp"
#include "gui/viewport/render_types.hpp"

#include <Eigen/Geometry>

#include <cstdint>
#include <filesystem>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace kpt::gui {

using LayerId = std::uint64_t;
using MeasurementId = std::uint64_t;

// Per-layer display state.  It deliberately excludes viewport-global settings
// such as background and guides: a review scene can draw several layers in one
// viewport, but it still has exactly one background and one set of guides.
struct LayerStyle {
  ColorBy color_by = ColorBy::Intensity;
  ColorMap color_map = ColorMap::Turbo;
  float point_size = 3.0F;
  float opacity = 1.0F;
  float scalar_min = 0.0F;
  float scalar_max = 1.0F;
  Eigen::Vector3f fixed_color = Eigen::Vector3f::Ones();
  Eigen::Vector3f noise_color = Eigen::Vector3f{1.0F, 0.0F, 0.0F};
  bool highlight_noise = true;
  bool intensity_equalize = true;
};

// Reject non-finite colour/scalar state before it reaches a renderer.  Opacity
// is closed [0, 1], so a layer is unambiguously opaque or transparent.
[[nodiscard]] bool isValidLayerStyle(const LayerStyle &style) noexcept;

// Stable source keys are logical identities, not a URI or content-integrity
// check. Review Share v1 accepts `path:`, `opaque:`, and
// `sha256:<64 lower-case hex>` keys unchanged across endpoints. Path keys use
// normalized generic absolute POSIX or drive-rooted syntax; keys made locally
// always use "path:<absolute, lexically-normal generic path>".
// Relative paths are resolved against the caller-supplied absolute base
// directory; absolute input deliberately ignores that base. This is lexical
// normalization only: sources may be unresolved when a shared review is
// opened.
[[nodiscard]] std::string pathSourceKey(const std::filesystem::path &path,
                                        const std::filesystem::path &base_directory);

// Opaque keys identify non-file sources (for example, a streamed capture). The
// payload is a logical identity, not interpreted as a filesystem path, and is
// stored as "opaque:<payload>". It may contain '/' or '\\', but not control
// characters.
[[nodiscard]] std::string opaqueSourceKey(std::string_view payload);

// Rejects empty, malformed, and non-canonical namespaced keys. In particular,
// sha256 keys require exactly 64 lower-case hexadecimal characters. Legacy
// unprefixed values are accepted only at Scene API boundaries and normalized
// to opaque keys for compatibility with existing callers.
[[nodiscard]] bool isCanonicalSourceKey(std::string_view source_key);

// Applies only a finite affine transform. Invalid local positions and matrices
// return no world point rather than leaking NaN/Inf into selection or export.
[[nodiscard]] std::optional<Eigen::Vector3d>
transformLocalToWorld(const Eigen::Vector3d &local_point,
                      const Eigen::Affine3d &local_to_world) noexcept;

// Application-level inspect state. Persistence is intentionally owned by the
// application shell rather than the ImGui layout SettingsStore.
class CameraBookmark {
public:
  CameraBookmark(std::string name, CameraSnapshot camera);

  [[nodiscard]] const std::string &name() const noexcept;
  [[nodiscard]] const CameraSnapshot &camera() const noexcept;

private:
  std::string name_;
  CameraSnapshot camera_;
};

// A closed, world-space box.  Points on every face belong to the ROI.
class RoiBox {
public:
  RoiBox(Eigen::Vector3d minimum, Eigen::Vector3d maximum);

  [[nodiscard]] const Eigen::Vector3d &minimum() const noexcept;
  [[nodiscard]] const Eigen::Vector3d &maximum() const noexcept;
  [[nodiscard]] bool contains(const Eigen::Vector3d &world_point) const noexcept;
  [[nodiscard]] bool containsTransformedLocal(
      const Eigen::Vector3d &local_point,
      const Eigen::Affine3d &local_to_world) const noexcept;

private:
  Eigen::Vector3d minimum_;
  Eigen::Vector3d maximum_;
};

class CloudLayer {
public:
  [[nodiscard]] LayerId id() const noexcept;
  // Stable across a share-file round trip. Runtime LayerId values are not.
  [[nodiscard]] const std::string &sourceKey() const noexcept;
  [[nodiscard]] const std::shared_ptr<const PointCloudIRGB> &cloud() const noexcept;
  [[nodiscard]] const Eigen::Affine3d &localToWorld() const noexcept;
  [[nodiscard]] const LayerStyle &style() const noexcept;
  [[nodiscard]] bool visible() const noexcept;

  // A layer identity is immutable. Scene owns source-key uniqueness; callers
  // must supply a stable non-empty key (normally normalized source identity).
  void setLocalToWorld(Eigen::Affine3d transform);
  void setStyle(LayerStyle style);
  void setCloud(std::shared_ptr<const PointCloudIRGB> cloud) noexcept;
  void setVisible(bool visible) noexcept;

private:
  friend class Scene;
  CloudLayer(LayerId id, std::string source_key,
             std::shared_ptr<const PointCloudIRGB> cloud);

  LayerId id_;
  std::string source_key_;
  std::shared_ptr<const PointCloudIRGB> cloud_;
  Eigen::Affine3d local_to_world_ = Eigen::Affine3d::Identity();
  LayerStyle style_;
  bool visible_ = true;
};

// Picked points are copied in world coordinates.  Layer transform edits must
// never move an already completed measurement.
class Measurement {
public:
  // Legacy same-source constructor.  The second endpoint inherits
  // `source_key` when supplied.
  Measurement(MeasurementId id, std::string source_key,
              Eigen::Vector3d first_world,
              std::optional<Eigen::Vector3d> second_world = std::nullopt);
  Measurement(MeasurementId id, std::string first_source_key,
              Eigen::Vector3d first_world, std::string second_source_key,
              Eigen::Vector3d second_world);

  [[nodiscard]] MeasurementId id() const noexcept;
  // Compatibility name for the first endpoint's source key.
  [[nodiscard]] const std::string &sourceKey() const noexcept;
  [[nodiscard]] const std::string &firstSourceKey() const noexcept;
  [[nodiscard]] const std::optional<std::string> &secondSourceKey() const noexcept;
  [[nodiscard]] const Eigen::Vector3d &firstWorld() const noexcept;
  [[nodiscard]] const std::optional<Eigen::Vector3d> &secondWorld() const noexcept;
  [[nodiscard]] std::optional<double> distance() const noexcept;

private:
  MeasurementId id_;
  std::string first_source_key_;
  std::optional<std::string> second_source_key_;
  Eigen::Vector3d first_world_;
  std::optional<Eigen::Vector3d> second_world_;
};

class UndoStack {
public:
  static constexpr std::size_t kCapacity = 100;

  struct Command {
    std::function<void()> undo;
    std::function<void()> redo;
  };

  // Reserves the bounded history before any command callback may mutate state.
  // This makes execute/undo/redo bookkeeping allocation-free.
  UndoStack();

  // Runs redo, then records command. An exception leaves history untouched.
  void execute(Command command);
  [[nodiscard]] bool undo();
  [[nodiscard]] bool redo();
  void clear() noexcept;
  [[nodiscard]] std::size_t undoCount() const noexcept;
  [[nodiscard]] std::size_t redoCount() const noexcept;

private:
  std::vector<Command> undo_;
  std::vector<Command> redo_;
};

class InspectionSettings {
public:
  InspectionSettings();
  InspectionSettings(const InspectionSettings &) = delete;
  InspectionSettings &operator=(const InspectionSettings &) = delete;
  InspectionSettings(InspectionSettings &&) noexcept = default;
  InspectionSettings &operator=(InspectionSettings &&) noexcept = default;

  // Replaces an existing bookmark of the same name; names are stable keys.
  // The mutation is command-backed and can be reverted through undo()/redo().
  // A byte-for-byte identical replacement is intentionally a no-op.
  void saveBookmark(CameraBookmark bookmark);
  [[nodiscard]] bool removeBookmark(const std::string &name);
  [[nodiscard]] const CameraBookmark *findBookmark(const std::string &name) const noexcept;
  [[nodiscard]] const std::vector<CameraBookmark> &bookmarks() const noexcept;
  [[nodiscard]] bool undo();
  [[nodiscard]] bool redo();
  [[nodiscard]] bool canUndo() const noexcept;
  [[nodiscard]] bool canRedo() const noexcept;

  // Settings loaded from disk establish a new history root; they must not
  // become undoable mutations in the current application session.
  void clearHistory() noexcept;

private:
  struct State;

  std::shared_ptr<State> state_;
  UndoStack undo_stack_;

  static bool equalBookmark(const CameraBookmark &left,
                            const CameraBookmark &right) noexcept;
  static void applyBookmarks(
      const std::shared_ptr<State> &state,
      const std::shared_ptr<const std::vector<CameraBookmark>> &bookmarks);
  void commitBookmarks(std::vector<CameraBookmark> after);
};

class Scene {
public:
  [[nodiscard]] LayerId addLayer(
      std::string source_key,
      std::shared_ptr<const PointCloudIRGB> cloud = {});
  [[nodiscard]] bool removeLayer(LayerId id);
  // Source replacement clears every runtime layer and active-layer reference.
  // Measurements remain as detached world-space history.
  void clearLayers() noexcept;
  // Starts a new externally loaded review document. Unlike clearLayers(),
  // this intentionally drops detached measurement history and ROI/undo state;
  // runtime ID counters remain monotonic and imported layers receive fresh IDs.
  void resetForImport() noexcept;

  // Layers expose no mutable identity. Apply edits through Scene mutators.
  [[nodiscard]] const CloudLayer *findLayer(LayerId id) const noexcept;
  [[nodiscard]] const CloudLayer *
  findLayerBySourceKey(const std::string &source_key) const noexcept;
  [[nodiscard]] const std::vector<CloudLayer> &layers() const noexcept;
  // Share-file import first creates unresolved layers with a null cloud, then
  // hydrates each one as asynchronous loading succeeds. Identity, transform,
  // style and visibility remain unchanged and the operation is undoable.
  [[nodiscard]] bool
  setLayerCloud(LayerId id, std::shared_ptr<const PointCloudIRGB> cloud);
  [[nodiscard]] bool setLayerTransform(LayerId id, Eigen::Affine3d transform);
  [[nodiscard]] bool setLayerStyle(LayerId id, LayerStyle style);
  [[nodiscard]] bool setLayerVisible(LayerId id, bool visible);
  [[nodiscard]] std::optional<LayerId> activeLayer() const noexcept;
  [[nodiscard]] bool setActiveLayer(std::optional<LayerId> id);

  [[nodiscard]] MeasurementId addMeasurement(
      std::string source_key, Eigen::Vector3d first_world,
      std::optional<Eigen::Vector3d> second_world = std::nullopt);
  [[nodiscard]] MeasurementId addMeasurement(
      std::string first_source_key, Eigen::Vector3d first_world,
      std::string second_source_key, Eigen::Vector3d second_world);
  // Interactive picking creates one pending measurement then completes that
  // same ID on its second pick. A completed measurement is never recreated.
  [[nodiscard]] MeasurementId beginMeasurement(std::string source_key,
                                                Eigen::Vector3d first_world);
  [[nodiscard]] bool completeMeasurement(MeasurementId id,
                                         Eigen::Vector3d second_world);
  [[nodiscard]] bool completeMeasurement(MeasurementId id,
                                         std::string second_source_key,
                                         Eigen::Vector3d second_world);
  [[nodiscard]] bool clearMeasurements();
  [[nodiscard]] const std::vector<Measurement> &measurements() const noexcept;
  [[nodiscard]] bool measurementDetached(const Measurement &measurement) const noexcept;

  // Measurement edits are command-backed. The history is bounded by
  // UndoStack::kCapacity; failed callbacks leave its bookkeeping unchanged.
  [[nodiscard]] bool undo();
  [[nodiscard]] bool redo();
  // Imported documents establish a new history root. This never changes
  // visible scene data; it only prevents an import from undoing into a prior
  // review document or into its own construction steps.
  void clearHistory() noexcept;

  // Review edits can be grouped while an ImGui drag is active. Mutators apply
  // immediately inside a transaction and add exactly one bounded undo command
  // when committed. Measurement edits remain independently command-backed.
  [[nodiscard]] bool beginTransaction();
  [[nodiscard]] bool commitTransaction();
  [[nodiscard]] bool cancelTransaction();
  [[nodiscard]] bool transactionActive() const noexcept;

  void setRoi(std::optional<RoiBox> roi);
  [[nodiscard]] const std::optional<RoiBox> &roi() const noexcept;

private:
  struct ReviewState;

  std::vector<CloudLayer> layers_;
  std::vector<Measurement> measurements_;
  std::optional<RoiBox> roi_;
  UndoStack undo_stack_;
  LayerId next_layer_id_ = 1;
  MeasurementId next_measurement_id_ = 1;
  std::optional<LayerId> active_layer_id_;
  std::shared_ptr<const ReviewState> transaction_before_;

  [[nodiscard]] std::shared_ptr<const ReviewState> reviewState() const;
  [[nodiscard]] static bool reviewStatesEqual(const ReviewState &left,
                                              const ReviewState &right) noexcept;
  void applyReviewState(const std::shared_ptr<const ReviewState> &state);
  void commitReviewState(std::shared_ptr<const ReviewState> before,
                         std::shared_ptr<const ReviewState> after);
  void applyOrCommitReviewState(std::shared_ptr<const ReviewState> before,
                                std::shared_ptr<const ReviewState> after);
  void applyMeasurements(
      const std::shared_ptr<const std::vector<Measurement>> &snapshot);
  void commitMeasurements(std::vector<Measurement> after);
};

} // namespace kpt::gui
