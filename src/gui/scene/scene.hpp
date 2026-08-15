#pragma once

#include "kpt/types.hpp"

#include <Eigen/Geometry>

#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace kpt::gui {

using LayerId = std::uint64_t;
using MeasurementId = std::uint64_t;

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

// Picked points are copied in world coordinates.  Layer transform edits must
// never move an already completed measurement.
class Measurement {
public:
  Measurement(MeasurementId id, std::string source_key,
              Eigen::Vector3d first_world,
              std::optional<Eigen::Vector3d> second_world = std::nullopt);

  [[nodiscard]] MeasurementId id() const noexcept;
  [[nodiscard]] const std::string &sourceKey() const noexcept;
  [[nodiscard]] const Eigen::Vector3d &firstWorld() const noexcept;
  [[nodiscard]] const std::optional<Eigen::Vector3d> &secondWorld() const noexcept;
  [[nodiscard]] std::optional<double> distance() const noexcept;

private:
  MeasurementId id_;
  std::string source_key_;
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

  [[nodiscard]] MeasurementId addMeasurement(
      std::string source_key, Eigen::Vector3d first_world,
      std::optional<Eigen::Vector3d> second_world = std::nullopt);
  [[nodiscard]] const std::vector<Measurement> &measurements() const noexcept;
  [[nodiscard]] bool measurementDetached(const Measurement &measurement) const noexcept;

  void setRoi(std::optional<RoiBox> roi);
  [[nodiscard]] const std::optional<RoiBox> &roi() const noexcept;

private:
  std::vector<CloudLayer> layers_;
  std::vector<Measurement> measurements_;
  std::optional<RoiBox> roi_;
  LayerId next_layer_id_ = 1;
  MeasurementId next_measurement_id_ = 1;
};

} // namespace kpt::gui
