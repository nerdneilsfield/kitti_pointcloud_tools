#pragma once

#include "gui/viewport/render_types.hpp"
#include "kpt/types.hpp"

#include <cstdint>
#include <memory>

namespace kpt::gui {

// Adapt the core cloud representation to the immutable viewport contract.
CloudBounds calculateBounds(const PointCloudIRGB &cloud);

std::shared_ptr<const ViewportCloudSnapshot>
makeViewportCloudSnapshot(const PointCloudIRGBConstPtr &cloud,
                          std::uint64_t request_generation);

} // namespace kpt::gui
