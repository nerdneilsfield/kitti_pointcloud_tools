#pragma once

#include "gui/viewport/render_types.hpp"
#include "kpt/types.hpp"

#include <cstdint>
#include <memory>
#include <stop_token>

namespace kpt::gui {

// Adapt the core cloud representation to the immutable viewport contract.
CloudBounds calculateBounds(const PointCloudIRGB &cloud);

std::shared_ptr<const ViewportCloudSnapshot>
makeViewportCloudSnapshot(const PointCloudIRGBConstPtr &cloud,
                          std::uint64_t request_generation,
                          std::stop_token stop = {});

} // namespace kpt::gui
