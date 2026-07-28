#pragma once

#include "gui/viewport/render_types.hpp"
#include "kpt/types.hpp"

#include <cstdint>
#include <memory>

namespace kpt::gui {

std::shared_ptr<const ViewportCloudSnapshot>
makeViewportCloudSnapshot(const PointCloudIRGBConstPtr &cloud,
                          std::uint64_t request_generation);

} // namespace kpt::gui
