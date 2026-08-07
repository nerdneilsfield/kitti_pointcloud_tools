#pragma once

#include "gui/viewport/render_types.hpp"

#include <algorithm>
#include <array>
#include <cstddef>

namespace kpt::gui::detail {

inline bool vectorEqual(const Eigen::Vector3f &left,
                        const Eigen::Vector3f &right) {
  return (left.array() == right.array()).all();
}

inline bool styleEqual(const ViewportStyle &left, const ViewportStyle &right) {
  return left.color_by == right.color_by &&
         left.color_map == right.color_map &&
         left.point_size == right.point_size &&
         vectorEqual(left.background, right.background) &&
         left.scalar_min == right.scalar_min &&
         left.scalar_max == right.scalar_max &&
         vectorEqual(left.fixed_color, right.fixed_color) &&
         vectorEqual(left.noise_color, right.noise_color) &&
         left.highlight_noise == right.highlight_noise &&
         left.intensity_equalize == right.intensity_equalize &&
         left.show_coordinate_axes == right.show_coordinate_axes &&
         left.show_scale_grid == right.show_scale_grid;
}

inline bool cdfEqual(const std::array<float, 256> &left,
                     const std::array<float, 256> &right) {
  return std::equal(left.begin(), left.end(), right.begin());
}

inline bool framesRenderEqual(const ViewportFrame &left,
                              const ViewportFrame &right) {
  if (!(left.view_projection.array() == right.view_projection.array()).all() ||
      !vectorEqual(left.world_origin, right.world_origin) ||
      left.world_scale != right.world_scale ||
      left.interactive_lod != right.interactive_lod ||
      !styleEqual(left.style, right.style) ||
      left.intensity_cdf_valid != right.intensity_cdf_valid ||
      left.guides.size() != right.guides.size()) {
    return false;
  }
  if (left.intensity_cdf_valid && !cdfEqual(left.intensity_cdf, right.intensity_cdf))
    return false;
  for (std::size_t index = 0; index < left.guides.size(); ++index) {
    if (!vectorEqual(left.guides[index].position,
                     right.guides[index].position) ||
        !vectorEqual(left.guides[index].color, right.guides[index].color)) {
      return false;
    }
  }
  return true;
}

} // namespace kpt::gui::detail
