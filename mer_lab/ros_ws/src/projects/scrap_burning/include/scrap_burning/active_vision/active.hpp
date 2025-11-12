#ifndef SCRAP_BURNING_ACTIVE_VISION_ACTIVE_HPP
#define SCRAP_BURNING_ACTIVE_VISION_ACTIVE_HPP

#include "scrap_burning/active_vision/core.hpp"
#include "scrap_burning/active_vision/grid.hpp"
#include "scrap_burning/active_vision/camera.hpp"

namespace scrap_burning {
  namespace active_vision {
    constexpr double UNKNOWN_CELL_STD_DEV = 1000.0;

    // Compute the quality of a ray r using Map tree
    // The previous variance is the current map variances
    double computeRayQuality(const OctoRay &r, const Map *tree, const KeyMap &proxMap, double rayLength);

    std::pair<std::vector<double>, std::vector<octomap::OcTreeKey>> getRayCellScores(const OctoRay &r, const Map *tree, const KeyMap &proxMap, double rayLength);

    double evalViewpoint(const scrap_burning::active_vision::Camera &cam, const Map *tree,
			 KeyMap &cache, const KeyMap &proxMap, double rayLength);
  } // active_vision
}   // scrap_burning

#endif	// SCRAP_BURNING_ACTIVE_VISION_ACTIVE_HPP
