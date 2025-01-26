#pragma once

#include <vector>

#include "pose.h"

namespace cev_planner {

    /**
     * @brief Grid represents an occupancy grid with probability of occupancy at each cell
     *
     */
    struct Grid {
        // Probability of occupancy at each cell. X axis is the outer index, Y axis is the inner
        // index
        std::vector<std::vector<float>> data;

        // Origin of the grid in meters
        Pose origin;

        // Resolution of the grid in meters per cell
        double resolution;
    };

}  // namespace cev_planner