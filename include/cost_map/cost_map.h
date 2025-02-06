#pragma once

#include <vector>
#include "grid.h"
#include "trajectory.h"
#include <memory>

namespace cev_planner::cost_map {

    /**
     * @brief CostMap allows for cost queries within an occupancy grid
     *
     */
    class CostMap {
    public:
        virtual double cost(State state) = 0;
    };

    /**
     * @brief CostMapGenerator generates a cost map from an occupancy grid and a set of waypoints
     *
     */
    class CostMapGenerator {
    public:
        /**
         * @brief Construct a new Cost Map Generator
         */
        CostMapGenerator() {}

        virtual std::unique_ptr<CostMap> generate_cost_map(Grid grid) = 0;
    };

}  // namespace cev_planner::cost_map