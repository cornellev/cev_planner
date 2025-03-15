#pragma once

#include <vector>
#include "grid.h"
#include "trajectory.h"
#include <memory>
#include "cost_map.h"

namespace cev_planner::cost_map {

    class SimpleCostMap : public CostMap {
    private:
        Grid cost_map;

    public:
        /**
         * @brief Construct a new Scan Cost Map object
         *
         * @param cost_map Cost map
         * @param kernel Cost kernel
         * @param store_radius Radius to cache costs
         */
        SimpleCostMap(Grid cost_map) {
            this->cost_map = cost_map;
        }

        double cost(State state) override {
            // Convert the state to grid coordinates
            int x = (state.pose.x - cost_map.origin.x) / cost_map.resolution;
            int y = (state.pose.y - cost_map.origin.y) / cost_map.resolution;

            // Check if the state is within the bounds of the cost map
            if (x < 0 || x >= cost_map.data.rows() || y < 0 || y >= cost_map.data.cols()) {
                return std::numeric_limits<double>::max();
            }

            return cost_map.data(x, y);
        }

        double debug_(int i, int j) override {
            return cost_map.data(i, j);
        }

        std::shared_ptr<CostMap> toCostmap() {
            return std::make_shared<SimpleCostMap>(cost_map);
        }
    };

}  // namespace cev_planner::cost_map