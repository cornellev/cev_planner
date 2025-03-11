#pragma once

#include <Eigen/Dense>
#include <vector>
#include <cmath>

#include <iostream>

#include "cost_map.h"
#include "vis/vis.h"

namespace cev_planner::cost_map {

    class GlobalCostMap : public CostMap {
    private:
        Grid cost_map;

    public:
        GlobalCostMap(Grid cost_map): cost_map(cost_map) {}
        double cost(State state) override;

        double debug_(int i, int j) override;
    };

    /**
     * @brief Generates a CostMap from a grid by dilating obstacles outwards
     *
     */
    class GlobalCostMapGenerator : public CostMapGenerator {
    private:
        int search_radius;

    public:
        GlobalCostMapGenerator(int dilation): CostMapGenerator() {
            search_radius = dilation;
        }

        std::shared_ptr<CostMap> generate_cost_map(Grid grid, Grid* scan = nullptr) override;
    };

}  // namespace cev_planner::cost_map