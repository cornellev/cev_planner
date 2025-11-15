#pragma once

#include <bits/stdc++.h>

#include "cost_map.h"
#include "vis/vis.h"

namespace cev_planner::cost_map {

    class DistCostMap : public CostMap {
    private:
        Grid cost_map;

    public:
        DistCostMap(Grid cost_map): cost_map(cost_map) {}
        double cost(State state) override;
    };

        /**
     * @brief DistGenerator cost map generator generates a cost map by calculating distances to nearest obstacles.
     *
     */
    class DistGenerator : public CostMapGenerator {
    private:
        int radius; // car radius
        int k; // heuristic classifier

    public:

        DistGenerator(int radius, int k) : radius(radius), k(k) {}

        std::shared_ptr<CostMap> generate_cost_map(Grid grid) override;
    };

}  // namespace cev_planner::cost_map