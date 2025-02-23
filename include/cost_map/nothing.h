#pragma once

#include <Eigen/Dense>
#include <vector>
#include <cmath>

#include <iostream>

#include "cost_map.h"
#include "vis/vis.h"

namespace cev_planner::cost_map {

    class NothingCostMap : public CostMap {
    private:
        Grid cost_map;

    public:
        NothingCostMap(Grid cost_map): cost_map(cost_map) {}
        double cost(State state) override;
    };

    class Nothing : public CostMapGenerator {
    private:
        int search_radius;
        Eigen::VectorXf kernel;

        Eigen::VectorXf gen_kernel(int search_radius, float power);

    public:
        Nothing(int search_radius, float power): CostMapGenerator() {
            this->search_radius = search_radius;
            kernel = gen_kernel(search_radius, power);
        }

        std::shared_ptr<CostMap> generate_cost_map(Grid grid) override;
    };

}  // namespace cev_planner::cost_map