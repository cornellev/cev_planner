#pragma once

#include <Eigen/Dense>
#include <vector>
#include <cmath>

#include <iostream>

#include "cost_map.h"
#include "vis/vis.h"

namespace cev_planner::cost_map {

    class EuclideanCostMap : public CostMap {
    private:
        Grid cost_map;

    public:
        EuclideanCostMap(Grid cost_map): cost_map(cost_map) {}
        double cost(State state) override;
    };

    class Euclidean : public CostMapGenerator {
    private:
        void edt_1d(const Eigen::Ref<const Eigen::VectorXf>& f, Eigen::Ref<Eigen::VectorXf> out);
        Grid last_cost_map_;
        bool has_last_cost_map_ = false;
 
    public:
        Euclidean(): CostMapGenerator() {}

        std::shared_ptr<CostMap> generate_cost_map(Grid grid) override;
        const Grid& grid() const { return last_cost_map_; }
        bool has_grid() const { return has_last_cost_map_; }
    };

}  // namespace cev_planner::cost_map
