#include "cost_map/global.h"

#include <iostream>

namespace cev_planner::cost_map {

    double GlobalCostMap::cost(State state) {
        // Convert the state to grid coordinates
        int x = (state.pose.x - cost_map.origin.x) / cost_map.resolution;
        int y = (state.pose.y - cost_map.origin.y) / cost_map.resolution;

        // Check if the state is within the bounds of the cost map
        if (x < 0 || x >= cost_map.data.rows() || y < 0 || y >= cost_map.data.cols()) {
            return std::numeric_limits<double>::max();
        }

        return cost_map.data(x, y);
    }

    std::shared_ptr<CostMap> GlobalCostMapGenerator::generate_cost_map(Grid grid, Grid* scan) {
        // Convolution along rows
        Eigen::MatrixXf cost_map = Eigen::MatrixXf::Zero(grid.data.rows(), grid.data.cols());

        for (int i = search_radius; i < grid.data.rows() - search_radius - 1; i++) {
            for (int j = search_radius; j < grid.data.cols() - search_radius - 1; j++) {
                if (grid.data(i, j) < 0.0) {
                    cost_map(i, j) = .3;
                } else if (grid.data(i, j) < .5) {
                    cost_map(i, j) = 0;
                } else {
                    cost_map(i, j) = 1;
                }

                // Expand outward
                for (int a = -search_radius; a < search_radius + 1; a++) {
                    for (int b = -search_radius; b < search_radius + 1; b++) {
                        if (cost_map(i + a, j + b) < cost_map(i, j)) {
                            cost_map(i + a, j + b) = cost_map(i, j);
                        }
                    }
                }
            }
        }

        Grid cost_map_ = Grid();
        cost_map_.data = cost_map;
        cost_map_.origin = grid.origin;
        cost_map_.resolution = grid.resolution;

        // cev_planner::vis::vis_costmap(grid, cost_map_);

        return std::make_shared<GlobalCostMap>(cost_map_);
    }
}