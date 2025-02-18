#include "cost_map/nothing.h"

#include <iostream>

namespace cev_planner::cost_map {

    double NothingCostMap::cost(State state) {
        // Convert the state to grid coordinates
        int x = (state.pose.x - cost_map.origin.x) / cost_map.resolution;
        int y = (state.pose.y - cost_map.origin.y) / cost_map.resolution;

        // Check if the state is within the bounds of the cost map
        if (x < 0 || x >= cost_map.data.rows() || y < 0 || y >= cost_map.data.cols()) {
            return std::numeric_limits<double>::max();
        }

        return cost_map.data(x, y);
    }

    // Eigen::VectorXf NearestGenerator::gen_kernel(int search_radius, float power) {
    //     int kernel_size = 2 * search_radius + 1;
    //     Eigen::VectorXf kernel(kernel_size);

    //     for (int i = 0; i < kernel_size; ++i) {
    //         int dist = i - search_radius;
    //         if (dist == 0) {
    //             kernel(i) = 1.0f;
    //         } else {
    //             kernel(i) = pow(1.0f / dist, power);
    //         }
    //     }
    //     return kernel;
    // }

    Eigen::VectorXf Nothing::gen_kernel(int search_radius, float sigma) {
        int kernel_size = 2 * search_radius + 1;
        Eigen::VectorXf kernel(kernel_size);
        float sum = 0.0f;

        for (int i = 0; i < kernel_size; ++i) {
            int dist = i - search_radius;
            kernel(i) = exp(-(dist * dist) / (2 * sigma * sigma));
            sum += kernel(i);
        }

        // Normalize the kernel to ensure the sum is 1
        // kernel /= sum;

        return kernel;
    }

    std::unique_ptr<CostMap> Nothing::generate_cost_map(Grid grid) {
        // Convolution along rows
        Eigen::MatrixXf cost_map = Eigen::MatrixXf::Zero(grid.data.rows(), grid.data.cols());

        for (int i = 0; i < grid.data.rows(); ++i) {
            for (int j = 0; j < grid.data.cols(); ++j) {
                if (grid.data(i, j) < 0.0) {
                    cost_map(i, j) = .3;
                } else if (grid.data(i, j) < .5) {
                    cost_map(i, j) = 0;
                } else {
                    cost_map(i, j) = 1;
                }
            }
        }

        Grid cost_map_ = Grid();
        cost_map_.data = cost_map;
        cost_map_.origin = grid.origin;
        cost_map_.resolution = grid.resolution;

        cev_planner::vis::vis_costmap(grid, cost_map_);

        return std::make_unique<NothingCostMap>(cost_map_);
    }
}