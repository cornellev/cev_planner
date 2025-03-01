#include "cost_map/nearest.h"

#include <iostream>

namespace cev_planner::cost_map {

    double NearestCostMap::cost(State state) {
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

    Eigen::VectorXf NearestGenerator::gen_kernel(int search_radius, float sigma) {
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

    std::shared_ptr<CostMap> NearestGenerator::generate_cost_map(Grid grid) {
        // Convolution along rows
        Eigen::MatrixXf row_conv = Eigen::MatrixXf::Zero(grid.data.rows(), grid.data.cols());

        for (int i = 0; i < grid.data.rows(); ++i) {
            for (int j = 0; j < grid.data.cols(); ++j) {
                if (grid.data(i, j) < 0.0) {
                    grid.data(i, j) = .3;
                }
            }
        }

        for (int i = 0; i < grid.data.rows(); ++i) {
            for (int j = 0; j < grid.data.cols(); ++j) {
                float sum = 0.0f;
                for (int k = -search_radius; k <= search_radius; ++k) {
                    int idx = j + k;
                    if (idx >= 0 && idx < grid.data.cols()) {
                        float tmp = grid.data(i, idx) * kernel(k + search_radius);

                        sum = std::max(sum, tmp);
                    }
                }
                row_conv(i, j) = sum;
            }
        }

        // Convolution along columns
        Eigen::MatrixXf cost_map = Eigen::MatrixXf::Zero(grid.data.rows(), grid.data.cols());
        for (int j = 0; j < row_conv.cols(); ++j) {
            for (int i = 0; i < row_conv.rows(); ++i) {
                float sum = 0.0f;
                for (int k = -search_radius; k <= search_radius; ++k) {
                    int idx = i + k;
                    if (idx >= 0 && idx < row_conv.rows()) {
                        float tmp = row_conv(idx, j) * kernel(k + search_radius);

                        sum = std::max(sum, tmp);
                    }
                }
                cost_map(i, j) = sum;
            }
        }

        Grid cost_map_ = Grid();
        cost_map_.data = cost_map;
        cost_map_.origin = grid.origin;
        cost_map_.resolution = grid.resolution;

        cev_planner::vis::vis_costmap(grid, cost_map_);

        return std::make_shared<NearestCostMap>(cost_map_);
    }
}