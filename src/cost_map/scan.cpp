#include "cost_map/scan.h"

#include <iostream>

namespace cev_planner::cost_map {
    ScanCostMap::ScanCostMap(Grid cost_map, Eigen::VectorXf kernel, int store_radius,
        Pose cache_origin)
        : cost_map(cost_map), kernel(kernel) {
        // Make a -1 filled cache of size (store_radius * 2 + 1) by (store_radius * 2 + 1) centered
        // at cache_origin
        // cache = Grid();
        // cache.origin = cache_origin;
        // cache.resolution = cost_map.resolution;
        // cache.data = Eigen::MatrixXf::Ones(store_radius * 2 + 1, store_radius * 2 + 1);
        // cache.data *= -1;
    }

    double ScanCostMap::cost(State state) {
        // Convert the state to grid coordinates
        int x = (state.pose.x - cost_map.origin.x) / cost_map.resolution;
        int y = (state.pose.y - cost_map.origin.y) / cost_map.resolution;

        // Check if the state is within the bounds of the cost map
        if (x < 0 || x >= cost_map.data.rows() || y < 0 || y >= cost_map.data.cols()) {
            return std::numeric_limits<double>::max();
        }

        return cost_map.data(x, y);
    }

    // double ScanCostMap::cost(State state) {
    //     // Convert the state to grid coordinates
    //     int x = (state.pose.x - cache.origin.x) / cache.resolution;
    //     int y = (state.pose.y - cache.origin.y) / cache.resolution;

    //     // Check if the state is within the bounds of the cache
    //     if (x < 0 || x >= cache.data.rows() || y < 0 || y >= cache.data.cols()) {
    //         return std::numeric_limits<double>::max();
    //     }

    //     // If the cache is filled, return the value
    //     if (cache.data(x, y) >= 0) {
    //         return cache.data(x, y);
    //     }

    //     // Otherwise, compute the cost by convolving kernel by position in original cost_map
    //     float sum = 0.0f;
    //     int kernel_radius = kernel.size() / 2;

    //     int cost_map_x = (state.pose.x - cost_map.origin.x) / cost_map.resolution;
    //     int cost_map_y = (state.pose.y - cost_map.origin.y) / cost_map.resolution;

    //     for (int i = -kernel_radius; i <= kernel_radius; ++i) {
    //         for (int j = -kernel_radius; j <= kernel_radius; ++j) {
    //             int x = cost_map_x + i;
    //             int y = cost_map_y + j;

    //             if (x >= 0 && x < cost_map.data.rows() && y >= 0 && y < cost_map.data.cols()) {
    //                 sum += kernel(i + kernel_radius) * cost_map.data(x, y);
    //             }
    //         }
    //     }

    //     cache.data(x, y) = sum;
    //     return sum;
    // }

    Eigen::VectorXf ScanCostMapGenerator::gen_kernel(int search_radius, float sigma) {
        int kernel_size = 2 * search_radius + 1;
        Eigen::VectorXf kernel(kernel_size);
        float sum = 0.0f;

        for (int i = 0; i < kernel_size; ++i) {
            int dist = i - search_radius;
            kernel(i) = exp(-(dist * dist) / (2 * sigma * sigma));
            sum += kernel(i);
        }

        // Normalize the kernel to ensure the sum is 1
        kernel /= sum;
        return kernel;
    }

    std::shared_ptr<CostMap> ScanCostMapGenerator::generate_cost_map(Grid grid, Grid* scan) {
        if (scan == nullptr) {
            return std::make_shared<ScanCostMap>(grid, kernel, store_radius, grid.origin);
        }

        // Temporarily clear the original grid so we can just see the scan overlay
        // grid.data = Eigen::MatrixXf::Zero(grid.data.rows(), grid.data.cols());

        // Iterate through the scan, using the scan origin, scan resolution, and map
        // resolution, to overlay the scan on the map with scan_weight
        // for (int i = 0; i < scan->data.rows(); ++i) {
        //     for (int j = 0; j < scan->data.cols(); ++j) {
        //         if (scan->data(i, j) > 0.0) {
        //             int x = (scan->origin.x + i * scan->resolution - grid.origin.x)
        //                     / grid.resolution;
        //             int y = (scan->origin.y + j * scan->resolution - grid.origin.y)
        //                     / grid.resolution;

        //             if (x >= 0 && x < grid.data.rows() && y >= 0 && y < grid.data.cols()) {
        //                 // std::cout << x << ", " << y << std::endl;
        //                 // std::cout << scan_weight << std::endl;
        //                 grid.data(x, y) = scan_weight;
        //             }
        //         }
        //     }
        // }

        // Convolution along rows
        Eigen::MatrixXf row_conv = Eigen::MatrixXf::Zero(grid.data.rows(), grid.data.cols());

        for (int i = 0; i < grid.data.rows(); ++i) {
            for (int j = 0; j < grid.data.cols(); ++j) {
                float sum = 0.0f;
                for (int k = -search_radius; k <= search_radius; ++k) {
                    int idx = j + k;
                    if (idx >= 0 && idx < grid.data.cols()) {
                        float tmp = grid.data(i, idx);
                        float tmp2 = kernel(k + search_radius);

                        sum += grid.data(i, idx) * kernel(k + search_radius);
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
                        sum += row_conv(idx, j) * kernel(k + search_radius);
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

        return std::make_shared<ScanCostMap>(cost_map_, kernel, store_radius, scan->origin);
    }
}