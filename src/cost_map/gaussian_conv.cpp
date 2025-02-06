#include "cost_map/gaussian_conv.h"

#include <iostream>

namespace cev_planner::cost_map {

    double GaussianCostMap::cost(State state) {
        // Convert the state to grid coordinates
        int x = (state.pose.x - cost_map.origin.x) / cost_map.resolution;
        int y = (state.pose.y - cost_map.origin.y) / cost_map.resolution;

        // Check if the state is within the bounds of the cost map
        if (x < 0 || x >= cost_map.data.rows() || y < 0 || y >= cost_map.data.cols()) {
            return std::numeric_limits<double>::max();
        }

        return cost_map.data(x, y);
    }

    Eigen::VectorXf GaussianConvolution::gen_kernel(int search_radius, float sigma) {
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

    std::unique_ptr<CostMap> GaussianConvolution::generate_cost_map(Grid grid) {
        std::cout << grid.data.cols() << std::endl;

        std::cout << "My search radius is: " << search_radius;

        // Convolution along rows
        Eigen::MatrixXf row_conv = Eigen::MatrixXf::Zero(grid.data.rows(), grid.data.cols());

        std::cout << "Made it here." << std::endl;

        std::cout << grid.data(0, 0) << std::endl;

        std::cout << kernel.rows() << std::endl;
        std::cout << kernel.cols() << std::endl;
        std::cout << kernel(1) << std::endl;

        for (int i = 0; i < grid.data.rows(); ++i) {
            std::cout << "here." << std::endl;
            for (int j = 0; j < grid.data.cols(); ++j) {
                std::cout << "here.." << std::endl;
                float sum = 0.0f;
                for (int k = -search_radius; k <= search_radius; ++k) {
                    int idx = j + k;
                    if (idx >= 0 && idx < grid.data.cols()) {
                        std::cout << "Getting grid cell: " << i << ", " << idx << std::endl;
                        float tmp = grid.data(i, idx);
                        std::cout << "this done." << std::endl;
                        std::cout << k << std::endl;
                        std::cout << search_radius << std::endl;
                        float tmp2 = kernel(k + search_radius);
                        std::cout << "this done too." << std::endl;

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

        std::cout << "Cost Map:\n" << cost_map << std::endl;

        Grid cost_map_ = Grid();
        cost_map_.data = cost_map;
        cost_map_.origin = grid.origin;
        cost_map_.resolution = grid.resolution;

        return std::make_unique<GaussianCostMap>(cost_map_);
    }

}