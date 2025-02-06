#pragma once

#include <Eigen/Dense>
#include <vector>
#include <cmath>

#include <iostream>

#include "cost_map.h"

namespace cev_planner::cost_map {

    class GaussianCostMap : public CostMap {
    private:
        Grid cost_map;

    public:
        GaussianCostMap(Grid cost_map): cost_map(cost_map) {}
        double cost(State state) override;
    };

    /**
     * @brief GaussianConvolution cost map generator generates a cost map by convolving a gaussian
     * kernel with an occupancy grid
     *
     */
    class GaussianConvolution : public CostMapGenerator {
    private:
        Eigen::VectorXf kernel;
        int search_radius;
        float sigma;

    public:
        /**
         * @brief Construct a new GaussianConvolution cost map generator
         */
        GaussianConvolution(int search_radius, float sigma): CostMapGenerator() {
            std::cout << "MY SEARCH RADIUS IS: " << search_radius << std::endl;
            kernel = gen_kernel(search_radius, sigma);
        }

        Eigen::VectorXf gen_kernel(int search_radius, float sigma);

        std::unique_ptr<CostMap> generate_cost_map(Grid grid) override;
    };

}  // namespace cev_planner::cost_map