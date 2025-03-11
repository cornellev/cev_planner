#pragma once

#include <Eigen/Dense>
#include <vector>
#include <cmath>

#include <iostream>

#include "cost_map.h"
#include "vis/vis.h"

namespace cev_planner::cost_map {

    class ScanCostMap : public CostMap {
    private:
        Grid cost_map;
        Eigen::VectorXf kernel;
        Grid cache;

    public:
        /**
         * @brief Construct a new Scan Cost Map object
         *
         * @param cost_map Cost map
         * @param kernel Cost kernel
         * @param store_radius Radius to cache costs
         */
        ScanCostMap(Grid cost_map, Eigen::VectorXf kernel, int store_radius, Pose cache_origin);
        double cost(State state) override;

        double debug_(int i, int j) override;
    };

    /**
     * @brief GaussianConvolution cost map generator generates a cost map by convolving a
     * gaussian kernel with an occupancy grid
     *
     * @param search_radius Search radius of cost kernel
     * @param sigma Standard deviation of cost kernel based on distance
     * @param scan_weight Weight of scan cost
     * @param store_radius Radius to cache costs
     *
     */
    class ScanCostMapGenerator : public CostMapGenerator {
    private:
        Eigen::VectorXf kernel;
        int search_radius;
        float scan_weight;
        int store_radius;

        Eigen::VectorXf gen_kernel(int search_radius, float sigma);

    public:
        /**
         * @brief GaussianConvolution cost map generator generates a cost map by convolving a
         * gaussian kernel with an occupancy grid
         *
         * @param search_radius Search radius of cost kernel
         * @param sigma Standard deviation of cost kernel based on distance
         * @param scan_weight Weight of scan cost
         * @param store_radius Radius to cache costs
         *
         */
        ScanCostMapGenerator(int search_radius, float sigma, float scan_weight, int store_radius)
            : CostMapGenerator() {
            this->search_radius = search_radius;
            this->scan_weight = scan_weight;
            this->store_radius = store_radius;
            kernel = gen_kernel(search_radius, sigma);
        }

        std::shared_ptr<CostMap> generate_cost_map(Grid grid, Grid* scan) override;
    };

}  // namespace cev_planner::cost_map