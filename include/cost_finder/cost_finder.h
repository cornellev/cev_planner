#pragma once

#include <vector>
#include "grid.h"
#include "trajectory.h"

namespace cev_planner::cost_finder {

    class Quadtree;

    /**
     * @brief CostFinder dynamically stores and queries cost information
     */
    class CostFinder {
    private:
        Quadtree* quadtree;
        double radius; // car radius
        int k;  // heuristic classifier

    public:
        /**
         * @brief Construct a new CostFinder object
         * @param radius car radius
         * @param k heuristic classifier
         */
        CostFinder(double radius, int k);

        /**
         * @brief Add a new point to the finder
         * @param state The state to be inserted
         */
        void addPoint(const State& state);

        /**
         * @brief Compute cost dynamically based on nearby obstacles
         * @param state The state for which cost is queried
         * @return Computed cost as a double
         */
        double cost(const State& state);
    };

}  // namespace cev_planner::cost_finder