#pragma once

#include "local_planner.h"

namespace cev_planner::local_planner {

    /**
     * @brief Model Predictive Control based local planner
     *
     */
    class MPC : public LocalPlanner {
    public:
        /**
         * @brief Construct a new MPC object
         *
         * @param dimensions Dimensions of the robot
         * @param constraints Constraints on the robot's motion
         * @param cost_map_generator Cost map generator
         */
        MPC(Dimensions dimensions, Constraints constraints,
            std::shared_ptr<CostMapGenerator> cost_map_generator)
            : LocalPlanner(dimensions, constraints, cost_map_generator) {}

        Trajectory calculate_trajectory();
    };
}  // namespace cev_planner::local_planner