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
         */
        MPC(Dimensions dimensions, Constraints constraints)
            : LocalPlanner(dimensions, constraints) {}

        Grid calculate_costmap(Grid grid, Pose start, Pose target, Trajectory waypoints) override;
        Trajectory calculate_trajectory(Pose start, Pose target) override;
    };
}  // namespace cev_planner::local_planner