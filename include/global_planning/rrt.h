#pragma once

#include "global_planner.h"

namespace cev_planner::global_planner {

    /**
     * @brief RRT is a global planner that plans a path through an occupancy grid to a target
     * pose while avoiding obstacles and obeying constraints
     *
     */
    class RRT : public GlobalPlanner {
    public:
        /**
         * @brief Construct a new RRT object
         *
         * @param dimensions Dimensions of the robot
         * @param constraints Constraints on the robot's motion
         */
        RRT(Dimensions dimensions, Constraints constraints)
            : GlobalPlanner(dimensions, constraints) {}

        /**
         * @brief Plan a `Trajectory` through an occupancy grid from a start to a target `State`
         * while avoiding obstacles and obeying constraints
         *
         * @param grid Occupancy grid with probability of occupancy at each cell
         * @param start Starting `State` of the robot
         * @param target Target `State` of the robot
         * @return `Trajectory` from the start pose to the target pose
         */
        Trajectory plan_path(Grid grid, State start, State target) override;
    };
}  // namespace cev_planner::global_planner