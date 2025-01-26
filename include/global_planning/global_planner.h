#pragma once

#include <vector>
#include "constraints.h"
#include "grid.h"
#include "trajectory.h"

namespace cev_planner::global_planner {

    /**
     * @brief GlobalPlanner plans a path through an occupancy grid to a target pose while
     * avoiding obstacles and obeying constraints
     *
     */
    class GlobalPlanner {
    private:
        Dimensions dimensions;
        Constraints constraints;

    public:
        /**
         * @brief Construct a new Global Planner object
         *
         * @param constraints Constraints on the robot's motion
         */
        GlobalPlanner(Dimensions dimensions, Constraints constraints) {
            this->dimensions = dimensions;
            this->constraints = constraints;
        }

        /**
         * @brief Plan a path through an occupancy grid to a target pose while avoiding
         * obstacles and obeying constraints
         *
         * @param grid Occupancy grid with probability of occupancy at each cell
         * @param start Starting pose of the robot
         * @param target Target pose of the robot
         * @return std::vector<Pose> Path from the start pose to the target pose
         */
        virtual Trajectory plan_path(Grid grid, Pose start, Pose target) = 0;
    };
}  // namespace cev_planner::global_planner