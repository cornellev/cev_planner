#pragma once

#include <vector>
#include "constraints.h"
#include "grid.h"
#include "trajectory.h"
#include <optional>

namespace cev_planner::global_planner {

    /**
     * @brief GlobalPlanner plans a `Trajectory` through an occupancy grid
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
         * @brief Plan a `Trajectory` through an occupancy grid from a start to a target `State`
         * while avoiding obstacles and obeying constraints
         *
         * @param grid Occupancy grid with probability of occupancy at each cell
         * @param start Starting `State` of the robot
         * @param target Target `State` of the robot
         * @return `Trajectory` from the start pose to the target pose
         */
        virtual std::optional<Trajectory> plan_path(Grid grid, State start, State target) = 0;
    };
}  // namespace cev_planner::global_planner