#pragma once

#include <vector>
#include "constraints.h"
#include "grid.h"
#include "trajectory.h"
#include "thread"

namespace cev_planner::local_planner {

    /**
     * @brief LocalPlanner plans a path through an occupancy grid to a target pose while
     * avoiding obstacles and obeying constraints
     *
     */
    class LocalPlanner {
    private:
        Dimensions dimensions;
        Constraints constraints;
        Grid grid;     // Current occupancy grid
        Grid costmap;  // Most recently calculated costmap
        Trajectory waypoints;

        Grid new_costmap;
        bool costmap_initialized = false;

        std::thread costmap_thread;

    public:
        /**
         * @brief Construct a new Local Planner object
         *
         * @param constraints Constraints on the robot's motion
         */
        LocalPlanner(Dimensions dimensions, Constraints constraints) {
            this->dimensions = dimensions;
            this->constraints = constraints;
        }

        virtual Grid calculate_costmap(Grid grid, Pose start, Pose target,
            Trajectory waypoints) = 0;

        virtual Trajectory calculate_trajectory(Pose start, Pose target) = 0;

        /**
         * @brief Plan a path through an occupancy grid to a target pose while avoiding
         * obstacles and obeying constraints
         *
         * @param grid Occupancy grid with probability of occupancy at each cell
         * @param start Starting pose of the robot
         * @param target Target pose of the robot
         * @return std::vector<Pose> Path from the start pose to the target pose
         */
        Trajectory plan_path(Grid grid, Pose start, Pose target, Trajectory waypoints) {
            this->grid = grid;
            this->waypoints = waypoints;

            if (costmap_initialized) {
                // Asynchronously calculate the costmap
                std::thread costmap_thread([this, grid, start, target, waypoints] {
                    this->new_costmap = this->calculate_costmap(grid, start, target, waypoints);
                });
            } else {
                this->new_costmap = this->calculate_costmap(grid, start, target, waypoints);
                this->costmap = this->new_costmap;
                costmap_initialized = true;
            }

            // Update costmap in case new one calculated
            this->costmap = this->new_costmap;

            Trajectory trajectory = this->calculate_trajectory(start, target);

            return trajectory;
        }
    };
}  // namespace cev_planner::local_planner