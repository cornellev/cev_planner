#pragma once

#include <vector>
#include "constraints.h"
#include "grid.h"
#include "trajectory.h"
#include "thread"
#include "cost_map/cost_map.h"
#include "vis/vis.h"

#include <iostream>
#include <chrono>

using namespace cev_planner::cost_map;

namespace cev_planner::local_planner {

    /**
     * @brief LocalPlanner plans a path through an occupancy grid to a target pose while
     * avoiding obstacles and obeying constraints
     *
     */
    class LocalPlanner {
    protected:
        Dimensions dimensions;
        Constraints constraints;

        Grid grid;  // Current occupancy grid
        State start;
        State target;
        Trajectory waypoints;

        std::shared_ptr<CostMap> costmap;

        std::thread costmap_thread;

    public:
        /**
         * @brief Construct a new Local Planner object
         *
         * @param dimensions Dimensions of the robot
         * @param constraints Constraints on the robot's motion
         * @param cost_map_generator Cost map generator
         */
        LocalPlanner(Dimensions dimensions, Constraints constraints) {
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
        Trajectory plan_path(Grid grid, State start, State target, Trajectory waypoints,
            Trajectory initial_guess, std::shared_ptr<CostMap> cost_map) {
            this->grid = grid;
            this->start = start;
            this->target = target;
            this->waypoints = waypoints;
            this->costmap = cost_map;

            auto start_time = std::chrono::high_resolution_clock::now();

            // std::cout << "Calculating trajectory..." << std::endl;
            Trajectory trajectory = this->calculate_trajectory(initial_guess);
            // std::cout << "Trajectory calculated in: "
            //           << std::chrono::duration_cast<std::chrono::milliseconds>(
            //                  std::chrono::high_resolution_clock::now() - start_time)
            //                  .count()
            //           << "ms" << std::endl;

            // cev_planner::vis::vis_trajectory(grid, start, trajectory, target);

            return trajectory;
        }

        virtual Trajectory calculate_trajectory(Trajectory initial_guess) = 0;
    };
}  // namespace cev_planner::local_planner