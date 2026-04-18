#pragma once

#include <vector>
#include "constraints.h"
#include "grid.h"
#include "trajectory.h"

#include <iostream>
#include <chrono>

namespace cev_planner::local_planner {

    class LocalPlanner {
    protected:
        Dimensions dimensions;
        Constraints constraints;

        Grid grid;
        State start;
        State target;
        Trajectory waypoints;

    public:
        LocalPlanner(Dimensions dimensions, Constraints constraints)
            : dimensions(dimensions), constraints(constraints) {}

        Trajectory plan_path(Grid grid, State start, State target, Trajectory waypoints,
            Trajectory initial_guess) {
            this->grid = grid;
            this->start = start;
            this->target = target;
            this->waypoints = waypoints;

            return this->calculate_trajectory(initial_guess);
        }

        virtual Trajectory calculate_trajectory(Trajectory initial_guess) = 0;
    };
}  // namespace cev_planner::local_planner
