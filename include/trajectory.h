#pragma once

#include <vector>
#include <pose.h>

namespace cev_planner {

    /**
     * @brief Waypoint represents a point in 2D space with a target velocity
     *
     */
    struct Waypoint {
        // State of the waypoint
        State state;
    };

    /**
     * @brief Trajectory represents a sequence of waypoints
     *
     */
    struct Trajectory {
        // Waypoints in the trajectory
        std::vector<State> waypoints;
        float cost;
        float timestep;
    };

}  // namespace cev_planner