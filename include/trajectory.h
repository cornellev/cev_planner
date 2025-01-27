#pragma once

#include <vector>
#include <pose.h>

namespace cev_planner {

    /**
     * @brief Waypoint represents a point in 2D space with a target velocity
     *
     */
    struct Waypoint {
        // Pose of the waypoint
        Pose pose;

        // Target velocity in meters per second
        double velocity;
    };

    /**
     * @brief Trajectory represents a sequence of waypoints
     *
     */
    struct Trajectory {
        // Waypoints in the trajectory
        std::vector<Waypoint> waypoints;
    };

}  // namespace cev_planner