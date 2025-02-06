#pragma once

namespace cev_planner {

    /**
     * @brief Dimensions of the robot
     *
     */
    struct Dimensions {
        // Width of the robot in meters (left to right side)
        double width;

        // Length of the robot in meters (front to back)
        double length;

        // Wheelbase of the robot in meters (distance between the front and rear axles)
        double wheelbase;
    };

    /**
     * @brief Constraints of the state space. [min, max]
     *
     */
    struct Constraints {
        // X coordinate in meters
        double x[2];
        // Y coordinate in meters
        double y[2];
        // Steering angle in radians
        double tau[2];
        // Velocity in m/s
        double vel[2];
        // Acceleration in m/s^2
        double accel[2];
        // Change of steering angle in radians
        double dtau[2];
    };

}  // namespace cev_planner
