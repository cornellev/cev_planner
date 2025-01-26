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
     * @brief Constraints on the robot's motion
     *
     */
    struct Constraints {
        // Maximum steering angle in radians
        double max_steering_angle;

        // Minimum steering angle in radians
        double min_steering_angle;
    };

}  // namespace cev_planner
