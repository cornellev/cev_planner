#pragma once

#include "angle.h"

namespace cev_planner {

    /**
     * @brief Pose represents position and orientation of an object in 2D space. Angles are
     * constrainted to the range [-pi, pi]
     *
     */
    struct Pose {
        Pose operator+(const Pose& other) const {
            return Pose{x + other.x, y + other.y, restrict_angle(theta + other.theta)};
        }

        Pose operator-(const Pose& other) const {
            return Pose{x - other.x, y - other.y, restrict_angle(theta - other.theta)};
        }

        Pose operator*(double scalar) const {
            return Pose{x * scalar, y * scalar, restrict_angle(theta * scalar)};
        }

        Pose operator/(double scalar) const {
            return Pose{x / scalar, y / scalar, restrict_angle(theta / scalar)};
        }

        float distance_to(const Pose& other) const {
            return std::hypot(x - other.x, y - other.y);
        }

        float angle_to(const Pose& other) const {
            return restrict_angle(std::atan2(other.y - y, other.x - x) - theta);
        }

        // X coordinate in meters
        double x;

        // Y coordinate in meters
        double y;

        // Orientation in radians
        double theta;
    };

}