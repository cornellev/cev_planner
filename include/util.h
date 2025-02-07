#pragma once

#include <cmath>

namespace cev_planner {
    /**
     * @brief Restrict an angle to the range [-pi, pi]
     *
     * @param angle Angle in radians
     * @return float Restricted angle
     */
    static float restrict_angle(float angle) {
        float new_angle = std::fmod(angle, 2 * M_PI);

        if (new_angle >= M_PI) {
            new_angle -= 2 * M_PI;
        } else if (new_angle < -M_PI) {
            new_angle += 2 * M_PI;
        }

        return new_angle;
    }
}  // namespace cev_planner