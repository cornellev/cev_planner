#pragma once

#include "util.h"
#include "constraints.h"

#include <iostream>

namespace cev_planner {
    /**
     * @brief Pose represents position of objects in 2D space. Angles are constrainted to the range
     * [-pi, pi]
     *
     */
    struct Pose {
        Pose(double x = 0, double y = 0, double theta = 0)
            : x(x), y(y), theta(restrict_angle(theta)) {}

        float distance_to(const Pose& other) const {
            return std::hypot(x - other.x, y - other.y);
        }

        // X coordinate in meters
        double x;

        // Y coordinate in meters
        double y;

        // Orientation in radians
        double theta;
    };

    /**
     * @brief Input represents the control input to a vehicle
     *
     */
    struct Input {
        // Steering angle in radians
        double tau;

        // Velocity in m/s
        double vel;
    };

    /**
     * @brief State represents position and orientation, and other state variables of a vehicle in
     * 2D space. Angles are constrainted to the range [-pi, pi]
     *
     */
    struct State {
        State(double x = 0, double y = 0, double theta = 0, double tau = 0, double vel = 0)
            : pose(Pose(x, y, theta)), tau(tau), vel(vel) {}

        // Pose of the vehicle
        Pose pose;

        // Steering angle in radians
        double tau;

        // Velocity in m/s
        double vel;

        State update(Input input, double dt, Dimensions& dimensions, Constraints& constraints) {
            State _state = *this;

            // double accel = std::clamp(input.vel - vel, constraints.accel[0],
            // constraints.accel[1]); double dtau = std::clamp(input.tau - tau, constraints.dtau[0],
            // constraints.dtau[1]);
            double accel = std::clamp(input.vel, constraints.accel[0], constraints.accel[1]);
            double dtau = std::clamp(input.tau, constraints.dtau[0], constraints.dtau[1]);

            _state.vel = std::clamp(vel + accel * dt, constraints.vel[0], constraints.vel[1]);
            _state.tau = std::clamp(tau + dtau * dt, constraints.tau[0], constraints.tau[1]);

            double avg_vel = (vel + _state.vel) / 2;

            double R = dimensions.wheelbase / tan(tau);
            double dtheta = (avg_vel / R) * dt;
            _state.pose.theta = restrict_angle(pose.theta + dtheta);

            double avg_theta = (pose.theta + _state.pose.theta) / 2;
            double avg_tau = (tau + _state.tau) / 2;

            _state.pose.x = pose.x + avg_vel * cos(avg_theta) * dt;
            _state.pose.y = pose.y + avg_vel * sin(avg_theta) * dt;

            return _state;
        }

        std::string to_string() {
            return "State: (" + std::to_string(pose.x) + ", " + std::to_string(pose.y) + ", "
                   + std::to_string(pose.theta) + ", " + std::to_string(tau) + ", "
                   + std::to_string(vel) + ")";
        }
    };
}