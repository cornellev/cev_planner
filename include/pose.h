#pragma once

#include "util.h"
#include "constraints.h"

#include <iostream>
#include <cmath>


// just reading, merged different pose file, changing for no reason
namespace cev_planner {

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
     * @brief Input represents the rate-based control input to a vehicle
     *
     */
    struct Input {
        // Steering rate in rad/s
        double dtau;

        // Longitudinal acceleration in m/s^2
        double accel;
    };

    struct State {
        State(double x = 0, double y = 0, double theta = 0, double tau = 0, double vel = 0)
            : pose(Pose(x, y, theta)), tau(tau), vel(vel) {}

        // Pose of the vehicle
        Pose pose;
        double tau;  // Steering angle (rad)
        double vel;  // Velocity (m/s)

        // Bicycle kinematic model.  Inputs are (steering_rate, acceleration).
        State update(Input input, double dt, Dimensions& dimensions, Constraints& constraints) {
            State next = *this;

            double accel = std::clamp(input.accel, constraints.accel[0], constraints.accel[1]);
            double dtau  = std::clamp(input.dtau,  constraints.dtau[0],  constraints.dtau[1]);

            next.vel = std::clamp(vel + accel * dt, constraints.vel[0], constraints.vel[1]);
            next.tau = std::clamp(tau + dtau  * dt, constraints.tau[0], constraints.tau[1]);

            double avg_vel = (vel + next.vel) / 2.0;
            double avg_tau = (tau + next.tau) / 2.0;

            double dtheta = (avg_vel * std::tan(avg_tau) / dimensions.wheelbase) * dt;
            next.pose.theta = restrict_angle(pose.theta + dtheta);

            double avg_theta = (pose.theta + next.pose.theta) / 2.0;

            next.pose.x = pose.x + avg_vel * std::cos(avg_theta) * dt;
            next.pose.y = pose.y + avg_vel * std::sin(avg_theta) * dt;

            return next;
        }

        std::string to_string() {
            return "State: (" + std::to_string(pose.x) + ", " + std::to_string(pose.y) + ", "
                   + std::to_string(pose.theta) + ", " + std::to_string(tau) + ", "
                   + std::to_string(vel) + ")";
        }
    };

}  // namespace cev_planner
