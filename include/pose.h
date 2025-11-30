#pragma once

#include "util.h"
#include "constraints.h"

#include <iostream>
#include <eigen3/Eigen/Dense>

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
     * @brief Input represents the rate-based control input to a vehicle
     *
     */
    struct Input {
        // Steering rate in rad/s
        double dtau;

        // Longitudinal acceleration in m/s^2
        double accel;
    };

    struct CubicPolynomial;

    /**
     * @brief State represents position and orientation, and other state variables of a vehicle in
     * 2D space. Angles are constrainted to the range [-pi, pi]
     *
     */
    struct State {
        State(double x = 0, double y = 0, double theta = 0, double tau = 0, double vel = 0, double cte = 0, double theta_e = 0)
            : pose(Pose(x, y, theta)), tau(tau), vel(vel), cte(cte), theta_e(theta_e) {}

        // Pose of the vehicle
        Pose pose;

        // Steering angle in radians
        double tau;

        // Velocity in m/s
        double vel;

        // State values for lane MPC        
        double cte;      // Cross track error
        double theta_e;  // Heading error

        State update(Input input, double dt, Dimensions& dimensions, Constraints& constraints);

        State update(Input input, double dt, CubicPolynomial& r, Dimensions& dimensions, Constraints& constraints);

        std::string to_string() {
            return "State: (" + std::to_string(pose.x) + ", " + std::to_string(pose.y) + ", "
                   + std::to_string(pose.theta) + ", " + std::to_string(tau) + ", "
                   + std::to_string(vel) + ")";
        }
    };

    // MPC utils
    struct CubicPolynomial {
        double a3=0, a2=0, a1=0, a0=0;
        double x0=0, y0=0, m0=0;     // left endpoint and slope
        double x1=0, y1=0, m1=0;     // right endpoint and slope

        bool fit(const std::vector<State>& pts, int start, int n){
            if (n < 4) return false;

            Eigen::MatrixXd X(n, 4);
            Eigen::VectorXd Y(n);

            for (int i = 0; i < n; ++i) {
                const auto& p = pts[start + i].pose;
                const double xi = p.x, xi2 = xi*xi;
                X(i,0)=xi2*xi; X(i,1)=xi2; X(i,2)=xi; X(i,3)=1.0;
                Y(i)=p.y;
            }
            Eigen::Vector4d a = X.colPivHouseholderQr().solve(Y);
            a3=a(0); a2=a(1); a1=a(2); a0=a(3);

            // Cache endpoint info for extrapolation
            const auto& L = pts[start].pose;
            const auto& R = pts[start + n - 1].pose;
            x0 = L.x; y0 = L.y; m0 = deriv(x0);
            x1 = R.x; y1 = R.y; m1 = deriv(x1);
            return true;
        }

        // cubic y = f(x) valid only within [x0, x1]
        double cubic(double x) const {
            return ((a3 * x + a2) * x + a1) * x + a0;
        }

        double deriv(double x) const {
            return (3.0 * a3 * x + 2.0 * a2) * x + a1;
        }

        // "Straight ends": linear extrapolation along endpoint tangents
        double at(double x) const {
            if (x <= x0) return y0 + m0 * (x - x0);
            if (x >= x1) return y1 + m1 * (x - x1);
            return cubic(x);
        }
    };

    inline State State::update(Input input, double dt, Dimensions& dimensions, Constraints& constraints) {
        State _state = *this;

        double accel = std::clamp(input.accel, constraints.accel[0], constraints.accel[1]);
        double dtau = std::clamp(input.dtau, constraints.dtau[0], constraints.dtau[1]);

        _state.vel = std::clamp(vel + accel * dt, constraints.vel[0], constraints.vel[1]);
        _state.tau = std::clamp(tau + dtau * dt, constraints.tau[0], constraints.tau[1]);

        double avg_vel = 0.5 * (vel + _state.vel);
        // double avg_tau = 0.5 * (tau + _state.tau);
        
        if (std::abs(_state.tau) < 1e-4) {
            _state.pose.theta = pose.theta;
        } else {
            double R = dimensions.wheelbase / std::tan(_state.tau);
            double dtheta = (avg_vel / R) * dt;
            _state.pose.theta = restrict_angle(pose.theta + dtheta);
        }

        double avg_theta = 0.5 * (pose.theta + _state.pose.theta);

        _state.pose.x = pose.x + avg_vel * cos(avg_theta) * dt;
        _state.pose.y = pose.y + avg_vel * sin(avg_theta) * dt;

        return _state;
    }

    inline State State::update(Input input, double dt, CubicPolynomial& r, Dimensions& dimensions, Constraints& constraints) {
        State s = update(input, dt, dimensions, constraints);

        s.cte = r.at(pose.x) - pose.y + vel * sin(theta_e) * dt;
        s.theta_e = restrict_angle(s.pose.theta - r.deriv(s.pose.x));

        return s;
    }

}
