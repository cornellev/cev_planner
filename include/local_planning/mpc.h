#pragma once

#include "local_planner.h"
#include <nlopt.hpp>
#include <vector>

namespace cev_planner::local_planner {

    // Decision variables: [tau_0, vel_0, tau_1, vel_1, ... tau_{n-1}, vel_{n-1}]
    // tau_i  = steering angle (rad) at step i  — bounded directly in [-tau_max, tau_max]
    // vel_i  = speed (m/s) at step i           — bounded directly in [0, vel_max]
    class CartesianMPC : public LocalPlanner {
    protected:
        int num_inputs = 8;
        float dt = 0.4f;
        nlopt::opt opt;

        // warm start
        std::vector<double> prev_x;

        // Grid-based obstacle cost.
        double grid_obstacle_cost(const std::vector<State>& path) const;

        // Progress + lateral-error waypoint cost, we'll see
        double waypoint_cost(const std::vector<State>& path) const;

        // steering-rate penalty
        double steering_rate_cost(const std::vector<double>& u) const;

        double costs(const std::vector<double>& x);

        // forward-simulate using (tau, vel) inputs.
        std::vector<State> decompose(const std::vector<double>& u) const;

        static double objective_function(const std::vector<double>& x,
                                         std::vector<double>& grad, void* data);

    public:

        bool gps_mode = false;

        CartesianMPC(Dimensions dimensions, Constraints constraints)
            : LocalPlanner(dimensions, constraints) {
            opt = nlopt::opt(nlopt::LN_BOBYQA, num_inputs * 2);
            opt.set_min_objective(objective_function, this);
            opt.set_xtol_rel(1e-4);
            opt.set_maxeval(500);
        }

        Trajectory calculate_trajectory(Trajectory initial_guess) override;
    };

}  // namespace cev_planner::local_planner
