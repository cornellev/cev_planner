#pragma once

#include "local_planner.h"

#include <nlopt.hpp>
#include <vector>

namespace cev_planner::local_planner {

    /**
     * @brief Model Predictive Control based local planner
     *
     */
    class MPC : public LocalPlanner {
    private:
        int num_states = 15;
        float dt = .1;
        nlopt::opt opt;

        double path_obs_cost(std::vector<State>& path);
        double path_waypoints_cost(std::vector<State>& path);
        std::vector<State> decompose(State start_state, std::vector<double> u);
        static double objective_function(const std::vector<double>& x, std::vector<double>& grad,
            void* data);
        void optimize_iter(nlopt::opt& opt, std::vector<double>& x);

    public:
        /**
         * @brief Construct a new MPC object
         *
         * @param dimensions Dimensions of the robot
         * @param constraints Constraints on the robot's motion
         * @param cost_map_generator Cost map generator
         */
        MPC(Dimensions dimensions, Constraints constraints,
            std::shared_ptr<CostMapGenerator> cost_map_generator)
            : LocalPlanner(dimensions, constraints, cost_map_generator) {
            opt = nlopt::opt(nlopt::LN_NELDERMEAD, num_states * 2);
            opt.set_min_objective(objective_function, this);
            opt.set_xtol_rel(1e-4);
        }

        Trajectory calculate_trajectory();
    };
}  // namespace cev_planner::local_planner