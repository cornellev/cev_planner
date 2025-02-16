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
        int num_inputs = 10;
        float dt = .25;
        int horizon_extension_iters = 1;  // 5 horizon extension steps
        int keep_per_extension = 10;      // keep 3/num_inputs of the best path
        int additionally_extend = 0;      // extend the final path by 5 more steps
        std::unique_ptr<State> temp_start;
        nlopt::opt opt;

        double path_obs_cost(std::vector<State>& path);
        double path_waypoints_cost(std::vector<State>& path);
        double costs(const std::vector<double>& x);
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
            opt = nlopt::opt(nlopt::LN_BOBYQA, num_inputs * 2);
            opt.set_min_objective(objective_function, this);
            opt.set_xtol_rel(1e-4);
            // TODO: Experiment with randomness settings, initial step based on angle?
        }

        Trajectory calculate_trajectory();
    };
}  // namespace cev_planner::local_planner