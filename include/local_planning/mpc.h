#pragma once

#include "local_planner.h"

#include <nlopt.hpp>
#include <vector>

namespace cev_planner::local_planner {

    /**
     * @brief Model Predictive Control based local planner
     *
     */
    class BaseMPC : public LocalPlanner {
    protected:
        int num_inputs = 10;
        float dt = .4;
        int horizon_extension_iters = 1;  // 5 horizon extension steps
        int keep_per_extension = 10;      // keep 3/num_inputs of the best path
        int additionally_extend = 0;      // extend the final path by 5 more steps
        nlopt::opt opt;

        double path_obs_cost(const std::vector<State>& path) const;
        double path_waypoints_cost(const std::vector<State>& path) const;
        virtual double costs(const std::vector<double>& x) = 0;
        virtual std::vector<State> decompose(State start_state, std::vector<double> u, double dt)=0;
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
        BaseMPC(Dimensions dimensions, Constraints constraints): LocalPlanner(dimensions, constraints) {
            // opt = nlopt::opt(nlopt::LN_SBPLX, num_inputs * 2);
            opt = nlopt::opt(nlopt::LN_BOBYQA, num_inputs * 2);
            opt.set_min_objective(objective_function, this);
            opt.set_xtol_rel(1e-8);
        }

        Trajectory calculate_trajectory(Trajectory initial_guess);
    };

    /**
     * @brief Cartesian-space Model Predictive Control based local planner
     */
    class CartesianMPC : public BaseMPC {
    public:
        CartesianMPC(Dimensions dimensions, Constraints constraints) : 
            BaseMPC(dimensions, constraints) {};
    protected:
        double costs(const std::vector<double>& x) override;
        std::vector<State> decompose(State start_state, std::vector<double> u, double dt) override;

    private:
        std::unique_ptr<State> temp_start;
    };

    /**
     * @brief Frenet-frame MPC.
     */
    class LaneFollowingMPC : public BaseMPC {
    public:
        double target_vel = 0;
        LaneFollowingMPC(Dimensions dimensions, Constraints constraints) :
            BaseMPC(dimensions, constraints) { 
                target_vel = constraints.vel[1] * 2.0 / 3;
            };

        void set_reference_polynomial(const Trajectory& reference, int start_index,
            int max_points);

    protected:
        double costs(const std::vector<double>& x) override;
        std::vector<State> decompose(State start_state, std::vector<double> u, double dt) override;

    private:
        double interpolated_costmap_penalty(const State& current, const State* next,
            double resolution) const;
        CubicPolynomial reference_polynomial;
    };
}  // namespace cev_planner::local_planner
