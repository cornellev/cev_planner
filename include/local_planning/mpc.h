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
        int num_inputs;
        // int horizon_extension_iters = 1;  // 5 horizon extension steps
        // int keep_per_extension = 10;      // keep 3/num_inputs of the best path
        // int additionally_extend = 0;      // extend the final path by 5 more steps
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
        float dt;
        BaseMPC(Dimensions dimensions, Constraints constraints, int num_inputs_ = 10, float dt_ = .4) 
            : LocalPlanner(dimensions, constraints), num_inputs(num_inputs_), dt(dt_) {
            // opt = nlopt::opt(nlopt::LN_SBPLX, num_inputs * 2);
            opt = nlopt::opt(nlopt::LN_BOBYQA, num_inputs * 2);
            opt.set_min_objective(objective_function, this);
            opt.set_xtol_rel(1e-8);
        }

        Trajectory calculate_trajectory(Trajectory initial_guess) override;

    };

    /**
     * @brief Cartesian-space Model Predictive Control based local planner
     */
    class CartesianMPC : public BaseMPC {
    public:
        CartesianMPC(Dimensions dimensions, Constraints constraints) : 
            BaseMPC(dimensions, constraints, 10, .4) {};
    protected:
        double costs(const std::vector<double>& x) override;
        std::vector<State> decompose(State start_state, std::vector<double> u, double dt) override;

    private:
        std::unique_ptr<State> temp_start;
    };

    /**
     * @brief Frenet-frame MPC.
     */
    struct LaneCostWeights {
        double w_along_track = 200.0;
        double w_cte = 50.0;
        double w_costmap = 50.0;
        double obs_threshold = 5.0;
        double cte_threshold = 1.5;
        double along_threshold = 0.1;
        double target_vel = 0.0;
    };

    class LaneFollowingMPC : public BaseMPC {
    public:
        double target_vel = 0;
        double extension_dist = 0; // extra heuristic
        std::vector<double> reference_waypoints;
        LaneFollowingMPC(Dimensions dimensions, Constraints constraints) :
            BaseMPC(dimensions, constraints, 6, .4) { 
                target_vel = constraints.vel[1];
                cost_weights_.target_vel = target_vel;
                cost_weights_.w_along_track = 200.0;
                cost_weights_.w_cte = 50.0;
                cost_weights_.w_costmap = 50.0;
                cost_weights_.obs_threshold = 5.0;
                cost_weights_.cte_threshold = 1.5;
                cost_weights_.along_threshold = 0.1;
                // extension_dist = 2 * dimensions.length * constraints.vel[1] * dt;
            };

        void set_reference_polynomial(const Trajectory& reference, int start_index,
            int max_points);
        void set_cost_weights(const LaneCostWeights& weights);
        LaneCostWeights get_cost_weights() const { return cost_weights_; }
        bool get_point_metrics(const State& state, size_t waypoint_index,
            double& along_track, double& cte, double& costmap_cost,
            double& cte_bad, double& obs_bad, double& along_track_penalty) const;

    protected:
        double costs(const std::vector<double>& x) override;
        std::vector<State> decompose(State start_state, std::vector<double> u, double dt) override;

    private:
        double interpolated_costmap_penalty(const State& current, const State* next,
            double resolution) const;
        CubicPolynomial reference_polynomial;
        LaneCostWeights cost_weights_;
    };
}  // namespace cev_planner::local_planner
