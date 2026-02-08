#include "local_planning/mpc.h"
#include <algorithm>
#include <iostream>
#include <limits>

namespace cev_planner::local_planner {

    double BaseMPC::path_obs_cost(const std::vector<State>& path) const {
        double cost = 0;

        if (!this->costmap) {
            return cost;
        }
        for (const auto& state: path) {
            cost += this->costmap->cost(state);
        }

        return cost;
    }

    std::pair<double, double> dist_vector(State a, State b) {
        // Make a distance vector from a to b given the angle of b
        double x = b.pose.x - a.pose.x;
        double y = b.pose.y - a.pose.y;
        double angle = b.pose.theta;

        double x_ = x * cos(angle) + y * sin(angle);
        double y_ = -x * sin(angle) + y * cos(angle);

        return {x_, y_};
    }

    float total_second_half_weight = .5;
    float second_half_weight_first = .2 * total_second_half_weight;
    float second_half_weight_second = .8 * total_second_half_weight;

    double BaseMPC::path_waypoints_cost(const std::vector<State>& path) const {
        int waypoints_size = waypoints.waypoints.size();
        int path_size = path.size();

        float cost = 0;

        // int end_waypoint_0 = path_size;
        // int end_waypoint_1 = path_size;

        // if (waypoints_size > 1 && path_size >= 10) {
        //     end_waypoint_0 = ceil(path_size * .8);
        //     end_waypoint_1 = path_size;
        // }

        // Divide the path up into parts, compute distance to first waypoint and next waypoint
        // separately
        for (int i = 1; i < path_size; i++) {
            float dist = ((float)i) * path[i].pose.distance_to(waypoints.waypoints[0].pose);
            cost += dist;

            if (waypoints_size > 1 && dist < 1.0) {
                cost += (i / 2.0) * path[i].pose.distance_to(waypoints.waypoints[1].pose);
            }
        }
        // for (int i = end_waypoint_0; i < end_waypoint_1; i++) {
        //     cost += second_half_weight_first
        //             * path[i].pose.distance_to(waypoints.waypoints[1].pose);
        //     cost += second_half_weight_second
        //             * path[i].pose.distance_to(waypoints.waypoints[1].pose);
        // }

        // float current_cost = 2;

        // for (int j = 0; j < waypoints.waypoints.size(); j++) {
        //     dist = path[path.size() - 1].pose.distance_to(waypoints.waypoints[j].pose);
        //     if (path.size() > 2) {
        //         dist += path[path.size() - 2].pose.distance_to(waypoints.waypoints[j].pose);
        //     }
        //     if (path.size() > 3) {
        //         dist += path[path.size() - 3].pose.distance_to(waypoints.waypoints[j].pose);
        //     }

        //     cost += current_cost * dist;

        //     if (dist < .2) {
        //         current_cost /= 4;
        //     } else {
        //         current_cost = 0;
        //     }
        // }

        // for (int i = 1; i < path.size(); i++) {
        //     for (int j = 0; j < waypoints.waypoints.size(); j++) {
        //         cost += current_cost * path[i].pose.distance_to(waypoints.waypoints[j].pose);
        //         current_cost /= 2;
        //     }
        //     current_cost = 2;
        // }

        // for (int i = 1; i < path.size(); i++) {
        //     if (current_waypoint < size) {
        //         dist = path[i].pose.distance_to(waypoints.waypoints[current_waypoint].pose);
        //     }
        //     while (current_waypoint < size && dist < within_waypoint) {
        //         current_waypoint += 1;
        //         dist = path[i].pose.distance_to(waypoints.waypoints[current_waypoint].pose);
        //     }

        //     if (current_waypoint < size) {
        //         cost += waypoint_weight * dist;
        //     } else {
        //         dist = path[i].pose.distance_to(target.pose);
        //         cost += 2 * waypoint_weight * dist;
        //     }
        // }

        return cost;
    }

    double BaseMPC::objective_function(const std::vector<double>& x, std::vector<double>& grad,
        void* data) {
        auto* mpc = static_cast<BaseMPC*>(data);

        return mpc->costs(x);
    }


    void BaseMPC::optimize_iter(nlopt::opt& opt, std::vector<double>& x) {
        double minf;
        try {
            opt.optimize(x, minf);
        } catch (const nlopt::roundoff_limited& e) {
            // Log and continue with current x; avoid crashing the node
            std::cerr << "MPC optimize roundoff_limited: " << e.what() << std::endl;
        } catch (const std::exception& e) {
            std::cerr << "MPC optimize failed: " << e.what() << std::endl;
        }
    }

    std::vector<State> CartesianMPC::decompose(State start_state, std::vector<double> u, double dt) {
        std::vector<State> path;
        State state = start_state;
        path.push_back(state);
        for (int i = 0; i < u.size(); i += 2) {
            Input input = {u[i], u[i + 1]};
            state = state.update(input, dt, dimensions, constraints);
            path.push_back(state);
        }
        return path;
    }

    // double CartesianMPC::costs(const std::vector<double>& x) {
    //     std::vector<State> path = this->decompose(*this->temp_start, x);
    //     return 50 * path_obs_cost(path) + path_waypoints_cost(path);
    // }

    double CartesianMPC::costs(const std::vector<double>& x) {
        // Show elements from second element of x onward
        // std::vector<double> x_ = std::vector<double>(x.begin() + 1, x.end());

        // float t = this->dt;

        // if (t > .75) {
        //     t = .75;
        // } else if (t < 0) {
        //     t = 0;
        // }

        std::vector<State> path = this->decompose(*this->temp_start, x, this->dt);
        return 8 * path_obs_cost(path) + 5 * path_waypoints_cost(path);
    }

    // Trajectory CartesianMPC::calculate_trajectory() {
    //     std::vector<double> path = {};
    //     std::vector<double> x = {};

    //     temp_start = std::make_unique<State>(start);

    //     // Create an initial guess straight forward from last point
    //     for (int i = 0; i < num_inputs; i++) {
    //         // Push 0 for steering angle
    //         x.push_back(0);
    //         // Push back current state velocity
    //         x.push_back(start.vel);
    //     }

    //     for (int i = 0; i < horizon_extension_iters; i++) {
    //         optimize_iter(opt, x);

    //         // Append the first `keep_per_extension` inputs to path
    //         for (int j = 0; j < keep_per_extension; j++) {
    //             path.push_back(x[j * 2]);
    //             path.push_back(x[j * 2 + 1]);
    //         }

    //         // Shift all inputs left by `keep_per_extension` input sets
    //         x.erase(x.begin(), x.begin() + keep_per_extension * 2);

    //         // Forward simulate the start_state with the last input
    //         std::vector<State> fin_path = decompose(*temp_start, x);
    //         this->temp_start = std::make_unique<State>(fin_path[fin_path.size() - 1]);
    //         float last_vel = temp_start->vel;

    //         // Add `keep_per_extension` new input sets to the end
    //         for (int j = 0; j < keep_per_extension; j++) {
    //             x.push_back(0);
    //             // Push back vel of the last input
    //             x.push_back(last_vel);
    //         }
    //     }

    //     // Extend the final path by `additionally_extend` more steps
    //     for (int i = 0; i < additionally_extend; i++) {
    //         path.push_back(x[i * 2]);
    //         path.push_back(x[i * 2 + 1]);
    //     }

    //     std::vector<State> fin_path = decompose(start, path);

    //     Trajectory traj;
    //     traj.waypoints = fin_path;
    //     traj.cost = path_obs_cost(fin_path) + path_waypoints_cost(fin_path);

    //     return traj;
    // }

    // Trajectory CartesianMPC::calculate_trajectory() {
    //     std::vector<std::vector<double>> paths;

    //     float start_angle = -.2;
    //     float angle_step = .2;

    //     for (int i = 0; i < 3; i++) {
    //         std::vector<double> path;
    //         for (int j = 0; j < num_inputs; j++) {
    //             path.push_back(start_angle + j * angle_step);
    //             path.push_back(.2);
    //         }
    //         paths.push_back(path);
    //     }

    //     temp_start = std::make_unique<State>(start);

    //     double min_cost = 100000000;
    //     std::vector<double> x;

    //     // Optimize each path
    //     for (int i = 0; i < paths.size(); i++) {
    //         std::vector<double> path = paths[i];
    //         optimize_iter(opt, path);

    //         double cost = costs(path);
    //         if (cost < min_cost) {
    //             min_cost = cost;
    //             x = path;
    //         }
    //     }

    //     // Decompose the optimized trajectory
    //     std::vector<State> path = decompose(start, x);

    //     Trajectory trajectory;
    //     trajectory.waypoints = path;
    //     trajectory.cost = min_cost;

    //     std::cout << "Min Cost: " << min_cost;

    //     return trajectory;
    // }

    Trajectory BaseMPC::calculate_trajectory(Trajectory initial_guess) {
        auto start_time = std::chrono::high_resolution_clock::now();
        std::cout << "Calculating trajectory..." << std::endl;
        (void)initial_guess;
        std::vector<double> x;

        // Initiate guess
        x.reserve(num_inputs * 2);
        for (int i = 0; i < num_inputs; ++i) {
            x.push_back(0);
            x.push_back(0);
        }

        // Set constraints
        std::vector<double> lb;
        std::vector<double> ub;
        lb.reserve(num_inputs * 2);
        ub.reserve(num_inputs * 2);
        for (int i = 0; i < num_inputs; ++i) {
            lb.push_back(constraints.tau[0]);
            lb.push_back(constraints.vel[0]);
            ub.push_back(constraints.tau[1]);
            ub.push_back(constraints.vel[1]);
        }
        opt.set_lower_bounds(lb);
        opt.set_upper_bounds(ub);

        // Optimize
        optimize_iter(opt, x);

        // Decompose and output
        Trajectory trajectory;
        trajectory.waypoints = decompose(start, x, this->dt);
        trajectory.cost = costs(x);
        trajectory.timestep = this->dt;
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
        std::cout << "Time: " << duration.count() << std::endl;
        std::cout << "nlopt result: " << opt.last_optimum_value() << std::endl;

        return trajectory;
    }

    // Lane MPC implementation
    void LaneFollowingMPC::set_reference_polynomial(const Trajectory& reference,
        int start_index, int n) {
        reference_polynomial.fit(reference.waypoints, start_index, n);
        reference_waypoints = reference_polynomial.genWaypoints(target_vel * dt, num_inputs);
    }

    void LaneFollowingMPC::set_cost_weights(const LaneCostWeights& weights) {
        cost_weights_ = weights;
        // Keep target_vel in sync for any external consumers
        target_vel = weights.target_vel;
    }

    bool LaneFollowingMPC::get_point_metrics(const State& state, size_t waypoint_index,
        double& along_track, double& cte, double& costmap_cost,
        double& cte_bad, double& obs_bad, double& along_track_penalty) const
    {
        const double nan = std::numeric_limits<double>::quiet_NaN();
        bool has_reference = !reference_waypoints.empty();
        if (has_reference) {
            const size_t ref_index = std::min(waypoint_index, reference_waypoints.size() - 1);
            const double waypoint_lx = reference_waypoints[ref_index];
            std::vector<double> errors =
                reference_polynomial.getErrorRelativeTo(state.pose.x, state.pose.y, waypoint_lx);
            along_track = errors[0];
            cte = errors[1];
        } else {
            along_track = nan;
            cte = nan;
        }

        const LaneCostWeights weights = cost_weights_;

        if (costmap) {
            costmap_cost = costmap->cost(state);
        } else {
            costmap_cost = nan;
        }

        if (std::isnan(costmap_cost)) {
                obs_bad = nan;
        } else {
            double d = costmap_cost;
            if (d <= 0.2251) obs_bad = 100 * std::exp(-3 * d);
            else obs_bad = std::exp(1/(5 * d * d)) - 1;
        }
        if (std::isnan(cte)) {
            cte_bad = nan;
        } else {
            cte_bad = weights.w_cte * std::exp(4*std::abs(cte)-4)-0.9;
        }

        if (std::isnan(along_track)) {
            along_track_penalty = nan;
        } else if (along_track < 0) {
            along_track_penalty = weights.w_along_track * along_track * along_track;
        } else {
            along_track_penalty = 0.0;
        }

        return has_reference || costmap;
    }


    // double LaneFollowingMPC::interpolated_costmap_penalty(const State& current, const State* next, 
    //     double resolution) const
    // {
    //     const Pose& start_pose = current.pose;
    //     const Pose& end_pose   = next ? next->pose : current.pose;

    //     const double dx     = end_pose.x - start_pose.x;
    //     const double dy     = end_pose.y - start_pose.y;
    //     const double length = std::hypot(dx, dy);

    //     // no segment -> no path-integrated obstacle cost
    //     if (!next || length < 1e-6) {
    //         return 0.0;
    //     }
    //     if (length < 0.1) return 10000;

    //     const int steps = std::max(1, static_cast<int>(std::ceil(length / resolution)));
    //     const double inv_steps = 1.0 / static_cast<double>(steps);

    //     auto sample_at = [&](double t) -> double {
    //         State probe = current;
    //         probe.pose.x = start_pose.x + t * dx;
    //         probe.pose.y = start_pose.y + t * dy;

    //         double c = this->costmap->cost(probe);
  
    //         if (c >= 0.4) return 10;
    //         if (c > 0.0) {
    //             return c * (1.0 + 0.48 * (1.0 - c));
    //         }
    //         return 0;
    //         if (c >= 0.2) return 5;
    //         if (c > 0.01) return c * 20;
    //         return 0;
    //     };

    //     double accum = 0.0;
    //     for (int step = 0; step <= steps; ++step) {
    //         const double t = static_cast<double>(step) * inv_steps;
    //         accum += sample_at(t);
    //     }

    //     // approx average * length
    //     const double avg_cost = accum * inv_steps;
    //     return avg_cost * length;
    // }

    double LaneFollowingMPC::costs(const std::vector<double>& x) {
        const LaneCostWeights weights = cost_weights_;

        double cost = 0.0;
        double path_length = 0.0;
        static constexpr double kInterpResolution = 0.05;

        State state = start;
        auto add_state_cost = [&](double& C, const State& s, const State* next_state) {
            // C += weights.w_cte * s.cte * s.cte;
            // C += weights.w_heading * s.theta_e * s.theta_e;
            // const double dv = s.vel - weights.target_vel;
            // C += weights.w_speed * dv * dv;  // W speed ❤️‍🩹
            // New try: remove w speed and maybe cte, and instead interpolate waypoints using speed from polynomial for n desired points, and then simply distance from those combined with distance to obstacle from costmap.

            // C += weights.w_costmap * interpolated_costmap_penalty(
            //         s, next_state, kInterpResolution);

            // cost += weights.w_costmap / std::max(1e-10, costmap->cost(state));
            // double c = costmap->cost(s);
            // C += weights.w_costmap / std::max(1e-10, c);
        };
        auto add_cost = [&](double& C, const State& s, double waypoint_lx) {
            std::vector<double> errors =
            reference_polynomial.getErrorRelativeTo(s.pose.x, s.pose.y, waypoint_lx);


            double along_track = errors[0];
            double cte = errors[1];

            if (along_track < 0) {
                C += weights.w_along_track * along_track * along_track;
            }
            double d = costmap->cost(s);
            if (d <= 0.2251) C += 100 * std::exp(-3 * d);
            else C += std::exp(1/(5 * d * d)) - 1;
            C += weights.w_cte * std::exp(4*std::abs(cte)-4)-0.9;

            // y=\left\{x\le0.224:53,x>0.224:e^{\frac{1}{5x^{2}}}-1\right\}


            weights.cte_threshold;
            weights.obs_threshold;
            weights.w_cte;
            weights.w_costmap;
        };
        for (int i = 0; i < x.size(); i += 2) {
            Input2 input = {x[i], x[i + 1]};
            State next_state = state.update(input, dt,
                reference_polynomial,
                dimensions, constraints);
            double c = 0;
            add_cost(c, state, reference_waypoints[i / 2]);
            if (c > cost) cost = c;
            // add_state_cost(cost, state, &next_state);
            state = next_state;
        }
        // add_state_cost(cost, state, nullptr);

        // // Extend last state and check that we not just easing up to an obstacle
        // {
        //     State lookahead = state;

        //     const double yaw = state.pose.theta;
            
        //     lookahead.pose.theta = yaw;
        //     lookahead.pose.x += extension_dist * std::cos(yaw);
        //     lookahead.pose.y += extension_dist * std::sin(yaw);

        //     cost += weights.w_lookahead *
        //             interpolated_costmap_penalty(state, &lookahead, kInterpResolution);
        // }

        return cost;
    }

    std::vector<State> LaneFollowingMPC::decompose(State start_state, std::vector<double> u, double dt) {
        std::vector<State> path;
        State state = start_state;
        for (int i = 0; i < u.size(); i += 2) {
            Input2 input = {u[i], u[i + 1]};
            State next_state = state.update(
                input, dt, reference_polynomial, dimensions, constraints);
            path.push_back(state);
            state = next_state;
        }
        return path;
    }

}  // namespace cev_planner::local_planner
