#include "local_planning/mpc.h"

namespace cev_planner::local_planner {

    double MPC::path_obs_cost(std::vector<State>& path) {
        double cost = 0;

        for (int i = 0; i < path.size(); i++) {
            cost += this->costmap->cost(path[i]);
        }

        return cost;
    }

    double MPC::path_waypoints_cost(std::vector<State>& path) {
        double cost = 0;
        for (int i = 1; i < path.size(); i++) {
            // Waypoints
            for (int j = 1; j < waypoints.waypoints.size(); j++) {
                cost += .3 * path[i].pose.distance_to(waypoints.waypoints[j].pose);
            }

            cost += .5 * path[i].pose.distance_to(target.pose);
        }

        // Additional cost for last node distance to goal
        cost += 2 * path[path.size() - 1].pose.distance_to(target.pose);

        // Ensure that final velocity low
        // cost += 1 * path[path.size() - 1].vel;

        return cost;
    }

    std::vector<State> MPC::decompose(State start_state, std::vector<double> u, double dt) {
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

    double MPC::costs(const std::vector<double>& x) {
        std::vector<State> path = this->decompose(*this->temp_start, x, this->dt);
        return 50 * path_obs_cost(path) + path_waypoints_cost(path);
    }

    // double MPC::costs(const std::vector<double>& x) {
    //     // Show elements from second element of x onward
    //     std::vector<double> x_ = std::vector<double>(x.begin() + 1, x.end());

    //     std::vector<State> path = this->decompose(*this->temp_start, x_, x[0]);
    //     return 50 * path_obs_cost(path) + path_waypoints_cost(path);
    // }

    double MPC::objective_function(const std::vector<double>& x, std::vector<double>& grad,
        void* data) {
        MPC* mpc = (MPC*)data;

        return mpc->costs(x);
    }

    void MPC::optimize_iter(nlopt::opt& opt, std::vector<double>& x) {
        double minf;
        opt.optimize(x, minf);
    }

    Trajectory MPC::calculate_trajectory() {
        std::vector<double> path = {};
        std::vector<double> x = {};

        temp_start = std::make_unique<State>(start);

        // Create an initial guess straight forward from last point
        for (int i = 0; i < num_inputs; i++) {
            // Push 0 for steering angle
            x.push_back(0);
            // Push back current state velocity
            x.push_back(start.vel);
        }

        for (int i = 0; i < horizon_extension_iters; i++) {
            optimize_iter(opt, x);

            // Append the first `keep_per_extension` inputs to path
            for (int j = 0; j < keep_per_extension; j++) {
                path.push_back(x[j * 2]);
                path.push_back(x[j * 2 + 1]);
            }

            // Shift all inputs left by `keep_per_extension` input sets
            x.erase(x.begin(), x.begin() + keep_per_extension * 2);

            // Forward simulate the start_state with the last input
            std::vector<State> fin_path = decompose(*temp_start, x, this->dt);
            this->temp_start = std::make_unique<State>(fin_path[fin_path.size() - 1]);
            float last_vel = temp_start->vel;

            // Add `keep_per_extension` new input sets to the end
            for (int j = 0; j < keep_per_extension; j++) {
                x.push_back(0);
                // Push back vel of the last input
                x.push_back(last_vel);
            }
        }

        // Extend the final path by `additionally_extend` more steps
        for (int i = 0; i < additionally_extend; i++) {
            path.push_back(x[i * 2]);
            path.push_back(x[i * 2 + 1]);
        }

        std::vector<State> fin_path = decompose(start, path, this->dt);

        Trajectory traj;
        traj.waypoints = fin_path;
        traj.cost = path_obs_cost(fin_path) + path_waypoints_cost(fin_path);

        return traj;
    }

    // Trajectory MPC::calculate_trajectory() {
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

    // Trajectory MPC::calculate_trajectory() {
    //     std::vector<double> x;

    //     x.push_back(dt);

    //     temp_start = std::make_unique<State>(start);

    //     for (int i = 0; i < num_inputs; i++) {
    //         x.push_back(0);
    //         x.push_back(start.vel);
    //     }

    //     // Optimize
    //     optimize_iter(opt, x);

    //     std::vector<double> x_ = std::vector<double>(x.begin() + 1, x.end());

    //     // Decompose the optimized trajectory
    //     std::vector<State> path = decompose(start, x_, x[0]);

    //     Trajectory trajectory;
    //     trajectory.waypoints = path;
    //     trajectory.cost = costs(x);

    //     return trajectory;
    // }

}  // namespace cev_planner::local_planner