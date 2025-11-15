#include "local_planning/mpc.h"

namespace cev_planner::local_planner {

    double MPC::path_obs_cost(std::vector<State>& path) {
        double cost = 0;

        for (int i = 0; i < path.size(); i++) {
            cost += this->costmap->cost(path[i]);
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

    double MPC::path_waypoints_cost(std::vector<State>& path) {
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

    std::vector<State> MPC::decompose(State start_state, std::vector<double> u, double dt) {
        std::vector<State> path;
        State state = start_state;
        path.push_back(state);
        for (int i = 0; i < u.size(); i += 2) {
            Input input = {u[i], u[i + 1]};
            state = state.update(input, this->dt, dimensions, constraints);
            path.push_back(state);
        }
        return path;
    }

    // double MPC::costs(const std::vector<double>& x) {
    //     std::vector<State> path = this->decompose(*this->temp_start, x);
    //     return 50 * path_obs_cost(path) + path_waypoints_cost(path);
    // }

    double MPC::costs(const std::vector<double>& x) {
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

    double MPC::objective_function(const std::vector<double>& x, std::vector<double>& grad,
        void* data) {
        MPC* mpc = (MPC*)data;

        return mpc->costs(x);
    }

    void MPC::optimize_iter(nlopt::opt& opt, std::vector<double>& x) {
        double minf;
        opt.optimize(x, minf);
    }

    // Trajectory MPC::calculate_trajectory() {
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

    Trajectory MPC::calculate_trajectory(Trajectory initial_guess) {
        std::vector<double> x;

        // x.push_back(dt);

        temp_start = std::make_unique<State>(start);

        // Fill initial guess

        // Fill with 0s
        for (int i = 0; i < num_inputs; i++) {
            x.push_back(0);
            x.push_back(0);
        }

        // for (int i = 1; i < initial_guess.waypoints.size(); i++) {
        //     int index = (i - 1) * 2;

        //     x[index] = (initial_guess.waypoints[i].tau - initial_guess.waypoints[i - 1].tau)
        //                / initial_guess.timestep;
        //     x[index + 1] = (initial_guess.waypoints[i].vel - initial_guess.waypoints[i -
        //     1].vel)
        //                    / initial_guess.timestep;
        // }

        // std::cout << "Starting optimization with x size:" << std::endl;

        // Define constraints
        std::vector<double> lb = {};
        std::vector<double> ub = {};

        for (int i = 0; i < num_inputs; i++) {
            // lb.push_back(constraints.tau[0]);
            // lb.push_back(constraints.vel[0]);
            // ub.push_back(constraints.tau[1]);
            // ub.push_back(constraints.vel[1]);
            lb.push_back(constraints.dtau[0]);
            lb.push_back(constraints.accel[0]);
            ub.push_back(constraints.dtau[1]);
            ub.push_back(constraints.accel[1]);
        }

        // std::cout << x.size() << std::endl;

        // std::cout << "Inputs: " << std::endl;
        // for (int i = 0; i < x.size(); i++) {
        //     std::cout << x[i] << " ";
        // }
        // std::cout << std::endl;

        // Optimize
        // nlopt::srand(0);
        optimize_iter(opt, x);
        // std::cout << "Optimization complete" << std::endl;

        // std::vector<double> x_ = std::vector<double>(x.begin() + 1, x.end());

        // Decompose the optimized trajectory
        std::vector<State> path = decompose(start, x, this->dt);

        Trajectory trajectory;
        trajectory.waypoints = path;
        trajectory.cost = path_waypoints_cost(path);
        // trajectory.timestep = x[0];
        trajectory.timestep = this->dt;

        // std::cout << "Cost: " << trajectory.cost << std::endl;

        return trajectory;
    }

}  // namespace cev_planner::local_planner