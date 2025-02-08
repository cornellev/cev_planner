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
        cost += 4 * path[path.size() - 1].pose.distance_to(target.pose);

        // Ensure that final velocity low
        // cost += 1 * path[path.size() - 1].vel;

        return cost;
    }

    std::vector<State> MPC::decompose(State start_state, std::vector<double> u) {
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

    double MPC::objective_function(const std::vector<double>& x, std::vector<double>& grad,
        void* data) {
        MPC* mpc = (MPC*)data;

        std::vector<State> path = mpc->decompose(mpc->start, x);
        double cost = 15 * mpc->path_obs_cost(path) + mpc->path_waypoints_cost(path);

        return cost;
    }

    void MPC::optimize_iter(nlopt::opt& opt, std::vector<double>& x) {
        double minf;
        opt.optimize(x, minf);
    }

    Trajectory MPC::calculate_trajectory() {
        std::vector<double> x = {};

        // Create an initial guess straight forward from last point
        for (int i = 0; i < num_states; i++) {
            // Push 0 for steering angle
            x.push_back(0);
            // Push back current state velocity
            x.push_back(start.vel);
        }

        optimize_iter(opt, x);

        std::vector<State> path = decompose(start, x);

        Trajectory traj;
        traj.waypoints = path;

        return traj;
    }

}  // namespace cev_planner::local_planner