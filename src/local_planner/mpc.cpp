#include "local_planning/mpc.h"
#include "util.h"
#include <iostream>
#include <chrono>
#include <cmath>
#include <algorithm>
#include <limits>

namespace cev_planner::local_planner {

    std::vector<State> CartesianMPC::decompose(const std::vector<double>& u) const {
        std::vector<State> path;
        path.reserve(num_inputs + 1);

        State state = start;
        path.push_back(state);

        for (int i = 0; i < num_inputs; ++i) {
            double tau_cmd = std::clamp(u[2*i],     constraints.tau[0], constraints.tau[1]);
            double vel_cmd = std::clamp(u[2*i + 1], constraints.vel[0], constraints.vel[1]);

            double avg_tau   = (state.tau + tau_cmd) / 2.0;
            double avg_vel   = (state.vel + vel_cmd)  / 2.0;
            double dtheta    = avg_vel * std::tan(avg_tau) / dimensions.wheelbase * dt;
            double avg_theta = state.pose.theta + dtheta / 2.0;

            State next;
            next.tau       = tau_cmd;
            next.vel       = vel_cmd;
            next.pose.theta = restrict_angle(state.pose.theta + dtheta);
            next.pose.x    = state.pose.x + avg_vel * std::cos(avg_theta) * dt;
            next.pose.y    = state.pose.y + avg_vel * std::sin(avg_theta) * dt;

            path.push_back(next);
            state = next;
        }
        return path;
    }

    double CartesianMPC::grid_obstacle_cost(const std::vector<State>& path) const {
        const double HL = dimensions.length * 0.5;
        const double HW = dimensions.width * 0.5;

        auto sample = [&](double x, double y) -> double {
            float c = grid.cost_at(x, y);
            if (c >= 1.0f)     return 5000.0;
            if (c > 0.0f)      return 200.0 * std::pow(c, 1.5);
            return 0.0;
        };

        double cost = 0.0;
        for (const auto& s : path) {
            double cx = s.pose.x, cy = s.pose.y, th = s.pose.theta;
            double ch = std::cos(th), sh = std::sin(th);
            // center
            cost += sample(cx, cy);
            // front-left / front-right / rear-left / rear-right corners, provided we update dims
            cost += sample(cx + HL * ch - HW * sh, cy + HL * sh + HW * ch);
            cost += sample(cx + HL * ch + HW * sh, cy + HL * sh - HW * ch);
            cost += sample(cx - HL * ch - HW * sh, cy - HL * sh + HW * ch);
            cost += sample(cx - HL * ch + HW * sh, cy - HL * sh - HW * ch);
        }
        return cost;
    }

    double CartesianMPC::waypoint_cost(const std::vector<State>& path) const {
        if (waypoints.waypoints.empty() || path.size() < 2) return 0.0;

        double cost = 0.0;
        const Pose& wp0 = waypoints.waypoints[0].pose;
        double dx = wp0.x - start.pose.x;
        double dy = wp0.y - start.pose.y;
        double dist_to_wp = std::hypot(dx, dy);

        // bad logic
        double heading_dot = dx * std::cos(start.pose.theta) + dy * std::sin(start.pose.theta);
        bool wp0_behind = (heading_dot < 0.0);
        if (dist_to_wp < 0.5 || wp0_behind) {
            if (waypoints.waypoints.size() > 1) {
                const Pose& wp1 = waypoints.waypoints[1].pose;
                double d1x = wp1.x - start.pose.x;
                double d1y = wp1.y - start.pose.y;
                double d1 = std::hypot(d1x, d1y);
                if (d1 > 1e-3) {
                    double t1x = d1x / d1, t1y = d1y / d1;
                    int n = static_cast<int>(path.size());
                    for (int i = 1; i < n; ++i) {
                        double rx = path[i].pose.x - start.pose.x;
                        double ry = path[i].pose.y - start.pose.y;
                        double progress = t1x * rx + t1y * ry;
                        double w = static_cast<double>(i) / static_cast<double>(n - 1);
                        cost += w * std::max(0.0, d1 - progress);
                    }
                }
            } else {
                cost += 2.0 * path.back().pose.distance_to(wp0);
            }
            return cost;
        }

        double tx = dx / dist_to_wp;
        double ty = dy / dist_to_wp;

        int n = static_cast<int>(path.size());
        for (int i = 1; i < n; ++i) {
            double rx = path[i].pose.x - start.pose.x;
            double ry = path[i].pose.y - start.pose.y;

            double progress = tx * rx + ty * ry;

            double progress_cost = std::max(0.0, dist_to_wp - progress);

            double w = static_cast<double>(i) / static_cast<double>(n - 1);

            cost += w * progress_cost;
        }

        if (waypoints.waypoints.size() > 1) {
            const Pose& wp1 = waypoints.waypoints[1].pose;
            cost += 1.0 * path.back().pose.distance_to(wp1);
        }

        return cost;
    }

    double CartesianMPC::steering_rate_cost(const std::vector<double>& u) const {
        double cost = 0.0;
        double dtau_max = constraints.dtau[1] * dt;   // max allowed |tau change| per step
        double tau_prev = start.tau;

        for (int i = 0; i < num_inputs; ++i) {
            double tau_i = std::clamp(u[2*i], constraints.tau[0], constraints.tau[1]);
            double delta = std::abs(tau_i - tau_prev) - dtau_max;
            if (delta > 0.0) cost += 20.0 * delta * delta;   // smooth quadratic penalty
            tau_prev = tau_i;
        }
        return cost;
    }

    double CartesianMPC::costs(const std::vector<double>& x) {
        std::vector<State> path = decompose(x);

        double obs_w = 10.0;
        double wp_w  = gps_mode ? 30.0 : 20.0;
        return obs_w * grid_obstacle_cost(path)
             + wp_w  * waypoint_cost(path)
             +  1.0  * steering_rate_cost(x);
    }

    double CartesianMPC::objective_function(const std::vector<double>& x,
                                            std::vector<double>& /*grad*/, void* data) {
        return static_cast<CartesianMPC*>(data)->costs(x);
    }

    Trajectory CartesianMPC::calculate_trajectory(Trajectory /*initial_guess*/) {
        auto t0 = std::chrono::high_resolution_clock::now();

        std::vector<double> x(num_inputs * 2, 0.0);

        const double target_speed = std::min(constraints.vel[1], 1.0);

        if (prev_x.size() == static_cast<size_t>(num_inputs * 2)) {
            for (int i = 0; i < num_inputs - 1; ++i) {
                x[2*i]     = prev_x[2*(i+1)];
                x[2*i + 1] = prev_x[2*(i+1) + 1];
            }
            x[2*(num_inputs-1)]     = prev_x[2*(num_inputs-1)];
            x[2*(num_inputs-1) + 1] = target_speed;
        } else {
            prev_x.clear();  // size mismatch, force cold start
            double desired_tau = 0.0;
            if (!waypoints.waypoints.empty()) {
                const Pose& wp = waypoints.waypoints[0].pose;
                double angle_to_wp  = std::atan2(wp.y - start.pose.y, wp.x - start.pose.x);
                double angle_diff   = restrict_angle(angle_to_wp - start.pose.theta);
                desired_tau = std::clamp(angle_diff, constraints.tau[0], constraints.tau[1]);
            }
            double dtau_step = constraints.dtau[1] * dt;   // max tau change per step
            double tau_cur = start.tau;
            for (int i = 0; i < num_inputs; ++i) {
                tau_cur += std::clamp(desired_tau - tau_cur, -dtau_step, dtau_step);
                x[2*i]     = std::clamp(tau_cur, constraints.tau[0], constraints.tau[1]);
                x[2*i + 1] = target_speed;
            }
        }

        std::vector<double> lb(num_inputs * 2), ub(num_inputs * 2);
        for (int i = 0; i < num_inputs; ++i) {
            lb[2*i]     = constraints.tau[0];
            lb[2*i + 1] = 0.0;
            ub[2*i]     = constraints.tau[1];
            ub[2*i + 1] = std::min(constraints.vel[1], 1.0); // will remove
        }
        opt.set_lower_bounds(lb);
        opt.set_upper_bounds(ub);

        for (int i = 0; i < num_inputs * 2; i++)
            x[i] = std::clamp(x[i], lb[i] + 1e-6, ub[i] - 1e-6);

        auto optimize_candidate = [&](std::vector<double>& candidate) -> double {
            double f = std::numeric_limits<double>::infinity();
            try {
                opt.optimize(candidate, f);
            } catch (const std::exception& e) {
                std::cerr << "NLopt exception: " << e.what() << std::endl;
                f = costs(candidate);
            }
            return f;
        };

        double minf = optimize_candidate(x);

        if (std::abs(start.vel) < 0.15 && minf > 2000.0) {
            std::vector<double> x2(num_inputs * 2, 0.0);
            double desired_tau = 0.0;
            if (!waypoints.waypoints.empty()) {
                const Pose& wp = waypoints.waypoints[0].pose;
                double angle_to_wp  = std::atan2(wp.y - start.pose.y, wp.x - start.pose.x);
                double angle_diff   = restrict_angle(angle_to_wp - start.pose.theta);
                desired_tau = std::clamp(angle_diff, constraints.tau[0], constraints.tau[1]);
            }
            double dtau_step = constraints.dtau[1] * dt;
            double tau_cur = start.tau;
            for (int i = 0; i < num_inputs; ++i) {
                tau_cur += std::clamp(desired_tau - tau_cur, -dtau_step, dtau_step);
                x2[2*i]     = std::clamp(tau_cur, constraints.tau[0], constraints.tau[1]);
                x2[2*i + 1] = target_speed;
            }
            for (int i = 0; i < num_inputs * 2; i++)
                x2[i] = std::clamp(x2[i], lb[i] + 1e-6, ub[i] - 1e-6);
            double minf2 = optimize_candidate(x2);
            if (minf2 < minf) { x = x2; minf = minf2; }
        }

        if (std::isfinite(minf)) prev_x = x;

        Trajectory trajectory;
        trajectory.waypoints  = decompose(x);
        trajectory.cost       = minf;
        trajectory.timestep   = this->dt;

        auto t1 = std::chrono::high_resolution_clock::now();
        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();
        std::cout << "MPC planning: " << ms << "ms, cost=" << minf << std::endl;

        return trajectory;
    }

}  // namespace cev_planner::local_planner

