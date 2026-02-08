#pragma once

#include "util.h"
#include "constraints.h"

#include <iostream>
#include <eigen3/Eigen/Dense>

namespace cev_planner {
    /**
     * @brief Pose represents position of objects in 2D space. Angles are constrainted to the range
     * [-pi, pi]
     *
     */
    struct Pose {
        Pose(double x = 0, double y = 0, double theta = 0)
            : x(x), y(y), theta(restrict_angle(theta)) {}

        float distance_to(const Pose& other) const {
            return std::hypot(x - other.x, y - other.y);
        }

        // X coordinate in meters
        double x;

        // Y coordinate in meters
        double y;

        // Orientation in radians
        double theta;
    };

    /**
     * @brief Input represents the rate-based control input to a vehicle
     *
     */
    struct Input {
        // Steering rate in rad/s
        double dtau;

        // Longitudinal acceleration in m/s^2
        double accel;
    };

    struct Input2 {
        double tau;
        double vel;
    };

    struct CubicPolynomial;

    /**
     * @brief State represents position and orientation, and other state variables of a vehicle in
     * 2D space. Angles are constrainted to the range [-pi, pi]
     *
     */
    struct State {
        State(double x = 0, double y = 0, double theta = 0, double tau = 0, double vel = 0, double cte = 0, double theta_e = 0)
            : pose(Pose(x, y, theta)), tau(tau), vel(vel), cte(cte), theta_e(theta_e) {}

        // Pose of the vehicle
        Pose pose;

        // Steering angle in radians
        double tau;

        // Velocity in m/s
        double vel;

        // State values for lane MPC        
        double cte;      // Cross track error
        double theta_e;  // Heading error

        
        State update(Input input, double dt, Dimensions& dimensions, Constraints& constraints) {
            State _state = *this;

            double accel = std::clamp(input.accel, constraints.accel[0], constraints.accel[1]);
            double dtau = std::clamp(input.dtau, constraints.dtau[0], constraints.dtau[1]);

            _state.vel = std::clamp(vel + accel * dt, constraints.vel[0], constraints.vel[1]);
            _state.tau = std::clamp(tau + dtau * dt, constraints.tau[0], constraints.tau[1]);

            double avg_vel = 0.5 * (vel + _state.vel);
            // double avg_tau = 0.5 * (tau + _state.tau);
            
            if (std::abs(_state.tau) < 1e-4) {
                _state.pose.theta = pose.theta;
            } else {
                double R = dimensions.wheelbase / std::tan(_state.tau);
                double dtheta = (avg_vel / R) * dt;
                _state.pose.theta = restrict_angle(pose.theta + dtheta);
            }

            double avg_theta = 0.5 * (pose.theta + _state.pose.theta);

            _state.pose.x += avg_vel * cos(avg_theta) * dt;
            _state.pose.y += avg_vel * sin(avg_theta) * dt;

            return _state;
        }

        State update(Input2 input, double dt, CubicPolynomial& r, Dimensions& dimensions, Constraints& constraints);

        std::string to_string() {
            return "State: (" + std::to_string(pose.x) + ", " + std::to_string(pose.y) + ", "
                   + std::to_string(pose.theta) + ", " + std::to_string(tau) + ", "
                   + std::to_string(vel) + ")";
        }
    };

    // MPC utils
    // struct CubicPolynomial {
    //     double a3=0, a2=0, a1=0, a0=0;
    //     double x0=0, y0=0, m0=0;     // start endpoint and slope
    //     double x1=0, y1=0, m1=0;     // finish endpoint and slope

    //     bool fit(const std::vector<State>& pts, int start, int n){
    //         if (n < 4) return false;

    //         Eigen::MatrixXd X(n, 4);
    //         Eigen::VectorXd Y(n);

    //         for (int i = 0; i < n; ++i) {
    //             const auto& p = pts[start + i].pose;
    //             const double xi = p.x, xi2 = xi*xi;
    //             X(i,0)=xi2*xi; X(i,1)=xi2; X(i,2)=xi; X(i,3)=1.0;
    //             Y(i)=p.y;
    //         }
    //         Eigen::Vector4d a = X.colPivHouseholderQr().solve(Y);
    //         a3=a(0); a2=a(1); a1=a(2); a0=a(3);

    //         // Cache endpoint info for extrapolation
    //         const auto& L = pts[start].pose;
    //         const auto& R = pts[start + n - 1].pose;
    //         x0 = L.x; y0 = L.y; m0 = deriv(x0);
    //         x1 = R.x; y1 = R.y; m1 = deriv(x1);
    //         return true;
    //     }

    //     double deriv(double x) const {
    //         if (x0 < x1 && x <= x0) return m0;
    //         if (x0 < x1 && x >= x1) return m1;
    //         if (x0 > x1 && x <= x1) return m1;
    //         if (x0 > x1 && x >= x0) return m0;
    //         return (3.0 * a3 * x + 2.0 * a2) * x + a1;
    //     }

    //     double at(double x) const {
    //         if (x0 < x1 && x <= x0) return y0 + m0 * (x - x0);
    //         if (x0 < x1 && x >= x1) return y1 + m1 * (x - x1);
    //         if (x0 > x1 && x <= x1) return y1 + m1 * (x - x1);
    //         if (x0 > x1 && x >= x0) return y0 + m0 * (x - x0);
    //         return ((a3 * x + a2) * x + a1) * x + a0;
    //     }

    //     std::vector<State> genWaypoints(double step_dist, int num_waypoints) const {
    //         std::vector<State> res;
    //         res.reserve(num_waypoints);

    //         double dist_target = 0.0;
    //         double cur_x = x0;
    //         double total_dist = 0.0;
            
    //         res.push_back({x0, y0, std::atan(m0)});

    //         const double dx = 0.05 * (x0 < x1 ? 1.0 : -1.0);
    //         const double dx_sq = dx * dx;

    //         for (int i = 1; i < num_waypoints; ++i) {
    //             dist_target += step_dist;
    //             double prev_y = at(cur_x);

    //             while (total_dist < dist_target) {             
    //                 cur_x += dx;
    //                 double dy = at(cur_x) - prev_y;
    //                 prev_y += dy;

    //                 double ds = std::sqrt(dx_sq + dy * dy);
    //                 total_dist += ds;
    //             }

    //             res.push_back({cur_x, at(cur_x), std::atan(deriv(cur_x))});
    //         }

    //         return res;
    //     }
    // };
    
    struct CubicPolynomial {
        // Coefficients: y = a3*x^3 + a2*x^2 + a1*x + a0
        double a3 = 0, a2 = 0, a1 = 0, a0 = 0;
    
        // Local Frame Metadata
        double x0_g = 0, y0_g = 0, yaw0_g = 0;
        double local_x1 = 0;   // end of polynomial segment in local-x
        double c1 = 0, c2 = 0; // endcap slope/pos at local_x1 (your original)
    
        // ---------- Arc-length cache (FAST along-track) ----------
        // s(lx) cached on uniform grid: lx_i = i * cache_dx
        double cache_dx = 0.01;          // tune: smaller -> more accurate, larger -> faster build
        double cache_inv_dx = 100.0;
        std::vector<double> s_cache;     // size = cache_N, cumulative arc-length from 0 to lx_i
        double s_total = 0.0;            // s(local_x1) approx
        bool cache_ready = false;
    
        // Clamp helper
        static inline double clamp(double v, double lo, double hi) {
            return (v < lo) ? lo : (v > hi ? hi : v);
        }
    
        // --- Fit polynomial in local frame (unchanged logic) ---
        // NOTE: you'll need your State type; here I only reference pose.x/pose.y like your code.
        template <typename State>
        bool fit(const std::vector<State>& pts, int start, int n) {
            if (n < 4) return false;
    
            x0_g = pts[start].pose.x;
            y0_g = pts[start].pose.y;
    
            // Orient local X-axis toward next point
            yaw0_g = std::atan2(pts[start + 1].pose.y - y0_g,
                                pts[start + 1].pose.x - x0_g);
    
            const double cos_y = std::cos(-yaw0_g);
            const double sin_y = std::sin(-yaw0_g);
    
            Eigen::MatrixXd X(n, 4);
            Eigen::VectorXd Y(n);
    
            for (int i = 0; i < n; ++i) {
                double dx_g = pts[start + i].pose.x - x0_g;
                double dy_g = pts[start + i].pose.y - y0_g;
    
                // Transform to Local Frame
                double lx = dx_g * cos_y - dy_g * sin_y;
                double ly = dx_g * sin_y + dy_g * cos_y;
    
                double lx2 = lx * lx;
                X(i, 0) = lx2 * lx;
                X(i, 1) = lx2;
                X(i, 2) = lx;
                X(i, 3) = 1.0;
                Y(i) = ly;
    
                if (i == n - 1) local_x1 = lx;
            }
    
            Eigen::Vector4d a = X.colPivHouseholderQr().solve(Y);
            a3 = a(0); a2 = a(1); a1 = a(2); a0 = a(3);
    
            // Endcap values for your piecewise extension (unchanged)
            c1 = (3.0 * a3 * local_x1 + 2.0 * a2) * local_x1 + a1;
            c2 = ((a3 * local_x1 + a2) * local_x1 + a1) * local_x1 + a0;
    
            // Build cache for fast arc-length queries
            buildArcLengthCache(cache_dx);
    
            return true;
        }
    
        // Local derivative (slope in body frame)
        inline double deriv_local(double lx) const {
            if (lx <= 0) return a1;
            if (lx >= local_x1) return c1;
            return (3.0 * a3 * lx + 2.0 * a2) * lx + a1;
        }
    
        // Local second derivative (needed for fast Newton projection)
        inline double second_deriv_local(double lx) const {
            // Inside the cubic segment: y'' = 6 a3 x + 2 a2
            // Outside: your endcaps are linear -> y'' = 0
            if (lx <= 0) return 0.0;
            if (lx >= local_x1) return 0.0;
            return 6.0 * a3 * lx + 2.0 * a2;
        }
    
        // Local position (y in body frame)
        inline double at_local(double lx) const {
            if (lx <= 0) return a0 + a1 * lx;
            if (lx >= local_x1) return c2 + c1 * (lx - local_x1);
            return ((a3 * lx + a2) * lx + a1) * lx + a0; // Horner
        }

    
        // Waypoints generator
        std::vector<double> genWaypoints(double step_dist, int num_waypoints) const {
            std::vector<double> res;
            res.reserve(num_waypoints);
    
            double cur_lx = 0.0;
            double total_s = 0.0;
            const double dx = 0.01;
            const double dx2 = dx * dx;
            res.push_back(cur_lx);
    
            for (int i = 1; i < num_waypoints; ++i) {
                double target_s = i * step_dist;
    
                double y_prev = at_local(cur_lx);
                while (total_s < target_s) {
                    cur_lx += dx;
                    double dy = at_local(cur_lx) - y_prev;
                    y_prev += dy;
                    total_s += std::sqrt(dx2 + dy * dy);
                }
                res.push_back(cur_lx);
            }
            return res;
        }
    
        // ---------- FAST cache build + query ----------
        void buildArcLengthCache(double dx) {
            cache_dx = std::max(1e-4, dx);
            cache_inv_dx = 1.0 / cache_dx;
    
            double x1 = std::max(local_x1, 0.0);
            int N = std::max(2, (int)std::ceil(x1 * cache_inv_dx) + 1);
    
            s_cache.assign(N, 0.0);
    
            double s = 0.0;
            double x = 0.0;
            double y_prev = at_local(0.0);
    
            for (int i = 1; i < N; ++i) {
                double x_next = i * cache_dx;
                if (x_next > x1) x_next = x1;
    
                double y_next = at_local(x_next);
                double dx_step = x_next - x;
                double dy = y_next - y_prev;
    
                s += std::sqrt(dx_step * dx_step + dy * dy);
                s_cache[i] = s;
    
                x = x_next;
                y_prev = y_next;
            }
    
            s_total = s_cache.back();
            cache_ready = true;
        }
    
        // O(1) arc-length query via linear interpolation
        inline double arclen_to(double lx) const {
            if (!cache_ready || s_cache.size() < 2) return 0.0;
    
            lx = clamp(lx, 0.0, local_x1);
            double f = lx * cache_inv_dx;
            int i = (int)f;
            if (i >= (int)s_cache.size() - 1) return s_cache.back();
            if (i < 0) return 0.0;
    
            double t = f - (double)i;
            return s_cache[i] + t * (s_cache[i + 1] - s_cache[i]);
        }
    
        // ---------- FAST projection (few Newton steps) ----------
        //
        // Minimize D(x) = (x - p_lx)^2 + (f(x) - p_ly)^2 over x in [0, local_x1]
        // Newton on D'(x) = 2(x - p_lx) + 2(f(x) - p_ly) f'(x)
        // D''(x) = 2 + 2 f'(x)^2 + 2(f(x) - p_ly) f''(x)
        //
        // This is very fast (constant work) and good enough for control / error decomposition.
        inline double project_local_newton(double p_lx, double p_ly) const {
            // Initial guess: clamp to segment
            double x = clamp(p_lx, 0.0, local_x1);
    
            // A few Newton steps; clamp each step to keep stable and fast.
            // 4–6 iterations is typically plenty.
            for (int it = 0; it < 6; ++it) {
                double y  = at_local(x);
                double dy = deriv_local(x);
                double ddy = second_deriv_local(x);
    
                double ex = x - p_lx;
                double ey = y - p_ly;
    
                double d1 = 2.0 * (ex + ey * dy);
                double d2 = 2.0 * (1.0 + dy * dy + ey * ddy);
    
                // Avoid divide-by-small
                if (std::abs(d2) < 1e-9) break;
    
                double step = d1 / d2;
                x = clamp(x - step, 0.0, local_x1);
    
                // Early exit if step tiny
                if (std::abs(step) < 1e-6) break;
            }
            return x;
        }
    
        // ---------- FIXED + FAST error function ----------
        //
        // Returns {along_track_error, cross_track_error}
        // along_track_error: arc-length difference between projection and reference lx_ref
        // cross_track_error: signed normal distance at the projection point
        //
        // This is designed to be called at high rate:
        // - just a transform + ~6 Newton steps + 2 arc-length lookups + a few flops
        inline std::vector<double> getErrorRelativeTo(double x_global, double y_global, const double lx_ref) const {
            // 1) Transform Global Point to the Polynomial's Local Frame
            double dx_g = x_global - x0_g;
            double dy_g = y_global - y0_g;
            double cos_y = std::cos(-yaw0_g);
            double sin_y = std::sin(-yaw0_g);
    
            double p_lx = dx_g * cos_y - dy_g * sin_y;
            double p_ly = dx_g * sin_y + dy_g * cos_y;
    
            // 2) Project point onto curve in local frame (closest point)
            double x_star = project_local_newton(p_lx, p_ly);
            double y_star = at_local(x_star);
    
            // 3) Signed CTE = dot((point - proj), unit_normal)
            double slope = deriv_local(x_star);
    
            // Tangent (1, slope) -> unit
            double tx = 1.0, ty = slope;
            double tnorm = std::sqrt(tx * tx + ty * ty);
            tx /= (tnorm + 1e-12);
            ty /= (tnorm + 1e-12);
    
            // Normal = (-ty, tx)
            double nx = -ty;
            double ny = tx;
    
            double vx = p_lx - x_star;
            double vy = p_ly - y_star;
            double cross_track = vx * nx + vy * ny;
    
            // 4) Along-track = arc-length difference along curve
            double along_track_error = arclen_to(x_star) - arclen_to(lx_ref);
    
            return {along_track_error, cross_track};
        }
    };

    inline State State::update(Input2 input, double dt, CubicPolynomial& r, Dimensions& dimensions, Constraints& constraints) {
        State s = *this;

        // double accel_ = input.vel;
        // double dtau_ = input.tau;

        // double accel = std::clamp((input.vel - vel) / dt, constraints.accel[0], constraints.accel[1]);
        // double dtau = std::clamp((input.tau - tau) / dt, constraints.dtau[0], constraints.dtau[1]);

        // The velocity and acceleration at the current time step
        vel = std::clamp(input.vel, constraints.vel[0], constraints.vel[1]);
        tau = std::clamp(input.tau, constraints.tau[0], constraints.tau[1]);

        double dist = vel * dt;
        double dtheta = dist / dimensions.wheelbase * std::tan(tau);
        s.pose.theta = restrict_angle(pose.theta + dtheta);
        double avg_theta = pose.theta + (0.5 * dtheta);
        s.pose.x += dist * std::cos(avg_theta);
        s.pose.y += dist * std::sin(avg_theta); // FIND BEST

        s.vel = vel; // 0???
        s.tau = tau;

        // s.cte = r.at(s.pose.x) - s.pose.y;// + vel * sin(theta_e) * dt;
        // s.theta_e = restrict_angle(s.pose.theta - r.deriv(s.pose.x));

        return s;
    }

}
