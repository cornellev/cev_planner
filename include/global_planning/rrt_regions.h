#pragma once

#include "util.h"
using Eigen::Vector2d;
using std::vector;
using std::array;
using std::pair;
using std::max;
using std::min;

namespace cev_planner::global_planner {
    /**
     * EP-RRT* region
     */
    class SamplingRegion {
        public:
        SamplingRegion(double d, double k, const Trajectory& path, int maxX, int maxY)
        : d(d), d_base(d*k), maxX(maxX), maxY(maxY) {
            cache.reserve(path.waypoints.size());
            this->path.reserve(path.waypoints.size());
            for (const State& w : path.waypoints) {
                this->path.push_back({w.pose.x, w.pose.y});
            }
            expansion_region.reserve(path.waypoints.size());
            generate_expansion_region();
        };

        /**
         * Compresses sampling region by a factor k
         */
        void shorten_radius(double k) {
            d = d_base * k;
            expansion_region.clear();
            generate_expansion_region();
        }

        /** 
         * Generates a point in the sampling region
         */
        Pose generate_random_point();

        private:
            struct QuadRegion {
                public:
                QuadRegion(array<Vector2d, 4>& quad, double cumulative_area, double split_ratio)
                : quad(quad), cumulative_area(cumulative_area), split_ratio(split_ratio) { }
                array<Vector2d, 4> quad;
                double cumulative_area;
                double split_ratio;
            };
            double d_base, d;
            vector<Vector2d> path;
            int maxX, maxY;
            vector<QuadRegion> expansion_region;
            vector<array<Vector2d, 2>> cache;

            array<Vector2d, 4> generate_quad(int i);
            
            void generate_expansion_region();

            pair<Vector2d, double> direction_vector(Vector2d& v1, Vector2d& v2, Vector2d& v3);

            array<Vector2d, 2> extension(pair<Vector2d, double>& vec_and_dir, Vector2d& v);
            array<Vector2d, 2> extension_end(Vector2d& end, Vector2d& other);
            
            Pose random_point_in_triangle(Vector2d& p1, Vector2d& p2, Vector2d& p3);

            void clip(Vector2d& v) {
                v[0] = max(min(v[0], (double)maxX), 0.0);
                v[1] = max(min(v[1], (double)maxY), 0.0);
            }

            Random rand;

            array<double, 3> line_from_nodes(Vector2d& p1, Vector2d& p2) {
                double a = p2[1] - p1[1];
                double b = p1[1] - p2[1];
                return {a, b, -(a * p1[1] + b * p1[1])};
            };

            double point_line_side(array<double, 3>& line, Vector2d& p) {
                return line[0] * p[0] + line[1] * p[1] + line[2];
            }

            bool segment_intersects_infinite_line(Vector2d& p1, Vector2d& p2, array<double, 3>& line) {
                int side1 = line[0] * p1[0] + line[1] * p1[1] + line[2];
                int side2 = line[0] * p2[0] + line[1] * p2[1] + line[2];
                return side1 * side2 < 0;
            };

            double triangle_area(Vector2d& p1, Vector2d& p2, Vector2d& p3) {
                return 0.5 * abs(p1[0]*(p2[1] - p3[1]) + p2[0] * (p3[1] - p1[1]) + p3[0] * (p1[1] - p2[1]));
            };
    };

    /**
     * Elliptical Region
     */
    class Ellipse {
        public:
        Ellipse(const Pose& focus1, const Pose& focus2, double majorAxisLength, int maxX, int maxY, Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic>& obstacle_grid)
        : maxX(maxX), maxY(maxY), obstacle_grid(obstacle_grid) {
            centerX = (focus1.x + focus2.x) / 2;
            centerY = (focus1.y + focus2.y) / 2;
            a = majorAxisLength / 2;
            c = focus2.distance_to(focus1) / 2;
            b = sqrt(max(a*a - c*c, 0.01));
            rotation = atan2(focus2.y - focus1.y, focus2.x - focus1.x);
        };
        
        /** 
         * Generates a point in the sampling region
         */
        Pose generate_random_point();

        private:
            int maxX, maxY;
            double centerX, centerY, a, b, c, rotation;
            Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic> obstacle_grid;
            Random rand;
    };
}  // namespace cev_planner::global_planner