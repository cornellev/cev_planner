#pragma once

#include <vector>
#include <cmath>
#include <algorithm>
#include <eigen3/Eigen/Dense>

#include "pose.h"

// shouldn't be modifying this directly, will clean up in a bit

namespace cev_planner {

    struct Grid {
        Eigen::MatrixXf data;
        Eigen::MatrixXf cost_field;
        Pose origin;
        double resolution;

        float at(double x, double y) const {
            int i = static_cast<int>(std::round((x - origin.x) / resolution));
            int j = static_cast<int>(std::round((y - origin.y) / resolution));
            if (i < 0 || i >= data.rows() || j < 0 || j >= data.cols())
                return -1.0f;
            return data(i, j);
        }

        float cost_at(double x, double y) const {
            int i = static_cast<int>(std::round((x - origin.x) / resolution));
            int j = static_cast<int>(std::round((y - origin.y) / resolution));
            if (cost_field.size() == 0 ||
                i < 0 || i >= cost_field.rows() || j < 0 || j >= cost_field.cols())
                return 0.0f;
            return cost_field(i, j);
        }

        void compute_cost_field(double sigma_m = 0.35, double cutoff_m = 2.0) {
            int rows = data.rows(), cols = data.cols();
            if (rows == 0 || cols == 0) return;
            cost_field.resize(rows, cols);

            const float INF = static_cast<float>(rows + cols);
            const float SQ2 = 1.41421356f;

            Eigen::MatrixXf dist = Eigen::MatrixXf::Constant(rows, cols, INF);
            for (int i = 0; i < rows; ++i)
                for (int j = 0; j < cols; ++j)
                    if (data(i, j) >= 1.0f)
                        dist(i, j) = 0.0f;

            for (int i = 0; i < rows; ++i) {
                for (int j = 0; j < cols; ++j) {
                    float d = dist(i, j);
                    if (i > 0)                       d = std::min(d, dist(i-1, j)   + 1.0f);
                    if (j > 0)                     d = std::min(d, dist(i,   j-1) + 1.0f);
                    if (i > 0 && j > 0)        d = std::min(d, dist(i-1, j-1) + SQ2);
                    if (i > 0 && j < cols - 1)        d = std::min(d, dist(i-1, j+1) + SQ2);
                    dist(i, j) = d;
                }
            }

            for (int i = rows - 1; i >= 0; --i) {
                for (int j = cols - 1; j >= 0; --j) {
                    float d = dist(i, j);
                    if (i < rows - 1)                  d = std::min(d, dist(i+1, j)   + 1.0f);
                    if (j < cols - 1)                  d = std::min(d, dist(i,   j+1) + 1.0f);
                    if (i < rows - 1 && j < cols - 1)  d = std::min(d, dist(i+1, j+1) + SQ2);
                    if (i < rows - 1 && j > 0)         d = std::min(d, dist(i+1, j-1) + SQ2);
                    dist(i, j) = d;
                }
            }

            float res    = static_cast<float>(resolution);
            float sigma  = static_cast<float>(sigma_m);
            float cutoff = static_cast<float>(cutoff_m);
            float inv2s2 = -0.5f / (sigma * sigma);

            for (int i = 0; i < rows; ++i) {
                for (int j = 0; j < cols; ++j) {
                    float dm = dist(i, j) * res;
                    if (dm <= 0.0f)
                        cost_field(i, j) = 1.0f;
                    else if (dm >= cutoff)
                        cost_field(i, j) = 0.0f;
                    else
                        cost_field(i, j) = std::exp(inv2s2 * dm * dm);
                }
            }
        }
    };

}  // namespace cev_planner
