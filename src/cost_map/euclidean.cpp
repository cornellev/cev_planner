#include "cost_map/euclidean.h"

#include <iostream>

namespace cev_planner::cost_map {
    static constexpr float INF = 1e10f;

    double EuclideanCostMap::cost(State state) {
        // Convert the state to grid coordinates
        int x = (state.pose.x - cost_map.origin.x) / cost_map.resolution;
        int y = (state.pose.y - cost_map.origin.y) / cost_map.resolution;

        // Check if the state is within the bounds of the cost map
        if (x < 0 || x >= cost_map.data.rows() || y < 0 || y >= cost_map.data.cols()) {
            return 0;
        }

        return cost_map.data(x, y);
    }

    void Euclidean::edt_1d(const Eigen::Ref<const Eigen::VectorXf>& f, Eigen::Ref<Eigen::VectorXf> out) {
        // Imagine every cell is the minimum point of a parabola.
        // We care about the height of the lowest parabola's value at each point. (infinity means no parabola)
        // Let v[k] = index of best parabola at k, z[k] = intersection boundary of best parabolas v[k-1] and v[k].
        // Each parabola is defined by y = (x - v[k])^2 + f[v[k]]
        int n = f.size();
        if (n == 0) return;

        std::vector<int> v(n);
        std::vector<float> z(n + 1);
        
        int k = 0;  // Index of rightmost parabola
        v[0] = 0;
        z[0] = -INF;
        z[1] = +INF;

        for (int q = 1; q < n; q++) {
            // intersection coordinate between q parabola and v[k] parabola
            float x = ((f[q] + q * q) - (f[v[k]] + v[k] * v[k])) / (2.0f * q - 2.0f * v[k]);
            
            // while new intersection is to the left of previous (new parabola is lower)
            while (x <= z[k]) {
                k--; // remove parabola v[k]
                x = ((f[q] + q * q) - (f[v[k]] + v[k] * v[k])) / (2.0f * q - 2.0f * v[k]);
            }

            // Add new parabola
            k++;
            v[k] = q;
            z[k] = x;
            z[k + 1] = INF;
        }

        int k_idx = 0;
        for (int q = 0; q < n; q++) {
            while (z[k_idx + 1] < q) {
                k_idx++; // move to next parabola
            }
            // output is dx^2 + parabola height (output is square of distance)
            float dx = static_cast<float>(q - v[k_idx]);
            out[q] = dx * dx + f[v[k_idx]];
        }
    }

    // Euclidean Distance Transform
    std::shared_ptr<CostMap> Euclidean::generate_cost_map(Grid grid) {
        int rows = grid.data.rows();
        int cols = grid.data.cols();
        
        // 0 = occupied, INF = free
        Eigen::MatrixXf cost_map = (grid.data.array() < 0.5f).select(Eigen::ArrayXXf::Constant(rows, cols, INF), 0.0f);

        // Pass 1: rows
        for (int i = 0; i < rows; i++) {
            Eigen::VectorXf row_in = cost_map.row(i);
            Eigen::VectorXf row_out(cols);
            edt_1d(row_in, row_out);
            cost_map.row(i) = row_out;
        }

        // Pass 2: cols, using previous values as vertical parabola offsets
        for (int j = 0; j < cols; j++) {
            Eigen::VectorXf col_in = cost_map.col(j);
            Eigen::VectorXf col_out(rows);
            edt_1d(col_in, col_out);
            cost_map.col(j) = col_out;
        }

        Grid cost_map_ = Grid();
        cost_map_.data = cost_map.cwiseSqrt() * grid.resolution;
        cost_map_.origin = grid.origin;
        cost_map_.resolution = grid.resolution;
        last_cost_map_ = cost_map_;
        has_last_cost_map_ = true;

        cev_planner::vis::vis_costmap(grid, cost_map_);

        return std::make_shared<EuclideanCostMap>(cost_map_);
    }
}
