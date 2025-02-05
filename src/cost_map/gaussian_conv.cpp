#include "cost_map/gaussian_conv.h"

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <iostream>

namespace cev_planner::cost_map {

    double GaussianCostMap::cost(State state) {
        // Convert the state to grid coordinates
        int x = (state.pose.x - cost_map.origin.x) / cost_map.resolution;
        int y = (state.pose.y - cost_map.origin.y) / cost_map.resolution;

        // Check if the state is within the bounds of the cost map
        if (x < 0 || x >= cost_map.data.rows() || y < 0 || y >= cost_map.data.cols()) {
            return std::numeric_limits<double>::max();
        }

        return cost_map.data(x, y);
    }

    Eigen::VectorXf GaussianConvolution::gen_kernel(int search_radius, float sigma) {
        int kernel_size = 2 * search_radius + 1;
        Eigen::VectorXf kernel(kernel_size);
        float sum = 0.0f;

        for (int i = 0; i < kernel_size; ++i) {
            int dist = i - search_radius;
            kernel(i) = exp(-(dist * dist) / (2 * sigma * sigma));
            sum += kernel(i);
        }

        // Normalize the kernel to ensure the sum is 1
        kernel /= sum;
        return kernel;
    }

    void visualizeCostmap(const std::vector<std::vector<double>>& costmap,
        const Eigen::MatrixXf& grid,
        const std::string& filename =
            "/home/sloth/Programming/CEV/rc-local-planning-workspace/costmap.png") {
        int rows = costmap.size();
        int cols = costmap[0].size();

        // Flatten the 2D vector into a 1D array for cv::Mat
        std::vector<uchar> flatCostmap;
        for (int i = 0; i < rows; ++i) {
            for (int j = 0; j < cols; ++j) {
                // Normalize cost values to range [0, 255] and add them to flatCostmap
                flatCostmap.push_back(static_cast<uchar>(costmap[i][j]
                                                         * 255.0));  // Assuming 0 <= cost <= 1
                // cout << costmap[i][j] << endl;
            }
        }

        // Create a cv::Mat from the flattened 1D array
        cv::Mat image(rows, cols, CV_8UC1, flatCostmap.data());  // 8-bit single-channel image

        // Apply a colormap (e.g., 'JET', 'HOT', 'PLASMA')
        cv::Mat coloredImage;
        cv::applyColorMap(image, coloredImage, cv::COLORMAP_HOT);

        // Overlay the original grid on the colored image
        for (int i = 0; i < grid.rows(); ++i) {
            for (int j = 0; j < grid.cols(); ++j) {
                if (grid(i, j) > 0) {
                    // cv::circle(coloredImage, cv::Point(j, i), 1, cv::Scalar(200, 200, 200), -1);
                    cv::rectangle(coloredImage, cv::Point(j, i), cv::Point(j + 1, i + 1),
                        cv::Scalar(200, 200, 200), -1);
                }
            }
        }

        // Invert the image (optional)
        // cv::bitwise_not(coloredImage, coloredImage);

        // Make the red stronger
        // cv::Mat channels[3];
        // cv::split(coloredImage, channels);
        // channels[2] = channels[2] * 2.0;
        // cv::merge(channels, 3, coloredImage);

        // Save the colored image to a file
        cv::imwrite(filename, coloredImage);

        std::cout << "Costmap image saved as " << filename << std::endl;
    }

    std::unique_ptr<CostMap> GaussianConvolution::generate_cost_map(Grid grid) {
        // Convolution along rows
        Eigen::MatrixXf row_conv = Eigen::MatrixXf::Zero(grid.data.rows(), grid.data.cols());

        for (int i = 0; i < grid.data.rows(); ++i) {
            for (int j = 0; j < grid.data.cols(); ++j) {
                float sum = 0.0f;
                for (int k = -search_radius; k <= search_radius; ++k) {
                    int idx = j + k;
                    if (idx >= 0 && idx < grid.data.cols()) {
                        float tmp = grid.data(i, idx);
                        float tmp2 = kernel(k + search_radius);

                        sum += grid.data(i, idx) * kernel(k + search_radius);
                    }
                }
                row_conv(i, j) = sum;
            }
        }

        // Convolution along columns
        Eigen::MatrixXf cost_map = Eigen::MatrixXf::Zero(grid.data.rows(), grid.data.cols());
        for (int j = 0; j < row_conv.cols(); ++j) {
            for (int i = 0; i < row_conv.rows(); ++i) {
                float sum = 0.0f;
                for (int k = -search_radius; k <= search_radius; ++k) {
                    int idx = i + k;
                    if (idx >= 0 && idx < row_conv.rows()) {
                        sum += row_conv(idx, j) * kernel(k + search_radius);
                    }
                }
                cost_map(i, j) = sum;
            }
        }

        Grid cost_map_ = Grid();
        cost_map_.data = cost_map;
        cost_map_.origin = grid.origin;
        cost_map_.resolution = grid.resolution;

        // Convert the Eigen matrix to a 2D vector for visualization
        std::vector<std::vector<double>> cost_map_vec(cost_map.rows(),
            std::vector<double>(cost_map.cols()));

        for (int i = 0; i < cost_map.rows(); ++i) {
            for (int j = 0; j < cost_map.cols(); ++j) {
                cost_map_vec[i][j] = cost_map(i, j);
            }
        }

        visualizeCostmap(cost_map_vec, grid.data);

        return std::make_unique<GaussianCostMap>(cost_map_);
    }

}