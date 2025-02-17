#include "vis/vis.h"

namespace cev_planner::vis {
    void vis_costmap(Grid grid, Grid costs) {
        cv::Mat img(grid.data.rows(), grid.data.cols(), CV_8UC3, cv::Vec3b(0, 0, 0));

        for (int i = 0; i < costs.data.rows(); ++i) {
            for (int j = 0; j < costs.data.cols(); ++j) {
                img.at<cv::Vec3b>(i, j) = cv::Vec3b(0, 0, costs.data(i, j) * 255);
            }
        }

        for (int i = 0; i < grid.data.rows(); ++i) {
            for (int j = 0; j < grid.data.cols(); ++j) {
                if (grid.data(i, j) > 0.0) {
                    img.at<cv::Vec3b>(i, j) = cv::Vec3b(grid.data(i, j) * 255,
                        grid.data(i, j) * 255, grid.data(i, j) * 255);
                }
            }
        }

        cv::imwrite("./costmap.png", img);
    }

    void vis_trajectory(Grid grid, State start, Trajectory trajectory, State target) {
        cv::Mat img(grid.data.rows(), grid.data.cols(), CV_8UC3, cv::Vec3b(0, 0, 0));

        for (int i = 0; i < grid.data.rows(); ++i) {
            for (int j = 0; j < grid.data.cols(); ++j) {
                if (grid.data(i, j) > 0.5) {
                    img.at<cv::Vec3b>(i, j) = cv::Vec3b(0, 0, 0);
                } else {
                    img.at<cv::Vec3b>(i, j) = cv::Vec3b(255, 255, 255);
                }
            }
        }

        for (State waypoint: trajectory.waypoints) {
            int x = (waypoint.pose.x - grid.origin.x) / grid.resolution;
            int y = (waypoint.pose.y - grid.origin.y) / grid.resolution;

            img.at<cv::Vec3b>(x, y) = cv::Vec3b(0, 255, 0);
        }

        int x = (start.pose.x - grid.origin.x) / grid.resolution;
        int y = (start.pose.y - grid.origin.y) / grid.resolution;
        img.at<cv::Vec3b>(x, y) = cv::Vec3b(0, 0, 255);

        x = (target.pose.x - grid.origin.x) / grid.resolution;
        y = (target.pose.y - grid.origin.y) / grid.resolution;
        img.at<cv::Vec3b>(x, y) = cv::Vec3b(255, 0, 0);

        cv::imwrite("./trajectory.png", img);
    }
}