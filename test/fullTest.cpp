#include <iostream>
#include "../include/global_planning/rrt.h"
using cev_planner::Pose;
using namespace cev_planner::global_planner;
using namespace cev_planner;

int main() {
    Grid sampleGrid;
    sampleGrid.resolution = 1.0;
    sampleGrid.origin.x = 0.0;
    sampleGrid.origin.y = 0.0;
    sampleGrid.data = Eigen::MatrixXf::Zero(500, 500);
    int x, y;
    for (int i = 0; i < 200; ++i) {
        do {
          x = rand() % 500;
          y = rand() % 500;
        } while ((x < 5 && y < 5) || (x > 494 && y > 494));
        sampleGrid.data(x, y) = -1.0;
    }
    RRT rrt = RRT{{1.0, 1.0, 1.0}, {1.0, 1.0, 1.0, 1.0, 1.0}};

    // Trajectory traj = {{{4.0, 4.0} {3.0, 2.0} {1.0,1.0}, {0.0, 1.0}, {0, 0} }};
    // double z =  0;

    // SamplingRegion sampling_region = SamplingRegion{
    //     max(sampleGrid.data.rows(), sampleGrid.data.cols()) / 10.0,
    //     1/(2*M_PI) * ((z!=0) ? (z<=0) * M_PI + atan(1/z) : M_PI/2),
    //     traj,
    //     5-1, 5-1
    // };
    // Pose point;
    // for (int i = 0; i < 1000000; i++) {
    //     point = sampling_region.generate_random_point();        
    // }

    // std::cout << point.x << " " << point.y << std::endl;
    


    Trajectory res;
    for (int i = 0; i < 1; i++) {
        std::cout << i << std::endl;
        res = rrt.plan_path(sampleGrid, {0.0, 0.0}, {490.0,490.0});
    }

    // // Print the result
    for (const auto& waypoint : res.waypoints) {
        std::cout << waypoint.pose.x << " " << waypoint.pose.y << std::endl;
    }
    
    return 0;
}