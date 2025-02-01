#include "local_planning/mpc.h"

namespace cev_planner::local_planner {
    Grid MPC::calculate_costmap(Grid grid, Pose start, Pose target, Trajectory waypoints) {
        return Grid();
    }

    Trajectory MPC::calculate_trajectory(Pose start, Pose target) {
        return Trajectory();
    }

}  // namespace cev_planner::local_planner