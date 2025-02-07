#pragma once

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include "pose.h"
#include "trajectory.h"
#include "grid.h"

using namespace cev_planner;

namespace cev_planner::vis {

    void vis_costmap(Grid grid, Grid costs);

    void vis_trajectory(Grid grid, State start, Trajectory trajectory, State target);

}  // namespace cev_planner::vis