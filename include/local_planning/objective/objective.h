// #pragma once

// #include "local_planner.h"

// #include <nlopt.hpp>
// #include <vector>

// namespace cev_planner::local_planner {

//     /**
//      * @brief Model Predictive Control based local planner
//      *
//      */
//     class MPC : public LocalPlanner {
//     private:
//         int num_inputs = 10;
//         float dt = .25;
//         int horizon_extension_iters = 2;  // 5 horizon extension steps
//         int keep_per_extension = 5;       // keep 3/num_inputs of the best path
//         int additionally_extend = 5;      // extend the final path by 5 more steps
//         volatile int test_updating = 0;
//         State temp_start;
//         nlopt::opt opt;

//         double path_obs_cost(std::vector<State>& path);
//         double path_waypoints_cost(std::vector<State>& path);
//         std::vector<State> decompose(State start_state, std::vector<double> u);
//         static double objective_function(const std::vector<double>& x, std::vector<double>& grad,
//             void* data);
//         void optimize_iter(nlopt::opt& opt, std::vector<double>& x);

//     public:
//         /**
//          * @brief Construct a new MPC object
//          *
//          * @param dimensions Dimensions of the robot
//          * @param constraints Constraints on the robot's motion
//          * @param cost_map_generator Cost map generator
//          */
//         MPC(Dimensions dimensions, Constraints constraints,
//             std::shared_ptr<CostMapGenerator> cost_map_generator) {}

//         Trajectory calculate_trajectory();
//     };
// }  // namespace cev_planner::local_planner