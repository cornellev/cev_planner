#pragma once

#include "global_planner.h"
#include "kd_tree.h"
#include "rrt_regions.h"
#include <random>
#include <optional>
#include <chrono>

#include <unordered_set>
using std::unordered_set;

namespace cev_planner::global_planner {
    /**
     * @brief RRT is a global planner that plans a path through an occupancy grid to a target
     * pose while avoiding obstacles and obeying constraints
     *
     */
    class RRT : public GlobalPlanner {
    public:
        /**
         * @brief Construct a new RRT object
         *
         * @param dimensions Dimensions of the robot
         * @param constraints Constraints on the robot's motion
         */
        Dimensions robotDimensions;
        Constraints constraints;
        RRT(Dimensions dimensions, Constraints constraints)
            : GlobalPlanner(dimensions, constraints) {
            this->robotDimensions = dimensions;
            this->constraints = constraints;
        }

        /**
         * @brief Plan a `Trajectory` through an occupancy grid from a start to a target `State`
         * while avoiding obstacles and obeying constraints
         *
         * @param grid Occupancy grid with probability of occupancy at each cell
         * @param start Starting `State` of the robot
         * @param target Target `State` of the robot
         * @return `Trajectory` from the start pose to the target pose
         */
        std::optional<Trajectory> plan_path(Grid grid, State start, State target) override {
            resolution = grid.resolution;
            origin = grid.origin;
            goalFlag = false;
            mapw = grid.data.rows();
            maph = grid.data.cols();
            nodes = {
                {0, {max(min((int)round((start.pose.x - grid.origin.x) / resolution), mapw - 1), 0),
                        max(min((int)round((start.pose.y - grid.origin.y) / resolution), maph - 1),
                            0)}}};
            goal_nodes = {{0,
                {max(min((int)round((target.pose.x - grid.origin.x) / resolution), mapw - 1), 0),
                    max(min((int)round((target.pose.y - grid.origin.y) / resolution), maph - 1),
                        0)}}};
            this->start = nodes[0];
            this->goal = goal_nodes[0];
            startCoordPose = this->start.as_is_pose();
            goalCoordPose = this->goal.as_is_pose();
            iter = 0;
            num_not_in_region = 0;
            num_max_in_region = 2000;
            d_base = max(mapw, maph) / 10;
            cur_index = 0;
            cur_goal_index = 0;
            best = 0;
            bestCost = inf;
            kdTree = makeKDTree(nodes);
            goal_kdTree = makeKDTree(goal_nodes);
            cur_tree = &nodes;
            goalstate = unordered_set<int>();
            startstate = unordered_set<int>();
            path = vector<int>();
            pathCoords = vector<Coordinate>();
            from_goal = false;
            cache_obstacle_grid(grid);
            double distToGoal = start.pose.distance_to(target.pose);
            bias();
            int iteration = 0;
            auto start_time = std::chrono::steady_clock::now();
            while (true) {
                double in_region = num_in_region();
                if (iteration % 500 == 0 and goalFlag
                    and (abs(bestCost - distToGoal) < 1
                         or (in_region / log(bestCost + 2)
                             > min(max(bestCost / 2000 * 75 + 275, 275.0), 375.0))))
                    break;

                expand();
                if (!goalFlag and iteration % 500 == 0) updateKDTree();

                if (iteration % 1000 == 0) {
                    // if time elapsed > 2 seconds, break
                    if (std::chrono::steady_clock::now() - start_time > std::chrono::seconds(2))
                        break;

                    if (goalFlag and in_region / bestCost < 2) {
                        pruneTrees();
                    }
                }
                iteration++;
            }
            delete sampling_region;
            delete ellipse;
            sampling_region = nullptr;
            ellipse = nullptr;
            Trajectory traj = getPathCoords(true);
            if (traj.waypoints.empty()) return std::nullopt;

            return std::make_optional(traj);
        }

        class Coordinate {
        public:
            int x, y;
            Coordinate(int x = 0, int y = 0): x(x), y(y) {}
            double distance_to(const Coordinate& other) const {
                return std::hypot(x - other.x, y - other.y);
            }
            bool operator==(const Coordinate& other) const {
                return x == other.x && y == other.y;
            }

            Pose tf_pose(double resolution, Pose& origin) const {
                return {x * resolution + origin.x, y * resolution + origin.y};
            }

            Pose as_is_pose() const {
                return {static_cast<double>(x), static_cast<double>(y)};
            }
        };

        class Node {
        public:
            int x, y;
            Coordinate coordinate;
            int parent;
            unordered_set<int> children;
            double cost;
            Node(int x = 0, int y = 0, double cost = 0)
                : coordinate(x, y), x(x), y(y), cost(cost), parent(-1) {}
            Node(Coordinate& coordinate, double cost = 0)
                : coordinate(coordinate),
                  x(coordinate.x),
                  y(coordinate.y),
                  cost(cost),
                  parent(-1) {}

            void set_parent(int parent) {
                this->parent = parent;
            }
            void set_cost(double cost) {
                this->cost = cost;
            }
            void add_child(int child) {
                this->children.insert(child);
            }
            void remove_child(int child) {
                this->children.erase(child);
            }
            double distance_to(const Node& other) const {
                return coordinate.distance_to(other.coordinate);
            }
            Pose as_is_pose() const {
                return coordinate.as_is_pose();
            }
            Pose tf_pose(double resolution, Pose& origin) const {
                return coordinate.tf_pose(resolution, origin);
            }
        };

    private:
        Node start, goal;
        Pose startCoordPose, goalCoordPose;
        unordered_map<int, Node> nodes, goal_nodes;
        KDTree kdTree, goal_kdTree;
        unordered_map<int, Node>* cur_tree;
        unordered_set<int> goalstate, startstate;
        vector<int> path;
        vector<Coordinate> pathCoords;
        bool goalFlag, from_goal;

        static constexpr double inf = std::numeric_limits<double>::infinity();
        int mapw, maph, iter, num_not_in_region, num_max_in_region, cur_index, cur_goal_index, best;
        double d_base, bestCost = inf;

        SamplingRegion* sampling_region;
        Ellipse* ellipse;
        Random rand;
        double resolution;
        Pose origin;
        // Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic> obstacle_grid;
        Grid* grid;

        void cache_obstacle_grid(Grid& grid) {
            this->grid = &grid;
            // dilate_grid(grid, start, goal, create_circular_structure(0.0), 5.0, 0.7f);
        }

        void dilate_grid(Grid& grid, const Node& start, const Node& goal,
            const std::vector<array<int, 2>>& structure = {{0, 1}, {1, 0}, {0, 0}, {-1, 0},
                {0, -1}},
            double safe_zone_radius = 5.0, float occupation_threshold = 0.7f);

        vector<array<int, 2>> create_circular_structure(double radius = 3.0);

        double calculate_k() {
            double z = num_in_region() - num_max_in_region / 2;
            return 1 / (2 * M_PI) * ((z != 0) ? (z <= 0) * M_PI + atan(1 / z) : M_PI / 2);
        }

        void step(bool bias = false, bool from_goal = false);

        Trajectory getPathCoords(bool corrected_tf = false);

        Coordinate sample_envir();

        void pruneTrees();

        int merge_trees(int node, int cur_other, KDTree& kdTree, unordered_map<int, Node>& nodes,
            unordered_map<int, Node>& other_nodes);

        bool cross_obstacle_points(Coordinate& startPoint, Coordinate& endPoint);

        bool cross_obstacle(int startNode, int endNode, unordered_map<int, Node>* nodes = nullptr);

        bool is_ancestor(int potential_ancestor, int node,
            unordered_map<int, Node>* nodes = nullptr);

        bool is_surrounded(int x1, int y1);

        bool is_occupied(Coordinate& coordinate, bool allow_unknown = false) {
            if (coordinate.x < 0 || coordinate.x >= mapw || coordinate.y < 0
                || coordinate.y >= maph)
                return true;
            double val = grid->data(coordinate.x, coordinate.y);
            return val > 0.7f || !(allow_unknown || val >= 0);
        }

        bool is_occupied(int x, int y, bool allow_unknown = false) {
            if (x < 0 || x >= mapw || y < 0 || y >= maph)
                return true;
            double val = grid->data(x, y);
            return val > 0.7f || !(allow_unknown || val >= 0);
        }

        int get_next_index(bool from_goal) {
            return from_goal ? ++cur_goal_index : ++cur_index;
        }

        int get_cur_index(bool from_goal) {
            return cur_goal_index * from_goal + cur_index * !from_goal;
        }

        double distance(int n1, int n2, unordered_map<int, Node>* nodes = nullptr) {
            nodes = nodes ?: cur_tree;
            return nodes->at(n1).distance_to(nodes->at(n2));
        }

        double get_cost(int parent, int child, unordered_map<int, Node>* nodes = nullptr) {
            nodes = nodes ?: cur_tree;
            return nodes->at(parent).cost + distance(parent, child, nodes);
        }

        double estimatedCost(const Coordinate& coordinate) {
            return coordinate.distance_to(start.coordinate)
                   + coordinate.distance_to(goal.coordinate);
        }

        void add_node(int n, Coordinate& coordinate, unordered_map<int, Node>* nodes = nullptr) {
            nodes = nodes ?: cur_tree;
            bool from_goal = nodes == &goal_nodes;
            nodes->emplace(n, Node(coordinate));
            if (n == get_cur_index(from_goal) + 1) get_next_index(from_goal);
        }

        void add_edge(int parent, int child, unordered_map<int, Node>* nodes = nullptr) {
            nodes = nodes ?: cur_tree;
            nodes->at(child).set_parent(parent);
            nodes->at(parent).add_child(child);
        }

        void rewire_edge(int parent, int old_child, int new_child,
            unordered_map<int, Node>* nodes = nullptr) {
            nodes = nodes ?: cur_tree;
            nodes->at(parent).remove_child(old_child);
            nodes->at(old_child).set_parent(new_child);
            nodes->at(new_child).add_child(old_child);
        }

        double num_in_region() {
            return goalFlag ? nodes.size() + goal_nodes.size() - num_not_in_region : 0;
        }

        void bias() {
            step(true, false);
        }

        void expand() {
            if (iter % 2 == 0) {
                iter = 0;
                cur_tree = &nodes;
                step();
            } else {
                cur_tree = &goal_nodes;
                step(false, true);
            }
            iter++;
        }

        KDTree makeKDTree(unordered_map<int, Node>& nodes) {
            std::unordered_map<int, Pose> poseMap;
            for (const auto& [key, node]: nodes) poseMap[key] = node.as_is_pose();
            return KDTree(poseMap);
        }

        void filter_nodes(unordered_map<int, Node>& nodes, unordered_set<int>& goalstate);

        void updateKDTree() {
            kdTree.clear();
            goal_kdTree.clear();
            kdTree = makeKDTree(nodes);
            goal_kdTree = makeKDTree(goal_nodes);
        }

        bool path_to_goal();
    };
}  // namespace cev_planner::global_planner