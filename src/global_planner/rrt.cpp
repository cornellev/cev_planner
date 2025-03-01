#include "global_planning/rrt.h"
#include <eigen3/Eigen/Dense>

namespace cev_planner::global_planner {
    bool RRT::path_to_goal() {
        if (goalFlag) {
            path.clear();
            int nodes_best = (goalstate.empty())
                                 ? -1
                                 : *std::min_element(goalstate.begin(), goalstate.end(),
                                       [&](int a, int b) {
                                           return nodes.at(a).cost < nodes.at(b).cost;
                                       });

            int goal_best = (startstate.empty())
                                ? -1
                                : *std::min_element(startstate.begin(), startstate.end(),
                                      [&](int a, int b) {
                                          return goal_nodes.at(a).cost < goal_nodes.at(b).cost;
                                      });
            if (nodes_best == goal_best == -1) return goalFlag;
            from_goal = goal_best != -1
                        && (nodes_best == -1
                            || goal_nodes.at(goal_best).cost < nodes.at(nodes_best).cost);
            best = from_goal ? goal_best : nodes_best;
            unordered_map<int, Node>& nodes = from_goal ? goal_nodes : this->nodes;
            bestCost = nodes.at(best).cost;
            path.push_back(best);

            int newpoint = nodes.at(best).parent;
            while (newpoint != 0) {
                path.push_back(newpoint);
                newpoint = nodes.at(newpoint).parent;
            }
            path.push_back(0);
        }
        return goalFlag;
    }

    void RRT::step(bool bias, bool from_goal) {
        int node = get_cur_index(from_goal) + 1;
        unordered_map<int, Node>& nodes = *cur_tree;
        unordered_map<int, Node>& other_nodes = from_goal ? this->nodes : goal_nodes;
        KDTree& kdTree = from_goal ? goal_kdTree : this->kdTree;
        KDTree& other_kdTree = from_goal ? this->kdTree : goal_kdTree;
        Coordinate& target = from_goal ? start.coordinate : goal.coordinate;
        unordered_set<int>& goalstate = from_goal ? startstate : this->goalstate;
        unordered_set<int>& other_goalstate = from_goal ? this->goalstate : startstate;
        Coordinate point = bias ? target : sample_envir();
        Pose pointPose = point.as_is_pose();

        bool foundGoal = bias;
        vector<int> neighbors;
        for (pair<int, Pose>& pr:
            kdTree.get_knn(pointPose, min(10, max(5, (int)(nodes.size() / 10))))) {
            Coordinate c = {(int)round(pr.second.x), (int)round(pr.second.y)};
            if (pr.first != node && !(pr.second.x == point.x && pr.second.y == point.y)
                && !(foundGoal && goalstate.count(pr.first)) && !cross_obstacle_points(c, point))
                neighbors.push_back(pr.first);
        }
        if (neighbors.empty()) return;

        int bestNeighbor = *std::min_element(neighbors.begin(), neighbors.end(),
            [&nodes](int a, int b) { return nodes.at(a).cost < nodes.at(b).cost; });
        add_node(node, point);
        add_edge(bestNeighbor, node);
        double dist = nodes.at(bestNeighbor).distance_to(point);
        if (node == get_cur_index(from_goal))
            nodes.at(node).set_cost(dist + nodes.at(nodes.at(node).parent).cost);
        for (int neighbor: neighbors) {
            if (neighbor == bestNeighbor) continue;
            double neighborCost = get_cost(node, neighbor);
            if (neighborCost < nodes.at(neighbor).cost
                && !((nodes.at(neighbor).x == point.x && nodes.at(neighbor).y == point.y)
                     || is_ancestor(neighbor, node))) {
                nodes.at(neighbor).set_cost(neighborCost);

                rewire_edge(nodes.at(neighbor).parent, neighbor, node);

                vector<unordered_set<int>*> stack = {&nodes.at(neighbor).children};
                while (!stack.empty()) {
                    std::unordered_set<int>* children = stack.back();
                    stack.pop_back();
                    for (int child: *children) {
                        nodes.at(child).set_cost(get_cost(nodes.at(child).parent, child));
                        stack.push_back(&nodes.at(child).children);
                    }
                }
            }
        }
        int cur_this = node;
        if (!foundGoal) {
            auto [nearest_other, nearest_other_pose] = other_kdTree.get_nearest(pointPose);
            if (nearest_other && nodes.at(node).cost + other_nodes.at(nearest_other).cost < bestCost
                && pointPose.distance_to(nearest_other_pose) < 20
                && !cross_obstacle_points(point, other_nodes.at(nearest_other).coordinate)) {
                cur_this = merge_trees(node, nearest_other, kdTree, nodes, other_nodes);
                other_goalstate.insert(merge_trees(nearest_other, node, other_kdTree, other_nodes,
                    nodes));
                foundGoal = true;
            }
        }
        kdTree.add_point({node, pointPose});
        if (foundGoal) {
            goalFlag = true;
            goalstate.insert(cur_this);

            double old_best = bestCost;
            path_to_goal();
            if (bestCost < old_best) {
                if (abs(bestCost - old_best) > 3) num_not_in_region = nodes.size();
                delete sampling_region;
                delete ellipse;
                sampling_region = new SamplingRegion{
                    d_base, calculate_k(), getPathCoords(), mapw - 1, maph - 1};
                ellipse = new Ellipse{
                    startCoordPose, goalCoordPose, bestCost, mapw, maph};
            } else
                sampling_region->shorten_radius(calculate_k());
        }
        if (!foundGoal && !from_goal && point.distance_to(target) <= 50) {
            this->bias();
        }
    }

    double RRT::obstacle_path_cost(int x1, int y1, int x2, int y2, int radius) {
        int dx = abs(x2 - x1);
        int dy = abs(y2 - y1);
        int sx = (x1 < x2) ? 1 : -1;
        int sy = (y1 < y2) ? 1 : -1;
        int err = dx - dy;
        double cost = 0;

        while (true) {
            for (int i = -radius; i <= radius; i++) {
                for (int j = -radius; j <= radius; j++) {
                    int x = x1 + i;
                    int y = y1 + j;
                    if (is_occupied(x, y)) {
                        cost += exp(-0.1 * (i * i + j * j) / (!is_occupied(x,y,true) + 1));
                    }
                }
            }
    
            if (x1 == x2 && y1 == y2)
                break;
    
            int e2 = 2 * err;
    
            if (e2 > -dy) {
                err -= dy;
                x1 += sx;
            }
    
            if (e2 < dx) {
                err += dx;
                y1 += sy;
            }
        }
        
        return cost;
    }

    void RRT::adjust_point(Coordinate& coordinate, Coordinate& prev, Coordinate& next, int radius) {
        double cost = (obstacle_path_cost(coordinate.x, coordinate.y, next.x, next.y, radius) + obstacle_path_cost(coordinate.x, coordinate.y, prev.x, prev.y, radius)) / coordinate.distance_to(next);
        int best_x = coordinate.x;
        int best_y = coordinate.y;
        for (int i = -radius; i <= radius; i++) {
            for (int j = -radius; j <= radius; j++) {
                int x = coordinate.x + i;
                int y = coordinate.y + j;
                if (!(i==0 && j==0) && !is_occupied(x, y) && !cross_obstacle_points(x, y, prev.x, prev.y) && !cross_obstacle_points(x, y, next.x, next.y)) {
                    double new_cost = log(1+sqrt(i * i + j * j)) * (obstacle_path_cost(x, y, next.x, next.y, radius) + obstacle_path_cost(x, y, prev.x, prev.y, radius)) / coordinate.distance_to(next);
                    if (new_cost < cost) {
                        best_x = x;
                        best_y = y;
                        cost = new_cost;
                    }
                }
            }
        }
        coordinate.x = best_x;
        coordinate.y = best_y;
    }

    Trajectory RRT::interpolate_trajectory(Trajectory& input, double max_dist) {
        Trajectory interpolated;
        interpolated.waypoints.push_back(input.waypoints[0]);
        
        for (int i = 0; i < input.waypoints.size() - 1; i++) {
            double dx = input.waypoints[i+1].pose.x - input.waypoints[i].pose.x;
            double dy = input.waypoints[i+1].pose.y - input.waypoints[i].pose.y;
            double dist = sqrt(dx*dx + dy*dy);
            
            if (dist > max_dist) {
                int num_interp_points = std::ceil(dist / max_dist);
                for (int j = 1; j < num_interp_points; j++) {
                    double t = j / (double)num_interp_points;
                    double x = input.waypoints[i].pose.x + t * dx;
                    double y = input.waypoints[i].pose.y + t * dy;
                    interpolated.waypoints.push_back({x, y});
                }
            }
            interpolated.waypoints.push_back(input.waypoints[i+1]);
        }
        return interpolated;
    }

    Trajectory RRT::angle_interpolate_trajectory(Trajectory& input) {
        Trajectory angle_interpolated;
        angle_interpolated.waypoints.push_back(input.waypoints[0]);
        int j = 1;
        while (j < input.waypoints.size() - 1) {
            double angle;
            Pose current, next, prev;
            prev = input.waypoints[j-1].pose;
            current = input.waypoints[j].pose;
            angle_interpolated.waypoints.push_back({current.x, current.y});
            Vector2d v1 = {current.x - prev.x, current.y - prev.y};
            bool x = false;
            while (true) {
                next = input.waypoints[++j].pose;
                Vector2d v2 = {current.x - next.x, current.y - next.y};
                double angle = acos(std::clamp(v1.dot(v2) / (v1.norm() * v2.norm()), -1.0, 1.0));
                if (j == input.waypoints.size() - 1 || angle < 3 || cross_obstacle_tf_points(current.x, current.y, next.x, next.y)) {
                    break;
                }
            }
        }

        if (j != input.waypoints.size() - 1) angle_interpolated.waypoints.push_back(input.waypoints[j]);
        angle_interpolated.waypoints.push_back(input.waypoints.back());
        return interpolate_trajectory(angle_interpolated);
    }

    Trajectory RRT::getPathCoords(bool corrected_tf) {
        unordered_map<int, Node>* nodes = from_goal ? &goal_nodes : &this->nodes;
        Trajectory res;
        
        if (path.empty()) return res;
    
        int cur_node = path[0];
        Pose p = nodes->at(cur_node).as_is_pose();
        res.waypoints.push_back(State(p.x, p.y));
    
        int i = 0;
        while (i < path.size() - 1) {
            int cur = path[i];
            int next_node = path[i + 1];
    
            if (cross_obstacle_nodes(cur_node, next_node, nodes)) {
                p = nodes->at(cur).as_is_pose();
                res.waypoints.push_back(State(p.x, p.y));
                cur_node = cur;
            }
            i++;
        }
    
        p = nodes->at(path[path.size() - 1]).as_is_pose();
        if (!(p.x == res.waypoints.back().pose.x && p.y == res.waypoints.back().pose.y))
            res.waypoints.push_back(State(p.x, p.y));
    
        if (corrected_tf) {
            if (res.waypoints[0].pose.distance_to(this->startCoordPose) > res.waypoints[0].pose.distance_to(this->goalCoordPose)) {
                std::reverse(res.waypoints.begin(), res.waypoints.end());
            }

            Trajectory optimized_coords;
            optimized_coords.waypoints.reserve(res.waypoints.size());
            int old_x, old_y;
            int rad = 5;
            if (!res.waypoints.empty()) {
                old_x = res.waypoints.front().pose.x;
                old_y = res.waypoints.front().pose.y;
                Pose p = Coordinate(res.waypoints.front().pose.x, res.waypoints.front().pose.y).tf_pose(resolution, origin);
                optimized_coords.waypoints.push_back({p.x, p.y});
            }

            for (size_t i = 1; i < res.waypoints.size() - 1; i++) {
                Coordinate current(res.waypoints[i].pose.x, res.waypoints[i].pose.y);
                Coordinate prev = Coordinate(old_x, old_y);
                Coordinate next(res.waypoints[i + 1].pose.x, res.waypoints[i + 1].pose.y);

                adjust_point(current, prev, next, rad);
                old_x = current.x;
                old_y = current.y;
                Pose p = current.tf_pose(resolution, origin);
                optimized_coords.waypoints.push_back({ p.x, p.y });
            }

            if (res.waypoints.size() > 1) {
                Pose p = Coordinate(res.waypoints.back().pose.x, res.waypoints.back().pose.y).tf_pose(resolution, origin);
                optimized_coords.waypoints.push_back({p.x, p.y});
            }

            Trajectory interpolated = interpolate_trajectory(optimized_coords);

            Trajectory angle_interpolated = angle_interpolate_trajectory(interpolated);

            return interpolated;
        }
        return res;
    }

    RRT::Coordinate RRT::sample_envir() {
        Coordinate res;
        Random randWidth{0, static_cast<double>(mapw - 1)};
        Random randHeight{0, static_cast<double>(maph - 1)};
        while (true) {
            bool sampling = (rand.Uniform() < 0.87);
            if (goalFlag) {
                Pose p = sampling ? sampling_region->generate_random_point()
                                  : ellipse->generate_random_point();
                res.x = p.x;
                res.y = p.y;
            } else {
                res.x = randWidth.Int();
                res.y = randHeight.Int();
            }
            if (!is_occupied(res)) break;
        }
        return res;
    }

    void RRT::filter_nodes(unordered_map<int, RRT::Node>& nodes, unordered_set<int>& goalstate) {
        for (auto it = nodes.begin(); it != nodes.end();) {
            auto& [n, v] = *it;
            bool keep = !v.children.empty() || (estimatedCost(v.coordinate) <= bestCost);

            if (!keep) {
                nodes.at(v.parent).remove_child(n);
                goalstate.erase(n);
                it = nodes.erase(it);
            } else {
                ++it;
            }
        }
    }

    void RRT::pruneTrees() {
        filter_nodes(nodes, goalstate);
        filter_nodes(goal_nodes, startstate);
        updateKDTree();
    }

    int RRT::merge_trees(int node, int cur_other, KDTree& kdTree, unordered_map<int, Node>& nodes,
        unordered_map<int, Node>& other_nodes) {
        int cur_this = get_cur_index(&nodes == &this->goal_nodes);
        int prev = node;
        int i = 0;
        while (true) {
            if (i == 1) prev = cur_this;
            cur_this++;
            RRT::Node& other_node = other_nodes.at(cur_other);
            add_node(cur_this, other_node.coordinate, &nodes);
            add_edge(prev, cur_this, &nodes);
            nodes.at(cur_this).set_cost(get_cost(prev, cur_this, &nodes));
            kdTree.add_point({cur_this, other_node.as_is_pose()});
            if (cur_other == 0) break;
            cur_other = other_node.parent;
            i = 1;
        }
        return cur_this;
    }

    bool RRT::is_surrounded(int x1, int y1, int radius) {
        for (int i = -radius; i <= radius; i++) {
            for (int j = -radius; j <= radius; j++) {
                if (i == 0 && j == 0) continue;
                if (is_occupied(x1 + i, y1 + j, true))
                    return true;
            }
        }
        return false;
    }

    bool RRT::cross_obstacle_points(int x1, int y1, int x2, int y2) {
        int dx = abs(x2 - x1);
        int dy = abs(y2 - y1);
        int sx = (x1 < x2) ? 1 : -1;
        int sy = (y1 < y2) ? 1 : -1;
        int err = dx - dy;

        while (true) {
            if (is_occupied(x1, y1) || is_surrounded(x1, y1)) {
                return true;
            }
    
            if (x1 == x2 && y1 == y2) {
                break;
            }
    
            int e2 = 2 * err;
            if (e2 > -dy) {
                err -= dy;
                x1 += sx;
            }
    
            if (e2 < dx) {
                err += dx;
                y1 += sy;
            }
        }
    
        return false;
    }

    bool RRT::cross_obstacle_nodes(int startNode, int endNode, unordered_map<int, RRT::Node>* nodes) {
        nodes = nodes ?: cur_tree;
        return cross_obstacle_points(nodes->at(startNode).coordinate,
            nodes->at(endNode).coordinate);
    }

    bool RRT::cross_obstacle_points(Coordinate& startPoint, Coordinate& endPoint) {
        return cross_obstacle_points(startPoint.x, startPoint.y, endPoint.x, endPoint.y);
    }

    bool RRT::cross_obstacle_tf_points(int x1, int y1, int x2, int y2) {
        x1 = std::clamp((int)round((x1 - origin.x) / resolution), 0, mapw - 1);
        y1 = std::clamp((int)round((y1 - origin.y) / resolution), 0, maph - 1);
        x2 = std::clamp((int)round((x2 - origin.x) / resolution), 0, mapw - 1);
        y2 = std::clamp((int)round((y2 - origin.y) / resolution), 0, mapw - 1);
        return cross_obstacle_points(x1, y1, x2, y2);
    }

    bool RRT::is_ancestor(int potential_ancestor, int node, unordered_map<int, RRT::Node>* nodes) {
        nodes = nodes ?: cur_tree;
        int current = node;
        unordered_set<int> visited = unordered_set<int>();
        while (current != 0) {
            current = nodes->at(current).parent;
            if (current == potential_ancestor) return true;
            if (visited.count(current)) return false;
            visited.insert(current);
        }
        return false;
    }

}  // namespace cev_planner::global_planner