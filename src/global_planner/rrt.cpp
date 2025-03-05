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

    double RRT::obstacle_path_cost(int x1, int y1, int x2, int y2, int radius, bool add_weight_endpoints) {
        int start_x = x1;
        int start_y = y1;
        int end_x = x2;
        int end_y = y2;
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
                        double distanceSq = i * i + j * j;
                        double penalty = (distanceSq <= 4) 
                            ? (10 / (0.5 + distanceSq)) 
                            : (1 / (1 + distanceSq));

                        if (!is_occupied(x, y, true)) penalty *= 0.5;

                        if (add_weight_endpoints && 
                            ((x1 == start_x && y1 == start_y) || (x2 == end_x && y2 == end_y))) {
                            penalty *= 2;
                        }
                        
                        cost += penalty;
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

    Trajectory RRT::round_trajectory(Trajectory& input, int radius) {
        if (input.waypoints.size() < 3) {
            return input;
        }

        Trajectory rounded;
        rounded.waypoints.push_back(input.waypoints[0]);

        int i = 1;
        while (i < (int)input.waypoints.size() - 1) {
            Pose prev = input.waypoints[i - 1].pose;
            Pose curr = input.waypoints[i].pose;
            Pose next = input.waypoints[i + 1].pose;

            double prev_dx = curr.x - prev.x;
            double prev_dy = curr.y - prev.y;
            double next_dx = next.x - curr.x;
            double next_dy = next.y - curr.y;

            double cross_product = prev_dx * next_dy - prev_dy * next_dx;

            if (std::fabs(cross_product) > 1e-3) {
                int left = i - 1;
                int right = i + 1;
                double best_cost = normalized_obstacle_path_cost(curr, input.waypoints[left].pose, input.waypoints[right].pose, radius);
                int best_right = right;

                while (right + 1 < (int)input.waypoints.size()) {
                    right++;
                    double shortcut_cost = 1.2 * normalized_obstacle_path_cost(input.waypoints[left].pose, input.waypoints[right].pose, radius);

                    if (shortcut_cost < best_cost) {
                        best_cost = shortcut_cost;
                        best_right = right;
                    } else {
                        break;
                    }
                }

                Pose start = input.waypoints[left].pose;
                Pose end = input.waypoints[best_right].pose;

                double entry_angle = atan2(curr.y - prev.y, curr.x - prev.x);
                double exit_angle = atan2(next.y - curr.y, next.x - curr.x);

                double max_radius = std::min(3.0, std::hypot(curr.x - start.x, curr.y - start.y));

                double control_scale = 0.5;
                double control_x = curr.x + control_scale * max_radius * cos((entry_angle + exit_angle) / 2);
                double control_y = curr.y + control_scale * max_radius * sin((entry_angle + exit_angle) / 2);

                int num_points = 5;
                for (int j = 1; j < num_points; j++) {
                    double t = (double)j / num_points;
                    double x = (1 - t) * (1 - t) * start.x + 2 * (1 - t) * t * control_x + t * t * end.x;
                    double y = (1 - t) * (1 - t) * start.y + 2 * (1 - t) * t * control_y + t * t * end.y;
                    rounded.waypoints.push_back({x, y});
                }

                rounded.waypoints.push_back(input.waypoints[best_right]);
                i = best_right;
            } else {
                rounded.waypoints.push_back(input.waypoints[i]);
                prev = curr;
            }
            i++;
        }

        if (rounded.waypoints.back().pose.x != input.waypoints.back().pose.x ||
            rounded.waypoints.back().pose.y != input.waypoints.back().pose.y) {
            rounded.waypoints.push_back(input.waypoints.back());
        }

        return rounded;
    }
    

    double RRT::normalized_obstacle_path_cost(int center_x, int center_y, int x2, int y2, int x3, int y3, int radius) {
        return (obstacle_path_cost(center_x, center_y, x2, y2, radius) + obstacle_path_cost(center_x, center_y, x3, y3, radius)) * (hypot(center_x - x2, center_y - y2) + hypot(center_x - x3, center_y - y3));
    }

    double RRT::normalized_obstacle_path_cost(Pose& center, Pose& p2, Pose& p3, int radius) {
        int center_x = tf_to_coord_horizontal(center.x);
        int center_y = tf_to_coord_vertical(center.y);
        int x2 = tf_to_coord_horizontal(p2.x);
        int y2 = tf_to_coord_vertical(p2.y);
        int x3 = tf_to_coord_horizontal(p3.x);
        int y3 = tf_to_coord_vertical(p3.y);
        return (obstacle_path_cost(center_x, center_y, x2, y2, radius, false) + obstacle_path_cost(center_x, center_y, x3, y3, radius, false)) * (hypot(center_x - x2, center_y - y2) + hypot(center_x - x3, center_y - y3));
    }
    double RRT::normalized_obstacle_path_cost(Pose& p1, Pose& p2, int radius) {
        int x1 = tf_to_coord_horizontal(p1.x);
        int y1 = tf_to_coord_vertical(p1.y);
        int x2 = tf_to_coord_horizontal(p2.x);
        int y2 = tf_to_coord_vertical(p2.y);
        return obstacle_path_cost(x1, y1, x2, y2, radius, false) * hypot(x1 - x2, y1 - y2);
    }

    void RRT::adjust_point(Pose& current, Pose& prev, Pose& next, int radius) {
        int starting_coord_x = tf_to_coord_horizontal(current.x); 
        int best_x = starting_coord_x;
        int starting_coord_y = tf_to_coord_vertical(current.y);
        int best_y = starting_coord_y;
        int next_coord_x = tf_to_coord_horizontal(next.x);
        int next_coord_y = tf_to_coord_vertical(next.y);
        int prev_coord_x = tf_to_coord_horizontal(prev.x);
        int prev_coord_y = tf_to_coord_vertical(prev.y);
        double cost = normalized_obstacle_path_cost(starting_coord_x, starting_coord_y, prev_coord_x, prev_coord_y, next_coord_x, next_coord_y, radius);
        for (int i = -radius; i <= radius; i++) {
            for (int j = -radius; j <= radius; j++) {
                int x = starting_coord_x + i;
                int y =  starting_coord_y + j;
                if (!(i==0 && j==0) && !is_occupied(x, y) && !cross_obstacle_points(x, y, prev_coord_x, prev_coord_y) && !cross_obstacle_points(x, y, next_coord_x, next_coord_y)) {
                    double new_cost = normalized_obstacle_path_cost(x, y, next_coord_x, next_coord_y, prev_coord_x, prev_coord_y, radius);
                    if (new_cost < cost) {
                        best_x = x;
                        best_y = y;
                        cost = new_cost;
                    }
                }
            }
        }
        current.x = coord_to_tf_horizontal(best_x);
        current.y = coord_to_tf_vertical(best_y);
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

    Trajectory RRT::reverse_interpolate_trajectory(Trajectory& input, double radius) {
        Trajectory interpolated;
        interpolated.waypoints.push_back(input.waypoints[0]);
        
        double accumulated_dist = 0.0;
        // bool add = true;
        // for (int i = 0; i < input.waypoints.size() - 1; i++) {
        for (int i = 1; i < input.waypoints.size() - 1; i++) {
            double dx = input.waypoints[i + 1].pose.x - input.waypoints[i].pose.x;
            double dy = input.waypoints[i + 1].pose.y - input.waypoints[i].pose.y;
            double dist = sqrt(dx * dx + dy * dy);
            
            interpolated.waypoints.push_back(input.waypoints[i]);
            if (dist <= radius) {
                i++;
            }
            
            // accumulated_dist += dist;
            
            // if (accumulated_dist >= radius) {
            //     interpolated.waypoints.push_back(input.waypoints[i + 1]);
            //     accumulated_dist = 0.0;
            // }
        }
        
        if (interpolated.waypoints.back().pose.x != input.waypoints.back().pose.x ||
            interpolated.waypoints.back().pose.y != input.waypoints.back().pose.y) {
            interpolated.waypoints.push_back(input.waypoints.back());
        }
        
        return interpolated;
    }
    

    Trajectory RRT::apply_obstacle_cost_trajectory(Trajectory& input, double radius) {
        Trajectory optimized_coords;
        optimized_coords.waypoints.reserve(input.waypoints.size());
        Pose prev;
        if (!input.waypoints.empty()) {
            prev = input.waypoints.front().pose;
            optimized_coords.waypoints.push_back({input.waypoints.front().pose.x, input.waypoints.front().pose.y});
        }

        for (size_t i = 1; i < input.waypoints.size() - 1; i++) {
            Pose current = input.waypoints[i].pose;
            adjust_point(current, prev, input.waypoints[i + 1].pose, radius);
            prev = current;
            optimized_coords.waypoints.push_back({ current.x, current.y });
        }

        if (input.waypoints.size() > 1)
            optimized_coords.waypoints.push_back({input.waypoints.back().pose.x, input.waypoints.back().pose.y});

        return optimized_coords;
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

            Trajectory tf_trajectory;
            for (State state : res.waypoints) {
                Pose p = Coordinate({int(state.pose.x), int(state.pose.y)}).tf_pose(resolution, origin);
                tf_trajectory.waypoints.push_back({p.x, p.y});
            }

            Trajectory interpolated;
            interpolated = apply_obstacle_cost_trajectory(tf_trajectory, 5);
            interpolated = interpolate_trajectory(interpolated);

            for (int i = 0; i < 2; i ++)
                interpolated = round_trajectory(interpolated);

            for (int i = 0; i < 3; i ++)
                interpolated = reverse_interpolate_trajectory(interpolated, 0.75);
            // interpolated = reverse_interpolate_trajectory(interpolated, 0.5);
            // interpolated = reverse_interpolate_trajectory(interpolated, 1);
            // for (int i = 0; i < 2; i ++)
            //     interpolated = reverse_interpolate_trajectory(interpolated);
            // interpolated = reverse_interpolate_trajectory(interpolated, 0.3);
            
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
        return cross_obstacle_points(tf_to_coord_horizontal(x1), tf_to_coord_vertical(y1), tf_to_coord_horizontal(x2), tf_to_coord_vertical(y2));
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