#include "global_planning/rrt.h"
#include <eigen3/Eigen/Dense>

namespace cev_planner::global_planner {

    void RRT::dilate_grid(
        Grid& grid,
        const RRT::Node& start,
        const RRT::Node& goal,
        const std::vector<array<int, 2>>& structure,
        double safe_zone_radius,
        float occupation_threshold
    ) {
        int rows = grid.data.rows();
        int cols = grid.data.cols();
        
        Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic> result(rows, cols);
        result.setConstant(false);
        
        Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic> safe_zones(rows, cols);
        safe_zones.setConstant(false);
        
        auto node_to_grid = [&](const RRT::Node& node) -> array<int, 2> {
            return {node.x, node.y};
        };
        
        std::queue<array<int, 2>> q;
        
        for (const RRT::Node& node : {start, goal}) {
            auto [start_i, start_j] = node_to_grid(node);
            
            Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic> visited(rows, cols);
            visited.setConstant(false);
            
            Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> distances(rows, cols);
            distances.setConstant(std::numeric_limits<double>::infinity());
            
            q.push({start_i, start_j});
            visited(start_i, start_j) = true;
            distances(start_i, start_j) = 0;
            while (!q.empty()) {
                auto [i, j] = q.front();
                q.pop();
                
                if (distances(i, j) <= safe_zone_radius && 
                    (grid.data(i, j) >= 0 && grid.data(i, j) < occupation_threshold)) {
                    safe_zones(i, j) = true;
                    
                    for (const auto& p : structure) {
                        int ni = i + p[0];
                        int nj = j + p[1];
                        
                        if (ni >= 0 && ni < rows && nj >= 0 && nj < cols && !visited(ni, nj)) {
                            double new_dist = std::sqrt(
                                std::pow(ni - start_i, 2) + 
                                std::pow(nj - start_j, 2)
                            );
                            
                            if (new_dist <= safe_zone_radius) {
                                distances(ni, nj) = new_dist;
                                visited(ni, nj) = true;
                                q.push({ni, nj});
                            }
                        }
                    }
                }
            }
        }

        for (int i = 0; i < rows; ++i) {
            for (int j = 0; j < cols; ++j) {
                if (grid.data(i, j) < 0 || grid.data(i, j) >= occupation_threshold) {
                    for (const auto& p : structure) {
                        int ni = i + p[0];
                        int nj = j + p[1];
                        
                        if (ni >= 0 && ni < rows && nj >= 0 && nj < cols) {
                            if (!safe_zones(ni, nj)) {
                                result(ni, nj) = true;
                            }
                        }
                    }
                }
            }
        }

        this->obstacle_grid = result;
    }

    vector<array<int, 2>> RRT::create_circular_structure(double radius) {
        vector<array<int, 2>> structure;
        int iradius = std::ceil(radius);
        double radius_sq = radius * radius;
        
        for (int i = -iradius; i <= iradius; ++i) {
            for (int j = -iradius; j <= iradius; ++j) {
                if (i*i + j*j <= radius_sq) {
                    structure.push_back({i, j});
                }
            }
        }
        
        return structure;
    }

    bool RRT::path_to_goal() {
        if (goalFlag) {
            path.clear();
            int nodes_best = (goalstate.empty()) ? -1 : *std::min_element(goalstate.begin(), goalstate.end(),
                            [&](int a, int b) { return nodes.at(a).cost < nodes.at(b).cost; });
            
            int goal_best = (startstate.empty()) ? -1 : *std::min_element(startstate.begin(), startstate.end(),
                            [&](int a, int b) { return goal_nodes.at(a).cost < goal_nodes.at(b).cost; });
            if (nodes_best == goal_best == -1) return goalFlag;
            from_goal = goal_best != -1 && (nodes_best == -1 || goal_nodes.at(goal_best).cost < nodes.at(nodes_best).cost);
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
        for (pair<int, Pose>& pr : kdTree.get_knn(pointPose, min(10, max(5, (int)(nodes.size() / 10))))) {
            Coordinate c = {(int)round(pr.second.x), (int)round(pr.second.y)};
            if (pr.first != node && !(pr.second.x == point.x && pr.second.y == point.y) && !(foundGoal && goalstate.count(pr.first)) && !cross_obstacle_points(c, point))
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
        for (int neighbor : neighbors) {
            if (neighbor == bestNeighbor) continue;
            double neighborCost = get_cost(node, neighbor);
            if (neighborCost < nodes.at(neighbor).cost && !((nodes.at(neighbor).x == point.x && nodes.at(neighbor).y == point.y) || is_ancestor(neighbor, node))) {
                nodes.at(neighbor).set_cost(neighborCost);
                
                rewire_edge(nodes.at(neighbor).parent, neighbor, node);

                vector<unordered_set<int>*> stack = {&nodes.at(neighbor).children};
                while (!stack.empty()) {
                    std::unordered_set<int>* children = stack.back();
                    stack.pop_back();
                    for (int child : *children) {
                        nodes.at(child).set_cost(get_cost(nodes.at(child).parent, child));
                        stack.push_back(&nodes.at(child).children);
                    }
                }
            }
        }
        int cur_this = node;
        if (!foundGoal) {
            auto [nearest_other,  nearest_other_pose] = other_kdTree.get_nearest(pointPose);
            if (nearest_other && nodes.at(node).cost + other_nodes.at(nearest_other).cost < bestCost 
                && pointPose.distance_to(nearest_other_pose) < 20 && !cross_obstacle_points(point, other_nodes.at(nearest_other).coordinate)) {
                    cur_this = merge_trees(node, nearest_other, kdTree, nodes, other_nodes);
                    other_goalstate.insert(merge_trees(nearest_other, node, other_kdTree, other_nodes, nodes));
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
                if (abs(bestCost - old_best) > 3)
                    num_not_in_region = nodes.size();
                delete sampling_region;
                delete ellipse;
                sampling_region = new SamplingRegion{
                    d_base,
                    calculate_k(),
                    getPathCoords(),
                    mapw-1, maph-1
                };
                ellipse = new Ellipse{
                    startCoordPose,
                    goalCoordPose,
                    bestCost,
                    mapw, maph,
                    obstacle_grid
                };
            }
            else
                sampling_region->shorten_radius(calculate_k()); 
        } 
        if (!foundGoal && !from_goal && point.distance_to(target) <= 50) {
            this->bias();
        }
    }


    Trajectory RRT::getPathCoords(bool corrected_tf) {
        unordered_map<int, Node>* nodes = from_goal ? &goal_nodes : &this->nodes;
        Trajectory res;
        if (path.empty())
            return res;

        int cur_node = path[0];
        Pose p = corrected_tf ? nodes->at(cur_node).tf_pose(resolution, origin) : nodes->at(cur_node).as_is_pose();
        res.waypoints.push_back({p.x, p.y});
        
        int i = 0;
        while (i < path.size() - 1) {
            int cur = path[i];
            int next_node = path[i + 1];

            if (cross_obstacle(cur_node, next_node, nodes)) {
                p = corrected_tf ? nodes->at(cur).tf_pose(resolution, origin) : nodes->at(cur).as_is_pose();
                res.waypoints.push_back({p.x, p.y});
                cur_node = cur;
            } 
            i++;
        }

        p = corrected_tf ? nodes->at(path[path.size()-1]).tf_pose(resolution, origin) : nodes->at(path[path.size()-1]).as_is_pose();
        if (!(p.x == res.waypoints.back().pose.x && p.y == res.waypoints.back().pose.y))
            res.waypoints.push_back({p.x, p.y});
        return res;                     
    }


    RRT::Coordinate RRT::sample_envir() {
        Coordinate res;
        Random randWidth{0, static_cast<double>(mapw - 1)};
        Random randHeight{0, static_cast<double>(maph - 1)};
        while (true) {
            bool sampling = (rand.Uniform() < 0.87);
            if (goalFlag) {
                Pose p = sampling ? sampling_region->generate_random_point() : ellipse->generate_random_point();
                res.x = p.x;
                res.y = p.y;
            }
            else {
                res.x = randWidth.Int();
                res.y = randHeight.Int();
            }
            if (!is_occupied(res)) break;
        }
        return res;
    }

    void RRT::filter_nodes(unordered_map<int, RRT::Node>& nodes, unordered_set<int>& goalstate) {
        for (auto it = nodes.begin(); it != nodes.end(); ) {
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

    int RRT::merge_trees(int node, int cur_other, KDTree& kdTree, unordered_map<int, Node>& nodes, unordered_map<int, Node>& other_nodes) {
        int cur_this = get_cur_index(&nodes==&this->goal_nodes);
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
            if (cur_other == 0)  break;
            cur_other = other_node.parent;
            i = 1;
        }
        return cur_this;
    }

    bool RRT::cross_obstacle_points(Coordinate& startPoint, Coordinate& endPoint) {
        int x1 = startPoint.x;
        int y1 = startPoint.y;
        int x2 = endPoint.x;
        int y2 = endPoint.y;

        int dx = abs(x2 - x1);
        int dy = abs(y2 - y1);
        int sx = (x1 < x2) * 2 -1;
        int sy = (y1 < y2) * 2 -1;
        int err = dx - dy;

        while (true) {
            if (is_occupied(x1, y1)) return true;
            if (x1 == x2 && y1 == y2) break;
            int e2 = err * 2;
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

    bool RRT::cross_obstacle(int startNode, int endNode, unordered_map<int, RRT::Node>* nodes) {
        nodes = nodes ?: cur_tree;
        return cross_obstacle_points(nodes->at(startNode).coordinate, nodes->at(endNode).coordinate);
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