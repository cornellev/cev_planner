#pragma once

#include "global_planner.h"
#include <queue>
#include <unordered_map>
using std::unordered_map;
using std::priority_queue;
using std::pair;
using std::vector;

namespace cev_planner::global_planner {
    /**
     * KD Tree for RRT Graph, handling RRT Nodes and associated index
     */
    class KDTree {
        public:
        KDTree(const unordered_map<int, Pose>& points={});

        void add_point(const pair<int, Pose>& point);
        vector<pair<int, Pose>> get_knn(const Pose& point, int k);
        pair<int, Pose> get_nearest(const Pose& point);

        void clear() {
            delete_tree(root);
            root = nullptr;
        }
    
        private:
            struct KDNode {
                std::pair<int, Pose> point;
                KDNode* left;
                KDNode* right;
    
                KDNode(const std::pair<int, Pose>& point={}) : point(point), left(nullptr), right(nullptr) {}
            };

            void delete_tree(KDNode* node) {
                if (node) {
                    delete_tree(node->left);
                    delete_tree(node->right);
                    delete node;
                }
            }

            KDNode* root;
            int dim;
            static constexpr double inf = std::numeric_limits<double>::infinity();
            struct heap_comp {
                bool operator()(const std::pair<double, std::pair<int, Pose>>& a, const std::pair<double, std::pair<int, Pose>>& b) const {
                    return a.first < b.first;
                }
            };
    
            double dist_sq(const Pose& a, const Pose& b) const {
                double dx = a.x - b.x;
                double dy = a.y - b.y;
                return (a.x == b.x && a.y==b.y) ? inf : (dx * dx + dy * dy);
            }
    
            KDNode* make(const vector<pair<int, Pose>>& points, int depth = 0);
            void add_point(KDNode* node, const pair<int, Pose>& point, int depth);
            void get_knn(KDNode* node, const Pose& point, int k, priority_queue<pair<double, pair<int, Pose>>, vector<pair<double, pair<int, Pose>>>, heap_comp>& heap, int depth);
        };
}  // namespace cev_planner::global_planner