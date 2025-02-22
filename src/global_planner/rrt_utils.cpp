#include "global_planning/kd_tree.h"
#include "global_planning/rrt_regions.h"
#include <algorithm>
#include <random>

using std::vector;
using std::pair;

namespace cev_planner::global_planner {
    KDTree::KDTree(const unordered_map<int, Pose>& points) : dim(2) {
        vector<pair<int, Pose>> pointList;
        for (const auto& pair : points) {
            pointList.push_back({pair.first, pair.second});
        }
        root = make(pointList);
    }

    KDTree::KDNode* KDTree::make(const vector<pair<int, Pose>>& points, int depth) {
        if (points.empty()) {
            return nullptr;
        }

        int axis = depth % dim;
        auto sortedPoints = points;
        std::sort(sortedPoints.begin(), sortedPoints.end(),
                  [axis](const pair<int, Pose>& a, const pair<int, Pose>& b) {
                      return (axis == 0 ? a.second.x : a.second.y) < (axis == 0 ? b.second.x : b.second.y);
                  });

        size_t median = sortedPoints.size() / 2;
        KDNode* node = new KDNode(sortedPoints[median]);

        node->left = make(vector<pair<int, Pose>>(sortedPoints.begin(), sortedPoints.begin() + median), depth + 1);
        node->right = make(vector<pair<int, Pose>>(sortedPoints.begin() + median + 1, sortedPoints.end()), depth + 1);

        return node;
    }

    void KDTree::add_point(const pair<int, Pose>& point) {
        if (!root) {
            root = new KDNode(point);
        } else {
            add_point(root, point, 0);
        }
    }

    void KDTree::add_point(KDNode* node, const pair<int, Pose>& point, int depth) {
        int axis = depth % dim;
        if ((axis == 0 ? point.second.x : point.second.y) < (axis == 0 ? node->point.second.x : node->point.second.y)) {
            if (node->left) {
                add_point(node->left, point, depth + 1);
            } else {
                node->left = new KDNode(point);
            }
        } else {
            if (node->right) {
                add_point(node->right, point, depth + 1);
            } else {
                node->right = new KDNode(point);
            }
        }
    }

    vector<pair<int, Pose>> KDTree::get_knn(const Pose& point, int k) {
        priority_queue<pair<double, pair<int, Pose>>, vector<pair<double, pair<int, Pose>>>, heap_comp> heap;
        get_knn(root, point, k, heap, 0);

        vector<pair<int, Pose>> result;
        while (!heap.empty()) {
            result.push_back(heap.top().second);
            heap.pop();
        }
        std::reverse(result.begin(), result.end());
        return result;
    }

    void KDTree::get_knn(KDNode* node, const Pose& point, int k,
        priority_queue<pair<double, pair<int, Pose>>, vector<pair<double, pair<int, Pose>>>, heap_comp>& heap, int depth) {
        if (!node) {
            return;
        }

        double distance = dist_sq(point, node->point.second);
        if (heap.size() < k) {
            heap.push({distance, node->point});
        } else if (distance < heap.top().first) {
            heap.pop();
            heap.push({distance, node->point});
        }

        int axis = depth % dim;
        double dx = (axis == 0 ? node->point.second.x : node->point.second.y) - (axis == 0 ? point.x : point.y);
        KDNode* near = dx > 0 ? node->left : node->right;
        KDNode* far = dx > 0 ? node->right : node->left;

        get_knn(near, point, k, heap, depth + 1);

        if (heap.size() < k || dx * dx < heap.top().first) {
            get_knn(far, point, k, heap, depth + 1);
        }
    }

    pair<int, Pose> KDTree::get_nearest(const Pose& point) {
        auto knn = get_knn(point, 1);
        return knn.empty() ? std::make_pair(-1, Pose()) : knn[0];
    }

    array<Vector2d, 4> SamplingRegion::generate_quad(int i) {
        array<Vector2d, 4> quad = {cache[i][0], cache[i][1], cache[i+1][0], cache[i+1][1]};
        array<double, 3> eq = line_from_nodes(path[i], path[i+1]);
        if (segment_intersects_infinite_line(quad[0], quad[3], eq)) {
            if (!segment_intersects_infinite_line(quad[0], quad[2], eq)) {
                quad[2],quad[3] = quad[3],quad[2];
            }
            else if (!segment_intersects_infinite_line(quad[0], quad[1], eq)) {
                quad[1],quad[3]=quad[3],quad[1];
            }
        }
        return quad;
    }
            
    void SamplingRegion::generate_expansion_region() {
        if (cache.empty()) {
            cache.push_back(extension_end(path[0], path[1]));
            for (int i = 1; i < path.size() - 1; i++) {
                pair<Vector2d, double> dir_vec = direction_vector(path[i-1], path[i], path[i+1]);
                cache.push_back(extension(dir_vec, path[i]));
            }
            cache.push_back(extension_end(path[path.size() - 1], path[path.size() - 2]));
        }

        double cumulative = 0;
        for (int i = 0; i < path.size() - 1; i++) {
            array<Vector2d, 4> quad = generate_quad(i);
            double area1 = triangle_area(quad[0], quad[1], quad[2]);
            double area2 = triangle_area(quad[0], quad[2], quad[3]);
            double total = area1 + area2;
            cumulative += total;
            if (total > 0) {
                expansion_region.push_back({
                    quad,
                    cumulative,
                    area1 / max(total, 0.001)
                });
            }
        }
    }

    pair<Vector2d, double> SamplingRegion::direction_vector(Vector2d& v1, Vector2d& v2, Vector2d& v3) {
        Vector2d vec1 = (v1 - v2).normalized();
        Vector2d vec2 = (v3 - v2).normalized();
        Vector2d e_i = vec1 + vec2;
        if (e_i.norm() < 1e-10) {
            e_i = {-vec1[1], vec1[0]};
        }
        e_i.normalize();
        return {e_i, sin(acos(max(min(vec1.dot(vec2), 1.0), -1.0)) / 2)};
    }

    array<Vector2d, 2> SamplingRegion::extension(pair<Vector2d, double>& vec_and_dir, Vector2d& v) {
        Vector2d e_i = vec_and_dir.first;
        e_i *= d / max(vec_and_dir.second, 0.1);

        Vector2d v_prime = v + e_i;
        Vector2d v_double_prime = v - e_i;
        clip(v_prime);
        clip(v_double_prime);
        return {v_prime, v_double_prime};
    }

    array<Vector2d, 2> SamplingRegion::extension_end(Vector2d& end, Vector2d& other) {
        Vector2d v_to_other = (other - end).normalized();
        Vector2d perpendicular = {-v_to_other[1], v_to_other[0]};
        Vector2d v_prime_dir = -((v_to_other + perpendicular).normalized());
        Vector2d v_double_prime_dir = (perpendicular - v_to_other).normalized();
        double z = d / 0.851;
        Vector2d v_prime = v_prime_dir * z + end;
        Vector2d v_double_prime = v_double_prime_dir * z + end;
        clip(v_prime);
        clip(v_double_prime);
        return {v_prime, v_double_prime};
    }
            
    Pose SamplingRegion::random_point_in_triangle(Vector2d& a, Vector2d& b, Vector2d& c) {
        double r1 = rand.Uniform();
        double r2 = rand.Uniform();
        if (r1 + r2 > 1) {
            r1 = 1 - r1;
            r2 = 1 - r2;
        }
        double x = a[0] + r1 * (b[0] - a[0]) + r2 * (c[0] - a[0]);
        double y = a[1] + r1 * (b[1] - a[1]) + r2 * (c[1] - a[1]);
        return {round(x), round(y)};
    }

    Pose SamplingRegion::generate_random_point() {
        double rand_val = rand.Double(0, expansion_region.back().cumulative_area);
        int left = 0;
        int right = expansion_region.size() - 1;
        while (left < right) {
            int mid = (left + right) / 2;
            if (expansion_region[mid].cumulative_area < rand_val) {
                left = mid + 1;
            } else {
                right = mid;
            }
        }
        int selected_region = left;
        array<Vector2d, 4> quad = expansion_region[selected_region].quad;
        if (rand.Uniform() < expansion_region[selected_region].split_ratio) {
            return random_point_in_triangle(quad[0], quad[1], quad[2]);
        } else {
            return random_point_in_triangle(quad[0], quad[2], quad[3]);
        }
    }

    Pose Ellipse::generate_random_point() {
        while (true) {
            double r = sqrt(rand.Uniform());
            double theta = 2 * M_PI * rand.Uniform();

            double x = r * a * cos(theta);
            double y = r * b * sin(theta);

            double x_rotated = x * cos(rotation) - y * sin(rotation);
            double y_rotated = x * sin(rotation) + y * cos(rotation);

            double final_x = round(x_rotated + centerX);
            double final_y = round(y_rotated + centerY);

            if ((0 <= final_x && final_x < maxX) && (0 <= final_y && final_y < maxY)) {
                return {abs(final_x), abs(final_y)};
            }
        }
    }

}  // namespace cev_planner::global_planner