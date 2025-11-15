#include <bits/stdc++.h>
using namespace std;

#include "cost_finder/cost_finder.h"

namespace cev_planner::cost_finder {

    // basic point structure
    struct Point {
        double x, y;

        Point(double x_val, double y_val) : x(x_val), y(y_val) {}

        bool operator==(const Point &other) const {
            return x == other.x && y == other.y;
        }

        double distance_to(const Point &other) const {
            return sqrt(pow(x - other.x, 2) + pow(y - other.y, 2));
        }

        bool operator<(const Point &other) const {
            if (x != other.x) return x < other.x;
            return y < other.y;
        }
    };

    // AABB collision from somewhere on the internet
    class AABB {

        public: 

        double x_min, y_min, x_max, y_max;

        AABB() : x_min(0), y_min(0), x_max(0), y_max(0) {}

        AABB(double x1, double y1, double x2, double y2)
            : x_min(x1), y_min(y1), x_max(x2), y_max(y2) {}

        bool contains(const Point &pt) const {
            return pt.x >= x_min && pt.x <= x_max && pt.y >= y_min && pt.y <= y_max;
        }

        bool intersects(const AABB &other) const {
            return !(x_max < other.x_min || x_min > other.x_max || y_max < other.y_min || y_min > other.y_max);
        }
    };

    // dsu from cp-algorithms with rank
    class DSU {

        public:

        vector<int> parent, rankOf;
        unordered_map<int, int> componentNum;
        unordered_map<int, AABB> componentBoundaries;


        void make_set(int v, const Point &pt) {
            parent[v] = v;
            rankOf[v] = 0;
            componentNum[v] = 1;
            componentBoundaries[v] = AABB(pt.x, pt.y, pt.x, pt.y);
        }

        int find_set(int v) {
            // defensive: ensure parent vector has been initialized and v is in range
            if (v < 0 || v >= (int)parent.size()) {
                return v;
            }

            if (v == parent[v]) {
                return v;
            }

            // if parent[v] is out of range for some reason, reset it
            if (parent[v] < 0 || parent[v] >= (int)parent.size()) {
                parent[v] = v;
                return v;
            }

            return parent[v] = find_set(parent[v]);
        }
        
        void union_sets(int a, int b) {
            a = find_set(a);
            b = find_set(b);

            // defensive: ensure indices are valid
            if (a < 0 || a >= (int)parent.size() || b < 0 || b >= (int)parent.size()) {
                return;
            }

            if (a != b) {
                if (rankOf[a] < rankOf[b])
                    swap(a, b);
                parent[b] = a;

                if (componentNum.count(a) && componentNum.count(b)) {
                    componentNum[a] += componentNum[b];
                    componentNum.erase(b);
                }

                if (componentBoundaries.count(a) && componentBoundaries.count(b)) {
                    auto &boundaryA = componentBoundaries[a];
                    auto &boundaryB = componentBoundaries[b];

                    boundaryA.x_min = min(boundaryA.x_min, boundaryB.x_min);
                    boundaryA.x_max = max(boundaryA.x_max, boundaryB.x_max);
                    boundaryA.y_min = min(boundaryA.y_min, boundaryB.y_min);
                    boundaryA.y_max = max(boundaryA.y_max, boundaryB.y_max);

                    componentBoundaries.erase(b);
                }

                if (rankOf[a] == rankOf[b])
                    rankOf[a]++;
            }
        }
    };

    // quadtree code inspired by: https://lisyarus.github.io/blog/posts/building-a-quadtree.html
    class QuadtreeNode {
    public:
        // given a boundary defined in the manner below (recursing 4 times sorta hardcoded)
        AABB boundary;
        static const int MAX_POINTS = 4;
        vector<Point> points;
        vector<QuadtreeNode *> children;

        // init
        QuadtreeNode(AABB boundary) : boundary(boundary) {
            children.resize(MAX_POINTS);
            for (int i = 0; i < MAX_POINTS; i++) {
                children[i] = nullptr;
            }
        }

        // 4 new quadtrees
        void subdivide() {
            double x_mid = (boundary.x_min + boundary.x_max) / 2;
            double y_mid = (boundary.y_min + boundary.y_max) / 2;

            // hardcoded for 4 (and they capture equal area)
            children[0] = new QuadtreeNode(AABB(boundary.x_min, boundary.y_min, x_mid, y_mid));
            children[1] = new QuadtreeNode(AABB(x_mid, boundary.y_min, boundary.x_max, y_mid));
            children[2] = new QuadtreeNode(AABB(boundary.x_min, y_mid, x_mid, boundary.y_max));
            children[3] = new QuadtreeNode(AABB(x_mid, y_mid, boundary.x_max, boundary.y_max));
        }

        bool insert(const Point &pt) {

            // made a mistake then
            if (!boundary.contains(pt)) {
                return false;
            }

            // if we got less than 4, then just add it
            if (points.size() < MAX_POINTS) {
                points.push_back(pt);
                return true;
            }

            // otherwise, subdivide
            if (children[0] == nullptr) {
                subdivide();
            }

            for (int i = 0; i < MAX_POINTS; ++i) {
                if (children[i]->insert(pt)) {
                    return true;
                }
            }

            // prayyy
            return false;
        }

        // get the points in a range
        void rangeQuery(const AABB &range, vector<Point> &result) {

            // not useful points
            if (!boundary.intersects(range)) {
                return;
            }

            // add points
            for (const auto &pt : points) {
                if (range.contains(pt)) {
                    result.push_back(pt);
                }
            }

            // if there are children (not a leaf node so we aren't done), recurse
            if (children[0] != nullptr) {
                for (int i = 0; i < MAX_POINTS; i++) {
                    children[i]->rangeQuery(range, result);
                }
            }
        }

        // idea: https://stackoverflow.com/questions/6698484/using-a-quadtree-to-get-all-points-within-a-bounding-circle
        void queryCircle(const Point &center, double radius, vector<Point> &result) {

            AABB queryRange(center.x - radius, center.y - radius, center.x + radius, center.y + radius);

            // first, find points in the square
            vector<Point> potentialPoints;
            rangeQuery(queryRange, potentialPoints);

            // check if each point is also inside the circle (we want those points)
            for (const Point &pt : potentialPoints) {
                double distanceSquared = pow(pt.x - center.x, 2) + pow(pt.y - center.y, 2);
                if (distanceSquared <= pow(radius, 2)) {
                    result.push_back(pt);
                }
            }
        }
    };

    // problem here that i'll fix in python
    class Quadtree {
    private:
        QuadtreeNode *root;

        // needed for dsu purposes
        vector<Point> allPoints;
        map<Point, int> pointIndex;

        DSU dsu;

    public:
        Quadtree(AABB boundary) {
            root = new QuadtreeNode(boundary);
        }

        // same stuff
        void insert(const Point &pt) {
            if (pointIndex.count(pt)) {
                return;
            }
            pointIndex[pt] = allPoints.size();
            allPoints.push_back(pt);
            root->insert(pt);
        }

        vector<Point> queryCircle(const Point &center, double radius) {
            vector<Point> result;
            root->queryCircle(center, radius, result);
            return result;
        }

        // dsu time
        void findConnectedComponents(double radius) {
            int n = allPoints.size();
            dsu.parent.resize(n);
            dsu.rankOf.resize(n);

            for (int i = 0; i < n; ++i) {
                dsu.make_set(i, allPoints[i]);
            }

            vector<bool> visited(n);

                // iterative DFS using a stack to avoid deep recursion
                for (int i = 0; i < n; ++i) {
                    if (visited[i]) continue;

                    std::stack<int> st;
                    st.push(i);
                    visited[i] = true;

                    while (!st.empty()) {
                        int cur = st.top();
                        st.pop();

                        // query neighbors within radius
                        vector<Point> neighbors = queryCircle(allPoints[cur], radius);
                        for (const Point &pt : neighbors) {
                            int nIndex = pointIndex[pt];
                            if (nIndex < 0 || nIndex >= n) continue;
                            if (!visited[nIndex]) {
                                dsu.union_sets(cur, nIndex);
                                visited[nIndex] = true;
                                st.push(nIndex);
                            }
                        }
                    }
                }

            // debug
            for (int i = 0; i < n; i++) {
                int root = dsu.find_set(i);
                // cout << "Point (" << allPoints[i].x << ", " << allPoints[i].y << ") is in component " << root << endl;

                AABB boundary = dsu.componentBoundaries[root];

                // debug
                // cout << "Component " << root << " Boundary: ";
                // cout << "X: [" << boundary.x_min << ", " << boundary.x_max << "], ";
                // cout << "Y: [" << boundary.y_min << ", " << boundary.y_max << "]" << endl;
                // cout << "Number of Points: " << dsu.componentNum[root] << endl;
                // cout << endl;
            }
        }

        void dfs(int index, vector<bool> &visited, double radius) {
            visited[index] = true;
            vector<Point> neighbors = queryCircle(allPoints[index], radius);

            for (auto point : neighbors) {
                int nIndex = pointIndex[point];
                if (!visited[nIndex]) {
                    dsu.union_sets(index, nIndex);
                    dfs(nIndex, visited, radius);
                }
            }
        }

        vector<Point> findClosestComponent(Point point, int k) {
            double l = 0, r = root->boundary.x_max;
            vector<Point> closestPoints;
            
            while (l < r) {
                double m = l + (r - l) / 2;
                vector<Point> pointArray = queryCircle(point, m);

                vector<Point> pts;
                for (const Point &pt : pointArray) {
                    int componentIndex = pointIndex[pt];
                    int componentParent = dsu.find_set(componentIndex);
                    int numPointsInComponent = dsu.componentNum[componentParent];

                    if (numPointsInComponent > k) {
                        pts.push_back(pt);
                    }
                }

                if (pts.empty()) {
                    l = m;
                } else {
                    r = m;
                }

                if (pts.size() == 1) {
                    break;
                }
            }

            vector<Point> closePoints = queryCircle(point, r);
            vector<pair<double, Point>> ans;

            for (const Point &pt : closePoints) {
                ans.push_back({point.distance_to(pt), pt});
            }

            sort(ans.begin(), ans.end(), [](const pair<double, Point> &a, const pair<double, Point> &b) {
                return a.first < b.first;
            });

            double bestDist = ans[0].first;

            for (const auto &pair : ans) {
                if (pair.first == bestDist) {
                    closestPoints.push_back(pair.second);
                } else {
                    break;
                }
            }

            return closestPoints;
        }

        unordered_map<int, double> queryHeuristic(const Point &queryPoint, double radius, int k) {
            //cout << "Hello900\n";
            unordered_map<int, double> componentDistances;
            //cout << "Hello901\n";
            double queryRadius = k * radius;
            vector<Point> pointsInRange = queryCircle(queryPoint, queryRadius);
            //cout << "Hello902\n";
            for (const Point &pt : pointsInRange) {
                //cout << "Hello903\n";
                int componentIndex = pointIndex[pt];
                //cout << componentIndex << " Hello903b\n";
                int componentParent = dsu.find_set(componentIndex);
                //cout << "Hello904\n";
                double distance = pt.distance_to(queryPoint);
                //cout << "Hello905\n";
                if (!componentDistances.count(componentParent) || componentDistances[componentParent] > distance) {
                    componentDistances[componentParent] = distance;
                }
                //cout << "Hello906\n";
            }

            for (const auto& [componentParent, boundary] : dsu.componentBoundaries) {
                //cout << "Hello907\n";
                if (!componentDistances.count(componentParent)) {

                    double distance = 0.0;

                    if (queryPoint.x < boundary.x_min && queryPoint.y < boundary.y_min) {
                        distance = sqrt(pow(boundary.x_min - queryPoint.x, 2) + pow(boundary.y_min - queryPoint.y, 2));
                    }
                    else if (queryPoint.x >= boundary.x_min && queryPoint.x <= boundary.x_max && queryPoint.y < boundary.y_min) {
                        distance = boundary.y_min - queryPoint.y;
                    }
                    else if (queryPoint.x > boundary.x_max && queryPoint.y < boundary.y_min) {
                        distance = sqrt(pow(boundary.x_max - queryPoint.x, 2) + pow(boundary.y_min - queryPoint.y, 2));
                    }
                    else if (queryPoint.x < boundary.x_min && queryPoint.y >= boundary.y_min && queryPoint.y <= boundary.y_max) {
                        distance = boundary.x_min - queryPoint.x;
                    }
                    else if (queryPoint.x > boundary.x_max && queryPoint.y >= boundary.y_min && queryPoint.y <= boundary.y_max) {
                        distance = queryPoint.x - boundary.x_max;
                    }
                    else if (queryPoint.x < boundary.x_min && queryPoint.y > boundary.y_max) {
                        distance = sqrt(pow(boundary.x_min - queryPoint.x, 2) + pow(boundary.y_max - queryPoint.y, 2));
                    }
                    else if (queryPoint.x >= boundary.x_min && queryPoint.x <= boundary.x_max && queryPoint.y > boundary.y_max) {
                        distance = queryPoint.y - boundary.y_max;
                    }
                    else if (queryPoint.x > boundary.x_max && queryPoint.y > boundary.y_max) {
                        distance = sqrt(pow(boundary.x_max - queryPoint.x, 2) + pow(boundary.y_max - queryPoint.y, 2));
                    }
                    else {
                        // incorrect, but i'll deal with it later
                        // interior point will be green or something I think
                        distance = 0.0;
                    }

                    componentDistances[componentParent] = distance;
                }
            }

            return componentDistances;
        }

    };

    CostFinder::CostFinder(double radius, int k) {
        this->radius = radius;
        this->k = k;
        this->quadtree = new Quadtree(AABB(0, 0, 1000, 1000));
    }
        
    void CostFinder::addPoint(const State& state) {
        quadtree->insert(Point(state.pose.x, state.pose.y));
    }

    double CostFinder::cost(const State& state) {
        auto distances = quadtree->queryHeuristic(Point(state.pose.x, state.pose.y), radius, k);
        double total_cost = 0;

        for (const auto& [component, dist] : distances) {
            total_cost += std::min(1.0 / (dist * dist), 10.0);
        }

        return total_cost;
    }

} // namespace cev_planner::cost_finder