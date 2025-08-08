#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <queue>
#include <algorithm>
#include <iomanip>
#include <unordered_map>
#include <string>
#include <chrono>

using namespace std;
using namespace std::chrono;

struct Point { int id; double x, y; };
struct KDPoint { int idx; double x, y; };
struct KDNode {
    KDPoint p; KDNode* l; KDNode* r;
    KDNode(const KDPoint& pt) :p(pt), l(nullptr), r(nullptr) {}
};
struct Edge { int to; double t, d; };

struct KDCompare {
    int axis;
    KDCompare(int a) :axis(a) {}
    bool operator()(const KDPoint& a, const KDPoint& b)const {
        return axis == 0 ? a.x < b.x : a.y < b.y;
    }
};

KDNode* buildKD(vector<KDPoint>& pts, int l, int r, int depth) {
    if (l >= r) return nullptr;
    int axis = depth & 1, m = (l + r) >> 1;
    nth_element(pts.begin() + l, pts.begin() + m, pts.begin() + r, KDCompare(axis));
    KDNode* node = new KDNode(pts[m]);
    node->l = buildKD(pts, l, m, depth + 1);
    node->r = buildKD(pts, m + 1, r, depth + 1);
    return node;
}

// r2 = R*R, r = R
void queryKD(KDNode* node, double qx, double qy, double r2, double r, int depth, vector<int>& out) {
    if (!node) return;
    double dx = node->p.x - qx, dy = node->p.y - qy;
    if (dx * dx + dy * dy <= r2) out.push_back(node->p.idx);
    int axis = depth & 1;
    double delta = (axis == 0 ? qx - node->p.x : qy - node->p.y);
    if (delta <= r)   queryKD(node->l, qx, qy, r2, r, depth + 1, out);
    if (delta >= -r)  queryKD(node->r, qx, qy, r2, r, depth + 1, out);
}

int main() {
    
    ios::sync_with_stdio(false);
    cin.tie(nullptr);
    
    string mapPath, queryPath, outPath;
    cout << "MAP file path: ";     getline(cin, mapPath);
    cout << "QUERIES file path: "; getline(cin, queryPath);
    cout << "OUTPUT file path: ";  getline(cin, outPath);

    auto execTimeStart = high_resolution_clock::now();
    ifstream fin(mapPath);
    if (!fin) { cerr << "Cannot open " << mapPath << "\n"; return 1; }
    int N; fin >> N;
    vector<Point> pts(N);
    unordered_map<int, int> id2idx; id2idx.reserve(N * 2);
    for (int i = 0; i < N; i++) {
        fin >> pts[i].id >> pts[i].x >> pts[i].y;
        id2idx[pts[i].id] = i;
    }
    int M; fin >> M;
    vector<vector<Edge>> adj(N);
    for (int i = 0, u, v; i < M; i++) {
        double len, spd; fin >> u >> v >> len >> spd;
        int ui = id2idx[u], vi = id2idx[v];
        double tmin = (len / spd) * 60.0;
        adj[ui].push_back({ vi,tmin,len });
        adj[vi].push_back({ ui,tmin,len });
    }
    fin.close();

    // build KD-tree
    vector<KDPoint> kdpts(N);
    for (int i = 0; i < N; i++) kdpts[i] = { i,pts[i].x,pts[i].y };
    KDNode* kdroot = buildKD(kdpts, 0, N, 0);

    ifstream finQ(queryPath);
    ofstream fout(outPath);
    if (!finQ || !fout) { cerr << "Cannot open query/output\n"; return 1; }
    int Q; finQ >> Q;

    // working arrays
    vector<int> starts, ends;
    starts.reserve(512); ends.reserve(512);

    vector<double> walkStart(N), walkEnd(N), dist(N), pedge(N);
    vector<int> parent(N);
    vector<char> isTarget(N);         // faster than vector<bool>
    const double walkInv = 60.0 / 5.0, INF = 1e18;

    vector<int> path; path.reserve(N); // reuse to avoid realloc

    using P = pair<double, int>;

    auto t0 = high_resolution_clock::now();
    for (int qi = 0; qi < Q; qi++) {
        double sx, sy, dx_, dy_, Rm;
        finQ >> sx >> sy >> dx_ >> dy_ >> Rm;
        double R = Rm / 1000.0, R2 = R * R;

        starts.clear(); ends.clear();
        queryKD(kdroot, sx, sy, R2, R, 0, starts);
        queryKD(kdroot, dx_, dy_, R2, R, 0, ends);

        // compute walking distances
        for (int u : starts) {
            double dx = pts[u].x - sx, dy = pts[u].y - sy;
            walkStart[u] = sqrt(dx * dx + dy * dy) * walkInv;
        }
        for (int v : ends) {
            double dx = pts[v].x - dx_, dy = pts[v].y - dy_;
            walkEnd[v] = sqrt(dx * dx + dy * dy) * walkInv;
        }

        // reset Dijkstra arrays
        fill(dist.begin(), dist.end(), INF);
        fill(parent.begin(), parent.end(), -1);
        fill(pedge.begin(), pedge.end(), 0.0);
        fill(isTarget.begin(), isTarget.end(), 0);

        // build fresh pq
        priority_queue<P, vector<P>, greater<P>> pq;
        for (int u : starts) {
            dist[u] = walkStart[u];
            pq.push(P(dist[u], u));
        }
        for (int v : ends) isTarget[v] = 1;

        double best = INF;
        int bestEnd = -1;
        while (!pq.empty()) {
            P pr = pq.top(); pq.pop();
            double cd = pr.first;
            int u = pr.second;
            if (cd > dist[u] || cd > best) continue;

            if (isTarget[u]) {
                double cand = cd + walkEnd[u];
                if (cand < best) {
                    best = cand;
                    bestEnd = u;
                }
            }
            for (auto& e : adj[u]) {
                double nd = cd + e.t;
                if (nd < dist[e.to]) {
                    dist[e.to] = nd;
                    parent[e.to] = u;
                    pedge[e.to] = e.d;
                    pq.push(P(nd, e.to));
                }
            }
        }

        // reconstruct path
        path.clear();
        for (int cur = bestEnd; cur != -1; cur = parent[cur])
            path.push_back(cur);
        reverse(path.begin(), path.end());

        // output IDs
        for (size_t i = 0; i < path.size(); i++) {
            fout << pts[path[i]].id
                << (i + 1 < path.size() ? ' ' : '\n');
        }

        // compute kms
        double walkKm = 0, vehKm = 0;
        if (!path.empty()) {
            double dx = pts[path.front()].x - sx,
                dy = pts[path.front()].y - sy;
            walkKm += sqrt(dx * dx + dy * dy);
            dx = pts[path.back()].x - dx_;
            dy = pts[path.back()].y - dy_;
            walkKm += sqrt(dx * dx + dy * dy);
            for (int i = 1; i < (int)path.size(); i++) {
                vehKm += pedge[path[i]];
            }
        }
        double totKm = walkKm + vehKm;

        fout << fixed << setprecision(2)
            << best << " mins\n"
            << totKm << " km\n"
            << walkKm << " km\n"
            << vehKm << " km\n\n";
    }
    auto t1 = high_resolution_clock::now();
    double ms = duration_cast<chrono::milliseconds>(t1 - t0).count();
    auto execTimeEnd = high_resolution_clock::now();
    double execTimeDiff = duration_cast<chrono::milliseconds>(execTimeEnd - execTimeStart).count();

    
    fout << fixed << setprecision(0)
         << ms << " ms\n\n"
         << execTimeDiff << " ms";

    fout.close();

    return 0;
}