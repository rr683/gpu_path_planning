#include "astar.h"
#include <queue>
#include <limits>
#include <unordered_map>
#include <functional>
#include <cmath>


// Disclaimer -- not quite done yet...
struct Node {
    int x,y;
    float f,g;
};

static inline float heuristic(GridIndex a, GridIndex b) {
    return std::abs(a.x - b.x) + std::abs(a.y - b.y);
}

Path astarPlan(const OccupancyGrid &grid, GridIndex start, GridIndex goal, float occ_threshold) {
    if (!grid.inBounds(start.x, start.y) || !grid.inBounds(goal.x, goal.y)) return {};
    auto key = [&](int x,int y){ return (x << 16) ^ y; };

    using PQItem = std::pair<float, int>; // f, packed key
    std::priority_queue<PQItem, std::vector<PQItem>, std::greater<PQItem>> open;
    std::unordered_map<int, float> gscore;
    std::unordered_map<int, int> came_from;

    int sk = key(start.x, start.y);
    gscore[sk] = 0.0f;
    open.emplace(heuristic(start, goal), sk);

    const int dx[4] = {1,-1,0,0};
    const int dy[4] = {0,0,1,-1};

    while (!open.empty()) {
        auto [f, packed] = open.top(); open.pop();
        int cx = packed >> 16;
        int cy = packed & 0xFFFF;
        if (cx == goal.x && cy == goal.y) break;
        int curk = packed;
        float cg = gscore[curk];

        for (int i=0;i<4;i++) {
            int nx = cx + dx[i];
            int ny = cy + dy[i];
            if (!grid.inBounds(nx, ny)) continue;
            if (grid.probAt(nx, ny) > occ_threshold) continue;
            int nk = key(nx, ny);
            float tentative_g = cg + 1.0f;
            if (!gscore.count(nk) || tentative_g < gscore[nk]) {
                gscore[nk] = tentative_g;
                float nf = tentative_g + heuristic({nx,ny}, goal);
                came_from[nk] = curk;
                open.emplace(nf, nk);
            }
        }
    }

    int gk = key(goal.x, goal.y);
    if (!gscore.count(gk)) return {};
    Path path;
    int cur = gk;
    while (cur != sk) {
        int x = cur >> 16;
        int y = cur & 0xFFFF;
        path.push_back({x,y});
        cur = came_from[cur];
    }
    path.push_back(start);
    std::reverse(path.begin(), path.end());
    return path;
}