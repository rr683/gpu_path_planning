#include "occupancy_grid.h"
#include "raycast_cpu.h"
#include "astar.h"
#include <chrono>
#include <iostream>
#include <vector>
#include <cmath>
#include <corecrt_math_defines.h>

int main() {
    int W = 200, H = 200;
    float res = 0.05f;
    OccupancyGrid grid(W, H, res, -5.0f, -5.0f);

    // create a simple obstacle: vertical wall at x=0 from y=-1 to y=1
    for (int gx = 80; gx < 82; ++gx) {
        for (int gy = 50; gy < 150; ++gy) {
            grid.setLogOddsAt(gx, gy, 8.0f);
        }
    }

    // robot pose
    float rx = 0.0f, ry = 0.0f;
    GridIndex start = grid.worldToGrid(rx, ry);
    GridIndex goal = grid.worldToGrid(4.0f, 0.0f);

    // simulate a full 360-degree LiDAR with many beams
    int beams = 720;
    float max_range = 8.0f;
    float free_delta = -0.4f;
    float occ_delta = 0.85f;

    auto t0 = std::chrono::high_resolution_clock::now();
    for (int b=0;b<beams;++b) {
        float angle = (2.0f * M_PI) * (b / static_cast<float>(beams));
        raycastAndIntegrate(grid, rx, ry, angle, max_range, free_delta, occ_delta);
    }
    auto t1 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> ray_time = t1 - t0;
    std::cout << "CPU raycasting time for " << beams << " beams: " << ray_time.count() << " s\n";

    auto t2 = std::chrono::high_resolution_clock::now();
    auto path = astarPlan(grid, start, goal);
    auto t3 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> plan_time = t3 - t2;
    if (path.empty()) {
        std::cout << "No path found\n";
    } else {
        std::cout << "Path found length: " << path.size() << ", planning time: " << plan_time.count() << " s\n";
    }

    return 0;
}