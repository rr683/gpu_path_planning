#include "occupancy_grid.h"
#include "raycast_cpu.h"
#include "astar.h"
#include <chrono>
#include <iostream>
#include <vector>
#include <cmath>

// Standard way to get M_PI on all platforms if not already defined
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

int main() {
    // STRESS TEST CONFIGURATION
    int W = 1024, H = 1024;
    float res = 0.05f;
    OccupancyGrid grid(W, H, res, -25.0f, -25.0f); // Centered roughly

    // Obstacle
    for (int gx = W/2 - 10; gx < W/2 + 10; ++gx) {
        for (int gy = H/4; gy < 3*H/4; ++gy) {
            grid.setLogOddsAt(gx, gy, 8.0f);
        }
    }

    float rx = 0.0f, ry = 0.0f;
    GridIndex start = grid.worldToGrid(rx, ry);
    GridIndex goal = grid.worldToGrid(10.0f, 0.0f);

    // High-density scan
    int beams = 10000; 
    float max_range = 20.0f; // Longer range for larger grid
    float free_delta = -0.4f;
    float occ_delta = 0.85f;

    std::cout << "=== CPU Baseline (Stress Test) ===\n";
    std::cout << "Grid: " << W << "x" << H << "\n";
    std::cout << "Beams: " << beams << "\n";

    auto t0 = std::chrono::high_resolution_clock::now();
    for (int b=0;b<beams;++b) {
        float angle = (2.0f * M_PI) * (b / static_cast<float>(beams));
        raycastAndIntegrate(grid, rx, ry, angle, max_range, free_delta, occ_delta);
    }
    auto t1 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> ray_time = t1 - t0;
    std::cout << "CPU Raycasting Time: " << ray_time.count() * 1000.0 << " ms\n";

    auto t2 = std::chrono::high_resolution_clock::now();
    auto path = astarPlan(grid, start, goal);
    auto t3 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> plan_time = t3 - t2;
    
    if (path.empty()) std::cout << "No path found\n";
    else std::cout << "Path found (" << path.size() << " steps). Planning Time: " << plan_time.count() * 1000.0 << " ms\n";

    return 0;
}