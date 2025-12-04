#include "cuda/device_grid.h"
#include "cuda/raycast_gpu.h"
#include "cuda/planner_gpu.h"
#include "src/occupancy_grid.h"
#include "src/astar.h"
#include <iostream>
#include <chrono>
#include <vector>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

int main() {
    // STRESS TEST CONFIGURATION
    int W = 1024, H = 1024;
    float res = 0.05f;
    OccupancyGrid grid(W, H, res, -25.0f, -25.0f);
    
    for (int gx = W/2 - 10; gx < W/2 + 10; ++gx) {
        for (int gy = H/4; gy < 3*H/4; ++gy) {
            grid.setLogOddsAt(gx, gy, 8.0f);
        }
    }
    
    float rx = 0.0f, ry = 0.0f;
    int num_beams = 10000;
    float max_range = 20.0f;
    float free_delta = -0.4f;
    float occ_delta = 0.85f;
    
    std::vector<float> h_angles(num_beams);
    for (int i = 0; i < num_beams; ++i) {
        h_angles[i] = 2.0f * M_PI * i / static_cast<float>(num_beams);
    }
    
    DeviceGrid d_grid = allocateDeviceGrid(grid);
    copyGridToDevice(d_grid, grid);
    
    std::cout << "=== GPU Pipeline (Stress Test) ===" << std::endl;
    std::cout << "Grid: " << W << "x" << H << ", Resolution: " << res << " m/cell" << std::endl;
    std::cout << "Beams: " << num_beams << ", Max range: " << max_range << " m" << std::endl;
    
    // 1. Raycasting Warmup
    raycastGPU(d_grid, rx, ry, h_angles.data(), num_beams, max_range, free_delta, occ_delta);
    
    // 1. Raycasting Timing
    auto t0 = std::chrono::high_resolution_clock::now();
    raycastGPU(d_grid, rx, ry, h_angles.data(), num_beams, max_range, free_delta, occ_delta);
    auto t1 = std::chrono::high_resolution_clock::now();
    
    std::chrono::duration<double> gpu_ray_time = t1 - t0;
    std::cout << "GPU Raycasting Time: " << gpu_ray_time.count() * 1000.0 << " ms" << std::endl;
    
    // 2. Path Planning Setup
    GridIndex start = grid.worldToGrid(rx, ry);
    GridIndex goal = grid.worldToGrid(10.0f, 0.0f);
    int start_idx = d_grid.idx(start.x, start.y);
    int goal_idx = d_grid.idx(goal.x, goal.y);
    
    GPUPlanner planner;
    planner.init(W, H);
    
    // 2. Path Planning Warmup
    planPathGPU(planner, d_grid, start_idx, goal_idx);
    
    // Reset planner state (implicitly handled by initBuffersKernel inside planPathGPU, 
    // but we need to make sure we don't reuse dirty buffers if logic assumes clean state.
    // Our planPathGPU calls initBuffersKernel first thing, so it's safe to call again.)
    
    // 2. Path Planning Timing
    auto t2 = std::chrono::high_resolution_clock::now();
    std::vector<int> path_indices = planPathGPU(planner, d_grid, start_idx, goal_idx);
    auto t3 = std::chrono::high_resolution_clock::now();
    
    std::chrono::duration<double> gpu_plan_time = t3 - t2;
    
    if (path_indices.empty()) {
        std::cout << "No path found" << std::endl;
    } else {
        std::cout << "Path found, length: " << path_indices.size() << " cells" << std::endl;
        std::cout << "GPU Planning Time: " << gpu_plan_time.count() * 1000.0 << " ms" << std::endl;
    }
    
    std::cout << "Total GPU Pipeline Time: " << (gpu_ray_time.count() + gpu_plan_time.count()) * 1000.0 << " ms" << std::endl;

    planner.cleanup();
    freeDeviceGrid(d_grid);
    
    return 0;
}