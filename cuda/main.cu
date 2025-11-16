#include "cuda/device_grid.h"
#include "cuda/raycast_gpu.h"
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
    int W = 512, H = 512;
    float res = 0.05f;
    OccupancyGrid grid(W, H, res, -12.8f, -12.8f);
    
    for (int gx = 240; gx < 244; ++gx) {
        for (int gy = 150; gy < 350; ++gy) {
            grid.setLogOddsAt(gx, gy, 8.0f);
        }
    }
    
    float rx = 0.0f, ry = 0.0f;
    int num_beams = 720;
    float max_range = 10.0f;
    float free_delta = -0.4f;
    float occ_delta = 0.85f;
    
    std::vector<float> h_angles(num_beams);
    for (int i = 0; i < num_beams; ++i) {
        h_angles[i] = 2.0f * M_PI * i / static_cast<float>(num_beams);
    }
    
    DeviceGrid d_grid = allocateDeviceGrid(grid);
    copyGridToDevice(d_grid, grid);
    
    std::cout << "=== GPU Raycasting ===" << std::endl;
    std::cout << "Grid: " << W << "x" << H << ", Resolution: " << res << " m/cell" << std::endl;
    std::cout << "Beams: " << num_beams << ", Max range: " << max_range << " m" << std::endl;
    
    raycastGPU(d_grid, rx, ry, h_angles.data(), num_beams, max_range, free_delta, occ_delta);
    
    auto t0 = std::chrono::high_resolution_clock::now();
    raycastGPU(d_grid, rx, ry, h_angles.data(), num_beams, max_range, free_delta, occ_delta);
    auto t1 = std::chrono::high_resolution_clock::now();
    
    std::chrono::duration<double> gpu_time = t1 - t0;
    std::cout << "GPU raycasting time: " << gpu_time.count() * 1000.0 << " ms" << std::endl;
    
    copyGridToHost(grid, d_grid);
    
    GridIndex start = grid.worldToGrid(rx, ry);
    GridIndex goal = grid.worldToGrid(8.0f, 0.0f);
    
    auto t2 = std::chrono::high_resolution_clock::now();
    Path path = astarPlan(grid, start, goal);
    auto t3 = std::chrono::high_resolution_clock::now();
    
    std::chrono::duration<double> plan_time = t3 - t2;
    
    if (path.empty()) {
        std::cout << "No path found" << std::endl;
    } else {
        std::cout << "Path found, length: " << path.size() << " cells" << std::endl;
        std::cout << "CPU A* planning time: " << plan_time.count() * 1000.0 << " ms" << std::endl;
    }
    
    freeDeviceGrid(d_grid);
    
    return 0;
}