#pragma once
#include "device_grid.h"
#include <vector>

// Structure to hold GPU planning resources (buffers)
// This avoids re-allocating memory on every frame
struct GPUPlanner {
    int width;
    int height;
    int size; // width * height

    // Device pointers
    float* d_gscore;
    int* d_parent;
    bool* d_closed;
    
    // Frontier queues (double buffering)
    int* d_frontier_current;
    int* d_frontier_next;
    int* d_frontier_count; // Pointer to a single int on device

    // Host helper
    int* h_frontier_count;

    void init(int w, int h);
    void cleanup();
};

// Main entry point for GPU A*
// Returns path as a vector of indices (y*w + x)
std::vector<int> planPathGPU(GPUPlanner& planner, const DeviceGrid& grid, int start_idx, int goal_idx);