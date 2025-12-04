#include "planner_gpu.h"
#include "cuda_utils.h"
#include <cuda_runtime.h>
#include <iostream>
#include <vector>
#include <algorithm>

// Initialize resources
void GPUPlanner::init(int w, int h) {
    width = w;
    height = h;
    size = w * h;

    CUDA_CHECK(cudaMalloc(&d_gscore, size * sizeof(float)));
    CUDA_CHECK(cudaMalloc(&d_parent, size * sizeof(int)));
    CUDA_CHECK(cudaMalloc(&d_closed, size * sizeof(bool)));
    
    // Max possible frontier size is the grid size
    CUDA_CHECK(cudaMalloc(&d_frontier_current, size * sizeof(int)));
    CUDA_CHECK(cudaMalloc(&d_frontier_next, size * sizeof(int)));
    CUDA_CHECK(cudaMalloc(&d_frontier_count, sizeof(int)));
    
    CUDA_CHECK(cudaMallocHost(&h_frontier_count, sizeof(int)));
}

void GPUPlanner::cleanup() {
    CUDA_CHECK(cudaFree(d_gscore));
    CUDA_CHECK(cudaFree(d_parent));
    CUDA_CHECK(cudaFree(d_closed));
    CUDA_CHECK(cudaFree(d_frontier_current));
    CUDA_CHECK(cudaFree(d_frontier_next));
    CUDA_CHECK(cudaFree(d_frontier_count));
    CUDA_CHECK(cudaFreeHost(h_frontier_count));
}

// Kernel to initialize buffers
__global__ void initBuffersKernel(float* gscore, int* parent, bool* closed, int size, int start_idx) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= size) return;

    gscore[idx] = 1e10f; // Infinity
    parent[idx] = -1;
    closed[idx] = false;

    if (idx == start_idx) {
        gscore[idx] = 0.0f;
    }
}

// OPTIMIZED Kernel
__global__ void expandKernel(
    const int* __restrict__ current_frontier, // Hint: Read-only
    int current_count,
    int* __restrict__ next_frontier,
    int* __restrict__ next_count,
    float* __restrict__ gscore, // We read and write, but mostly read
    int* __restrict__ parent,
    DeviceGrid grid,
    int goal_idx
) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= current_count) return;

    // Load current node using read-only cache (LDG)
    int current_node = __ldg(&current_frontier[idx]);
    
    int cx = current_node % grid.width;
    int cy = current_node / grid.width;
    
    // Load g-score via cache
    float current_g = gscore[current_node];

    // 4-neighbor expansion
    int dx[4] = {1, -1, 0, 0};
    int dy[4] = {0, 0, 1, -1};

    for (int i = 0; i < 4; ++i) {
        int nx = cx + dx[i];
        int ny = cy + dy[i];

        if (grid.inBounds(nx, ny)) {
            int neighbor_idx = ny * grid.width + nx;
            
            // OPTIMIZATION 1: Use __ldg for grid read (Read-Only Cache)
            float logodds = __ldg(&grid.logodds[neighbor_idx]);
            float prob = 1.0f - 1.0f / (1.0f + expf(logodds));
            
            if (prob < 0.65f) {
                float new_g = current_g + 1.0f;
                
                // OPTIMIZATION 2: Check before Atomic
                // Only attempt atomic if we have a chance of improving
                // This reduces memory contention significantly
                if (new_g < gscore[neighbor_idx]) {
                    
                    int* address_as_int = (int*)&gscore[neighbor_idx];
                    int old = *address_as_int;
                    int assumed;
                    
                    do {
                        assumed = old;
                        float assumed_float = __int_as_float(assumed);
                        if (new_g >= assumed_float) break; 
                        old = atomicCAS(address_as_int, assumed, __float_as_int(new_g));
                    } while (assumed != old);

                    if (__int_as_float(old) > new_g) {
                        // We won the race!
                        parent[neighbor_idx] = current_node;
                        int pos = atomicAdd(next_count, 1);
                        next_frontier[pos] = neighbor_idx;
                    }
                }
            }
        }
    }
}

std::vector<int> planPathGPU(GPUPlanner& planner, const DeviceGrid& grid, int start_idx, int goal_idx) {
    // 1. Initialize buffers
    int threads = 256;
    int blocks = (planner.size + threads - 1) / threads;
    initBuffersKernel<<<blocks, threads>>>(planner.d_gscore, planner.d_parent, planner.d_closed, planner.size, start_idx);
    CUDA_CHECK(cudaDeviceSynchronize());

    // 2. Setup initial frontier
    int initial_count = 1;
    CUDA_CHECK(cudaMemcpy(planner.d_frontier_current, &start_idx, sizeof(int), cudaMemcpyHostToDevice));
    CUDA_CHECK(cudaMemcpy(planner.d_frontier_count, &initial_count, sizeof(int), cudaMemcpyHostToDevice));

    int iter = 0;
    int* d_next_count;
    CUDA_CHECK(cudaMalloc(&d_next_count, sizeof(int)));

    while (true) {
        CUDA_CHECK(cudaMemcpy(planner.h_frontier_count, planner.d_frontier_count, sizeof(int), cudaMemcpyDeviceToHost));
        int current_count = *planner.h_frontier_count;

        if (current_count == 0) break;
        
        CUDA_CHECK(cudaMemset(d_next_count, 0, sizeof(int)));

        int expand_blocks = (current_count + threads - 1) / threads;
        expandKernel<<<expand_blocks, threads>>>(
            planner.d_frontier_current,
            current_count,
            planner.d_frontier_next,
            d_next_count,
            planner.d_gscore,
            planner.d_parent,
            grid,
            goal_idx
        );
        CUDA_CHECK(cudaDeviceSynchronize());

        std::swap(planner.d_frontier_current, planner.d_frontier_next);
        CUDA_CHECK(cudaMemcpy(planner.d_frontier_count, d_next_count, sizeof(int), cudaMemcpyDeviceToDevice));

        iter++;
        if (iter > planner.width * planner.height) break;
    }
    
    CUDA_CHECK(cudaFree(d_next_count));

    // 3. Reconstruct path
    std::vector<int> h_parent(planner.size);
    CUDA_CHECK(cudaMemcpy(h_parent.data(), planner.d_parent, planner.size * sizeof(int), cudaMemcpyDeviceToHost));

    std::vector<int> path;
    int curr = goal_idx;
    
    if (h_parent[curr] == -1 && curr != start_idx) {
        return {}; 
    }

    while (curr != -1) {
        path.push_back(curr);
        if (curr == start_idx) break;
        curr = h_parent[curr];
    }
    std::reverse(path.begin(), path.end());
    return path;
}