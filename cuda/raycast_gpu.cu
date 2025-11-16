#include "raycast_gpu.h"
#include "cuda_utils.h"
#include <cmath>

// Bread and butter of this project

__device__ void clampLogOdds(float* val) {
    if (*val < L_MIN) *val = L_MIN;
    if (*val > L_MAX) *val = L_MAX;
}

__device__ void atomicAddClamped(float* addr, float delta) {
    float old = atomicAdd(addr, delta);
    float newval = old + delta;
    clampLogOdds(&newval);
    if (newval != old + delta) {
        atomicExch(addr, newval);
    }
}

__global__ void raycastKernel(
    DeviceGrid grid,
    float robot_x,
    float robot_y,
    float* angles,
    int num_beams,
    float max_range,
    float free_delta,
    float occ_delta
) {
    int beam_id = blockIdx.x * blockDim.x + threadIdx.x;
    if (beam_id >= num_beams) return;
    
    float angle = angles[beam_id];
    float dx = cosf(angle);
    float dy = sinf(angle);
    float step = grid.resolution * 0.5f;
    float traveled = 0.0f;
    
    while (traveled <= max_range) {
        float wx = robot_x + dx * traveled;
        float wy = robot_y + dy * traveled;
        
        int gx = __float2int_rd((wx - grid.origin_x) / grid.resolution);
        int gy = __float2int_rd((wy - grid.origin_y) / grid.resolution);
        
        if (!grid.inBounds(gx, gy)) break;
        
        int cell_idx = grid.idx(gx, gy);
        float current_logodds = grid.logodds[cell_idx];
        float prob = 1.0f - 1.0f / (1.0f + expf(current_logodds));
        
        if (prob > 0.65f) {
            atomicAddClamped(&grid.logodds[cell_idx], occ_delta);
            break;
        } else {
            atomicAddClamped(&grid.logodds[cell_idx], free_delta);
        }
        
        traveled += step;
    }
}

void raycastGPU(
    DeviceGrid& d_grid,
    float robot_x,
    float robot_y,
    const float* h_angles,
    int num_beams,
    float max_range,
    float free_delta,
    float occ_delta
) {
    float* d_angles;
    CUDA_CHECK(cudaMalloc(&d_angles, num_beams * sizeof(float)));
    CUDA_CHECK(cudaMemcpy(d_angles, h_angles, num_beams * sizeof(float), cudaMemcpyHostToDevice));
    
    int threads_per_block = 256;
    int num_blocks = (num_beams + threads_per_block - 1) / threads_per_block;
    
    raycastKernel<<<num_blocks, threads_per_block>>>(
        d_grid, robot_x, robot_y, d_angles, num_beams, max_range, free_delta, occ_delta
    );
    CUDA_CHECK(cudaDeviceSynchronize());
    
    CUDA_CHECK(cudaFree(d_angles));
}