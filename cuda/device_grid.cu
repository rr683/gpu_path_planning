#include "device_grid.h"
#include "cuda_utils.h"
#include "../src/occupancy_grid.h"
#include <vector>

DeviceGrid allocateDeviceGrid(const OccupancyGrid& host_grid) {
    DeviceGrid d_grid;
    d_grid.width = host_grid.width();
    d_grid.height = host_grid.height();
    d_grid.resolution = host_grid.resolution();
    
    auto [ox, oy] = host_grid.gridToWorld(0, 0);
    d_grid.origin_x = ox - 0.5f * d_grid.resolution;
    d_grid.origin_y = oy - 0.5f * d_grid.resolution;
    
    size_t grid_bytes = d_grid.width * d_grid.height * sizeof(float);
    CUDA_CHECK(cudaMalloc(&d_grid.logodds, grid_bytes));
    CUDA_CHECK(cudaMemset(d_grid.logodds, 0, grid_bytes));
    
    return d_grid;
}

void copyGridToDevice(DeviceGrid& d_grid, const OccupancyGrid& host_grid) {
    std::vector<float> host_logodds(d_grid.width * d_grid.height);
    for (int gy = 0; gy < d_grid.height; ++gy) {
        for (int gx = 0; gx < d_grid.width; ++gx) {
            host_logodds[gy * d_grid.width + gx] = host_grid.logOddsAt(gx, gy);
        }
    }
    size_t bytes = host_logodds.size() * sizeof(float);
    CUDA_CHECK(cudaMemcpy(d_grid.logodds, host_logodds.data(), bytes, cudaMemcpyHostToDevice));
}

void copyGridToHost(OccupancyGrid& host_grid, const DeviceGrid& d_grid) {
    std::vector<float> host_logodds(d_grid.width * d_grid.height);
    size_t bytes = host_logodds.size() * sizeof(float);
    CUDA_CHECK(cudaMemcpy(host_logodds.data(), d_grid.logodds, bytes, cudaMemcpyDeviceToHost));
    
    for (int gy = 0; gy < d_grid.height; ++gy) {
        for (int gx = 0; gx < d_grid.width; ++gx) {
            host_grid.setLogOddsAt(gx, gy, host_logodds[gy * d_grid.width + gx]);
        }
    }
}

void freeDeviceGrid(DeviceGrid& d_grid) {
    CUDA_CHECK(cudaFree(d_grid.logodds));
    d_grid.logodds = nullptr;
}