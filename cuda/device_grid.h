#pragma once
#include <cuda_runtime.h>

struct DeviceGrid {
    float* logodds;
    int width;
    int height;
    float resolution;
    float origin_x;
    float origin_y;
    
    __device__ __host__ int idx(int gx, int gy) const {
        return gy * width + gx;
    }
    
    __device__ __host__ bool inBounds(int gx, int gy) const {
        return gx >= 0 && gx < width && gy >= 0 && gy < height;
    }
};

class OccupancyGrid;

DeviceGrid allocateDeviceGrid(const OccupancyGrid& host_grid);
void copyGridToDevice(DeviceGrid& d_grid, const OccupancyGrid& host_grid);
void copyGridToHost(OccupancyGrid& host_grid, const DeviceGrid& d_grid);
void freeDeviceGrid(DeviceGrid& d_grid);