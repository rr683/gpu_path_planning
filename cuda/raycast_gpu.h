#pragma once
#include "device_grid.h"

void raycastGPU(
    DeviceGrid& d_grid,
    float robot_x,
    float robot_y,
    const float* h_angles,
    int num_beams,
    float max_range,
    float free_delta,
    float occ_delta
);