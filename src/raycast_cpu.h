#pragma once
#include "occupancy_grid.h"
#include <utility>

// single beam: origin (wx,wy) and angle or endpoint; Returns the grid index of hit
GridIndex raycastAndIntegrate(OccupancyGrid &grid,
                              float origin_x, float origin_y,
                              float angle_rad,
                              float max_range,
                              float free_logodds_delta,
                              float occ_logodds_delta);
        