#pragma once
#include "occupancy_grid.h"
#include <vector>

using Path = std::vector<GridIndex>;

// simple 4-neighbor A*; returns empty path if none
Path astarPlan(const OccupancyGrid &grid, 
            GridIndex start, GridIndex goal, 
            float occ_threshold = 0.65f);
