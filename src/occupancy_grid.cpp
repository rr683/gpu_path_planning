#include "occupancy_grid.h"
#include <algorithm>
#include <cmath>



OccupancyGrid::OccupancyGrid(int width, int height, float resolution, float origin_x, float origin_y)
    : w_(width), h_(height), res_(resolution), ox_(origin_x), oy_(origin_y),
      logodds_(width * height, 0.0f) {}

GridIndex OccupancyGrid::worldToGrid(float wx, float wy) const {return ;}

int OccupancyGrid::height() const {return;}

int OccupancyGrid::width() const {return ;}

float OccupancyGrid::logOddsAt(int wx, int wy) const {return ;}

float OccupancyGrid::probAt(int gx, int gy) const {return;}

void OccupancyGrid::setLogOddsAt(int gx, int gy, float val) const {
    
}

void OccupancyGrid::addLogOdds(int gx, int gy, float delta) {}