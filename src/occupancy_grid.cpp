#include "occupancy_grid.h"
#include <algorithm>
#include <cmath>

OccupancyGrid::OccupancyGrid(int width, int height, float resolution, float origin_x, float origin_y)
    : w_(width), h_(height), res_(resolution), ox_(origin_x), oy_(origin_y),
      logodds_(width * height, 0.0f) {}

int OccupancyGrid::width() const { return w_; }
int OccupancyGrid::height() const { return h_; }
float OccupancyGrid::resolution() const { return res_; }

GridIndex OccupancyGrid::worldToGrid(float wx, float wy) const {
    int gx = static_cast<int>(std::floor((wx - ox_) / res_));
    int gy = static_cast<int>(std::floor((wy - oy_) / res_));
    return {gx, gy};
}

std::pair<float,float> OccupancyGrid::gridToWorld(int gx, int gy) const {
    float wx = ox_ + (gx + 0.5f) * res_;
    float wy = oy_ + (gy + 0.5f) * res_;
    return {wx, wy};
}

float OccupancyGrid::logOddsAt(int gx, int gy) const {
    if (!inBounds(gx, gy)) return 0.0f;
    return logodds_[idx(gx, gy)];
}

float OccupancyGrid::probAt(int gx, int gy) const {
    float l = logOddsAt(gx, gy);
    float p = 1.0f - 1.0f / (1.0f + std::exp(l));
    return p;
}

void OccupancyGrid::setLogOddsAt(int gx, int gy, float val) {
    if (!inBounds(gx, gy)) return;
    val = std::max(L_MIN, std::min(L_MAX, val));
    logodds_[idx(gx, gy)] = val;
}

void OccupancyGrid::addLogOdds(int gx, int gy, float delta) {
    if (!inBounds(gx, gy)) return;
    float v = logodds_[idx(gx, gy)] + delta;
    v = std::max(L_MIN, std::min(L_MAX, v));
    logodds_[idx(gx, gy)] = v;
}

bool OccupancyGrid::inBounds(int gx, int gy) const {
    return gx >= 0 && gx < w_ && gy >= 0 && gy < h_;
}