#include "raycast_cpu.h"
#include <cmath>

// Bresenham-like sampling along ray (digital differential analyzer).
GridIndex raycastAndIntegrate(OccupancyGrid &grid,
                              float origin_x, float origin_y,
                              float angle_rad,
                              float max_range,
                              float free_logodds_delta,
                              float occ_logodds_delta) {
    float dx = std::cos(angle_rad);
    float dy = std::sin(angle_rad);
    float step = grid.resolution() * 0.5f;
    float traveled = 0.0f;
    GridIndex lastHit{-1,-1};
    while (traveled <= max_range) {
        float wx = origin_x + dx * traveled;
        float wy = origin_y + dy * traveled;
        GridIndex g = grid.worldToGrid(wx, wy);
        if (!grid.inBounds(g.x, g.y)) {
            break;
        }
        // check for obstacle by thresholding probability
        float prev = grid.logOddsAt(g.x, g.y);
        // In CPU baseline we assume static obstacles are encoded as very high logodds (> L_MAX/2)
        // but here we'll treat existing high logodds as obstacles for hit termination.
        if (grid.probAt(g.x, g.y) > 0.65f) {
            grid.addLogOdds(g.x, g.y, occ_logodds_delta);
            lastHit = g;
            return lastHit;
        } else {
            grid.addLogOdds(g.x, g.y, free_logodds_delta);
        }
        traveled += step;
    }
    return lastHit;
}