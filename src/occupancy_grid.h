#pragma once
#include <vector>
#include <cstdint>
#include <optional>
#include <utility>

struct GridIndex { int x; int y; };

class OccupancyGrid {
public:
    OccupancyGrid(int width, int height, float resolution, float origin_x = 0.0f, float origin_y = 0.0f);

    int width() const;
    int height() const;
    float resolution() const;
    GridIndex worldToGrid(float wx, float wy) const;
    std::pair<float,float> gridToWorld(int gx, int gy) const;

    float logOddsAt(int gx, int gy) const;
    float probAt(int gx, int gy) const;
    void setLogOddsAt(int gx, int gy, float val);

    // integrate a cell as free or occupied (delta in log-odds)
    void addLogOdds(int gx, int gy, float delta);

    // clamp limits for log odds
    static constexpr float L_MIN = -10.0f;
    static constexpr float L_MAX = 10.0f;

    bool inBounds(int gx, int gy) const;
private:
    int w_, h_;
    float res_, ox_, oy_;
    std::vector<float> logodds_; // row-major [y*w + x]
    inline int idx(int gx, int gy) const { return gy * w_ + gx; }
};