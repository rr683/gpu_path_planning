#pragma once
#include <vector>
#include <cstdint>
#include <optional>
#include <utility>

//set up: constructor and classes 
struct GridIndex {int x; int y;};

class OccupancyGrid {
    // Grid Occupancy class here -- 
    public:
    OccupancyGrid(int width, int height, float resolution, float origin_x = 0.0f, float origin_y = 0.0f);

    int width() const; 
    int height () const; 

    GridIndex worldToGrid(float wx, float wy) const; 
    std::pair<float, float> gridToWorld(int gx, int gy); 

    float logOddsAt(int gx, int gy) const;
    float probAt(int gx, int gy) const; 
    void setLogOddsAt(int gx, int gy, float val) const;

    void addLogOdds(int gx, int gy, float delta);

    static constexpr float L_MIN = -10.0f;
    static constexpr float L_MAX = 10.0f;

    bool inBounds(int gx, int gy) const;



    private:
    int w_; int h_;
    float res_; float ox_; float oy_;
    std::vector<float> logodds_; //row major: y*w +x
    inline int idx(int gx, int gy) const {return gy*w_ + gx;}
};