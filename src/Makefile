# Makefile: unsure if ill use this
CXX = cl
NVCC = nvcc
CXXFLAGS = /std:c++17 /O2 /EHsc
NVCCFLAGS = -O3

# CPU baseline
planner: src/main.cpp src/occupancy_grid.cpp src/raycast_cpu.cpp src/astar.cpp
    $(CXX) $(CXXFLAGS) /Feplanner.exe src/*.cpp /Isrc

# CUDA version (later)
planner_gpu: main.cu src/occupancy_grid.cpp src/astar.cpp
    $(NVCC) $(NVCCFLAGS) -o planner_gpu.exe main.cu src/occupancy_grid.cpp src/astar.cpp -Isrc

run_cpu: planner
    .\planner.exe

run_gpu: planner_gpu
    .\planner_gpu.exe

clean:
    del planner.exe planner_gpu.exe