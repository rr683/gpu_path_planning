cmake_minimum_required(VERSION 3.18)
project(gpu_path_planning LANGUAGES CXX CUDA)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CUDA_STANDARD 17)
set(CMAKE_CUDA_STANDARD_REQUIRED ON)

# CPU Baseline Executable
add_executable(planner
  src/main.cpp
  src/occupancy_grid.cpp
  src/raycast_cpu.cpp
  src/astar.cpp
)
target_include_directories(planner PRIVATE src)

# GPU Executable
add_executable(planner_gpu
  main.cu
  cuda/device_grid.cu
  cuda/raycast_gpu.cu
  cuda/planner_gpu.cu
  src/occupancy_grid.cpp
  src/astar.cpp
)
target_include_directories(planner_gpu PRIVATE ${CMAKE_SOURCE_DIR} src)
set_target_properties(planner_gpu PROPERTIES CUDA_SEPARABLE_COMPILATION ON)