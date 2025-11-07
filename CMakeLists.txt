cmake_minimum_required(VERSION 3.13)
project(gpu_path_planning_cpu_baseline LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(planner
  src/main.cpp
  src/occupancy_grid.cpp
  src/raycast_cpu.cpp
  src/astar.cpp
)

target_include_directories(planner PRIVATE src)