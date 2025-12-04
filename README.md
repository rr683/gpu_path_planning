# CUDA-Accelerated 2D LiDAR Occupancy Grid Mapping & Path Planning

A high-performance GPU pipeline for real-time occupancy grid mapping and path planning. This project implements parallel beam raycasting and a parallel wavefront path planner using CUDA C++, achieving significant speedups over a CPU baseline.

## Key Features

*   **GPU Raycasting:** Parallelizes LiDAR beam casting (10,000+ beams) using a custom CUDA kernel.
    *   *Performance:* **>50x speedup** vs CPU (4ms vs 300ms).
*   **GPU Path Planning:** Implements a parallel frontier-based BFS planner on the GPU.
    *   *Performance:* **~1.5x speedup** vs CPU A*.
*   **Optimizations:**
    *   `__ldg()` / Read-Only Cache for grid access.
    *   Atomic operation reduction (Check-before-Atomic).
    *   Modular C++/CUDA architecture.


## Build & Run

### Prerequisites
*   NVIDIA CUDA Toolkit (11.0+)
*   C++17 Compiler (GCC, Clang, or MSVC)
*   CMake (3.18+)

### Option 1: Linux / WSL (Recommended)

```bash
# 1. Create build directory
mkdir build && cd build

# 2. Configure and Build
cmake ..
cmake --build . --config Release

# 3. Run CPU Baseline
./planner

# 4. Run GPU Pipeline
./planner_gpu
```
