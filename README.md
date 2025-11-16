 
# CUDA-Accelerated 2D LiDAR Occupancy Grid Mapping with Real-Time Path Planning

GPU-accelerated pipeline for real-time occupancy grid mapping from LiDAR scans and A* path planning using CUDA C++.

## Project Goal

Implement parallel raycasting and path planning on GPU to achieve ≥10× speedup over CPU baseline for large grids (512×512 to 1024×1024) with dense LiDAR scans (720+ beams) running at 10+ Hz.

## Current Status

### Completed-ish
- **CPU Baseline**: Functional reference implementation
  - Occupancy grid with log-odds updates
  - Sequential raycasting (Bresenham/DDA traversal)
  - A* path planning (4-neighbor)
  - Timing harness for performance comparison
  
- **GPU Skeleton**: Modular CUDA implementation
  - Device grid structure and memory management
  - Parallel per-beam raycasting kernel
  - Host↔device transfer utilities
  - Basic atomic operations for grid updates

### 🚧 In Progress
- Testing and debugging GPU raycasting correctness
- Performance profiling and baseline comparison

### 📋 TODO
1. **GPU Optimizations**
   - [ ] Shared memory tile caching for grid reads
   - [ ] Texture memory binding for occupancy grid
   - [ ] CUDA streams for overlapped transfers/compute
   - [ ] Pinned host memory for faster transfers
   - [ ] Warp-level optimizations

2. **GPU Path Planning**
   - [ ] GPU A* implementation with batched frontier expansion
   - [ ] Compare GPU vs CPU planning performance

3. **Integration & Testing**
   - [ ] End-to-end pipeline with multiple scans
   - [ ] Correctness validation (CPU vs GPU results)
   - [ ] Performance benchmarks at different grid sizes

4. **Deliverables**
   - [ ] Performance report with speedup analysis
   - [ ] Demo video with visualization
   - [ ] (Optional) ROS 2/RViz integration

## Project Structure
