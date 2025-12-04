# Experiment Results Log

## Experiment 1: Baseline vs. Naive GPU (Small Scale)
**Date:** Dec 2, 2025
**Configuration:**
- Grid: 200x200 (CPU) / 512x512 (GPU)
- Beams: 720
- Max Range: 8.0m

**Results:**
- **CPU Raycasting:** 8.34 ms
- **GPU Raycasting (Naive):** 0.65 ms
- **Speedup:** ~12.8x

**Observation:**
Even a naive per-beam kernel significantly outperforms the CPU for small beam counts. The GPU overhead (memory transfer, kernel launch) is amortized well enough.

---

## Experiment 2: Stress Test (Large Scale)
**Date:** Dec 3, 2025
**Configuration:**
- Grid: 1024x1024
- Beams: 10,000
- Max Range: 20.0m

**Results:**
- **CPU Raycasting:** ~308 ms
- **GPU Raycasting:** ~5.7 ms
- **Raycasting Speedup:** ~54x

- **CPU Planning (A*):** ~238 ms
- **GPU Planning (Naive BFS):** ~183 ms
- **Planning Speedup:** ~1.3x

- **Total Pipeline Speedup:** ~2.9x (550ms vs 189ms)

**Observation:**
The naive GPU raycasting scales incredibly well, achieving >50x speedup. However, the naive GPU planner (Parallel BFS) is bottlenecked by kernel launch overhead (one launch per wavefront step) and host-device synchronization. It is barely faster than the CPU A*.

**Next Steps:**
Optimize the GPU planner to reduce synchronization and memory contention, or implement a more efficient parallel search algorithm.