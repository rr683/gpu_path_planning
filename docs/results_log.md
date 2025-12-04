# Experiment Results Log

## Experiment 1: Baseline vs. Naive GPU (Small Scale)
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

---

## Experiment 3: Optimized GPU Planner
**Date:** Dec 3, 2025
**Configuration:**
- Grid: 1024x1024
- Beams: 10,000
- Optimizations: `__ldg()` (Read-Only Cache), Check-before-Atomic

**Results:**
- **GPU Raycasting:** ~3.95 ms
- **GPU Planning:** ~155 ms (vs 183 ms naive)
- **Planning Speedup vs CPU:** ~1.53x (238ms / 155ms)
- **Total Pipeline Speedup:** ~3.45x (550ms / 159ms)

**Conclusion:**
Optimizing memory access patterns (using texture cache and reducing atomic contention) yielded a ~15% improvement in planning time. The remaining bottleneck is the algorithmic overhead of launching hundreds of kernels for the wavefront expansion. To achieve >10x planning speedup, a different algorithmic approach (e.g., block-based expansion or persistent threads) would be required. However, the raycasting component has exceeded expectations with >50x speedup.