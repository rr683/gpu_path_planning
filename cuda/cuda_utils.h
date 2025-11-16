#pragma once
#include <cuda_runtime.h>
#include <iostream>
#include <cstdlib>

#define CUDA_CHECK(call) \
    do { \
        cudaError_t err = call; \
        if (err != cudaSuccess) { \
            std::cerr << "CUDA error in " << __FILE__ << ":" << __LINE__ \
                      << " - " << cudaGetErrorString(err) << std::endl; \
            exit(EXIT_FAILURE); \
        } \
    } while(0)

constexpr float L_MIN = -10.0f;
constexpr float L_MAX = 10.0f;