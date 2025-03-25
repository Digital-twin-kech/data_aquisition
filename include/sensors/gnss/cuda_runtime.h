#ifndef CUDA_RUNTIME_H
#define CUDA_RUNTIME_H

// Mock CUDA runtime for development without actual CUDA toolkit

typedef enum {
    cudaSuccess = 0,
    cudaErrorInvalidValue = 1,
    cudaErrorMemoryAllocation = 2,
    cudaErrorInitializationError = 3,
    cudaErrorCudartUnloading = 4,
    cudaErrorNoDevice = 5
} cudaError_t;

typedef struct {
    char name[256];
    int major;
    int minor;
    int multiProcessorCount;
    int clockRate;
    int totalGlobalMem;
    int sharedMemPerBlock;
    int maxThreadsPerBlock;
} cudaDeviceProp;

inline cudaError_t cudaGetDeviceCount(int* count) {
    // Mock implementation: Pretend we have 1 CUDA device
    if (count) {
        *count = 1;
    }
    return cudaSuccess;
}

inline cudaError_t cudaGetDeviceProperties(cudaDeviceProp* prop, int device) {
    if (!prop || device < 0) {
        return cudaErrorInvalidValue;
    }
    
    // Populate with mock Jetson Xavier values
    strcpy(prop->name, "NVIDIA Jetson Xavier NX");
    prop->major = 7;
    prop->minor = 2;
    prop->multiProcessorCount = 6;
    prop->clockRate = 1109000;  // 1.1 GHz
    prop->totalGlobalMem = 2147483647;  // Use INT_MAX instead of 8GB to avoid overflow
    prop->sharedMemPerBlock = 49152;  // 48 KB
    prop->maxThreadsPerBlock = 1024;
    
    return cudaSuccess;
}

#endif // CUDA_RUNTIME_H