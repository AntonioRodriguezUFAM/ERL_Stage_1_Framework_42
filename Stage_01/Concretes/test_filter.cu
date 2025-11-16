// test_filter.cu
#include <cuda_runtime.h>
#include <iostream>

__global__ void dummy_kernel() {
    printf("CUDA kernel running on Jetson Nano!\n");
}

void test_cuda() {
    dummy_kernel<<<1, 1>>>();
    cudaDeviceSynchronize();
    std::cout << "CUDA test completed successfully!\n";
}