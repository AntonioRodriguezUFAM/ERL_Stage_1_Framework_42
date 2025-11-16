// cuda_kernel.cu

// cuda_kernel.cu
#include <cstdio>

__global__ void helloWorldKernel() {
    printf("Hello from GPU!\n");
}

void runCudaHelloWorld() {
    helloWorldKernel<<<1, 1>>>();
    cudaDeviceSynchronize();
}