
//===============================================================
// AlgorithmConcreteKernels.cu
#include "AlgorithmConcreteKernels.cuh"
#include "../SharedStructures/ZeroCopyFrameData.h"

#include <cuda_runtime.h>
#include <spdlog/spdlog.h>

#include <cstdint>
#include <vector>
#include <stdexcept>
#include <cmath>

// =============================== CUDA error helper ===============================
static inline void checkCuda(cudaError_t result,
                             char const* const func,
                             char const* const file,
                             int const line) {
    if (result != cudaSuccess) {
        spdlog::error("CUDA error at {}:{} code={}({}) \"{}\"",
                      file, line,
                      static_cast<unsigned int>(result),
                      cudaGetErrorName(result),
                      func);
        cudaDeviceReset();
        std::terminate();
    }
}
#define checkCudaErrors(val) checkCuda((val), #val, __FILE__, __LINE__)

// =============================== device helpers =================================
__device__ __forceinline__ int d_clamp_i(int x, int lo, int hi) {
    return x < lo ? lo : (x > hi ? hi : x);
}

__device__ __forceinline__ uint8_t d_sat_u8(int x) {
    return static_cast<uint8_t>(x < 0 ? 0 : (x > 255 ? 255 : x));
}

// ================================================================================
// KERNELS (assume YUYV: 2 bytes/pixel, Y at even byte, keep chroma=128)
// ================================================================================

__global__ void sobelEdgeKernel(const uint8_t* input, uint8_t* output, int width, int height) {
    const int x = blockIdx.x * blockDim.x + threadIdx.x;
    const int y = blockIdx.y * blockDim.y + threadIdx.y;
    if (x >= width || y >= height) return;

    const int ystride = width * 2;
    const int idx = y * ystride + x * 2; // Y byte location for (x,y)

    if (x > 0 && x < width - 1 && y > 0 && y < height - 1) {
        const int gx =
            -int(input[idx - 2 - ystride]) + int(input[idx + 2 - ystride]) +
            -2 * int(input[idx - 2])       + 2 * int(input[idx + 2]) +
            -int(input[idx - 2 + ystride]) + int(input[idx + 2 + ystride]);

        const int gy =
            -int(input[idx - 2 - ystride]) - 2 * int(input[idx - ystride]) - int(input[idx + 2 - ystride]) +
             int(input[idx - 2 + ystride]) + 2 * int(input[idx + ystride]) + int(input[idx + 2 + ystride]);

        const float magf = sqrtf(float(gx * gx + gy * gy));
        const int   mag  = d_clamp_i(int(magf), 0, 255);

        output[idx]     = static_cast<uint8_t>(mag);
        output[idx + 1] = 128; // neutral chroma
    } else {
        // borders: passthrough
        output[idx]     = input[idx];
        output[idx + 1] = input[idx + 1];
    }
}

__global__ void medianFilterKernel(const uint8_t* input, uint8_t* output,
                                   int width, int height, int windowSize) {
    const int x = blockIdx.x * blockDim.x + threadIdx.x;
    const int y = blockIdx.y * blockDim.y + threadIdx.y;
    if (x >= width || y >= height) return;

    const int radius  = windowSize / 2;
    const int ystride = width * 2;
    const int idx     = y * ystride + x * 2;

    if (windowSize > 5) return; // max 5x5 supported

    uint8_t window[25];
    int count = 0;

    for (int j = -radius; j <= radius; ++j) {
        const int yy = (y + j < 0) ? 0 : ((y + j >= height) ? (height - 1) : (y + j));
        for (int i = -radius; i <= radius; ++i) {
            const int xx = (x + i < 0) ? 0 : ((x + i >= width) ? (width - 1) : (x + i));
            window[count++] = input[yy * ystride + xx * 2];
        }
    }

    // bubble sort (small count)
    for (int i = 0; i < count - 1; ++i) {
        for (int j = 0; j < count - i - 1; ++j) {
            if (window[j] > window[j + 1]) {
                const uint8_t t = window[j];
                window[j] = window[j + 1];
                window[j + 1] = t;
            }
        }
    }

    output[idx]     = window[count / 2];
    output[idx + 1] = 128;
}

__global__ void histogramKernel(const uint8_t* input, unsigned int* hist,
                                int width, int height) {
    const int x = blockIdx.x * blockDim.x + threadIdx.x;
    const int y = blockIdx.y * blockDim.y + threadIdx.y;
    if (x >= width || y >= height) return;

    const int idx = (y * width + x) * 2; // Y byte
    atomicAdd(&hist[input[idx]], 1);
}

__global__ void remapKernel(const uint8_t* input, uint8_t* output,
                            int width, int height,
                            const int* cdf, int minCdf) {
    const int x = blockIdx.x * blockDim.x + threadIdx.x;
    const int y = blockIdx.y * blockDim.y + threadIdx.y;
    if (x >= width || y >= height) return;

    const int idx = (y * width + x) * 2;
    const int totalPixels = width * height;

    const int c = cdf[input[idx]];
    const int denom = totalPixels - minCdf;
    uint8_t newY = 0;
    if (denom > 0) {
        const float norm = float(c - minCdf) / float(denom);
        newY = d_sat_u8(int(norm * 255.0f));
    }

    output[idx]     = newY;
    output[idx + 1] = 128;
}

__global__ void gaussianBlurHorizontalKernel(const uint8_t* input, uint8_t* output,
                                             int width, int height,
                                             int radius, const float* kernel) {
    const int x = blockIdx.x * blockDim.x + threadIdx.x;
    const int y = blockIdx.y * blockDim.y + threadIdx.y;
    if (x >= width || y >= height) return;

    const int ystride = width * 2;

    float acc = 0.0f;
    for (int k = -radius; k <= radius; ++k) {
        int xx = x + k;
        xx = (xx < 0) ? 0 : ((xx >= width) ? (width - 1) : xx);
        acc += float(input[y * ystride + xx * 2]) * kernel[k + radius];
    }

    const int outIdx = y * ystride + x * 2;
    output[outIdx]     = d_sat_u8(int(acc));
    output[outIdx + 1] = 128;
}

__global__ void gaussianBlurVerticalKernel(const uint8_t* input, uint8_t* output,
                                           int width, int height,
                                           int radius, const float* kernel) {
    const int x = blockIdx.x * blockDim.x + threadIdx.x;
    const int y = blockIdx.y * blockDim.y + threadIdx.y;
    if (x >= width || y >= height) return;

    const int ystride = width * 2;

    float acc = 0.0f;
    for (int k = -radius; k <= radius; ++k) {
        int yy = y + k;
        yy = (yy < 0) ? 0 : ((yy >= height) ? (height - 1) : yy);
        acc += float(input[yy * ystride + x * 2]) * kernel[k + radius];
    }

    const int outIdx = y * ystride + x * 2;
    output[outIdx]     = d_sat_u8(int(acc));
    output[outIdx + 1] = 128;
}


//================================================================================
// ---- WRONG (assumes RGB) ----
__global__ void invertKernel(unsigned char* in, unsigned char* out,
                             int width, int height)
{
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;
    if (x >= width || y >= height) return;

    int idx = y * width * 3 + x * 3;   // <-- 3 bytes per pixel
    out[idx]   = 255 - in[idx];       // R
    out[idx+1] = 255 - in[idx+1];     // G
    out[idx+2] = 255 - in[idx+2];     // B
}
//===============================================================================

__global__ void invertYUYVKernel(const unsigned char* in,
                                 unsigned char* out,
                                 int width, int height)
{
    // One thread processes two pixels (4 bytes)
    int x2 = (blockIdx.x * blockDim.x + threadIdx.x) * 2; // pixel index *2
    int y  =  blockIdx.y * blockDim.y + threadIdx.y;
    if (x2 + 1 >= width || y >= height) return;

    int idx = y * width * 2 + x2 * 2;   // 4 bytes for Y0 U Y1 V
    out[idx]     = 255 - in[idx];       // Y0
    out[idx + 1] = in[idx + 1];         // U (unchanged)
    out[idx + 2] = 255 - in[idx + 2];   // Y1
    out[idx + 3] = in[idx + 3];         // V (unchanged)
}


// ================================================================================
// LAUNCHER WRAPPERS (host side). Assume frame->dataPtr is YUYV (width*height*2)
// ================================================================================
namespace AlgorithmConcreteKernels {

void launchSobelEdgeKernel(const std::shared_ptr<ZeroCopyFrameData>& frame,
                           std::vector<uint8_t>& processedBuffer) {
    if (!frame || !frame->dataPtr) {
        spdlog::warn("[CUDA Sobel] Invalid frame/dataPtr");
        return;
    }

    const int width  = frame->width;
    const int height = frame->height;
    const size_t dataSize = frame->size;

    uint8_t *d_in = nullptr, *d_out = nullptr;
    checkCudaErrors(cudaMalloc(&d_in,  dataSize));
    checkCudaErrors(cudaMalloc(&d_out, dataSize));
    checkCudaErrors(cudaMemcpy(d_in, frame->dataPtr, dataSize, cudaMemcpyHostToDevice));

    dim3 block(16, 16);
    dim3 grid((width + block.x - 1) / block.x,
              (height + block.y - 1) / block.y);

    sobelEdgeKernel<<<grid, block>>>(d_in, d_out, width, height);
    checkCudaErrors(cudaGetLastError());

    processedBuffer.resize(dataSize);
    checkCudaErrors(cudaMemcpy(processedBuffer.data(), d_out, dataSize, cudaMemcpyDeviceToHost));

    checkCudaErrors(cudaFree(d_in));
    checkCudaErrors(cudaFree(d_out));
}

void launchMedianFilterKernel(const std::shared_ptr<ZeroCopyFrameData>& frame,
                              std::vector<uint8_t>& processedBuffer,
                              int windowSize) {
    if (!frame || !frame->dataPtr) {
        spdlog::warn("[CUDA Median] Invalid frame/dataPtr");
        return;
    }
    if (windowSize != 3 && windowSize != 5) {
        spdlog::error("[CUDA Median] Only 3x3 or 5x5 windows are supported (got {}).", windowSize);
        return;
    }

    const int width  = frame->width;
    const int height = frame->height;
    const size_t dataSize = frame->size;

    uint8_t *d_in = nullptr, *d_out = nullptr;
    checkCudaErrors(cudaMalloc(&d_in,  dataSize));
    checkCudaErrors(cudaMalloc(&d_out, dataSize));
    checkCudaErrors(cudaMemcpy(d_in, frame->dataPtr, dataSize, cudaMemcpyHostToDevice));

    dim3 block(16, 16);
    dim3 grid((width + block.x - 1) / block.x,
              (height + block.y - 1) / block.y);

    medianFilterKernel<<<grid, block>>>(d_in, d_out, width, height, windowSize);
    checkCudaErrors(cudaGetLastError());

    processedBuffer.resize(dataSize);
    checkCudaErrors(cudaMemcpy(processedBuffer.data(), d_out, dataSize, cudaMemcpyDeviceToHost));

    checkCudaErrors(cudaFree(d_in));
    checkCudaErrors(cudaFree(d_out));
}

void launchHistogramEqualizationKernel(const std::shared_ptr<ZeroCopyFrameData>& frame,
                                       std::vector<uint8_t>& processedBuffer) {
    if (!frame || !frame->dataPtr) {
        spdlog::warn("[CUDA HistEq] Invalid frame/dataPtr");
        return;
    }

    const int width  = frame->width;
    const int height = frame->height;
    const size_t dataSize = frame->size;

    uint8_t     *d_in  = nullptr, *d_out = nullptr;
    unsigned int* d_hist = nullptr;
    int*          d_cdf  = nullptr;

    checkCudaErrors(cudaMalloc(&d_in,  dataSize));
    checkCudaErrors(cudaMalloc(&d_out, dataSize));
    checkCudaErrors(cudaMalloc(&d_hist, 256 * sizeof(unsigned int)));
    checkCudaErrors(cudaMalloc(&d_cdf,  256 * sizeof(int)));
    checkCudaErrors(cudaMemcpy(d_in, frame->dataPtr, dataSize, cudaMemcpyHostToDevice));
    checkCudaErrors(cudaMemset(d_hist, 0, 256 * sizeof(unsigned int)));

    dim3 block(16, 16);
    dim3 grid((width + block.x - 1) / block.x,
              (height + block.y - 1) / block.y);

    // 1) GPU histogram of Y
    histogramKernel<<<grid, block>>>(d_in, d_hist, width, height);
    checkCudaErrors(cudaGetLastError());

    // 2) Bring hist back and compute CDF on CPU
    std::vector<unsigned int> h_hist(256, 0);
    checkCudaErrors(cudaMemcpy(h_hist.data(), d_hist, 256 * sizeof(unsigned int), cudaMemcpyDeviceToHost));

    std::vector<int> h_cdf(256, 0);
    int minCdf = -1;
    for (int i = 0; i < 256; ++i) {
        h_cdf[i] = (i == 0) ? int(h_hist[0]) : (h_cdf[i - 1] + int(h_hist[i]));
        if (minCdf < 0 && h_cdf[i] > 0) minCdf = h_cdf[i];
    }
    if (minCdf < 0) minCdf = 0;

    // 3) Push CDF to GPU and remap Y
    checkCudaErrors(cudaMemcpy(d_cdf, h_cdf.data(), 256 * sizeof(int), cudaMemcpyHostToDevice));
    remapKernel<<<grid, block>>>(d_in, d_out, width, height, d_cdf, minCdf);
    checkCudaErrors(cudaGetLastError());

    processedBuffer.resize(dataSize);
    checkCudaErrors(cudaMemcpy(processedBuffer.data(), d_out, dataSize, cudaMemcpyDeviceToHost));

    checkCudaErrors(cudaFree(d_in));
    checkCudaErrors(cudaFree(d_out));
    checkCudaErrors(cudaFree(d_hist));
    checkCudaErrors(cudaFree(d_cdf));
}

void launchHeterogeneousGaussianBlurKernel(const std::shared_ptr<ZeroCopyFrameData>& frame,
                                           std::vector<uint8_t>& processedBuffer,
                                           int radius) {
    if (!frame || !frame->dataPtr) {
        spdlog::warn("[CUDA HGB] Invalid frame/dataPtr");
        return;
    }
    if (radius <= 0) {
        spdlog::warn("[CUDA HGB] Non-positive radius {}; passthrough.", radius);
        processedBuffer.assign(static_cast<const uint8_t*>(frame->dataPtr),
                               static_cast<const uint8_t*>(frame->dataPtr) + frame->size);
        return;
    }

    const int width  = frame->width;
    const int height = frame->height;
    const size_t dataSize = frame->size;

    // Build separable 1D Gaussian kernel on CPU
    const int ksize = radius * 2 + 1;
    std::vector<float> h_kernel(ksize);
    const float sigma = (radius <= 1) ? 1.0f : (float(radius) * 0.5f);
    float sum = 0.0f;
    for (int i = 0; i < ksize; ++i) {
        const float x = float(i - radius);
        const float w = expf(-(x * x) / (2.0f * sigma * sigma));
        h_kernel[i] = w;
        sum += w;
    }
    if (sum > 0.0f) {
        for (int i = 0; i < ksize; ++i) h_kernel[i] /= sum;
    }

    uint8_t *d_in = nullptr, *d_tmp = nullptr, *d_out = nullptr;
    float   *d_kernel = nullptr;

    checkCudaErrors(cudaMalloc(&d_in,  dataSize));
    checkCudaErrors(cudaMalloc(&d_tmp, dataSize));
    checkCudaErrors(cudaMalloc(&d_out, dataSize));
    checkCudaErrors(cudaMalloc(&d_kernel, ksize * sizeof(float)));
    checkCudaErrors(cudaMemcpy(d_in, frame->dataPtr, dataSize, cudaMemcpyHostToDevice));
    checkCudaErrors(cudaMemcpy(d_kernel, h_kernel.data(), ksize * sizeof(float), cudaMemcpyHostToDevice));

    dim3 block(16, 16);
    dim3 grid((width + block.x - 1) / block.x,
              (height + block.y - 1) / block.y);

    // Horizontal then vertical passes (on Y plane); keep chroma neutral
    gaussianBlurHorizontalKernel<<<grid, block>>>(d_in,  d_tmp, width, height, radius, d_kernel);
    checkCudaErrors(cudaGetLastError());
    gaussianBlurVerticalKernel  <<<grid, block>>>(d_tmp, d_out, width, height, radius, d_kernel);
    checkCudaErrors(cudaGetLastError());

    processedBuffer.resize(dataSize);
    checkCudaErrors(cudaMemcpy(processedBuffer.data(), d_out, dataSize, cudaMemcpyDeviceToHost));

    checkCudaErrors(cudaFree(d_in));
    checkCudaErrors(cudaFree(d_tmp));
    checkCudaErrors(cudaFree(d_out));
    checkCudaErrors(cudaFree(d_kernel));
}

} // namespace AlgorithmConcreteKernels
