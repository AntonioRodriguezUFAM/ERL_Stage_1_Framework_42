// AlgorithmConcreteKernels.cuh


//============================================================================
// This file defines the AlgorithmConcrete class, which implements various image
// processing algorithms using zero-copy frames. It includes methods for starting
// and stopping the algorithm, processing frames, configuring settings, and
// aggregating performance metrics.
//============================================================================
#pragma once
#include <cstdint>
#include <memory>
#include <vector>

// Forward declaration to avoid including the full header
class ZeroCopyFrameData;

namespace AlgorithmConcreteKernels {

// ---- Core API declarations (only declarations here) ----
void launchSobelEdgeKernel(
    const std::shared_ptr<ZeroCopyFrameData>& frame,
    std::vector<uint8_t>& processedBuffer);

void launchMedianFilterKernel(
    const std::shared_ptr<ZeroCopyFrameData>& frame,
    std::vector<uint8_t>& processedBuffer,
    int windowSize /* 3 or 5 */);

void launchHistogramEqualizationKernel(
    const std::shared_ptr<ZeroCopyFrameData>& frame,
    std::vector<uint8_t>& processedBuffer);

void launchHeterogeneousGaussianBlurKernel(
    const std::shared_ptr<ZeroCopyFrameData>& frame,
    std::vector<uint8_t>& processedBuffer,
    int radius);

// // ---- Back-compat inline wrappers (different overloads) ----
// inline void launchSobelEdgeKernel(
//     const std::shared_ptr<ZeroCopyFrameData>& frame,
//     std::vector<unsigned char>& processedBufferUc)
// {
//     std::vector<uint8_t> tmp;
//     launchSobelEdgeKernel(frame, tmp);   // calls the core API above
//     processedBufferUc.assign(tmp.begin(), tmp.end());
// }

// inline void launchMedianFilterKernel(
//     const std::shared_ptr<ZeroCopyFrameData>& frame,
//     std::vector<unsigned char>& processedBufferUc,
//     int windowSize)
// {
//     std::vector<uint8_t> tmp;
//     launchMedianFilterKernel(frame, tmp, windowSize);
//     processedBufferUc.assign(tmp.begin(), tmp.end());
// }

// inline void launchHistogramEqualizationKernel(
//     const std::shared_ptr<ZeroCopyFrameData>& frame,
//     std::vector<unsigned char>& processedBufferUc)
// {
//     std::vector<uint8_t> tmp;
//     launchHistogramEqualizationKernel(frame, tmp);
//     processedBufferUc.assign(tmp.begin(), tmp.end());
// }

// inline void launchHeterogeneousGaussianBlurKernel(
//     const std::shared_ptr<ZeroCopyFrameData>& frame,
//     std::vector<unsigned char>& processedBufferUc,
//     int radius)
// {
//     std::vector<uint8_t> tmp;
//     launchHeterogeneousGaussianBlurKernel(frame, tmp, radius);
//     processedBufferUc.assign(tmp.begin(), tmp.end());
// }

} // namespace AlgorithmConcreteKernels
