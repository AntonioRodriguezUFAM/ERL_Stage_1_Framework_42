// //AlgorithmConfig.h

// /**
//  * Key Features:
// Diverse Workloads:

// GaussianBlur: Memory bandwidth and cache utilization

// MatrixMultiply: CPU/FPU performance (potential for SIMD)

// Mandelbrot: Floating-point performance

// MultiThreadedInvert: Thread scaling efficiency

// Parallelization:

// Custom parallelFor implementation for workload distribution

// Support for configurable concurrency levels

// Async operations for multi-threaded processing

// Hardware Stress Tests:

// Memory-bound operations

// CPU-bound computations

// Floating-point intensive tasks

// Concurrent execution patterns

// Metrics Tracking:

// Time per operation

// Frame processing rate

// Thread utilization efficiency

// Memory bandwidth measurements

// To use this for hardware benchmarking:

// Create different configuration profiles

// Run each algorithm type with varying parameters

// Compare performance metrics across:

// Single-threaded vs multi-threaded

// CPU vs GPU (when implemented)

// Different hardware configurations

// Various problem sizes

// This framework now supports comprehensive hardware evaluation across multiple compute paradigms.
//  * 
//  */

// #pragma once
// #include <string>
// #include "OpticalFlowConfig.h"

// /**
// updating the AlgorithmConfig structure to include an algorithm type. Adding an enum AlgorithmType with entries like Invert, Grayscale, EdgeDetection, etc.,
//  will allow switching between different processing methods.
//  */


// /**
//  * @enum AlgorithmType
//  * @brief Example enumeration for choosing different algorithm processing types.
//  */

// enum class AlgorithmType {
//     Invert,             // Basic CPU-bound operation
//     Grayscale,          // Memory bandwidth test
//     EdgeDetection,      // Moderate compute
//     GaussianBlur,       // Memory-intensive
//     MatrixMultiply,     // CPU/FPU intensive
//     Mandelbrot,         // Floating-point heavy
//     PasswordHash,       // Integer operations
//     MultiPipeline,      // Combined pipeline
//     GPUMatrixMultiply,  // GPU offload placeholder
//     MultiThreadedInvert, // Concurrent processing
//     OpticalFlow_LucasKanade, // New type Optical Flow
//     // new types GPU-based filters
//     SobelEdge, // New GPU-based filter
//     MedianFilter, // New GPU-based filter
//     HistogramEqualization, // New GPU-based filter
//     HeterogeneousGaussianBlur // New heterogeneous filter
// };

// /**
//  * @brief Example configuration struct for an algorithm.
//  *        Add additional fields relevant to your algorithm.
//  */


// /**
//  * @struct AlgorithmConfig
//  * @brief Configuration for the algorithm pipeline.
//  */
// /**
//  * @brief Configuration structure for an algorithm component.
//  */
// struct AlgorithmConfig {
//     int concurrencyLevel = 1;   ///< Number of threads or concurrency level
//     AlgorithmType algorithmType = AlgorithmType::Invert; // Add this
//     std::string modelPath;      ///< Path to a model file (if using ML/DNN)

//     // Additional parameters for specific algorithms
//     int matrixSize = 512;       // For MatrixMultiply
//     int mandelbrotIter = 100;   // For Mandelbrot
//     int blurRadius = 5;         // For GaussianBlur
//     int medianWindowSize = 5;   // New parameter for median filter
//     bool useGPU = false;        // GPU acceleration flag
//     OpticalFlowConfig opticalFlowConfig; // For Optical Flow algorithms
//     // Add more fields as needed for your specific algorithms

// };

// //AlgorithmConfig.h


#pragma once
#include <string>
#include <cstdint>
#include "OpticalFlowConfig.h"  

enum class AlgorithmType {
    Invert,
    Grayscale,
    EdgeDetection,
    GaussianBlur,
    MatrixMultiply,
    Mandelbrot,
    PasswordHash,
    MultiPipeline,
    GPUMatrixMultiply,
    MultiThreadedInvert,
    OpticalFlow_LucasKanade,
    SobelEdge,
    MedianFilter,
    HistogramEqualization,
    HeterogeneousGaussianBlur
};

struct AlgorithmConfig {
    int32_t concurrencyLevel = 1; // Number of threads
    AlgorithmType algorithmType = AlgorithmType::Invert;
    std::string modelPath;
    int32_t matrixSize = 512;
    int32_t mandelbrotIter = 100;
    int32_t blurRadius = 5;
    int32_t medianWindowSize = 5;
    bool useGPU = false;
    OpticalFlowConfig opticalFlowConfig;

    bool validate() const {
        if (concurrencyLevel <= 0) return false;
        if (matrixSize <= 0) return false;
        if (mandelbrotIter <= 0) return false;
        if (blurRadius <= 0) return false;
        if (medianWindowSize <= 0) return false;
        if (useGPU) {
            // Placeholder: Check CUDA availability
            // if (!cudaAvailable()) return false;
        }
        return opticalFlowConfig.validate();
    }
};