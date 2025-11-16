

//=======================================================================================================================================

// AlgorithmConcrete_new.h
#pragma once

#define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_INFO

#include <cstddef>
#include <cstring>
#include <limits>     // for UINT32_MAX
#include <vector>
#include <mutex>
#include <atomic>
#include <thread>
#include <functional>
#include <chrono>
#include <future>
#include <cmath>
#include <algorithm>
#include <memory>

#include <spdlog/spdlog.h>
#include <cuda_runtime.h>
#include <device_launch_parameters.h>

#include "../Interfaces/IAlgorithm.h"
#include "../SharedStructures/FrameData.h"
#include "../SharedStructures/ZeroCopyFrameData.h"
#include "../SharedStructures/SharedQueue.h"
#include "../SharedStructures/ThreadManager.h"
#include "../SharedStructures/AlgorithmConfig.h"
#include "../SharedStructures/LucasKanadeOpticalFlow.h"
#include "../SharedStructures/allModulesStatcs.h"
#include "../Interfaces/ISystemMetricsAggregator.h"
#include "../Others/utils.h"
#include "AlgorithmConcreteKernels.cuh"  // CUDA kernels

/**
 * @class AlgorithmConcrete
 * @brief Implements image processing algorithms using zero-copy frames with metrics aggregation.
 */
class AlgorithmConcrete : public IAlgorithm {
public:
    explicit AlgorithmConcrete(ThreadManager& threadManager);
    AlgorithmConcrete(std::shared_ptr<SharedQueue<std::shared_ptr<ZeroCopyFrameData>>> inputQueue,
                      std::shared_ptr<SharedQueue<std::shared_ptr<ZeroCopyFrameData>>> outputQueue,
                      ThreadManager& threadManager,
                      std::shared_ptr<ISystemMetricsAggregator> aggregator);
    ~AlgorithmConcrete() override;

    static std::shared_ptr<IAlgorithm> createAlgorithmZeroCopy(
        AlgorithmType type,
        std::shared_ptr<SharedQueue<std::shared_ptr<ZeroCopyFrameData>>> inputQueue,
        std::shared_ptr<SharedQueue<std::shared_ptr<ZeroCopyFrameData>>> outputQueue,
        ThreadManager& threadManager,
        std::shared_ptr<ISystemMetricsAggregator> aggregator);

    void startAlgorithm() override;
    void stopAlgorithm() override;
    bool isRunning() const;
    bool processFrameZeroCopy(const std::shared_ptr<ZeroCopyFrameData>& inputFrame,
                              std::shared_ptr<ZeroCopyFrameData>& outputFrame) override;
    bool configure(const AlgorithmConfig& config) override;
    void setErrorCallback(std::function<void(const std::string&)>) override;
    std::tuple<double, double> getAlgorithmMetrics() const override;
    double getLastFPS() const override;
    double getFps() const override;
    double getAverageProcTime() const override;
    const uint8_t* getProcessedBuffer() const override;
    void setAlgorithmType(AlgorithmType newType);

private:
    void threadLoopZeroCopy();
    void updateMetrics(double elapsedSec, uint64_t frameSeq);
    std::string algorithmTypeToString(AlgorithmType type) const;
    void reportError(const std::string& msg);
    void parallelFor(size_t start, size_t end, std::function<void(size_t)> func);

    // CPU ops (YUYV; Y at even byte; UV neutral to 128)
    void processInvertZeroCopy(const std::shared_ptr<ZeroCopyFrameData>& frame);
    void processGrayscaleZeroCopy(const std::shared_ptr<ZeroCopyFrameData>& frame);
    void processEdgeDetectionZeroCopy(const std::shared_ptr<ZeroCopyFrameData>& frame);
    void processGaussianBlurZeroCopy(const std::shared_ptr<ZeroCopyFrameData>& frame);

    // CUDA wrappers
    void processSobelEdgeZeroCopy(const std::shared_ptr<ZeroCopyFrameData>& frame);
    void processMedianFilterZeroCopy(const std::shared_ptr<ZeroCopyFrameData>& frame);
    void processHistogramEqualizationZeroCopy(const std::shared_ptr<ZeroCopyFrameData>& frame);
    void processHeterogeneousGaussianBlurZeroCopy(const std::shared_ptr<ZeroCopyFrameData>& frame);

    void processOpticalFlow(const std::shared_ptr<ZeroCopyFrameData>& frame);

    // CUDA helpers
    void checkCudaError(cudaError_t err, const std::string& context);
    void allocateCudaMemory(void** devPtr, size_t size, const std::string& context);
    void freeCudaMemory(void* devPtr, const std::string& context);
    double timeCudaSectionMs(const std::function<void()>& launch);

    // State
    std::atomic<bool> running_{false};
    std::shared_ptr<SharedQueue<std::shared_ptr<ZeroCopyFrameData>>> inputQueueZeroCopy_;
    std::shared_ptr<SharedQueue<std::shared_ptr<ZeroCopyFrameData>>> outputQueueZeroCopy_;
    AlgorithmConfig algoConfig_;
    std::function<void(const std::string&)> errorCallback_;

    // Metrics
    mutable std::mutex metricMutex_;
    double fps_ = 0.0;
    double avgProcTime_ = 0.0;
    std::vector<uint8_t> processedBuffer_;
    mutable std::atomic<int> framesProcessed_{0};
    mutable std::chrono::steady_clock::time_point lastUpdateTime_;
    mutable double lastFPS_ = 0.0;
    mutable double lastProcessingTime_ = 0.0;
    uint64_t windowFrames_ = 0;
    double windowProcMs_ = 0.0;
    std::chrono::steady_clock::time_point windowStart_;
    uint64_t totalFrames_ = 0;
    double totalProcMs_ = 0.0;
    uint64_t lastFrameSeq_ = 0;
    bool haveLastSeq_ = false;

    // Dependencies
    ThreadManager& threadManager_;
    std::unique_ptr<LucasKanadeOpticalFlow> opticalFlowProcessor_;
    std::shared_ptr<ZeroCopyFrameData> previousFrame_;
    std::shared_ptr<ISystemMetricsAggregator> metricAggregator_;
};

/* ============================ Inline impl ============================ */

inline AlgorithmConcrete::AlgorithmConcrete(ThreadManager& threadManager)
    : running_(false),
      inputQueueZeroCopy_(nullptr),
      outputQueueZeroCopy_(nullptr),
      lastUpdateTime_(std::chrono::steady_clock::now()),
      windowStart_(std::chrono::steady_clock::now()),
      threadManager_(threadManager) {
    spdlog::warn("[AlgorithmConcrete] Default constructor: no queues provided.");
}

inline AlgorithmConcrete::AlgorithmConcrete(
    std::shared_ptr<SharedQueue<std::shared_ptr<ZeroCopyFrameData>>> inputQueue,
    std::shared_ptr<SharedQueue<std::shared_ptr<ZeroCopyFrameData>>> outputQueue,
    ThreadManager& threadManager,
    std::shared_ptr<ISystemMetricsAggregator> aggregator)
    : running_(false),
      inputQueueZeroCopy_(std::move(inputQueue)),
      outputQueueZeroCopy_(std::move(outputQueue)),
      lastUpdateTime_(std::chrono::steady_clock::now()),
      windowStart_(std::chrono::steady_clock::now()),
      threadManager_(threadManager),
      metricAggregator_(aggregator) {
    if (algoConfig_.algorithmType == AlgorithmType::OpticalFlow_LucasKanade) {
        opticalFlowProcessor_ = std::make_unique<LucasKanadeOpticalFlow>(algoConfig_.opticalFlowConfig);
    }
    if (!inputQueueZeroCopy_ || !outputQueueZeroCopy_) {
        spdlog::error("[AlgorithmConcrete] ZeroCopy queues must be non-null.");
    }
    spdlog::info("[AlgorithmConcrete] Constructed with ZeroCopy queues.");
}

inline AlgorithmConcrete::~AlgorithmConcrete() {
    stopAlgorithm();
}

inline std::shared_ptr<IAlgorithm> AlgorithmConcrete::createAlgorithmZeroCopy(
    AlgorithmType type,
    std::shared_ptr<SharedQueue<std::shared_ptr<ZeroCopyFrameData>>> inputQueue,
    std::shared_ptr<SharedQueue<std::shared_ptr<ZeroCopyFrameData>>> outputQueue,
    ThreadManager& threadManager,
    std::shared_ptr<ISystemMetricsAggregator> aggregator) {
    if (!inputQueue || !outputQueue) {
        spdlog::error("[AlgorithmConcrete] ZeroCopy queues must be non-null.");
        return nullptr;
    }
    auto algo = std::make_shared<AlgorithmConcrete>(inputQueue, outputQueue, threadManager, aggregator);
    AlgorithmConfig cfg;
    cfg.algorithmType = type;
    algo->configure(cfg);
    return algo;
}

inline bool AlgorithmConcrete::isRunning() const { return running_; }

inline void AlgorithmConcrete::startAlgorithm() {
    if (running_) {
        spdlog::warn("[AlgorithmConcrete] Already running.");
        return;
    }
    running_ = true;
    try {
        threadManager_.addThread("AlgorithmProcessingZeroCopy",
                                 std::thread(&AlgorithmConcrete::threadLoopZeroCopy, this));
        spdlog::info("[AlgorithmConcrete] Thread started, mode: {}.",
                     algorithmTypeToString(algoConfig_.algorithmType));
    } catch (const std::exception& ex) {
        spdlog::error("[AlgorithmConcrete] Exception during thread start: {}", ex.what());
        running_ = false;
    }
}

inline void AlgorithmConcrete::stopAlgorithm() {
    if (!running_) return;
    running_ = false;
    if (outputQueueZeroCopy_) outputQueueZeroCopy_->stop();
    threadManager_.joinThreadsFor("AlgorithmProcessingZeroCopy");
    spdlog::info("[AlgorithmConcrete] Algorithm thread stopped.");
}

inline bool AlgorithmConcrete::configure(const AlgorithmConfig& config) {
    algoConfig_ = config;
    if (algoConfig_.algorithmType == AlgorithmType::OpticalFlow_LucasKanade) {
        opticalFlowProcessor_ = std::make_unique<LucasKanadeOpticalFlow>(algoConfig_.opticalFlowConfig);
    }
    spdlog::info("[AlgorithmConcrete] Configured to {}.", algorithmTypeToString(config.algorithmType));
    return true;
}

inline void AlgorithmConcrete::setErrorCallback(std::function<void(const std::string&)> callback) {
    errorCallback_ = std::move(callback);
}

inline std::tuple<double, double> AlgorithmConcrete::getAlgorithmMetrics() const {
    return {getLastFPS(), lastProcessingTime_};
}
inline double AlgorithmConcrete::getLastFPS() const { return lastFPS_; }
inline double AlgorithmConcrete::getFps() const { return lastFPS_; }
inline double AlgorithmConcrete::getAverageProcTime() const { return avgProcTime_; }
inline const uint8_t* AlgorithmConcrete::getProcessedBuffer() const { return processedBuffer_.data(); }

inline void AlgorithmConcrete::threadLoopZeroCopy() {
    windowStart_ = std::chrono::steady_clock::now();
    while (running_) {
        std::shared_ptr<ZeroCopyFrameData> in, out;
        if (!inputQueueZeroCopy_->try_pop(in)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            continue;
        }
        if (!in || !in->isValid()) {
            spdlog::warn("[AlgorithmConcrete] Invalid input frame.");
            continue;
        }

        const bool produced = processFrameZeroCopy(in, out);
        if (produced && out && out->isValid()) {
            outputQueueZeroCopy_->push(out);
        }

        const auto now = std::chrono::steady_clock::now();
        const double elapsed = std::chrono::duration<double>(now - windowStart_).count();
        if (elapsed >= 1.0) {
            updateMetrics(elapsed, static_cast<uint64_t>(in->frameNumber));
            windowStart_ = now;
            windowFrames_ = 0;
            windowProcMs_ = 0.0;
        }
    }
    spdlog::info("[AlgorithmConcrete] Exiting thread loop (mode: {}).",
                 algorithmTypeToString(algoConfig_.algorithmType));
}

inline bool AlgorithmConcrete::processFrameZeroCopy(
    const std::shared_ptr<ZeroCopyFrameData>& inputFrame,
    std::shared_ptr<ZeroCopyFrameData>& outputFrame) {
    if (!inputFrame || !inputFrame->isValid() || !inputFrame->dataPtr || inputFrame->size == 0) {
        spdlog::error("[AlgorithmConcrete] Invalid input frame.");
        return false;
    }

    const uint64_t fid = static_cast<uint64_t>(inputFrame->frameNumber);
    processedBuffer_.resize(inputFrame->size);

    double cudaKernelTime = 0.0;
    auto t0 = std::chrono::steady_clock::now();
    // --- CAPTURE START TIMES ---
    auto t_start_steady = std::chrono::steady_clock::now();
    auto t_start_sys = std::chrono::system_clock::now(); // <-- This is what we'll send

    switch (algoConfig_.algorithmType) {
        case AlgorithmType::Invert:
            processInvertZeroCopy(inputFrame);
            break;
        case AlgorithmType::Grayscale:
            processGrayscaleZeroCopy(inputFrame);
            break;
        case AlgorithmType::EdgeDetection:
            processEdgeDetectionZeroCopy(inputFrame);
            break;
        case AlgorithmType::GaussianBlur:
            processGaussianBlurZeroCopy(inputFrame);
            break;
        case AlgorithmType::SobelEdge:
            cudaKernelTime = timeCudaSectionMs([&] {
                AlgorithmConcreteKernels::launchSobelEdgeKernel(inputFrame, processedBuffer_);
            });
            break;
        case AlgorithmType::MedianFilter:
            cudaKernelTime = timeCudaSectionMs([&] {
                AlgorithmConcreteKernels::launchMedianFilterKernel(inputFrame, processedBuffer_,
                                                                   algoConfig_.medianWindowSize);
            });
            break;
        case AlgorithmType::HistogramEqualization:
            cudaKernelTime = timeCudaSectionMs([&] {
                AlgorithmConcreteKernels::launchHistogramEqualizationKernel(inputFrame, processedBuffer_);
            });
            break;
        case AlgorithmType::HeterogeneousGaussianBlur:
            cudaKernelTime = timeCudaSectionMs([&] {
                AlgorithmConcreteKernels::launchHeterogeneousGaussianBlurKernel(inputFrame, processedBuffer_,
                                                                                 algoConfig_.blurRadius);
            });
            break;
        case AlgorithmType::OpticalFlow_LucasKanade:
            // This path pushes its own output
            processOpticalFlow(inputFrame);
            lastProcessingTime_ = std::chrono::duration<double, std::milli>(
                                      std::chrono::steady_clock::now() - t0)
                                      .count();
            goto after_output_build;
        default:
            std::memcpy(processedBuffer_.data(), inputFrame->dataPtr, inputFrame->size);
            break;
    }

    {
        // move processed bytes without copy
        auto processedData = std::make_shared<std::vector<uint8_t>>(std::move(processedBuffer_));

        // Create output frame (YUYV)
        outputFrame = std::make_shared<ZeroCopyFrameData>(
            processedData,
            processedData->size(),
            inputFrame->width,
            inputFrame->height,
            -1, // no physical buffer index for derived frames
            inputFrame->frameNumber,
            inputFrame->captureSys,
            inputFrame->captureSteady);
    }

after_output_build:
    // --- CAPTURE END TIMES ---
    auto t_end_steady = std::chrono::steady_clock::now();
    auto t_end_sys = std::chrono::system_clock::now(); // <-- This is what we'll send

    // lastProcessingTime_ = std::chrono::duration<double, std::milli>(
    //                           std::chrono::steady_clock::now() - t0)
    //                           .count();

    lastProcessingTime_ = std::chrono::duration<double, std::milli>(
                              t_end_steady - t_start_steady
                          ).count();

    // ... (window metrics, dropped frames logic) ...
    // Window/lifetime metrics
    windowFrames_++;
    windowProcMs_ += lastProcessingTime_;
    totalFrames_++;
    totalProcMs_ += lastProcessingTime_;

    // Dropped frames (camera sequence)
    uint32_t dropped = 0;
    if (haveLastSeq_) {
        if (fid > lastFrameSeq_ + 1) {
            const uint64_t gap = fid - lastFrameSeq_ - 1;
            dropped = (gap > UINT32_MAX) ? UINT32_MAX : static_cast<uint32_t>(gap);
        }
    }
    lastFrameSeq_ = fid;
    haveLastSeq_ = true;

    // Aggregation
    if (metricAggregator_) {
        // Use the NEW constructor to pass the (start, end) time window
        AlgorithmStats as{ t_start_sys, t_end_sys };

        // AlgorithmStats as{ inputFrame->captureTime };
        as.inferenceTimeMs   = lastProcessingTime_;
        as.confidenceScore   = 1.0;
        as.fps               = getLastFPS();
        as.avgProcTimeMs     = avgProcTime_;
        as.totalProcTimeMs   = totalProcMs_;
        as.cudaKernelTimeMs  = cudaKernelTime;
        as.droppedFrames     = dropped;

#ifdef __CUDACC__
        size_t freeB = 0, totalB = 0;
        if (cudaMemGetInfo(&freeB, &totalB) == cudaSuccess) {
            as.gpuFreeMemory  = static_cast<uint64_t>(freeB);
            as.gpuTotalMemory = static_cast<uint64_t>(totalB);
        } else {
            as.gpuFreeMemory = as.gpuTotalMemory = 0;
        }
#endif
        
        spdlog ::info ("[DebugID] mergeAlgorithm frameId={}",fid);
        // Send the complete stats object (with time window) to the aggregator
        metricAggregator_->mergeAlgorithm(fid, as);
        spdlog::debug("[AlgorithmConcrete] Merged stats for frame {}.", fid);
        metricAggregator_->overlayStats("algorithm", {
            {"AlgorithmFPS",    as.fps},
            {"AvgProcTimeMs",   as.avgProcTimeMs},
            {"TotalProcTimeMs", as.totalProcTimeMs},
            {"CudaKernelTimeMs",as.cudaKernelTimeMs}
        }, as.timestamp);
        spdlog::debug("[AlgorithmConcrete] Overlayed stats for frame {}, AlgorithmFPS{}.", fid, as.fps);
    }

    return algoConfig_.algorithmType != AlgorithmType::OpticalFlow_LucasKanade;
}

inline void AlgorithmConcrete::updateMetrics(double elapsedSec, uint64_t frameSeq) {
    std::lock_guard<std::mutex> lock(metricMutex_);
    if (elapsedSec <= 0.0) elapsedSec = 1.0;

    const double windowFps   = (windowFrames_ > 0) ? (windowFrames_ / elapsedSec) : 0.0;
    const double windowAvgMs = (windowFrames_ > 0) ? (windowProcMs_ / windowFrames_)
                                                   : (lastProcessingTime_ > 0.0 ? lastProcessingTime_ : 0.0);

    fps_         = windowFps;
    lastFPS_     = windowFps;
    avgProcTime_ = windowAvgMs;

    spdlog::info("[AlgorithmConcrete] Frame {}: windowFPS={:.1f}, windowAvg={:.2f}ms, mode={}",
                 frameSeq, windowFps, windowAvgMs, algorithmTypeToString(algoConfig_.algorithmType));

    if (metricAggregator_) {
        const auto ts = std::chrono::system_clock::now();
        metricAggregator_->overlayStats("algorithmWindow", {
            {"AlgorithmFPS",  windowFps},
            {"AvgProcTimeMs", windowAvgMs}
        }, ts);
    }
}

inline std::string AlgorithmConcrete::algorithmTypeToString(AlgorithmType type) const {
    switch (type) {
        case AlgorithmType::Invert: return "Invert";
        case AlgorithmType::Grayscale: return "Grayscale";
        case AlgorithmType::EdgeDetection: return "EdgeDetection";
        case AlgorithmType::GaussianBlur: return "GaussianBlur";
        case AlgorithmType::SobelEdge: return "SobelEdge";
        case AlgorithmType::MedianFilter: return "MedianFilter";
        case AlgorithmType::HistogramEqualization: return "HistogramEqualization";
        case AlgorithmType::HeterogeneousGaussianBlur: return "HeterogeneousGaussianBlur";
        case AlgorithmType::OpticalFlow_LucasKanade: return "OpticalFlow_LucasKanade";
        default: return "Unknown";
    }
}

inline void AlgorithmConcrete::reportError(const std::string& msg) {
    if (errorCallback_) errorCallback_(msg);
    else spdlog::error("[AlgorithmConcrete] {}", msg);
}

inline void AlgorithmConcrete::parallelFor(size_t start, size_t end, std::function<void(size_t)> func) {
    const size_t numThreads = algoConfig_.concurrencyLevel;
    if (numThreads <= 1) {
        for (size_t i = start; i < end; i++) func(i);
        return;
    }
    const size_t chunkSize = (end - start + numThreads - 1) / numThreads;
    std::vector<std::thread> workers;
    workers.reserve(numThreads);
    for (size_t t = 0; t < numThreads; ++t) {
        size_t s = start + t * chunkSize;
        if (s >= end) break;
        size_t e = std::min(end, s + chunkSize);
        workers.emplace_back([=] { for (size_t j = s; j < e; j++) func(j); });
    }
    for (auto& w : workers) w.join();
}

/* ============================ CPU ops ============================ */

inline void AlgorithmConcrete::processInvertZeroCopy(const std::shared_ptr<ZeroCopyFrameData>& frame) {
    if (!frame || !frame->dataPtr) return;
    const uint8_t* in_data = static_cast<const uint8_t*>(frame->dataPtr);
    parallelFor(0, frame->size, [&](size_t i) { processedBuffer_[i] = ~in_data[i]; });
}

inline void AlgorithmConcrete::processGrayscaleZeroCopy(const std::shared_ptr<ZeroCopyFrameData>& frame) {
    if (!frame || !frame->dataPtr) return;
    const uint8_t* in_data = static_cast<const uint8_t*>(frame->dataPtr);
    parallelFor(0, frame->size, [&](size_t i) { processedBuffer_[i] = (i % 2 == 0) ? in_data[i] : 128; });
}

inline void AlgorithmConcrete::processEdgeDetectionZeroCopy(const std::shared_ptr<ZeroCopyFrameData>& frame) {
    if (!frame || !frame->dataPtr) return;
    const uint8_t* in_data = static_cast<const uint8_t*>(frame->dataPtr);
    const int width = frame->width, height = frame->height;

    parallelFor(0, height, [&](size_t y) {
        for (int x = 0; x < width; ++x) {  // per-pixel iteration
            const int idx = static_cast<int>(y) * width * 2 + x * 2;
            if (x > 0 && x < width - 1) {
                const int grad = std::abs(int(in_data[idx]) - int(in_data[idx - 2]));
                processedBuffer_[idx]     = static_cast<uint8_t>(grad);
                processedBuffer_[idx + 1] = 128;
            } else {
                processedBuffer_[idx]     = in_data[idx];
                processedBuffer_[idx + 1] = in_data[idx + 1];
            }
        }
    });
}

inline void AlgorithmConcrete::processGaussianBlurZeroCopy(const std::shared_ptr<ZeroCopyFrameData>& frame) {
    if (!frame || !frame->dataPtr) return;

    const int radius = algoConfig_.blurRadius;
    const int width = frame->width, height = frame->height;
    std::vector<uint8_t> temp(frame->size);
    const uint8_t* in_data = static_cast<const uint8_t*>(frame->dataPtr);

    // Horizontal pass
    parallelFor(0, height, [&](size_t y) {
        for (int x = 0; x < width; ++x) {
            float sum = 0.0f, wsum = 0.0f;
            for (int dx = -radius; dx <= radius; ++dx) {
                const int xx = std::clamp(x + dx, 0, width - 1);
                const float w = std::exp(-(dx * dx) / (2.0f * radius * radius));
                sum  += in_data[static_cast<int>(y) * width * 2 + xx * 2] * w;
                wsum += w;
            }
            temp[static_cast<int>(y) * width * 2 + x * 2]     = static_cast<uint8_t>(sum / wsum);
            temp[static_cast<int>(y) * width * 2 + x * 2 + 1] = 128;
        }
    });

    // Vertical pass
    parallelFor(0, width, [&](size_t x) {
        for (int y = 0; y < height; ++y) {
            float sum = 0.0f, wsum = 0.0f;
            for (int dy = -radius; dy <= radius; ++dy) {
                const int yy = std::clamp(y + dy, 0, height - 1);
                const float w = std::exp(-(dy * dy) / (2.0f * radius * radius));
                sum  += temp[yy * width * 2 + static_cast<int>(x) * 2] * w;
                wsum += w;
            }
            processedBuffer_[y * width * 2 + static_cast<int>(x) * 2]     = static_cast<uint8_t>(sum / wsum);
            processedBuffer_[y * width * 2 + static_cast<int>(x) * 2 + 1] = 128;
        }
    });
}

/* ============================ CUDA wrappers ============================ */

inline void AlgorithmConcrete::processSobelEdgeZeroCopy(const std::shared_ptr<ZeroCopyFrameData>& frame) {
    AlgorithmConcreteKernels::launchSobelEdgeKernel(frame, processedBuffer_);
}
inline void AlgorithmConcrete::processMedianFilterZeroCopy(const std::shared_ptr<ZeroCopyFrameData>& frame) {
    AlgorithmConcreteKernels::launchMedianFilterKernel(frame, processedBuffer_, algoConfig_.medianWindowSize);
}
inline void AlgorithmConcrete::processHistogramEqualizationZeroCopy(const std::shared_ptr<ZeroCopyFrameData>& frame) {
    AlgorithmConcreteKernels::launchHistogramEqualizationKernel(frame, processedBuffer_);
}
inline void AlgorithmConcrete::processHeterogeneousGaussianBlurZeroCopy(const std::shared_ptr<ZeroCopyFrameData>& frame) {
    AlgorithmConcreteKernels::launchHeterogeneousGaussianBlurKernel(frame, processedBuffer_, algoConfig_.blurRadius);
}

/* ============================ Optical Flow ============================ */

inline void AlgorithmConcrete::processOpticalFlow(const std::shared_ptr<ZeroCopyFrameData>& frame) {
    if (!frame || !frame->dataPtr) {
        spdlog::error("[AlgorithmConcrete] Invalid frame for optical flow.");
        return;
    }
    if (previousFrame_) {
        std::shared_ptr<ZeroCopyFrameData> flowOutput;
        opticalFlowProcessor_->computeOpticalFlow(previousFrame_, frame, flowOutput);
        if (outputQueueZeroCopy_) outputQueueZeroCopy_->push(flowOutput);
    }
    previousFrame_ = frame;
}

/* ============================ CUDA helpers ============================ */

inline void AlgorithmConcrete::checkCudaError(cudaError_t err, const std::string& context) {
    if (err != cudaSuccess) {
        reportError("[AlgorithmConcrete] CUDA error in " + context + ": " + std::string(cudaGetErrorString(err)));
    }
}
inline void AlgorithmConcrete::allocateCudaMemory(void** devPtr, size_t size, const std::string& context) {
    checkCudaError(cudaMalloc(devPtr, size), context);
}
inline void AlgorithmConcrete::freeCudaMemory(void* devPtr, const std::string& context) {
    (void)context;
    checkCudaError(cudaFree(devPtr), "cudaFree");
}

inline void AlgorithmConcrete::setAlgorithmType(AlgorithmType newType) {
    std::lock_guard<std::mutex> lock(metricMutex_);
    if (algoConfig_.algorithmType == newType) {
        spdlog::info("[AlgorithmConcrete] Already set to {}.", algorithmTypeToString(newType));
        return;
    }
    spdlog::info("[AlgorithmConcrete] Switching from {} to {}.",
                 algorithmTypeToString(algoConfig_.algorithmType),
                 algorithmTypeToString(newType));
    algoConfig_.algorithmType = newType;
    if (newType == AlgorithmType::OpticalFlow_LucasKanade) {
        opticalFlowProcessor_ = std::make_unique<LucasKanadeOpticalFlow>(algoConfig_.opticalFlowConfig);
        previousFrame_.reset();
    } else {
        opticalFlowProcessor_.reset();
    }
}

inline double AlgorithmConcrete::timeCudaSectionMs(const std::function<void()>& launch) {
#ifdef __CUDACC__
    cudaEvent_t start{}, stop{};
    cudaEventCreate(&start);
    cudaEventCreate(&stop);
    cudaEventRecord(start, 0);

    launch();

    cudaEventRecord(stop, 0);
    cudaEventSynchronize(stop);
    float ms = 0.0f;
    cudaEventElapsedTime(&ms, start, stop);
    cudaEventDestroy(start);
    cudaEventDestroy(stop);
    return static_cast<double>(ms);
#else
    auto t0 = std::chrono::high_resolution_clock::now();
    launch();
    auto t1 = std::chrono::high_resolution_clock::now();
    return std::chrono::duration<double, std::milli>(t1 - t0).count();
#endif
}
