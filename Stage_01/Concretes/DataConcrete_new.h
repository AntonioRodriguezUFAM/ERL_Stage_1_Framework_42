//===========================================================================================================
// DataConcrete_new.h
// Updated: 16/11/2025
//===========================================================================================================
// This file implements the DataConcrete class, which provides a concrete implementation of the IData interface
// for camera data acquisition using V4L2 with zero-copy frame handling. It supports advanced features such as
// dynamic reconfiguration, error handling, and performance metrics.
//===========================================================================================================

//===========================================================================================================
//==================================== FINAL VERSION ========================================================
//==================================== Date: 09-21-2025 ====================================================
//==================================== OPTIMIZED WITH ZERO-COPY AND INDEPENDENCE ===========================
//==================================== TESTING: Final Version ===============================================

/*
 * @brief Manages a V4L2 camera device for capturing frames and pushing them to shared queues with zero-copy.
 * @details This class implements the IData interface, handling V4L2 camera operations with memory-mapped buffers.
 *          It uses a dedicated capture thread for asynchronous frame and metrics production, managed by ThreadManager.
 *          Frames are passed to algoQueue_ and displayQueue_ using zero-copy via std::shared_ptr<void>.
 *          Metrics are pushed per frame to ISystemMetricsAggregator, with robust timestamp normalization.
 *
 * Fixes and Upgrades:
 * - [MOD 09-09-2025] Normalized timestamps at camera source (captureSys for logs, captureSteady for FPS).
 * - [MOD 09-20-2025] Implemented true zero-copy using std::shared_ptr<void> with custom deleter.
 * - [MOD 09-20-2025] Enhanced independence with dedicated capture thread managed by ThreadManager.
 * - [MOD 09-20-2025] Added configurable NUM_BUFFERS via CameraConfig.numBuffers with validation.
 * - [MOD 09-20-2025] Added backpressure checks for queues to handle slow consumers.
 * - [MOD 09-20-2025] Improved metrics logging with per-frame pushes and null checks.
 * - [MOD 09-20-2025] Adjusted for updated ZeroCopyFrameData.h with bufferGuard and dual timestamps.
 * - [MOD 09-20-2025] Enhanced numBuffers handling with validation in CameraConfig.
 * - [MOD 09-21-2025] Removed redundant lock in startStreaming() to prevent self-deadlock when called from startCapture() under lock.
 */

#pragma once

#define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_INFO

#include "../Interfaces/IData.h"
#include "../SharedStructures/FrameData.h"
#include "../SharedStructures/SharedQueue.h"
#include "../SharedStructures/CameraConfig.h"
#include "../SharedStructures/ZeroCopyFrameData.h"
#include "../SharedStructures/ThreadManager.h"
#include "../Interfaces/ISystemMetricsAggregator.h"
#include "../SharedStructures/allModulesStatcs.h"
#include "../Others/utils.h"

#include <vector>
#include <atomic>
#include <mutex>
#include <thread>
#include <functional>
#include <string>
#include <spdlog/spdlog.h>

#include <linux/videodev2.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <cerrno>
#include <cstring>
#include <chrono>

#include <fmt/format.h>

// 1. Add includes (top of file, after existing includes)
#include <sys/eventfd.h>
#include <sys/select.h>


/**
 * @class DataConcrete
 * @brief Manages a V4L2 camera device for capturing frames and pushing them to shared queues.
 *
 * This class handles the interaction with a V4L2 camera device, memory-mapping buffers to capture frames.
 * Two SharedQueue<std::shared_ptr<ZeroCopyFrameData>> are injected:
 * - algoQueue_        (frames for algorithm processing)
 * - displayOrigQueue_ (original frames for display)
 * The class memory-maps a fixed number of V4L2 buffers. For each dequeued buffer,
 * the code copies the camera data into a std::vector<uint8_t> and immediately
 * re-queues the buffer. The name "ZeroCopyFrameData" is preserved for legacy,
 * but it is not truly zero-copy.
 * It pushes frames to two shared queues: one for algorithm processing (`algoQueue_`) and one for display
 * (`displayOrigQueue_`). The "ZeroCopyFrameData" name is retained for legacy reasons, though the implementation
 * involves copying data into a `std::vector<uint8_t>` rather than true zero-copy.
 */
class DataConcrete : public IData {
public:
    /**
     * @brief Constructor for DataConcrete.
     * @param config Camera configuration parameters (resolution, format, etc.).
     * @param tm Shared pointer to ThreadManager for thread handling.
     * @param algoQueue Shared queue for algorithm processing.
     * @param displayOrigQueue Shared queue for display.
     * @param aggregator Shared pointer to ISystemMetricsAggregator for metrics logging.
     */
    DataConcrete(const CameraConfig& config,
                 std::shared_ptr<ThreadManager> tm,
                 std::shared_ptr<SharedQueue<std::shared_ptr<ZeroCopyFrameData>>> algoQueue,
                 std::shared_ptr<SharedQueue<std::shared_ptr<ZeroCopyFrameData>>> displayOrigQueue,
                 std::shared_ptr<ISystemMetricsAggregator> aggregator);

    /**
     * @brief Destructor: stops streaming if active and closes device
     */
    ~DataConcrete() override;

    /**
     * @brief Opens the camera device
     * @param path  The device path (e.g., "/dev/video0")
     * @return True on success, false on failure
     */
    bool openDevice(const std::string& path) override;

    /**
     * @brief Configures the camera (resolution, format, etc.)
     * @param config  Camera configuration
     * @return True on success, false otherwise
     */
    bool configure(const CameraConfig& config) override;

    /**
     * @brief Starts the camera stream (queues buffers, calls STREAMON)
     * @return True on success, false otherwise
     */
    bool startStreaming() override;

    /**
     * @brief Stops the camera stream (calls STREAMOFF)
     * @return True on success (or if already stopped)
     */
    bool stopStreaming() override;

    /**
     * @brief Starts the internal capture thread
     * @return True on success, false otherwise
     */ 
    bool startCapture() override;

    void pauseCapture();
    void resumeCapture();

    /**
     * @brief Stops the capture thread and cleans up resources
     * @return True on success, false otherwise
     */ 
    bool stopCapture() override;

    /**
     * @brief Dequeues one frame, copies it to a vector, pushes it to both queues, re-queues buffer
     * @return True if successfully dequeued, false otherwise
     */
    bool dequeFrame() override;

    /**
     * @brief Queues a specific buffer index. Used internally, but exposed if needed externally.
     * @param bufferIndex  The buffer index
     * @return True on success, false otherwise
     */
    bool queueBuffer(size_t bufferIndex) override;

    /**
     * @brief Sets an error callback for handling error messages
     * @param callback  A function that takes a const std::string&
     */
    void setErrorCallback(std::function<void(const std::string&)> callback) override;

    /**
     * @brief Returns true if streaming is active
     */
    bool isStreaming() const override;

    /**
     * @brief Sends camera metrics (FPS, etc.) to the PerformanceLogger
     */
    void pushCameraMetrics() override;

    /**
     * @brief Computes or returns the most recently computed FPS
     * @return Current frames-per-second estimate
     */
    double getLastFPS() override;

    /**
     * @brief Returns the number of frames queued (if you update framesQueued_)
     */
    int getQueueSize() const override;

    /**
     * @brief Closes the device (unmaps buffers, resets file descriptor)
     */
    void closeDevice();

    /**
     * @brief Stops streaming (if active), then closes and resets the device
     */
    void resetDevice();

    /**
     * @brief Getters for the two shared queues
     */
    std::shared_ptr<SharedQueue<std::shared_ptr<ZeroCopyFrameData>>> getDisplayQueue() const {
        return displayOrigQueue_;
    }
    std::shared_ptr<SharedQueue<std::shared_ptr<ZeroCopyFrameData>>> getProcessingQueue() const {
        return algoQueue_;
    }

private:
    enum BufferState { AVAILABLE, QUEUED, DEQUEUED, PROCESSING, IN_USE };

    // Aggregator
    std::shared_ptr<ISystemMetricsAggregator> metricAggregator_;

    struct Buffer {
        void*       start = nullptr;
        size_t      length = 0;
        BufferState state  = AVAILABLE;
        std::chrono::steady_clock::time_point captureTime;
    };

    bool initializeBuffers();
    bool queueBufferInternal(size_t index);
    void unmapBuffers();
    void reportError(const std::string& msg);
    void cleanUpResources() {}
     // Capture thread
    void captureThreadFunc();

    // [MOD 09-22-2025] Renamed core logic to internal methods to clarify public API vs. implementation.
    bool internalStartStreaming();
    bool internalStopStreaming();
    bool internalDequeFrame();


    // Device file descriptor
    int fd_ = -1;

    // Streaming state
    std::atomic<bool> streaming_{false};

    // Configuration state
    bool configured_ = false;

    // Device path (e.g., "/dev/video0")
    std::string devicePath_;

    // Camera configuration parameters
    CameraConfig cameraConfig_{};

    // Memory-mapped buffers
    std::vector<Buffer> buffers_;

    // Locks
    mutable std::mutex mutex_;
    mutable std::mutex bufferMutex_;

    static constexpr int NUM_BUFFERS = 4;

    std::function<void(const std::string&)> errorCallback_;

    bool forceReleaseDevice();

    // FPS tracking
    mutable std::mutex fpsMutex_;
    mutable double lastFPS_ = 0.0;
    mutable std::atomic<int> framesDequeued_{0};
    mutable std::chrono::steady_clock::time_point lastUpdateTime_;
    std::chrono::steady_clock::time_point lastFpsTime_;
    uint32_t frameCountInSecond_ = 0;
    double lastFps_ = 0.0;

    std::atomic<int> framesQueued_{0};

    // Shared queues
    std::shared_ptr<SharedQueue<std::shared_ptr<ZeroCopyFrameData>>> algoQueue_;
    std::shared_ptr<SharedQueue<std::shared_ptr<ZeroCopyFrameData>>> displayOrigQueue_;

   // ---------------------------------------------------------------
	// 2. Class members (add inside private:)
	int wakeupFd_ = -1;                 // <-- NEW
    std::shared_ptr<ThreadManager> tm_;
    std::atomic<bool> running_{false};
    std::atomic<uint64_t> frameCounter_{0};

    // Pause/resume flag
    std::atomic<bool> capturePaused_{false};

    // Metric pacing
    std::chrono::system_clock::time_point lastMetricTime_{};
    int framesSinceLast_ = 0;

    // [MOD] Track previous capture timestamp for accurate FPS
    std::chrono::system_clock::time_point prevCaptureTs_{};

    // [MOD] Track last FPS calculation time
    std::chrono::steady_clock::time_point  monoBase_;
    std::chrono::system_clock::time_point  sysBase_;
    std::chrono::nanoseconds               sysMinusMono_;
    std::atomic<bool>                      basesInitialized_{false};
    std::chrono::steady_clock::time_point  prevCaptureTsSteady_{};
    bool usesMonotonicTs_ = true;
};

// --------------------------- Implementation ----------------------------------------
//====================================================================================
// --------------------------- Implementation ----------------------------------------
//====================================================================================
// ---------------------------------------------------------------
// 3. Constructor (add eventfd)

inline DataConcrete::DataConcrete(const CameraConfig& config,
                                 std::shared_ptr<ThreadManager> tm,
                                 std::shared_ptr<SharedQueue<std::shared_ptr<ZeroCopyFrameData>>> algoQueue,
                                 std::shared_ptr<SharedQueue<std::shared_ptr<ZeroCopyFrameData>>> displayOrigQueue,
                                 std::shared_ptr<ISystemMetricsAggregator> aggregator)
    : cameraConfig_(config)
    , tm_(tm)
    , algoQueue_(algoQueue)
    , displayOrigQueue_(displayOrigQueue)
    , metricAggregator_(aggregator)
    , mutex_()
    , fd_(-1)
    , streaming_(false)
    , configured_(false)
    , lastFPS_(0.0)
{
    if (!tm_) {
        spdlog::error("[DataConcrete] ThreadManager is null!");
        throw std::runtime_error("ThreadManager is null");
    }
	wakeupFd_ = eventfd(0, EFD_NONBLOCK);
    if (wakeupFd_ == -1) throw std::runtime_error("eventfd() failed");
	
    lastUpdateTime_ = std::chrono::steady_clock::now();
    spdlog::debug("[DataConcrete] Constructor complete.");
    lastFpsTime_ = std::chrono::steady_clock::now();
    lastFps_ = cameraConfig_.fps; // initial value
}
/ ---------------------------------------------------------------
// 4. Destructor – clean up
inline DataConcrete::~DataConcrete() {
    if (running_) stopCapture();
    if (streaming_) stopStreaming();
	if (wakeupFd_ != -1) { ::close(wakeupFd_); wakeupFd_ = -1; }
    if (fd_ >= 0) closeDevice();
    spdlog::debug("[DataConcrete] Destructor complete.");
}

inline bool DataConcrete::openDevice(const std::string& path) {
    try {
        std::lock_guard<std::mutex> lock(mutex_);
        spdlog::debug("[DataConcrete] Entering openDevice for path {}.", path);

        if (fd_ != -1) {
            spdlog::warn("[DataConcrete] Device already open. Closing previous device with fd {}.", fd_);
            closeDevice();
        }

        spdlog::info("[DataConcrete] Attempting to open device at {}.", path);
        fd_ = ::open(path.c_str(), O_RDWR | O_NONBLOCK);
        if (fd_ < 0) {
            std::string errorMsg = fmt::format("Failed to open {}: {}", path, std::strerror(errno));
            reportError(errorMsg);
            spdlog::error("[DataConcrete] {} errno: {}.", errorMsg, errno);
            return false;
        }

        devicePath_ = path;
        spdlog::info("[DataConcrete] Device opened: {} (fd={})", path, fd_);
        return true;
    } catch (const std::exception& e) {
        reportError(fmt::format("Exception in openDevice for path {}: {}", path, e.what()));
        return false;
    } catch (...) {
        reportError(fmt::format("Unknown exception in openDevice for path {}.", path));
        return false;
    }
}

inline bool DataConcrete::forceReleaseDevice() {
    spdlog::info("[DataConcrete] forceReleaseDevice() – clearing any previous stream");
    v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (streaming_) {
        ioctl(fd_, VIDIOC_STREAMOFF, &type);
        streaming_ = false;
    }
    if (fd_ > 0) {
        close(fd_);
        fd_ = -1;
    }
    spdlog::info("[DataConcrete] Device properly released.");
    return true;
}

// ---------------------------------------------------------------
// 5. configure() – enforce 30 FPS + more buffers
inline bool DataConcrete::configure(const CameraConfig& config) {
    try {
        std::lock_guard<std::mutex> lock(mutex_);
        spdlog::debug("[DataConcrete] Entering configure with width {} and height {}.", config.width, config.height);

        if (fd_ < 0) {
            reportError("Device not opened; cannot configure.");
            return false;
        }

        if (configured_ || !buffers_.empty()) {
            spdlog::info("[DataConcrete] Re-configuring device. Unmapping old buffers.");
            unmapBuffers();
        }

        if (!config.validate()) {
            reportError(fmt::format("Invalid CameraConfig: width={}, height={}, fps={}, numBuffers={}",
                                    config.width, config.height, config.fps, config.numBuffers));
            return false;
        }
        cameraConfig_ = config;
		
		// ---- FORCE 30 FPS MIN ----
		if (cameraConfig_.fps < 30) {
			spdlog::warn("[DataConcrete] FPS <30 → clamping to 30");
			cameraConfig_.fps = 30;
		}

        // Reset any old stream/buffers before S_FMT
        v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        ioctl(fd_, VIDIOC_STREAMOFF, &type);
        v4l2_requestbuffers req_clear{};
        req_clear.count  = 0;
        req_clear.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        req_clear.memory = V4L2_MEMORY_MMAP;
        ioctl(fd_, VIDIOC_REQBUFS, &req_clear);

        // Set format
        struct v4l2_format fmt = {};
        fmt.type                = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        fmt.fmt.pix.width       = config.width;
        fmt.fmt.pix.height      = config.height;
        fmt.fmt.pix.pixelformat = static_cast<uint32_t>(config.pixelFormat);
        fmt.fmt.pix.field       = V4L2_FIELD_NONE;

        if (ioctl(fd_, VIDIOC_S_FMT, &fmt) < 0) {
            reportError(fmt::format("VIDIOC_S_FMT failed: {}", std::strerror(errno)));
            return false;
        }

        // Validate format
        struct v4l2_format currentFmt = {};
        currentFmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        if (ioctl(fd_, VIDIOC_G_FMT, &currentFmt) < 0) {
            reportError(fmt::format("VIDIOC_G_FMT failed after S_FMT: {}", std::strerror(errno)));
            return false;
        }
        if (currentFmt.fmt.pix.pixelformat != static_cast<uint32_t>(config.pixelFormat)) {
            reportError(fmt::format("Format mismatch: expected {}, got 0x{:X}",
                                    static_cast<uint32_t>(config.pixelFormat), currentFmt.fmt.pix.pixelformat));
            return false;
        }

        // Set frame rate
		// ---- SET FPS (V4L2) ----
        struct v4l2_streamparm parm = {};
        parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        parm.parm.capture.timeperframe.numerator = 1;
        parm.parm.capture.timeperframe.denominator = config.fps;
        if (ioctl(fd_, VIDIOC_S_PARM, &parm) < 0) {
            spdlog::warn("[DataConcrete] VIDIOC_S_PARM failed to set {} FPS: {}. Continuing with driver default FPS.",
                         config.fps, std::strerror(errno));
        } else {
            spdlog::info("[DataConcrete] Frame rate set to {} FPS.", config.fps);
        }

        // Verify frame rate
        struct v4l2_streamparm gparm = {};
        gparm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        if (ioctl(fd_, VIDIOC_G_PARM, &gparm) < 0) {
            spdlog::warn("[DataConcrete] VIDIOC_G_PARM failed: {}. Assuming {} FPS.", std::strerror(errno), config.fps);
        } else {
            uint32_t denom = gparm.parm.capture.timeperframe.denominator;
            uint32_t num = gparm.parm.capture.timeperframe.numerator;
            if (denom > 0 && num > 0) {
                double actual_fps = static_cast<double>(denom) / num;
                if (std::abs(actual_fps - config.fps) > 0.1) {
                    spdlog::warn("[DataConcrete] Requested {} FPS, but driver set {:.2f} FPS.", config.fps, actual_fps);
                } else {
                    spdlog::info("[DataConcrete] Verified frame rate: {:.2f} FPS.", actual_fps);
                }
            } else {
                spdlog::warn("[DataConcrete] Invalid FPS returned by VIDIOC_G_PARM, assuming {} FPS.", config.fps);
            }
        }

        // Request buffers
		// ---- REQUEST 8 BUFFERS (anti-starving) ----
        struct v4l2_requestbuffers req = {};
        req.count  = config.numBuffers;
        req.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        req.memory = V4L2_MEMORY_MMAP;
        if (ioctl(fd_, VIDIOC_REQBUFS, &req) < 0) {
            reportError(fmt::format("VIDIOC_REQBUFS failed for {} buffers: {}", config.numBuffers, std::strerror(errno)));
            return false;
        }

        if (!initializeBuffers()) {
            spdlog::error("[DataConcrete] Buffer initialization failed.");
            return false;
        }

        configured_ = true;
        spdlog::info("[DataConcrete] Device configured ({}x{}, {} buffers).", config.width, config.height, config.numBuffers);
        return true;
    } catch (const std::exception& e) {
        reportError(fmt::format("Exception in configure: {}", e.what()));
        return false;
    }
}

inline bool DataConcrete::initializeBuffers() {
    try {
        buffers_.clear();
        buffers_.resize(cameraConfig_.numBuffers);

        for (int i = 0; i < cameraConfig_.numBuffers; ++i) {
            struct v4l2_buffer buf = {};
            buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            buf.memory = V4L2_MEMORY_MMAP;
            buf.index  = i;

            if (ioctl(fd_, VIDIOC_QUERYBUF, &buf) < 0) {
                reportError(fmt::format("VIDIOC_QUERYBUF failed for buffer {}: {}", i, std::strerror(errno)));
                return false;
            }

            buffers_[i].length = buf.length;
            buffers_[i].start = mmap(nullptr, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, buf.m.offset);
            if (buffers_[i].start == MAP_FAILED) {
                reportError(fmt::format("mmap failed for buffer {}: {}", i, std::strerror(errno)));
                return false;
            }
            buffers_[i].state = AVAILABLE;
        }
        spdlog::info("[DataConcrete] Initialized {} buffers.", cameraConfig_.numBuffers);
        return true;
    } catch (const std::exception& e) {
        reportError(fmt::format("Exception in initializeBuffers: {}", e.what()));
        return false;
    }
}

inline bool DataConcrete::startStreaming() {
    spdlog::debug("[DataConcrete] Entering startStreaming with {} buffers.", cameraConfig_.numBuffers);
    if (streaming_.load()) {
        spdlog::warn("[DataConcrete] Already streaming, skipping initialization.");
        return true;
    }

    if (!configured_ || buffers_.empty()) {
        reportError("Device not configured or no buffers allocated.");
        return false;
    }

    // Queue all buffers
    for (size_t i = 0; i < buffers_.size(); ++i) {
        if (!queueBufferInternal(i)) {
            spdlog::error("[DataConcrete] Failed to queue buffer {}, aborting.", i);
            return false;
        }
    }

    // Start the stream
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(fd_, VIDIOC_STREAMON, &type) < 0) {
        reportError(fmt::format("VIDIOC_STREAMON failed: {}", std::strerror(errno)));
        return false;
    }

    // Initialize timestamp bases
    auto s1 = std::chrono::system_clock::now();
    auto m  = std::chrono::steady_clock::now();
    auto s2 = std::chrono::system_clock::now();
    sysBase_ = s1 + (s2 - s1) / 2;
    monoBase_ = m;
    sysMinusMono_ = std::chrono::duration_cast<std::chrono::nanoseconds>(sysBase_.time_since_epoch() - monoBase_.time_since_epoch());
    basesInitialized_.store(true, std::memory_order_release);

    // Reset timing helpers
    prevCaptureTsSteady_ = {};
    lastMetricTime_ = sysBase_;
    framesSinceLast_ = 0;

    streaming_ = true;
    spdlog::info("[DataConcrete] Streaming started with {} buffers.", buffers_.size());
    return true;
}

inline bool DataConcrete::stopStreaming() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!streaming_) return true;
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(fd_, VIDIOC_STREAMOFF, &type) < 0) {
        reportError(fmt::format("VIDIOC_STREAMOFF failed: {}", std::strerror(errno)));
    }
    streaming_ = false;
    spdlog::info("[DataConcrete] Streaming stopped.");
    return true;
}

// inline bool DataConcrete::startCapture() {
//     // [MOD 09-20-2025] Independence: Starts async capture thread via ThreadManager
//     std::lock_guard<std::mutex> lock(mutex_);
//     if (running_) return true;
//     if (!configured_) {
//         reportError("Device not configured; cannot start capture.");
//         return false;
//     }
//     if (!startStreaming()) return false;

//     running_ = true;
//     try {
//         std::thread t(&DataConcrete::captureThreadFunc, this);
//         tm_->addThread("CameraCaptureZeroCopy", std::move(t));
//         spdlog::info("[DataConcrete] Camera capture thread started with {} buffers.", cameraConfig_.numBuffers);
//     } catch (const std::exception& e) {
//         running_ = false;
//         streaming_ = false;
//         reportError(fmt::format("Failed to create capture thread: {}", e.what()));
//         return false;
//     }
//     return true;
// }


inline bool DataConcrete::startCapture() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (running_) return true;
    if (!configured_) {
        reportError("Cannot start capture: not configured.");
        return false;
    }
    if (!internalStartStreaming()) return false;
    running_ = true;
    tm_->addThread(Component::Camera, std::thread(&DataConcrete::captureThreadFunc, this));
    return true;
}

inline void DataConcrete::pauseCapture() {
    capturePaused_ = true;
    spdlog::info("[DataConcrete] Capture paused.");
}

inline void DataConcrete::resumeCapture() {
    capturePaused_ = false;
    spdlog::info("[DataConcrete] Capture resumed.");
}

// inline bool DataConcrete::stopCapture() {
//     // [MOD 09-20-2025] Independence: Safely stops async thread via ThreadManager
//     std::lock_guard<std::mutex> lock(mutex_);
//     if (!running_) return true;

//     running_ = false;
//     tm_->joinThreadsFor("CameraCaptureZeroCopy");
//     stopStreaming();
//     spdlog::info("[DataConcrete] Camera capture thread stopped.");
//     return true;
// }


inline bool DataConcrete::stopCapture() {
    if (!running_.exchange(false)) return true;
    
    spdlog::info("Stopping camera capture thread...");
    tm_->joinThreadsFor(Component::Camera);
    
    {
        std::lock_guard<std::mutex> lock(mutex_);
        internalStopStreaming();
    }
    spdlog::info("[DataConcrete] Camera capture thread stopped.");
    return true;
}


// inline void DataConcrete::captureThreadFunc() {
//     // [MOD 09-20-2025] Independence: Async thread loop produces frames and metrics
//     spdlog::info("[DataConcrete] Capture thread started.");
//     try {
//         while (running_) {
//             if (capturePaused_) {
//                 std::this_thread::sleep_for(std::chrono::milliseconds(100));
//                 continue;
//             }
//             if (!dequeFrame()) {
//                 if (errno == EAGAIN) {
//                     std::this_thread::yield();
//                     continue;
//                 }
//                 reportError("Failed to dequeue frame");
//                 break;
//             }
//         }
//     } catch (const std::exception& e) {
//         spdlog::error("[DataConcrete] Exception in capture thread: {}", e.what());
//     }
//     spdlog::info("[DataConcrete] Capture thread exiting.");
// }

// inline void DataConcrete::captureThreadFunc() {
//     spdlog::info("[DataConcrete] Capture thread started.");
//     try {
//         while (running_) {
//             if (capturePaused_) {
//                 std::this_thread::sleep_for(std::chrono::milliseconds(100));
//                 continue;
//             }

//             fd_set fds;
//             FD_ZERO(&fds);
//             FD_SET(fd_, &fds);
//             struct timeval tv = {2, 0}; // 2-second timeout

//             // 1. Sleep efficiently until a frame is ready or a timeout occurs.
//             int r = select(fd_ + 1, &fds, nullptr, nullptr, &tv);

//             if (r < 0) { // Error
//                 reportError("select() failed in capture thread");
//                 break;
//             }
//             if (r == 0) { // Timeout
//                 spdlog::warn("[DataConcrete] select() timeout in capture thread, no frame received in 2s.");
//                 continue;
//             }
            
//             // 2. A frame is ready. Now we can call dequeFrame() to process it.
//             if (!dequeFrame()) {
//                 if (errno != EAGAIN) {
//                     reportError("dequeFrame returned a fatal error.");
//                     break;
//                 }
//             }
//         }
//     } catch (const std::exception& e) {
//         spdlog::error("[DataConcrete] Exception in capture thread: {}", e.what());
//     }
//     spdlog::info("[DataConcrete] Capture thread exiting.");
// }


// inline void DataConcrete::captureThreadFunc() {
//     spdlog::info("[DataConcrete] Capture thread started.");
//     while (running_) {
//         fd_set fds;
//         FD_ZERO(&fds);
//         FD_SET(fd_, &fds);
//         struct timeval tv = {2, 0}; // 2-second timeout
//         int r = select(fd_ + 1, &fds, nullptr, nullptr, &tv);
//         if (r > 0) {
//             internalDequeFrame();
//         } else if (r == 0) {
//             spdlog::warn("[DataConcrete] select() timeout in capture thread, no frame received in 2s.");
//         } else {
//             if (errno != EINTR) {
//                 reportError("select() failed in capture thread");
//                 running_ = false; // Stop thread on critical error
//             }
//         }
//     }
//     spdlog::info("[DataConcrete] Capture thread exiting.");
// }


// ==================================================================================================================
// ------------------------------------------------------------
//  DataConcrete  capture thread + throttling
// ------------------------------------------------------------
inline void DataConcrete::captureThreadFunc() {
    spdlog::info("[DataConcrete] Capture thread started.");
    
    // --- THROTTLING SETUP ---
    const double target_fps = cameraConfig_.fps > 0 ? cameraConfig_.fps : 30.0;
    const auto   min_interval = std::chrono::nanoseconds(static_cast<long long>(1e9 / target_fps));
    auto         next_allowed = std::chrono::steady_clock::now();

    while (running_) {
        fd_set fds;
        FD_ZERO(&fds);
        FD_SET(fd_, &fds);

        // Dynamic timeout: remaining time until next allowed frame
        const auto now = std::chrono::steady_clock::now();
        const auto timeout_ns = (next_allowed > now)
            ? std::chrono::duration_cast<std::chrono::nanoseconds>(next_allowed - now)
            : std::chrono::nanoseconds::zero();

        struct timeval tv = {
            .tv_sec  = static_cast<__time_t>(timeout_ns.count() / 1'000'000'000),
            .tv_usec = static_cast<__suseconds_t>((timeout_ns.count() % 1'000'000'000) / 1000)
        };

        int r = select(fd_ + 1, &fds, nullptr, nullptr, &tv);
        if (r > 0) {
            // A buffer is ready  dequeue and process
            if (internalDequeFrame()) {
                // Frame successfully pushed ? advance slot
                next_allowed += min_interval;
                // If we fell behind, clamp to now+interval to avoid backlog
                if (next_allowed < std::chrono::steady_clock::now() + min_interval)
                    next_allowed = std::chrono::steady_clock::now() + min_interval;
            }
        }
        else if (r == 0) {
            // Timeout  either we were waiting for the slot or no frame arrived
            if (timeout_ns.count() > 0) {
                // We just hit the throttling sleep  continue
                continue;
            }
            spdlog::warn("[DataConcrete] select() timeout, no frame in 2s.");
        }
        else {
            if (errno != EINTR) {
                reportError("select() failed in capture thread");
                running_ = false;
            }
        }
    }
    spdlog::info("[DataConcrete] Capture thread exiting.");
}

//====================================================================================================================

// DELETE THIS FUNCTION  it duplicates internalDequeFrame

inline bool DataConcrete::dequeFrame() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!streaming_) {
        spdlog::warn("[DataConcrete] dequeFrame called but not streaming.");
        return false;
    }

    // Dequeue buffer
    struct v4l2_buffer buf{};
    buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;

    if (ioctl(fd_, VIDIOC_DQBUF, &buf) < 0) {
        if (errno != EAGAIN) {
            reportError(fmt::format("VIDIOC_DQBUF failed: {}", std::strerror(errno)));
        }
        return false;
    }

    const size_t bufferIndex = buf.index;
    Buffer& currentBuffer = buffers_[bufferIndex];

    // Guard against empty frames
    if (buf.bytesused == 0) {
        spdlog::warn("[DataConcrete] Dropping empty frame idx={} (bytesused=0).", bufferIndex);
        if (ioctl(fd_, VIDIOC_QBUF, &buf) < 0) {
            reportError(fmt::format("VIDIOC_QBUF failed (after empty frame): {}", std::strerror(errno)));
        }
        return false;
    }

    // [MOD 09-20-2025] Zero-Copy: Use shared_ptr for MMAP buffer with custom deleter
    // auto bufferGuard = std::shared_ptr<void>(currentBuffer.start, [this, bufferIndex](void*) {
    //     queueBufferInternal(bufferIndex); // Auto-requeue when refcount hits 0
    // });

     // [MOD 09-22-2025] CRITICAL FIX: The custom deleter now calls the public, thread-safe queueBuffer method.
    // This prevents the race condition that was causing buffer starvation.
    auto bufferGuard = std::shared_ptr<void>(currentBuffer.start, [this, bufferIndex](void*) {
        this->queueBuffer(bufferIndex);
    });


    // auto bufferGuard = std::shared_ptr<void>(currentBuffer.start, [this, bufferIndex](void*) {
    //     // Call the public method, which correctly locks the mutex.
    //     this->queueBuffer(bufferIndex);
    // });

    // [MOD 09-09-2025] Timestamp normalization
    std::chrono::steady_clock::time_point capMono;
    std::chrono::system_clock::time_point capSys;
    bool haveDriverTs = (buf.timestamp.tv_sec != 0 || buf.timestamp.tv_usec != 0);
    if (usesMonotonicTs_ && haveDriverTs && basesInitialized_) {
        auto mono_ns = std::chrono::seconds(buf.timestamp.tv_sec) + std::chrono::microseconds(buf.timestamp.tv_usec);
        capMono = std::chrono::steady_clock::time_point(std::chrono::duration_cast<std::chrono::steady_clock::duration>(mono_ns));
        capSys = std::chrono::system_clock::time_point(capMono.time_since_epoch() + sysMinusMono_);
    } else {
        capMono = std::chrono::steady_clock::now();
        capSys = std::chrono::system_clock::now();
    }

    uint64_t frameId = frameCounter_.fetch_add(1);

    // [MOD 09-20-2025] Use updated ZeroCopyFrameData with bufferGuard and dual timestamps
    auto frame = std::make_shared<ZeroCopyFrameData>(
        bufferGuard,
        bufferGuard.get(),
        buf.bytesused,
        cameraConfig_.width,
        cameraConfig_.height,
        bufferIndex,
        frameId,
        capSys,
        capMono
    );

    if (!frame->isValid()) {
        spdlog::warn("[DataConcrete] Dropped invalid frame (frame={}).", frameId);
        return false; // Deleter auto-requeues
    }

    // [MOD 09-20-2025] Accurate FPS using monotonic clock
    double fpsMeasured = cameraConfig_.fps > 0 ? static_cast<double>(cameraConfig_.fps) : 30.0;
    if (prevCaptureTsSteady_.time_since_epoch().count() != 0) {
        auto dt_ms = std::chrono::duration_cast<std::chrono::milliseconds>(capMono - prevCaptureTsSteady_).count();
        if (dt_ms > 0) fpsMeasured = 1000.0 / static_cast<double>(dt_ms);
    }
    prevCaptureTsSteady_ = capMono;

    // [MOD 09-20-2025] Push metrics for every frame
    if (metricAggregator_) {
        CameraStats cs{};
        cs.timestamp = capSys;
        cs.frameNumber = frameId;
        cs.fps = fpsMeasured;
        cs.frameWidth = static_cast<uint32_t>(cameraConfig_.width);
        cs.frameHeight = static_cast<uint32_t>(cameraConfig_.height);
        cs.frameSize = static_cast<uint64_t>(buf.bytesused);

        if (cs.isValid()) {
            spdlog::debug("[DataConcrete] Pushing metrics for frame {}: FPS={:.2f}, Size={}, Buffers={}",
                          cs.frameNumber, cs.fps, cs.frameSize, cameraConfig_.numBuffers);

            spdlog ::info ("[DebugID] beginFrame frameId={}",cs.frameNumber);
            
            metricAggregator_->beginFrame(cs.frameNumber, cs);
            metricAggregator_->overlayStats("camera", {
                {"FPS", cs.fps},
                {"FrameWidth", static_cast<double>(cs.frameWidth)},
                {"FrameHeight", static_cast<double>(cs.frameHeight)},
                {"FrameSize", static_cast<double>(cs.frameSize)},
                {"NumBuffers", static_cast<double>(cameraConfig_.numBuffers)}
            }, capSys);
        } else {
            spdlog::warn("[DataConcrete] Invalid CameraStats for frame {}", cs.frameNumber);
        }
    } else {
        spdlog::warn("[DataConcrete] metricAggregator_ is null, skipping metrics push.");
    }

    // [MOD 09-20-2025] Backpressure checks for queues
    if (displayOrigQueue_ && displayOrigQueue_->size() < cameraConfig_.numBuffers * 2) {
        displayOrigQueue_->push(frame);
    } else {
        spdlog::warn("[DataConcrete] Display queue full, dropping frame {}.", frameId);
    }
    if (algoQueue_ && algoQueue_->size() < cameraConfig_.numBuffers * 2) {
        algoQueue_->push(frame);
    } else {
        spdlog::warn("[DataConcrete] Algo queue full, dropping frame {}.", frameId);
    }

    return true;
}
//====================================================================================================================

// inline bool DataConcrete::internalDequeFrame() {
//     std::lock_guard<std::mutex> lock(mutex_);
    
//     struct v4l2_buffer buf{};
//     buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
//     buf.memory = V4L2_MEMORY_MMAP;
//     if (ioctl(fd_, VIDIOC_DQBUF, &buf) < 0) {
//         if (errno != EAGAIN) reportError(fmt::format("VIDIOC_DQBUF failed: {}", std::strerror(errno)));
//         return false;
//     }

//     const size_t bufferIndex = buf.index;
//     Buffer& currentBuffer = buffers_[bufferIndex];
    
//     currentBuffer.state = IN_USE;

//     if (buf.bytesused == 0) {
//         spdlog::warn("[DataConcrete] Dequeued empty frame (idx={}), re-queuing immediately.", bufferIndex);
//         queueBufferInternal(bufferIndex); // Requeue and continue
//         return true;
//     }
    
//     auto bufferGuard = std::shared_ptr<void>(currentBuffer.start, [this, bufferIndex](void*) {
//         this->queueBuffer(bufferIndex);
//     });

//     // [MOD 09-22-2025] Completed the implementation of this method with the logic from the previous, working version.
//     // This includes timestamp normalization, frame creation, metrics calculation, and pushing to queues.
    
//     // --- Timestamp Normalization ---
//     std::chrono::steady_clock::time_point capMono;
//     std::chrono::system_clock::time_point capSys;
//     const bool haveDriverTs = (buf.timestamp.tv_sec != 0 || buf.timestamp.tv_usec != 0);
//     const bool haveBases = basesInitialized_.load(std::memory_order_acquire);

//     if (usesMonotonicTs_ && haveDriverTs && haveBases) {
//         auto mono_ns = std::chrono::seconds(buf.timestamp.tv_sec) + std::chrono::microseconds(buf.timestamp.tv_usec);
//         capMono = std::chrono::steady_clock::time_point(std::chrono::duration_cast<std::chrono::steady_clock::duration>(mono_ns));
//         capSys = std::chrono::system_clock::time_point(capMono.time_since_epoch() + sysMinusMono_);
//     } else {
//         capMono = std::chrono::steady_clock::now();
//         capSys = std::chrono::system_clock::now();
//     }

//     // --- Frame Creation ---
//     const uint64_t frameId = frameCounter_.fetch_add(1);
//     auto frame = std::make_shared<ZeroCopyFrameData>(
//         bufferGuard,
//         currentBuffer.start,
//         buf.bytesused,
//         cameraConfig_.width,
//         cameraConfig_.height,
//         bufferIndex,
//         frameId,
//         capSys,
//         capMono
//     );

//     if (!frame->isValid()) {
//         spdlog::warn("[DataConcrete] Created an invalid frame (frameId={}). Buffer will be auto-requeued.", frameId);
//         return true; // Return true because the error is handled, and the loop can continue.
//     }

//     // --- Metrics Calculation and Pushing ---
//     double fpsMeasured = (cameraConfig_.fps > 0) ? static_cast<double>(cameraConfig_.fps) : 30.0;
//     if (prevCaptureTsSteady_.time_since_epoch().count() != 0) {
//         auto dt_ms = std::chrono::duration_cast<std::chrono::milliseconds>(capMono - prevCaptureTsSteady_).count();
//         if (dt_ms > 0) fpsMeasured = 1000.0 / static_cast<double>(dt_ms);
//     }
//     prevCaptureTsSteady_ = capMono;

//     if (metricAggregator_) {
//         CameraStats cs{};
//         cs.timestamp = capSys;
//         cs.frameNumber = frameId;
//         cs.fps = fpsMeasured;
//         cs.frameWidth = static_cast<uint32_t>(cameraConfig_.width);
//         cs.frameHeight = static_cast<uint32_t>(cameraConfig_.height);
//         cs.frameSize = static_cast<uint64_t>(buf.bytesused);
//         metricAggregator_->beginFrame(cs.frameNumber, cs);
//         spdlog::debug("[DataConcrete] Pushed metrics for frame {}: FPS={:.2f}, Size={}", 
//                       cs.frameNumber, cs.fps, cs.frameSize);
//         //metricAggregator_->mergeCamera(frameId, cs);   // <-- ADD THIS LINE
//     }
    
//     // --- Push to Queues ---
//     if (algoQueue_) {
//         algoQueue_->push(frame);
//     }
//     if (displayOrigQueue_) {
//         displayOrigQueue_->push(frame);
//     }
    
//     return true;
// }


// //==============================================================================================================
// inline bool DataConcrete::internalDequeFrame() {
//     std::lock_guard<std::mutex> lock(mutex_);
//     if (!streaming_) return false;

//     struct v4l2_buffer buf{};
//     buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
//     buf.memory = V4L2_MEMORY_MMAP;

//     if (ioctl(fd_, VIDIOC_DQBUF, &buf) < 0) {
//         if (errno != EAGAIN)
//             reportError(fmt::format("VIDIOC_DQBUF failed: {}", std::strerror(errno)));
//         return false;
//     }

//     const size_t bufferIndex = buf.index;
//     Buffer& cur = buffers_[bufferIndex];

//     // Empty frame ? requeue immediately
//     if (buf.bytesused == 0) {
//         spdlog::debug("[DataConcrete] Empty frame (idx={}), re-queue.", bufferIndex);
//         queueBufferInternal(bufferIndex);
//         return true;
//     }

//     // --- Zero-copy guard ---
//     auto bufferGuard = std::shared_ptr<void>(cur.start, [this, bufferIndex](void*) {
//         this->queueBuffer(bufferIndex);
//     });

//     // --- Timestamps ---
//     std::chrono::steady_clock::time_point capMono;
//     std::chrono::system_clock::time_point capSys;
//     const bool haveDriverTs = (buf.timestamp.tv_sec || buf.timestamp.tv_usec);
//     if (usesMonotonicTs_ && haveDriverTs && basesInitialized_) {
//         auto mono_ns = std::chrono::seconds(buf.timestamp.tv_sec)
//                      + std::chrono::microseconds(buf.timestamp.tv_usec);
//         capMono = std::chrono::steady_clock::time_point(
//             std::chrono::duration_cast<std::chrono::steady_clock::duration>(mono_ns));
//         capSys = std::chrono::system_clock::time_point(capMono.time_since_epoch() + sysMinusMono_);
//     } else {
//         capMono = std::chrono::steady_clock::now();
//         capSys  = std::chrono::system_clock::now();
//     }

//     // --- Frame ID ---
//     const uint64_t frameId = frameCounter_.fetch_add(1, std::memory_order_relaxed);

//     // --- Build ZeroCopyFrameData ---
//     auto frame = std::make_shared<ZeroCopyFrameData>(
//         bufferGuard,
//         cur.start,
//         buf.bytesused,
//         cameraConfig_.width,
//         cameraConfig_.height,
//         bufferIndex,
//         frameId,
//         capSys,
//         capMono
//     );

//     if (!frame->isValid()) {
//         spdlog::warn("[DataConcrete] Invalid frame (id={}), dropping.", frameId);
//         return true; // deleter re-queues
//     }

//     // --- FPS measurement (for logging & metrics) ---
//     double fpsMeasured = cameraConfig_.fps > 0 ? static_cast<double>(cameraConfig_.fps) : 30.0;
//     if (prevCaptureTsSteady_.time_since_epoch().count() != 0) {
//         auto dt_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
//                         capMono - prevCaptureTsSteady_).count();
//         if (dt_ms > 0) fpsMeasured = 1000.0 / dt_ms;
//     }
//     prevCaptureTsSteady_ = capMono;

//     // --- Push to Aggregator ---
//     if (metricAggregator_) {
//         CameraStats cs{};
//         cs.timestamp    = capSys;
//         cs.frameNumber  = frameId;
//         cs.fps          = fpsMeasured;
//         cs.frameWidth   = cameraConfig_.width;
//         cs.frameHeight  = cameraConfig_.height;
//         cs.frameSize    = buf.bytesused;

//         spdlog::info("[DebugID] beginFrame frameId={}", frameId);
//         metricAggregator_->beginFrame(frameId, cs);

//         metricAggregator_->overlayStats("camera", {
//             {"FPS", cs.fps},
//             {"Width", static_cast<double>(cs.frameWidth)},
//             {"Height", static_cast<double>(cs.frameHeight)},
//             {"Size", static_cast<double>(cs.frameSize)}
//         }, capSys);
//     }

//     // --- Push to downstream queues (with back-pressure) ---
//     const size_t max_q_size = cameraConfig_.numBuffers * 2;
//     bool pushed = true;

//     if (displayOrigQueue_ && displayOrigQueue_->size() < max_q_size)
//         displayOrigQueue_->push(frame);
//     else if (displayOrigQueue_)
//         pushed = false, spdlog::warn("[DataConcrete] Display queue full (frame {})", frameId);

//     if (algoQueue_ && algoQueue_->size() < max_q_size)
//         algoQueue_->push(frame);
//     else if (algoQueue_)
//         pushed = false, spdlog::warn("[DataConcrete] Algo queue full (frame {})", frameId);

//     return pushed;
// }
// //==============================================================================================================

//==============================================================================================================
// REPLACE the existing internalDequeFrame (lines 797-886) with this corrected version
//==============================================================================================================
inline bool DataConcrete::internalDequeFrame() {
    std::shared_ptr<ZeroCopyFrameData> frame;
    CameraStats cs{};
    bool pushed = true;
    bool frame_valid = false;
    std::chrono::system_clock::time_point capSys; // Need capSys outside lock for overlayStats

    { // --- Start Critical Section ---
        std::lock_guard<std::mutex> lock(mutex_);
        if (!streaming_) return false;

        struct v4l2_buffer buf{};
        buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;

        if (ioctl(fd_, VIDIOC_DQBUF, &buf) < 0) {
            if (errno != EAGAIN)
                reportError(fmt::format("VIDIOC_DQBUF failed: {}", std::strerror(errno)));
            return false; // Error or EAGAIN
        }

        const size_t bufferIndex = buf.index;
        if (bufferIndex >= buffers_.size()) {
            reportError(fmt::format("VIDIOC_DQBUF returned invalid index: {}", bufferIndex));
            // We can't requeue this, it's a critical driver-level error
            return false;
        }
        
        Buffer& cur = buffers_[bufferIndex];
        
        // --- FIX 1 (Starvation): Set state to IN_USE *after* dequeuing ---
        cur.state = IN_USE; 

        // Empty frame ? requeue immediately
        if (buf.bytesused == 0) {
            spdlog::debug("[DataConcrete] Empty frame (idx={}), re-queue.", bufferIndex);
            // This is safe: queueBufferInternal expects IN_USE and sets to QUEUED
            queueBufferInternal(bufferIndex); 
            return true; // Successfully handled, continue capture loop
        }

        // --- Zero-copy guard ---
        auto bufferGuard = std::shared_ptr<void>(cur.start, [this, bufferIndex](void*) {
            this->queueBuffer(bufferIndex);
        });

        // --- Timestamps ---
        std::chrono::steady_clock::time_point capMono;
        //std::chrono::system_clock::time_point capSys; // Declared outside
        const bool haveDriverTs = (buf.timestamp.tv_sec || buf.timestamp.tv_usec);
        if (usesMonotonicTs_ && haveDriverTs && basesInitialized_) {
            auto mono_ns = std::chrono::seconds(buf.timestamp.tv_sec)
                         + std::chrono::microseconds(buf.timestamp.tv_usec);
            capMono = std::chrono::steady_clock::time_point(
                std::chrono::duration_cast<std::chrono::steady_clock::duration>(mono_ns));
            capSys = std::chrono::system_clock::time_point(capMono.time_since_epoch() + sysMinusMono_);
        } else {
            capMono = std::chrono::steady_clock::now();
            capSys  = std::chrono::system_clock::now();
        }

        // --- Frame ID ---
        const uint64_t frameId = frameCounter_.fetch_add(1, std::memory_order_relaxed);

        // --- Build ZeroCopyFrameData ---
        frame = std::make_shared<ZeroCopyFrameData>( // Assign to outer scope var
            bufferGuard,
            cur.start,
            buf.bytesused,
            cameraConfig_.width,
            cameraConfig_.height,
            bufferIndex,
            frameId,
            capSys,
            capMono
        );
        
        frame_valid = frame->isValid();
        if (!frame_valid) {
            spdlog::warn("[DataConcrete] Invalid frame (id={}), dropping.", frameId);
            // Frame will be destroyed outside lock, deleter will run safely
            return true; 
        }

        // --- FPS measurement (for logging & metrics) ---
        double fpsMeasured = cameraConfig_.fps > 0 ? static_cast<double>(cameraConfig_.fps) : 30.0;
        if (prevCaptureTsSteady_.time_since_epoch().count() != 0) {
            auto dt_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                            capMono - prevCaptureTsSteady_).count();
            if (dt_ms > 0) fpsMeasured = 1000.0 / dt_ms;
        }
        prevCaptureTsSteady_ = capMono;

        // --- Prepare CameraStats struct (will be used outside lock) ---
        cs.timestamp    = capSys;
        cs.frameNumber  = frameId;
        cs.fps          = fpsMeasured;
        cs.frameWidth   = cameraConfig_.width;
        cs.frameHeight  = cameraConfig_.height;
        cs.frameSize    = buf.bytesused;

        // --- Push to downstream queues (with back-pressure) ---
        const size_t max_q_size = cameraConfig_.numBuffers * 2;
        pushed = true; // Assume pushed

        if (displayOrigQueue_ && displayOrigQueue_->size() < max_q_size)
            displayOrigQueue_->push(frame);
        else if (displayOrigQueue_)
            pushed = false, spdlog::warn("[DataConcrete] Display queue full (frame {})", frameId);

        if (algoQueue_ && algoQueue_->size() < max_q_size)
            algoQueue_->push(frame);
        else if (algoQueue_)
            pushed = false, spdlog::warn("[DataConcrete] Algo queue full (frame {})", frameId);

    } // --- End Critical Section: mutex_ is released ---

    // --- FIX 2 (Deadlock): Moved aggregator calls outside the lock ---
    // These calls can be slow and must not be inside the camera's main mutex.
    if (frame_valid && metricAggregator_) {
        spdlog::info("[DebugID] beginFrame frameId={}", cs.frameNumber);
        metricAggregator_->beginFrame(cs.frameNumber, cs);

        metricAggregator_->overlayStats("camera", {
            {"FPS", cs.fps},
            {"Width", static_cast<double>(cs.frameWidth)},
            {"Height", static_cast<double>(cs.frameHeight)},
            {"Size", static_cast<double>(cs.frameSize)}
        }, cs.timestamp); // Use capSys timestamp
    }

    // `frame` (and its bufferGuard) is destroyed here if !pushed. 
    // The deleter runs, calls queueBuffer(), and safely locks the mutex.
    return pushed;
}
//==============================================================================================================



// inline bool DataConcrete::queueBuffer(size_t bufferIndex) {
//     std::lock_guard<std::mutex> lock(mutex_);
//     if (bufferIndex >= buffers_.size()) {
//         reportError(fmt::format("Invalid buffer index: {}", bufferIndex));
//         return false;
//     }
//     return queueBufferInternal(bufferIndex);
// }


inline bool DataConcrete::queueBuffer(size_t bufferIndex) {
    // This is the PUBLIC, thread-safe method for requeuing.
    std::lock_guard<std::mutex> lock(mutex_);
    return queueBufferInternal(bufferIndex);
}

// inline bool DataConcrete::queueBufferInternal(size_t index) {
//     try {
//         if (index >= buffers_.size()) {
//             reportError(fmt::format("Index out of range: {}", index));
//             return false;
//         }
//         auto& buffer = buffers_[index];
//         if (buffer.state == QUEUED) {
//             spdlog::warn("[DataConcrete] Buffer {} already QUEUED.", index);
//             return true;
//         }

//         struct v4l2_buffer buf = {};
//         buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
//         buf.memory = V4L2_MEMORY_MMAP;
//         buf.index  = index;

//         if (ioctl(fd_, VIDIOC_QBUF, &buf) < 0) {
//             reportError(fmt::format("VIDIOC_QBUF failed for buffer {}: {}", index, std::strerror(errno)));
//             return false;
//         }
//         buffer.state = QUEUED;
//         return true;
//     } catch (const std::exception& e) {
//         reportError(fmt::format("Exception in queueBufferInternal for index {}: {}", index, e.what()));
//         return false;
//     }
// }


inline bool DataConcrete::queueBufferInternal(size_t index) {
    if (index >= buffers_.size()) return false;
    
    auto& buffer = buffers_[index];
    // [MOD 09-22-2025] Only queue if it's currently in use. This prevents double-queuing.
    if (buffer.state != IN_USE) {
        //spdlog::warn("[DataConcrete] Attempted to queue buffer {} which is not in use (state is {}).", index, buffer.state);
        spdlog::error("[DataConcrete] Attempted to queue buffer {} which is not in use (state is {}).",
              index, static_cast<int>(buffer.state));

        return true; 
    }

    struct v4l2_buffer buf = {};
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = index;
    if (ioctl(fd_, VIDIOC_QBUF, &buf) < 0) {
        reportError(fmt::format("VIDIOC_QBUF failed for buffer {}: {}", index, std::strerror(errno)));
        return false;
    }
    // [MOD 09-22-2025] Update state to show the buffer has been returned to the driver.
    buffer.state = QUEUED;
    return true;
}

// --- Other helpers ---
inline bool DataConcrete::internalStartStreaming() {
    for (size_t i = 0; i < buffers_.size(); ++i) {
        buffers_[i].state = IN_USE; // Temporarily mark as IN_USE to allow initial queuing
        if (!queueBufferInternal(i)) return false;
    }
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(fd_, VIDIOC_STREAMON, &type) < 0) {
        reportError(fmt::format("VIDIOC_STREAMON failed: {}", std::strerror(errno)));
        return false;
    }
    // ... timestamp base initialization ...
    return true;
}

inline bool DataConcrete::internalStopStreaming() {
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(fd_, VIDIOC_STREAMOFF, &type) < 0) {
        reportError(fmt::format("VIDIOC_STREAMOFF failed: {}", std::strerror(errno)));
        return false;
    }
    return true;
}

inline void DataConcrete::unmapBuffers() {
    for (auto& buf : buffers_) {
        if (buf.start && buf.start != MAP_FAILED) {
            munmap(buf.start, buf.length);
        }
        buf.start  = nullptr;
        buf.length = 0;
        buf.state  = AVAILABLE;
    }
    buffers_.clear();
    spdlog::debug("[DataConcrete] All buffers unmapped and cleared.");
}

inline void DataConcrete::reportError(const std::string& msg) {
    if (errorCallback_) errorCallback_(msg);
    else spdlog::error("[DataConcrete] {}", msg);
}

inline void DataConcrete::setErrorCallback(std::function<void(const std::string&)> callback) {
    std::lock_guard<std::mutex> lock(mutex_);
    errorCallback_ = std::move(callback);
}

inline bool DataConcrete::isStreaming() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return streaming_;
}

inline void DataConcrete::pushCameraMetrics() {
    // Intentionally blank (legacy logger unused)
}

inline double DataConcrete::getLastFPS() {
    std::lock_guard<std::mutex> lock(fpsMutex_);
    auto now = std::chrono::steady_clock::now();
    double elapsed = std::chrono::duration<double>(now - lastUpdateTime_).count();

    if (elapsed > 1.0) {
        double fps = frameCountInSecond_ / elapsed;
        int count = framesDequeued_.exchange(0);
        lastFPS_ = static_cast<double>(count) / elapsed;
        lastUpdateTime_ = now;
        frameCountInSecond_ = 0;
        lastFpsTime_ = now;
        return fps;
    }
    return lastFPS_;
}

inline int DataConcrete::getQueueSize() const {
    return framesQueued_.load();
}

inline void DataConcrete::closeDevice() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (fd_ == -1) return;
    unmapBuffers();
    ::close(fd_);
    fd_ = -1;
    spdlog::info("[DataConcrete] Device closed.");
}

inline void DataConcrete::resetDevice() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (streaming_) stopStreaming();
    closeDevice();
    spdlog::info("[DataConcrete] Device reset.");
}