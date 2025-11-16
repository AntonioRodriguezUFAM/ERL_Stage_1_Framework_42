//====================================================================================================
//  SystemMetricsAggregatorConcrete_v3_2.h
//====================================================================================================

//===============================================================================================================


// NEW PROPOSAL CODE WITH COMPILING ERROR !! ======================================
//====================================================================================================

#pragma once
// Set the active level to debug before including spdlog
#define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_INFO

#include <unordered_map>
#include <fstream>
#include <mutex>
#include <iomanip>
#include <sstream>
#include <condition_variable>
#include <deque>
#include <chrono>
#include <thread>
#include <algorithm>    // For std::remove
#include <ctime>        // For std::tm, localtime_r/localtime_s
#include <atomic>       // For std::atomic
#include <vector>       // For std::vector
#include <iterator>     // For std::make_move_iterator
#include <numeric>   // std::accumulate
#include <cmath>     // std::llabs (or cstdlib)

#include <cmath>     // math
#include <cstdlib>   // std::llabs
#include <functional> // std::function, std::reference_wrapper
#include <utility>    // std::pair
#include <cstdint>    // uint64_t


//#include <filesystem>
#include <experimental/filesystem>

#include "../Interfaces/ISystemMetricsAggregator.h"
#include "../SharedStructures/allModulesStatcs.h"  // Corrected filename
#include "../SharedStructures/AggregatorConfig.h"
#include "../Others/utils.h" // for formatTimestamp

#include "../nlohmann/json.hpp"
#include <spdlog/spdlog.h>

namespace fs = std::experimental::filesystem;
using namespace std::chrono;
using json = nlohmann::json;
namespace {
template<typename T>
T clamp(T value, T min, T max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}
} // namespace

/**
 * @class SystemMetricsAggregatorConcreteV3_2
 * @brief Frame-centric metrics aggregator with thread-safe operations and CSV/JSON export.
 * Camera thread calls beginFrame, algorithm thread calls mergeAlgorithm, and display thread
 * calls mergeDisplay to flush a complete snapshot. SoC and power threads push data
 * independently, with the latest values attached to each frame via overlayStats.
 * 
 * Note: Producers should avoid calling merge* or finalizeFrame after stop() is called or the
 * aggregator's destructor begins. Call stop() and join producer threads before destruction
 * for thread-safe shutdown.
 **/

class SystemMetricsAggregatorConcreteV3_2 : public ISystemMetricsAggregator {
private:
    struct PendingFrame {
        CameraStats cam;
        AlgorithmStats alg;
        DisplayStats disp;
        JetsonNanoInfo soc;
        PowerStats power;
        std::chrono::steady_clock::time_point arrived = std::chrono::steady_clock::now();
        bool hasCam = false;
        bool hasAlg = false;
        bool hasDisp = false;
        bool hasSoC = false;
        bool hasPower = false;

        // --- ADD THESE FIELDS to store the time window ---
        std::chrono::system_clock::time_point algStartTime;
        std::chrono::system_clock::time_point algEndTime;
        bool hasAlgTime = false;
        // --- END ADD ---

        std::unordered_map<std::string, double> overlays; // Store overlay data
    };

    struct PendingOverlay {
        std::chrono::system_clock::time_point ts;
        std::string module;
        std::unordered_map<std::string, double> kv;
    };

    // In SystemMetricsAggregatorConcreteV3_2.h, before the constructor
public:
    // Constructor with configuration
    

//======================================================================================================================
explicit SystemMetricsAggregatorConcreteV3_2(const json& config)
    : csvPath_(config.value("metrics_csv", "metrics_csv/realtime_metrics_010.csv")),
      jsonPath_(config.value("metrics_json", "metrics_json/realtime_metrics010.ndjson")),
      retentionWindow_(seconds(config.value("retention_window_sec", 2000))),
      pruneMaxAge_(seconds(std::max(1, config.value("prune_max_age_sec", 5)))),
      maxPendingFrames_(config.value("max_pending_frames", 2000)),
      maxHistorySize_(config.value("max_history_size", 10000)),
      csvHeaderWritten_(false),
      mergeWaitMs_(clamp(config.value("merge_wait_ms", 2000), 50, 5000)),
      flushPeriodMs_(clamp(config.value("flush_period_ms", 1000), 100, 10000)),
      flushThreshold_(std::max(1, config.value("json_flush_threshold", 2000))),
      dropEmptyCompat_(config.value("drop_empty_compat_rows", true)),
      dropEmptyOnFlush_(config.value("drop_empty_flush_rows", true)),
      aggConfig_(parseConfig(config)) {

    // [MOD] Validate size limits early
    if (maxPendingFrames_ <= 0 || maxHistorySize_ <= 0) {
        throw std::invalid_argument("[Aggregator] maxPendingFrames_ and maxHistorySize_ must be > 0");
    }
    if (retentionWindow_ < pruneMaxAge_) {
        spdlog::warn("[Aggregator] retention_window_sec < prune_max_age_sec; bumping retention to match prune");
        retentionWindow_ = pruneMaxAge_;
    }

    // [MOD] Helper: Convert relative ? absolute + sanitize
    auto make_absolute = [](const std::string& path) -> std::string {
        if (path.empty()) {
            spdlog::warn("[Aggregator] Path is empty, using default: /tmp/realtime_metrics.csv");
            return "/tmp/realtime_metrics.csv";
        }
        try {
            fs::path p(path);
            if (p.is_relative()) {
                fs::path abs_path = fs::absolute(p);
                spdlog::debug("[Aggregator] Converted relative path '{}' ? absolute: '{}'", path, abs_path.string());
                return abs_path.string();
            }
            return path;
        } catch (const fs::filesystem_error& e) {
            spdlog::error("[Aggregator] Invalid path '{}': {}. Using default: /tmp/realtime_metrics.csv", path, e.what());
            return "/tmp/realtime_metrics.csv";
        } catch (...) {
            spdlog::error("[Aggregator] Unexpected error processing path '{}'. Using default.", path);
            return "/tmp/realtime_metrics.csv";
        }
    };

    // [MOD] Convert paths to absolute
    csvPath_ = make_absolute(csvPath_);
    jsonPath_ = make_absolute(jsonPath_);

    // [MOD] Helper: Ensure directory + file is writable, fallback to /tmp
    auto ensure_writable = [this](std::string& path, const std::string& default_path, const std::string& type) -> void {
        if (path.empty()) {
            path = default_path;
            spdlog::warn("[Aggregator] {} path empty, using default: {}", type, path);
        }

        fs::path p(path);
        fs::path parent = p.parent_path();

        // Create parent directory
        if (!parent.empty()) {
            try {
                if (!fs::exists(parent)) {
                    spdlog::debug("[Aggregator] Creating {} directory: '{}'", type, parent.string());
                    fs::create_directories(parent);
                }
            } catch (const std::exception& e) {
                spdlog::error("[Aggregator] Failed to create {} directory '{}': {}. Falling back to /tmp", type, parent.string(), e.what());
                path = default_path;
                p = fs::path(path);
                parent = p.parent_path();
                fs::create_directories(parent); // Safe fallback
            }
        }

        // [MOD] Test file writability
        {
            std::ofstream test(p.string(), std::ios::out | std::ios::app);
            if (!test) {
                spdlog::error("[Aggregator] Cannot write to {} file '{}'. Falling back to {}", type, path, default_path);
                path = default_path;
                p = fs::path(path);
                parent = p.parent_path();
                fs::create_directories(parent);
            } else {
                test.close();
            }
        }

        // [MOD] Removed fs::perm_options for compatibility with std::experimental::filesystem
        // Default permissions (rwxr-xr-x) set by create_directories are sufficient
        spdlog::info("[Aggregator] {} path resolved to: '{}'", type, path);
    };

    // [MOD] Apply writability check + fallback
    ensure_writable(csvPath_,  "/tmp/realtime_metrics_010.csv",  "CSV");
    ensure_writable(jsonPath_, "/tmp/realtime_metrics010.ndjson", "JSON");

    // [MOD] Open files (now guaranteed writable)
    ofs_.open(csvPath_, std::ios::out | std::ios::app);
    if (!ofs_) {
        spdlog::error("[Aggregator] [FATAL] Failed to open CSV file even after fallback: {}", csvPath_);
        throw std::runtime_error("Failed to open CSV file");
    }

    jofs_.open(jsonPath_, std::ios::out | std::ios::app);
    if (!jofs_) {
        spdlog::error("[Aggregator] [FATAL] Failed to open JSON file even after fallback: {}", jsonPath_);
        throw std::runtime_error("Failed to open JSON file");
    }

    // [MOD] Start flush thread
    flushThread_ = std::thread([this]() {
        while (!stopping_) {
            std::this_thread::sleep_for(milliseconds(flushPeriodMs_));
            if (!stopping_) flushBatchBuffer();
        }
    });

    // [MOD] Final config log
    spdlog::info("[Aggregator] cfg: expectsPower={}, expectsCamera={}, expectsAlgorithm={}, expectsDisplay={}, expectsSoC={}, "
                 "merge_wait_ms={}ms, flush_period_ms={}ms, retention={}s, prune_max={}s, "
                 "dropEmptyCompat={}, dropEmptyOnFlush={}, json_flush_threshold={} flushes, csv='{}', json='{}'",
                 aggConfig_.expectsPower, aggConfig_.expectsCamera, aggConfig_.expectsAlgorithm,
                 aggConfig_.expectsDisplay, aggConfig_.expectsSoC,
                 mergeWaitMs_, flushPeriodMs_,
                 duration_cast<seconds>(retentionWindow_).count(),
                 duration_cast<seconds>(pruneMaxAge_).count(),
                 dropEmptyCompat_, dropEmptyOnFlush_, flushThreshold_,
                 csvPath_, jsonPath_);
}
//======================================================================================================================

  // 
  // Method to update AggregatorConfig
    void updateConfig(const AggregatorConfig& newConfig) {
        if (aggConfig_.expectsPower != newConfig.expectsPower) {
            aggConfig_ = newConfig;
            spdlog::info("[Aggregator] Updated expectsPower to: {}", aggConfig_.expectsPower);
            // Optional: Trigger reconfiguration of metrics collection
        }
    }


const AggregatorConfig& getConfig() const { return aggConfig_; }


// Deconstructor        
    // ~SystemMetricsAggregatorConcreteV3_2() override {
    //     stop();
    //     forceFlushBatch();
    //     if (ofs_.is_open()) ofs_.close();
    //     if (jofs_.is_open()) jofs_.close();
    // }

    // In destructor
    
~SystemMetricsAggregatorConcreteV3_2() override {
    stop();
    if (flushThread_.joinable()) flushThread_.join();
    forceFlushBatch();
    if (ofs_.is_open()) ofs_.close();
    if (jofs_.is_open()) jofs_.close();
}

//==========================================================================================================
void forceFlushBatch() {
    std::lock_guard<std::mutex> lock(batchMutex_);
    //flushBatchBuffer();
    if (!batchBuffer_.empty()) {
        flushBatchBuffer();
    }
}

//=================================================================================================
    void beginFrame(uint64_t frameId, const CameraStats& stats) override {
        if (stopping_) return;
        CameraStats s = stats;

        if (isAncientTs(s.timestamp)) {
            s.timestamp = std::chrono::system_clock::now();
            spdlog::warn("[Aggregator] beginFrame({}): epoch/bad camera timestamp -> fixed to now()", frameId);
        }

        const bool dimsMissing = (s.frameWidth == 0 || s.frameHeight == 0 || s.frameSize == 0);
        if (dimsMissing || s.fps < 1.0) {
            spdlog::warn("[Aggregator] beginFrame({}): low FPS ({}) or missing dims; accepting partial frame for CSV", frameId, s.fps);
        }

        std::unique_lock<std::mutex> lock(mutex_);
       
        // OLD (C++17)
       // auto [it, inserted] = pending_.emplace(frameId, PendingFrame{});

        // NEW (C++11)
        std::pair<std::unordered_map<uint64_t, PendingFrame>::iterator, bool> ins =
            pending_.emplace(frameId, PendingFrame());
        auto it = ins.first;
        bool inserted = ins.second;


        auto& pf = it->second;
        if (inserted) arrivalOrder_.push_back(frameId);
        pf.cam = s;
        pf.hasCam = true;
        pf.arrived = std::chrono::steady_clock::now();
        frameStarted_[frameId] = true;
        lock.unlock();
        cv_.notify_all();

        auto now = std::chrono::steady_clock::now();
        std::unique_lock<std::mutex> pruneLock(mutex_);
        while (!arrivalOrder_.empty()) {
            const auto oldestId = arrivalOrder_.front();
            auto it = pending_.find(oldestId);
            if (it == pending_.end()) {
                arrivalOrder_.pop_front();
                continue;
            }
            if ((now - it->second.arrived) > pruneMaxAge_) {
                spdlog::debug("[Aggregator] Pruned stale frame {} (age > {}s)", oldestId, pruneMaxAge_.count());
                frameStarted_.erase(oldestId);
                pending_.erase(it);
                arrivalOrder_.pop_front();
            } else {
                break;
            }
        }
        while (pending_.size() > maxPendingFrames_ && !arrivalOrder_.empty()) {
            uint64_t oldestId = arrivalOrder_.front();
            spdlog::debug("[Aggregator] Pruned oldest frame {} (size limit exceeded: {} > {})",
                          oldestId, pending_.size(), maxPendingFrames_);
            frameStarted_.erase(oldestId);
            pending_.erase(oldestId);
            arrivalOrder_.pop_front();
        }
        spdlog::debug("[Aggregator] beginFrame({}, timestamp={})", frameId, utils::formatTimestamp(stats.timestamp));
        // New
 //       spdlog::debug("[Aggregator] Received frame {} with FPS {}", frameId, stats.fps);
    }

    void mergeAlgorithm(uint64_t frameId, const AlgorithmStats& stats) override {
        if (stopping_) return;
        std::unique_lock<std::mutex> lock(mutex_);

        cv_.wait_for(lock, std::chrono::milliseconds(mergeWaitMs_),
                     [this, frameId] { return frameStarted_.count(frameId) && frameStarted_[frameId]; });
        if (!frameStarted_.count(frameId) || !frameStarted_[frameId]) {
            spdlog::warn("[Aggregator] mergeAlgorithm({}): beginFrame not called, proceeding with partial data", frameId);
        }

        // OLD (C++17)
       // auto [it, inserted] = pending_.emplace(frameId, PendingFrame{});

        // NEW (C++11)
        std::pair<std::unordered_map<uint64_t, PendingFrame>::iterator, bool> ins =
            pending_.emplace(frameId, PendingFrame());
        auto it = ins.first;
        bool inserted = ins.second;


        if (inserted) arrivalOrder_.push_back(frameId);
        it->second.alg = stats;
        it->second.hasAlg = true;

        // --- NEW LOGIC: Store the algorithm's execution window ---
        if (stats.startTime.time_since_epoch().count() > 0 &&
            stats.timestamp.time_since_epoch().count() > 0) {
            it->second.algStartTime = stats.startTime;
            it->second.algEndTime = stats.timestamp; // 'timestamp' is the end time
            it->second.hasAlgTime = true;
        }
        // --- END NEW LOGIC ---
        spdlog::debug("[Aggregator] mergeAlgorithm({}): merged AlgorithmStats", frameId);
    }


//================================================================================
// --- REPLACE mergeDisplay ---
    // This is the correct version from our previous discussion,
    // but now it will call the new integration functions.
    void mergeDisplay(uint64_t frameId, const DisplayStats& stats) override {
        if (stopping_) return;

        PendingFrame pf_to_finalize;
        bool should_finalize = false;

        { 
            std::unique_lock<std::mutex> lock(mutex_); // Lock for pending_ map
            
            cv_.wait_for(lock, std::chrono::milliseconds(mergeWaitMs_),
                         [this, frameId] { return frameStarted_.count(frameId) > 0; });

            auto it = pending_.find(frameId);
            if (it == pending_.end()) {
                spdlog::warn("[Aggregator] mergeDisplay({}): Frame not found. Discarding.", frameId);
                return;
            }

            it->second.disp = stats;
            it->second.hasDisp = true;
            
            // Lock the asynchronous data history
            // DELETE this lock in mergeDisplay:
            // std::lock_guard<std::mutex> asyncLock(asyncDataMutex_);


            // Check if we have a valid time window from the algorithm
            if (it->second.hasAlgTime) {
                // YES: Call the integration functions
                it->second.soc = integrateSoCInWindow(it->second.algStartTime, it->second.algEndTime);
                it->second.power = integratePowerInWindow(it->second.algStartTime, it->second.algEndTime);
            } else {
                // NO: Fall back to the "latest" sample (less accurate)
                if (!socHistory_.empty()) it->second.soc = socHistory_.back();
                if (!powerHistory_.empty()) it->second.power = powerHistory_.back();
            }
            it->second.hasSoC = true;
            it->second.hasPower = true;

            // Mark for finalization
            should_finalize = true;
            pf_to_finalize = std::move(it->second);
            
            pending_.erase(it);
            frameStarted_.erase(frameId);
            arrivalOrder_.erase(std::remove(arrivalOrder_.begin(), arrivalOrder_.end(), frameId), arrivalOrder_.end());
        } 

        if (should_finalize) {
            finalizeFrame(frameId, std::move(pf_to_finalize));
        }
    }
// ====================== WORKING CODE ======================================0

    //=================================================================================================================

    
    void mergeSoC(uint64_t frameId, const JetsonNanoInfo& stats) override {
    if (stopping_) return;
    PendingFrame pf_to_finalize;
    bool should_finalize = false;

    { // Scoped lock
        std::unique_lock<std::mutex> lock(mutex_);
        cv_.wait_for(lock, std::chrono::milliseconds(mergeWaitMs_),
                     [this, frameId] { return frameStarted_.count(frameId) > 0; });

        if (frameStarted_.count(frameId) == 0) {
            spdlog::warn("[Aggregator] mergeSoC({}): beginFrame not called, proceeding with partial data", frameId);
        }

        
        // OLD (C++17)
        //auto [it, inserted] = pending_.emplace(frameId, PendingFrame{});

        // NEW (C++11)
        std::pair<std::unordered_map<uint64_t, PendingFrame>::iterator, bool> ins =
            pending_.emplace(frameId, PendingFrame());
        auto it = ins.first;
        bool inserted = ins.second;


        if (inserted) arrivalOrder_.push_back(frameId);
        it->second.soc = stats;
        it->second.hasSoC = true;
        spdlog::debug("[Aggregator] mergeSoC({}): merged SoCStats", frameId);

        // Finalize ONLY if display is NOT expected AND power is NOT expected

        // Check if this is the final step
       // if (!aggConfig_.expectsPower) {
        if (!aggConfig_.expectsDisplay && !aggConfig_.expectsPower) {
            should_finalize = true;
            pf_to_finalize = std::move(it->second);
            pending_.erase(it);
            frameStarted_.erase(frameId);
            arrivalOrder_.erase(std::remove(arrivalOrder_.begin(), arrivalOrder_.end(), frameId), arrivalOrder_.end());
        }
    } // Lock released here

    if (should_finalize) {
        finalizeFrame(frameId, std::move(pf_to_finalize));
    }
    spdlog::debug("[Aggregator] mergeSoC({}): merged SoCStats", frameId);
}

    

    //=====================================================================================================
    void mergePower(uint64_t frameId, const PowerStats& stats) override {
    if (stopping_ || !aggConfig_.expectsPower) return;

    PendingFrame pf_to_finalize;
    bool should_finalize = false;

    { // Scoped lock
        std::unique_lock<std::mutex> lock(mutex_);
        cv_.wait_for(lock, std::chrono::milliseconds(mergeWaitMs_),
                     [this, frameId] { return frameStarted_.count(frameId) > 0; });

        if (frameStarted_.count(frameId) == 0) {
            spdlog::warn("[Aggregator] mergePower({}): beginFrame not called, proceeding with partial data", frameId);
        }
        
        // OLD (C++17)
        //auto [it, inserted] = pending_.emplace(frameId, PendingFrame{});

        // NEW (C++11)
        std::pair<std::unordered_map<uint64_t, PendingFrame>::iterator, bool> ins =
            pending_.emplace(frameId, PendingFrame());
        auto it = ins.first;
        bool inserted = ins.second;

        
        if (inserted) arrivalOrder_.push_back(frameId);
        it->second.power = stats;
        it->second.hasPower = true;
        spdlog::debug("[Aggregator] mergePower({}): merged PowerStats", frameId);
        
        // Power is the last step, so we always finalize here.
         // Finalize ONLY if display is NOT expected
        if (!aggConfig_.expectsDisplay) {
        should_finalize = true;
        pf_to_finalize = std::move(it->second);
        pending_.erase(it);
        frameStarted_.erase(frameId);
        arrivalOrder_.erase(std::remove(arrivalOrder_.begin(), arrivalOrder_.end(), frameId), arrivalOrder_.end());
        }
    } // Lock released here

    if (should_finalize) {
        // Attach latest SoC if it was missed
        if (!pf_to_finalize.hasSoC && latestSoC_.isValid()) {
            pf_to_finalize.soc = latestSoC_;
            pf_to_finalize.hasSoC = true;
        }
        finalizeFrame(frameId, std::move(pf_to_finalize));
    }
    spdlog::debug("[Aggregator] mergePower({}): merged PowerStats", frameId);
}
    //===========================================================================================================

//===============================================================================
// --- REPLACE pushSoCStats / pushPowerStats ---
    void pushSoCStats(const JetsonNanoInfo& stats) override {
        if (stopping_) return;
        std::lock_guard<std::mutex> lock(asyncDataMutex_); // Use new mutex
        socHistory_.push_back(stats);
        // Prune old data (e.g., keep 5 seconds)
        auto cutoff = stats.timestamp - asyncHistoryDuration_;
        while (!socHistory_.empty() && socHistory_.front().timestamp < cutoff) {
            socHistory_.pop_front();
        }
    }

    void pushPowerStats(const PowerStats& stats) override {
        if (stopping_) return;
        std::lock_guard<std::mutex> lock(asyncDataMutex_); // Use new mutex
        powerHistory_.push_back(stats);
        // Prune old data
        auto cutoff = stats.timestamp - asyncHistoryDuration_;
        while (!powerHistory_.empty() && powerHistory_.front().timestamp < cutoff) {
            powerHistory_.pop_front();
        }
    }
//==================================================================================


    void overlayStats(const std::string& module,
                      const std::unordered_map<std::string, double>& kv,
                      const std::chrono::system_clock::time_point& ts)  {
        std::lock_guard<std::mutex> lock(overlay_mtx_);
        overlay_q_.push_back(PendingOverlay{ts, module, kv});
    }

    void finalizeFrame(uint64_t id, PendingFrame&& pf) {
        if (!(pf.hasCam || pf.hasAlg || pf.hasDisp || pf.hasSoC || pf.hasPower)) {
            spdlog::warn("[Aggregator] finalizeFrame({}): nothing to write", id);
            return;
        }
        SystemMetricsSnapshot snap(pf.hasCam ? pf.cam.timestamp : std::chrono::system_clock::now());
        snap.frameId = pf.hasCam ? pf.cam.frameNumber : id;
        if (pf.hasCam) snap.cameraStats = pf.cam;
        if (pf.hasAlg) snap.algorithmStats = pf.alg;
        if (pf.hasDisp) snap.displayStats = pf.disp;
        if (pf.hasSoC) snap.socInfo = pf.soc;
        if (pf.hasPower) snap.powerStats = pf.power;

        // Merge overlay data into the snapshot
        for (const auto& [key, value] : pf.overlays) {
            // Camera fields
            if (key == "FrameNumber" && pf.hasCam) snap.cameraStats.frameNumber = static_cast<uint64_t>(value);
            if (key == "FPS" && pf.hasCam) snap.cameraStats.fps = value;
            if (key == "FrameWidth" && pf.hasCam) snap.cameraStats.frameWidth = static_cast<uint32_t>(value);
            if (key == "FrameHeight" && pf.hasCam) snap.cameraStats.frameHeight = static_cast<uint32_t>(value);
            if (key == "FrameSize" && pf.hasCam) snap.cameraStats.frameSize = static_cast<uint64_t>(value);

            // Algorithm fields
            if (key == "InferenceTimeMs" && pf.hasAlg) snap.algorithmStats.inferenceTimeMs = value;
            if (key == "ConfidenceScore" && pf.hasAlg) snap.algorithmStats.confidenceScore = value;
            if (key == "AlgorithmFPS" && pf.hasAlg) snap.algorithmStats.fps = value;
            if (key == "AvgProcTimeMs" && pf.hasAlg) snap.algorithmStats.avgProcTimeMs = value;
            if (key == "TotalProcTimeMs" && pf.hasAlg) snap.algorithmStats.totalProcTimeMs = value;
            if (key == "CudaKernelTimeMs" && pf.hasAlg) snap.algorithmStats.cudaKernelTimeMs = value;

            // Display fields
            if (key == "DisplayLatencyMsRaw" && pf.hasDisp) snap.displayStats.latencyMs = value;
            if (key == "DroppedFrames" && pf.hasDisp) snap.displayStats.droppedFrames = static_cast<uint32_t>(value);
            if (key == "RenderTimeMs" && pf.hasDisp) snap.displayStats.renderTimeMs = value;

            // Derived fields
            if (key == "ProcessingLatencyMs") snap.processingLatencyMs = value;
            if (key == "DisplayLatencyMs") snap.displayLatencyMs = value;
            if (key == "EndToEndLatencyMs") snap.endToEndLatencyMs = value;
            if (key == "JoulesPerFrame") snap.joulesPerFrame = value;

            // SoC fields
            if (key == "RAM_In_Use_MB" && pf.hasSoC) snap.socInfo.RAM_In_Use_MB = value;
            if (key == "Total_RAM_MB" && pf.hasSoC) snap.socInfo.Total_RAM_MB = value;
            if (key == "LFB_Size_MB" && pf.hasSoC) snap.socInfo.LFB_Size_MB = value;
            if (key == "Block_Max_MB" && pf.hasSoC) snap.socInfo.Block_Max_MB = value;
            if (key == "SWAP_In_Use_MB" && pf.hasSoC) snap.socInfo.SWAP_In_Use_MB = value;
            if (key == "Total_SWAP_MB" && pf.hasSoC) snap.socInfo.Total_SWAP_MB = value;
            if (key == "Cached_MB" && pf.hasSoC) snap.socInfo.Cached_MB = value;
            if (key == "used_IRAM_kB" && pf.hasSoC) snap.socInfo.used_IRAM_kB = value;
            if (key == "total_IRAM_kB" && pf.hasSoC) snap.socInfo.total_IRAM_kB = value;
            if (key == "lfb_kB" && pf.hasSoC) snap.socInfo.lfb_kB = value;
            if (key == "CPU1_Utilization_Percent" && pf.hasSoC) snap.socInfo.CPU1_Utilization_Percent = value;
            if (key == "CPU1_Frequency_MHz" && pf.hasSoC) snap.socInfo.CPU1_Frequency_MHz = value;
            if (key == "CPU2_Utilization_Percent" && pf.hasSoC) snap.socInfo.CPU2_Utilization_Percent = value;
            if (key == "CPU2_Frequency_MHz" && pf.hasSoC) snap.socInfo.CPU2_Frequency_MHz = value;
            if (key == "CPU3_Utilization_Percent" && pf.hasSoC) snap.socInfo.CPU3_Utilization_Percent = value;
            if (key == "CPU3_Frequency_MHz" && pf.hasSoC) snap.socInfo.CPU3_Frequency_MHz = value;
            if (key == "CPU4_Utilization_Percent" && pf.hasSoC) snap.socInfo.CPU4_Utilization_Percent = value;
            if (key == "CPU4_Frequency_MHz" && pf.hasSoC) snap.socInfo.CPU4_Frequency_MHz = value;
            if (key == "EMC_Frequency_Percent" && pf.hasSoC) snap.socInfo.EMC_Frequency_Percent = value;
            if (key == "GR3D_Frequency_Percent" && pf.hasSoC) snap.socInfo.GR3D_Frequency_Percent = value;
            if (key == "PLL_Temperature_C" && pf.hasSoC) snap.socInfo.PLL_Temperature_C = value;
            if (key == "CPU_Temperature_C" && pf.hasSoC) snap.socInfo.CPU_Temperature_C = value;
            if (key == "PMIC_Temperature_C" && pf.hasSoC) snap.socInfo.PMIC_Temperature_C = value;
            if (key == "GPU_Temperature_C" && pf.hasSoC) snap.socInfo.GPU_Temperature_C = value;
            if (key == "AO_Temperature_C" && pf.hasSoC) snap.socInfo.AO_Temperature_C = value;
            if (key == "Thermal_Temperature_C" && pf.hasSoC) snap.socInfo.Thermal_Temperature_C = value;

            // Power fields
            if (key == "PowerSensor0V" && pf.hasPower) snap.powerStats.voltages[0] = value;
            if (key == "PowerSensor0A" && pf.hasPower) snap.powerStats.currents[0] = value;
            if (key == "PowerSensor1V" && pf.hasPower) snap.powerStats.voltages[1] = value;
            if (key == "PowerSensor1A" && pf.hasPower) snap.powerStats.currents[1] = value;
            if (key == "PowerSensor2V" && pf.hasPower) snap.powerStats.voltages[2] = value;
            if (key == "PowerSensor2A" && pf.hasPower) snap.powerStats.currents[2] = value;
            if (key == "PowerSensor3V" && pf.hasPower) snap.powerStats.voltages[3] = value;
            if (key == "PowerSensor3A" && pf.hasPower) snap.powerStats.currents[3] = value;
            // Compute derived fields after all updates
            if (key == "PowerTotalW" && pf.hasPower) {
                auto powers = snap.powerStats.powerPerSensor();
                double total = std::accumulate(powers.begin(), powers.end(), 0.0);
                // Optionally store or log total if needed, but don't assign to totalPower()
                //snap.powerStats.totalPower () = total; // Ensure totalPower_ is set
                snap.powerStats.setTotalPower(total); // Assuming setter exists, adjust if different
            }
            if (key == "PowerAverageW" && pf.hasPower) {
                auto powers = snap.powerStats.powerPerSensor();
                double avg = powers.empty() ? 0.0 : std::accumulate(powers.begin(), powers.end(), 0.0) / powers.size();
                // Optionally store or log average if needed
                //snap.powerStats.averagePower = avg; // Ensure averagePower_ is set
                snap.powerStats.setAveragePower(avg); // Assuming setter exists, adjust if different
            }
        }

        snap.processingLatencyMs = pf.hasAlg ? std::max(0.0, pf.alg.inferenceTimeMs) : 0.0;
        snap.displayLatencyMs = pf.hasDisp ? std::max(0.0, pf.disp.renderTimeMs) : 0.0;
        if (pf.hasCam && pf.hasDisp) {
            double e2e = std::chrono::duration<double, std::milli>(pf.disp.timestamp - pf.cam.timestamp).count();
            snap.endToEndLatencyMs = std::max(0.0, e2e);
        }

        // -------- Correct Joules-per-frame --------
        // Prefer the true algorithm window if available (set in mergeAlgorithm and used in mergeDisplay)
        if (pf.hasPower && pf.hasAlgTime) {
            const double win_s = std::chrono::duration<double>(pf.algEndTime - pf.algStartTime).count();
            if (win_s > 0.0) {
                // pf.power is already the window-AVERAGED power (per-sensor + totals) after integratePowerInWindow()
                const double avg_total_W = pf.power.totalPower(); // derived in updateDerivedMetrics()
                snap.joulesPerFrame = std::max(0.0, avg_total_W * win_s);
            }
        }

        // Fallback ONLY if we couldn't compute with alg window
        if (snap.joulesPerFrame <= 0.0) {
            if (pf.hasCam && pf.hasPower && pf.cam.fps > 0.0) {
                // Legacy proxy: average total power multiplied by frame period
                snap.joulesPerFrame = pf.power.totalPower() * (1.0 / pf.cam.fps);
            } else {
                snap.joulesPerFrame = 0.0;
            }
        }
        // -------- end Joules-per-frame --------
        
        bool needsFlush = false;
        {
            std::lock_guard<std::mutex> bl(batchMutex_);
            batchBuffer_.push_back(std::move(snap));
            needsFlush = (batchBuffer_.size() >= BATCH_SIZE_LIMIT);
        }
        if (needsFlush) flushBatchBuffer();
    }

    inline void flushBatchBuffer() {
        std::vector<SystemMetricsSnapshot> snaps;
        {
            std::lock_guard<std::mutex> bl(batchMutex_);
            if (batchBuffer_.empty()) return;
             // move out
            snaps = std::move(batchBuffer_);
            batchBuffer_.clear();
        }
        // Optional extra guard: drop empties on flush as well
        if (dropEmptyOnFlush_) {
            snaps.erase(
                std::remove_if(snaps.begin(), snaps.end(),
                    [](const SystemMetricsSnapshot& s){ return !hasUsefulPayload(s); }),
                snaps.end()
            );
        }
        if (!snaps.empty()) {
            spdlog::debug("[Aggregator] flushBatchBuffer: no useful snapshots to flush after filtering");


             // Write out BEFORE moving
            for (const auto& s : snaps) {
                appendSnapshotToCSV(s);
                // appendSnapshotToJSON(s); // if you want JSON too
            }
         
          // Add to history
          // Now move into history
    
            std::lock_guard<std::mutex> lock(mutex_);
            history_.insert(history_.end(), std::make_move_iterator(snaps.begin()), std::make_move_iterator(snaps.end()));
            enforceRetentionPolicy();
    
        //   // Write out
        // for (auto& s : snaps) {
        //     appendSnapshotToCSV(s);
        //     //appendSnapshotToJSON(s);
        // }
    }
        flushCount_++;
        spdlog::debug("[Aggregator] Flushed {} snapshots", snaps.size());
    }

    inline void appendSnapshotToCSV(const SystemMetricsSnapshot& snap) {
        std::lock_guard<std::mutex> ioLock(ioMutex_);
        if (!ofs_.is_open()) {
            spdlog::error("[Aggregator] CSV file not open");
            return;
        }
        if (!csvHeaderWritten_) {
            if (ofs_.tellp() > 0) {
                csvHeaderWritten_ = true;
            } else {
                ofs_ << "Timestamp,FrameId,FrameNumber,FPS,FrameWidth,FrameHeight,FrameSize,"
                     << "InferenceTimeMs,ConfidenceScore,AlgorithmFPS,AvgProcTimeMs,TotalProcTimeMs,GPUFreeMemoryMB,GPUTotalMemoryMB,CudaKernelTimeMs,"
                     << "DisplayLatencyMsRaw,DroppedFrames,RenderTimeMs,ProcessingLatencyMs,DisplayLatencyMs,EndToEndLatencyMs,JoulesPerFrame,"
                     << "RAM_In_Use_MB,Total_RAM_MB,LFB_Size_MB,Block_Max_MB,SWAP_In_Use_MB,Total_SWAP_MB,Cached_MB,"
                     << "used_IRAM_kB,total_IRAM_kB,lfb_kB,"
                     << "CPU1_Utilization_Percent,CPU1_Frequency_MHz,CPU2_Utilization_Percent,CPU2_Frequency_MHz,"
                     << "CPU3_Utilization_Percent,CPU3_Frequency_MHz,CPU4_Utilization_Percent,CPU4_Frequency_MHz,"
                     << "EMC_Frequency_Percent,GR3D_Frequency_Percent,"
                     << "PLL_Temperature_C,CPU_Temperature_C,PMIC_Temperature_C,GPU_Temperature_C,AO_Temperature_C,Thermal_Temperature_C,"
                     << "PowerTotalW,PowerAverageW,PowerSensor0W,PowerSensor0V,PowerSensor0A,PowerSensor1W,PowerSensor1V,PowerSensor1A,"
                     << "PowerSensor2W,PowerSensor2V,PowerSensor2A,PowerSensor3W,PowerSensor3V,PowerSensor3A\n";
                csvHeaderWritten_ = true;
            }
        }
        ofs_ << utils::formatTimestamp(snap.timestamp) << "," << snap.frameId << "," << snap.cameraStats.frameNumber << ","
             << snap.cameraStats.fps << "," << snap.cameraStats.frameWidth << "," << snap.cameraStats.frameHeight << ","
             << snap.cameraStats.frameSize << "," << snap.algorithmStats.inferenceTimeMs << ","
             << snap.algorithmStats.confidenceScore << "," << snap.algorithmStats.fps << ","
             << snap.algorithmStats.avgProcTimeMs << "," << snap.algorithmStats.totalProcTimeMs << ","
             << (snap.algorithmStats.gpuFreeMemory / (1024.0 * 1024.0)) << ","
             << (snap.algorithmStats.gpuTotalMemory / (1024.0 * 1024.0)) << "," << snap.algorithmStats.cudaKernelTimeMs << ","
             << snap.displayStats.latencyMs << "," << snap.displayStats.droppedFrames << ","
             << snap.displayStats.renderTimeMs << "," << snap.processingLatencyMs << ","
             << snap.displayLatencyMs << "," << snap.endToEndLatencyMs << "," << snap.joulesPerFrame << ","
             << snap.socInfo.RAM_In_Use_MB << "," << snap.socInfo.Total_RAM_MB << "," << snap.socInfo.LFB_Size_MB << ","
             << snap.socInfo.Block_Max_MB << "," << snap.socInfo.SWAP_In_Use_MB << "," << snap.socInfo.Total_SWAP_MB << ","
             << snap.socInfo.Cached_MB << "," << snap.socInfo.used_IRAM_kB << "," << snap.socInfo.total_IRAM_kB << ","
             << snap.socInfo.lfb_kB << "," << snap.socInfo.CPU1_Utilization_Percent << "," << snap.socInfo.CPU1_Frequency_MHz << ","
             << snap.socInfo.CPU2_Utilization_Percent << "," << snap.socInfo.CPU2_Frequency_MHz << ","
             << snap.socInfo.CPU3_Utilization_Percent << "," << snap.socInfo.CPU3_Frequency_MHz << ","
             << snap.socInfo.CPU4_Utilization_Percent << "," << snap.socInfo.CPU4_Frequency_MHz << ","
             << snap.socInfo.EMC_Frequency_Percent << "," << snap.socInfo.GR3D_Frequency_Percent << ","
             << snap.socInfo.PLL_Temperature_C << "," << snap.socInfo.CPU_Temperature_C << ","
             << snap.socInfo.PMIC_Temperature_C << "," << snap.socInfo.GPU_Temperature_C << ","
             << snap.socInfo.AO_Temperature_C << "," << snap.socInfo.Thermal_Temperature_C << ","
             << snap.powerStats.totalPower() << "," << snap.powerStats.averagePower() << ","
             << snap.powerStats.sensorPower(0) << "," << (snap.powerStats.sensorCount() > 0 ? snap.powerStats.voltages[0] : 0.0) << ","
             << (snap.powerStats.sensorCount() > 0 ? snap.powerStats.currents[0] : 0.0) << ","
             << snap.powerStats.sensorPower(1) << "," << (snap.powerStats.sensorCount() > 1 ? snap.powerStats.voltages[1] : 0.0) << ","
             << (snap.powerStats.sensorCount() > 1 ? snap.powerStats.currents[1] : 0.0) << ","
             << snap.powerStats.sensorPower(2) << "," << (snap.powerStats.sensorCount() > 2 ? snap.powerStats.voltages[2] : 0.0) << ","
             << (snap.powerStats.sensorCount() > 2 ? snap.powerStats.currents[2] : 0.0) << ","
             << snap.powerStats.sensorPower(3) << "," << (snap.powerStats.sensorCount() > 3 ? snap.powerStats.voltages[3] : 0.0) << ","
             << (snap.powerStats.sensorCount() > 3 ? snap.powerStats.currents[3] : 0.0) << "\n";
        }
    

    void appendSnapshotToJSON(const SystemMetricsSnapshot& s) {
        std::lock_guard<std::mutex> ioLock(ioMutex_);
        if (!jofs_.is_open()) {
            spdlog::error("[Aggregator] JSON file not open");
            return;
        }
        jofs_ << s.toJson().dump() << "\n";
        if (++flushCount_ >= flushThreshold_) {
            jofs_.flush();  // Periodic flush for tail-ability
            flushCount_ = 0;
        }
    }

    inline void exportToJSON(const std::string& filePath) {
        forceFlushBatch();
        std::lock_guard<std::mutex> ioLock(ioMutex_);
        std::ofstream out(filePath, std::ios::trunc);
        if (!out) {
            spdlog::error("[Aggregator] exportToJSON: failed to open {}", filePath);
            return;
        }
        std::vector<SystemMetricsSnapshot> copy;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            copy = history_;
        }
        for (const auto& s : copy) out << s.toJson().dump() << "\n";
    }

    inline std::vector<SystemMetricsSnapshot> getAllSnapshots() const {
        const_cast<SystemMetricsAggregatorConcreteV3_2*>(this)->forceFlushBatch();
        std::lock_guard<std::mutex> lock(mutex_);
        return history_;
    }

    inline void pushCameraStats(const CameraStats& stats) {
        pushMetrics(std::chrono::system_clock::now(), [&](SystemMetricsSnapshot& s) { s.cameraStats = stats; });
    }

    inline void pushAlgorithmStats(const AlgorithmStats& stats) {
        pushMetrics(std::chrono::system_clock::now(), [&](SystemMetricsSnapshot& s) { s.algorithmStats = stats; });
    }

    inline void pushDisplayStats(const DisplayStats& stats) {
        pushMetrics(std::chrono::system_clock::now(), [&](SystemMetricsSnapshot& s) { s.displayStats = stats; });
    }

    inline SystemMetricsSnapshot getAggregatedAt(std::chrono::system_clock::time_point ts) const {
        const_cast<SystemMetricsAggregatorConcreteV3_2*>(this)->forceFlushBatch();
        std::lock_guard<std::mutex> lock(mutex_);
        if (history_.empty()) return SystemMetricsSnapshot(ts);

        auto it = std::min_element(history_.begin(), history_.end(),
            [&](const auto& a, const auto& b) {
                return std::llabs(std::chrono::duration_cast<std::chrono::milliseconds>(a.timestamp - ts).count()) <
                       std::llabs(std::chrono::duration_cast<std::chrono::milliseconds>(b.timestamp - ts).count());
            });
        return (it != history_.end()) ? *it : SystemMetricsSnapshot(ts);
    }

    void stop() {
        stopping_ = true;
    }
    /**
     * @brief Enforce retention policy by removing old snapshots and limiting history size.
     * This is called after each batch flush to keep memory usage in check.
     */ 

    void enforceRetentionPolicy() {
        auto cutoff = std::chrono::system_clock::now() - retentionWindow_;
        history_.erase(std::remove_if(history_.begin(), history_.end(), [&cutoff](const auto& snap) { return snap.timestamp < cutoff; }), history_.end());
        if (history_.size() > maxHistorySize_) {
            history_.erase(history_.begin(), history_.begin() + (history_.size() - maxHistorySize_));
        }
    }

    //void forceFlushBatch() { flushBatchBuffer(); }

    


    //====================================================================
    // --- REPLACE pushMetrics ---
    // This stops the "SoC-only" rows race condition
    void pushMetrics(
        const std::chrono::system_clock::time_point& ts,
        std::function<void(SystemMetricsSnapshot&)> mutate
    )  {
        if (stopping_) return;
        
        SystemMetricsSnapshot snap(ts);
        if (mutate) mutate(snap);

        // Check what kind of data this is and push it to the correct history
        // This is the crucial fix: DO NOT add to batchBuffer_.
        if (snap.socInfo.isValid()) {
             pushSoCStats(snap.socInfo);
        }
        if (snap.powerStats.isValid()) {
             pushPowerStats(snap.powerStats);
        }
    }

    //====================================================================

    SystemMetricsSnapshot getLatestSnapshot() const override {
        // Prefer most recent from batchBuffer_, else history_
        {
            std::lock_guard<std::mutex> bl(batchMutex_);
            if (!batchBuffer_.empty()) return batchBuffer_.back();
        }
        std::lock_guard<std::mutex> lock(mutex_);
        if (!history_.empty()) return history_.back();
        // If nothing, return a default snapshot "now"
        return SystemMetricsSnapshot(std::chrono::system_clock::now());
    }

    void exportToCSV(const std::string& filePath) override {
        // Minimal, safe implementation: flush, then write all via our own append
        forceFlushBatch();
        std::lock_guard<std::mutex> ioLock(ioMutex_);
        std::ofstream out(filePath, std::ios::trunc);
        if (!out) {
            spdlog::error("[Aggregator] exportToCSV: failed to open {}", filePath);
            return;
        }

        // Write header once
        out << "Timestamp,FrameId,FrameNumber,FPS,FrameWidth,FrameHeight,FrameSize,"
            << "InferenceTimeMs,ConfidenceScore,AlgorithmFPS,AvgProcTimeMs,TotalProcTimeMs,GPUFreeMemoryMB,GPUTotalMemoryMB,CudaKernelTimeMs,"
            << "DisplayLatencyMsRaw,DroppedFrames,RenderTimeMs,ProcessingLatencyMs,DisplayLatencyMs,EndToEndLatencyMs,JoulesPerFrame,"
            << "RAM_In_Use_MB,Total_RAM_MB,LFB_Size_MB,Block_Max_MB,SWAP_In_Use_MB,Total_SWAP_MB,Cached_MB,"
            << "used_IRAM_kB,total_IRAM_kB,lfb_kB,"
            << "CPU1_Utilization_Percent,CPU1_Frequency_MHz,CPU2_Utilization_Percent,CPU2_Frequency_MHz,"
            << "CPU3_Utilization_Percent,CPU3_Frequency_MHz,CPU4_Utilization_Percent,CPU4_Frequency_MHz,"
            << "EMC_Frequency_Percent,GR3D_Frequency_Percent,"
            << "PLL_Temperature_C,CPU_Temperature_C,PMIC_Temperature_C,GPU_Temperature_C,AO_Temperature_C,Thermal_Temperature_C,"
            << "PowerTotalW,PowerAverageW,PowerSensor0W,PowerSensor0V,PowerSensor0A,PowerSensor1W,PowerSensor1V,PowerSensor1A,"
            << "PowerSensor2W,PowerSensor2V,PowerSensor2A,PowerSensor3W,PowerSensor3V,PowerSensor3A\n";

        // Snapshot a copy of history under lock
        std::vector<SystemMetricsSnapshot> copy;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            copy = history_;
        }

        for (const auto& s : copy) {
            out << utils::formatTimestamp(s.timestamp) << "," << s.frameId << "," << s.cameraStats.frameNumber << ","
                << s.cameraStats.fps << "," << s.cameraStats.frameWidth << "," << s.cameraStats.frameHeight << ","
                << s.cameraStats.frameSize << "," << s.algorithmStats.inferenceTimeMs << ","
                << s.algorithmStats.confidenceScore << "," << s.algorithmStats.fps << ","
                << s.algorithmStats.avgProcTimeMs << "," << s.algorithmStats.totalProcTimeMs << ","
                << (s.algorithmStats.gpuFreeMemory / (1024.0 * 1024.0)) << ","
                << (s.algorithmStats.gpuTotalMemory / (1024.0 * 1024.0)) << "," << s.algorithmStats.cudaKernelTimeMs << ","
                << s.displayStats.latencyMs << "," << s.displayStats.droppedFrames << ","
                << s.displayStats.renderTimeMs << "," << s.processingLatencyMs << ","
                << s.displayLatencyMs << "," << s.endToEndLatencyMs << "," << s.joulesPerFrame << ","
                << s.socInfo.RAM_In_Use_MB << "," << s.socInfo.Total_RAM_MB << "," << s.socInfo.LFB_Size_MB << ","
                << s.socInfo.Block_Max_MB << "," << s.socInfo.SWAP_In_Use_MB << "," << s.socInfo.Total_SWAP_MB << ","
                << s.socInfo.Cached_MB << "," << s.socInfo.used_IRAM_kB << "," << s.socInfo.total_IRAM_kB << ","
                << s.socInfo.lfb_kB << "," << s.socInfo.CPU1_Utilization_Percent << "," << s.socInfo.CPU1_Frequency_MHz << ","
                << s.socInfo.CPU2_Utilization_Percent << "," << s.socInfo.CPU2_Frequency_MHz << ","
                << s.socInfo.CPU3_Utilization_Percent << "," << s.socInfo.CPU3_Frequency_MHz << ","
                << s.socInfo.CPU4_Utilization_Percent << "," << s.socInfo.CPU4_Frequency_MHz << ","
                << s.socInfo.EMC_Frequency_Percent << "," << s.socInfo.GR3D_Frequency_Percent << ","
                << s.socInfo.PLL_Temperature_C << "," << s.socInfo.CPU_Temperature_C << ","
                << s.socInfo.PMIC_Temperature_C << "," << s.socInfo.GPU_Temperature_C << ","
                << s.socInfo.AO_Temperature_C << "," << s.socInfo.Thermal_Temperature_C << ","
                << s.powerStats.totalPower() << "," << s.powerStats.averagePower() << ","
                << s.powerStats.sensorPower(0) << "," << (s.powerStats.sensorCount() > 0 ? s.powerStats.voltages[0] : 0.0) << ","
                << (s.powerStats.sensorCount() > 0 ? s.powerStats.currents[0] : 0.0) << ","
                << s.powerStats.sensorPower(1) << "," << (s.powerStats.sensorCount() > 1 ? s.powerStats.voltages[1] : 0.0) << ","
                << (s.powerStats.sensorCount() > 1 ? s.powerStats.currents[1] : 0.0) << ","
                << s.powerStats.sensorPower(2) << "," << (s.powerStats.sensorCount() > 2 ? s.powerStats.voltages[2] : 0.0) << ","
                << (s.powerStats.sensorCount() > 2 ? s.powerStats.currents[2] : 0.0) << ","
                << s.powerStats.sensorPower(3) << "," << (s.powerStats.sensorCount() > 3 ? s.powerStats.voltages[3] : 0.0) << ","
                << (s.powerStats.sensorCount() > 3 ? s.powerStats.currents[3] : 0.0) << "\n";
        }
    }


private:
    bool isAncientTs(const std::chrono::system_clock::time_point& ts) {
        static const auto minValid = std::chrono::system_clock::now() - std::chrono::hours(24 * 365 * 10);
        return ts.time_since_epoch().count() == 0 || ts < minValid;
    }


    // Attach SoC/Power sampled with time-window integration if we have the algorithm's window.
    // PRECONDITION: caller does NOT hold mutex_; this acquires asyncDataMutex_ internally.
    void attachWindowIntegratedAsync(PendingFrame& pf) {
        std::lock_guard<std::mutex> asyncLock(asyncDataMutex_);
        if (pf.hasAlgTime) {
            pf.soc   = integrateSoCInWindow(pf.algStartTime, pf.algEndTime);
            pf.power = integratePowerInWindow(pf.algStartTime, pf.algEndTime);
            pf.hasSoC = true;
            pf.hasPower = true;
        } else {
            // Fallback to latest samples if we don't know the window yet
            if (!socHistory_.empty())   { pf.soc = socHistory_.back(); pf.hasSoC = true; }
            if (!powerHistory_.empty()) { pf.power = powerHistory_.back(); pf.hasPower = true; }
        }
    }


    

    // PRECONDITION: caller holds mutex_.
    void applyOverlaysForRow(const std::chrono::system_clock::time_point& row_ts) {
        std::lock_guard<std::mutex> lock(overlay_mtx_);
        const auto tol = std::chrono::milliseconds(mergeWaitMs_);
        auto keep = std::deque<PendingOverlay>{};
        while (!overlay_q_.empty()) {
            auto& o = overlay_q_.front();
            auto dt = (o.ts > row_ts) ? (o.ts - row_ts) : (row_ts - o.ts);
            if (dt <= tol) {
                uint64_t closestFrameId = 0;
                auto minDt = std::chrono::milliseconds::max();
                for (const auto& [frameId, pf] : pending_) {
                    auto frameDt = (pf.cam.timestamp > o.ts) ? (pf.cam.timestamp - o.ts) : (o.ts - pf.cam.timestamp);
                    if (frameDt < minDt) {
                        //minDt = frameDt;
                        minDt = std::chrono::duration_cast<std::chrono::milliseconds>(frameDt);
                        closestFrameId = frameId;
                    }
                }
                if (closestFrameId != 0) {
                    auto it = pending_.find(closestFrameId);
                    if (it != pending_.end()) {
                        it->second.overlays.insert(o.kv.begin(), o.kv.end());
                    }
                }
            } else {
                keep.push_back(std::move(o));
            }
            overlay_q_.pop_front();
        }
        overlay_q_.swap(keep);
    }


    // --- add near private: helpers ---
    static inline bool hasUsefulPayload(const SystemMetricsSnapshot& s) {
        // Camera meaningful?
        if (s.cameraStats.frameNumber > 0 || s.cameraStats.fps > 0 ||
            s.cameraStats.frameWidth > 0 || s.cameraStats.frameHeight > 0 || s.cameraStats.frameSize > 0)
            return true;

        // Algorithm meaningful?
        if (s.algorithmStats.inferenceTimeMs > 0 || s.algorithmStats.fps > 0 ||
            s.algorithmStats.avgProcTimeMs > 0 || s.algorithmStats.totalProcTimeMs > 0 ||
            s.algorithmStats.cudaKernelTimeMs > 0 || s.algorithmStats.gpuTotalMemory > 0 || s.algorithmStats.gpuFreeMemory > 0)
            return true;

        // Display meaningful?
        if (s.displayStats.renderTimeMs > 0 || s.displayStats.latencyMs > 0 || s.displayStats.droppedFrames > 0)
            return true;

        // Derived
        if (s.processingLatencyMs > 0.0 ||
            s.displayLatencyMs > 0.0 ||
            s.endToEndLatencyMs > 0.0 ||
            s.joulesPerFrame > 0.0) return true;

        // SoC or Power meaningful (pick a few representative fields)
        if (s.socInfo.Total_RAM_MB > 0 || s.socInfo.RAM_In_Use_MB > 0 || s.socInfo.CPU1_Frequency_MHz > 0 || s.socInfo.CPU1_Utilization_Percent > 0)
            return true;

        // Power meaningful (pick a few representative fields)
        if (s.powerStats.totalPower() > 0 || s.powerStats.sensorCount() > 0)
            return true;

        return false;
    }
    // --- end near private: helpers ---

    // --- REPLACE latestSoC_/latestPower_ with HISTORY ---
    mutable std::mutex mutex_, batchMutex_, ioMutex_, overlay_mtx_;  // Added overlay_mtx_ for overlay queue
    std::condition_variable cv_;
    std::unordered_map<uint64_t, PendingFrame> pending_;
    std::unordered_map<uint64_t, bool> frameStarted_;
    std::deque<uint64_t> arrivalOrder_;  // FIFO for oldest frames
    std::vector<SystemMetricsSnapshot> history_, batchBuffer_;

    // --- NEW MEMBERS ---
    mutable std::mutex asyncDataMutex_; // New mutex for async histories
    std::deque<JetsonNanoInfo> socHistory_;
    std::deque<PowerStats> powerHistory_;
    std::chrono::seconds asyncHistoryDuration_{std::chrono::seconds(5)}; // Keep 5s of history
    // --- END NEW ---

    PowerStats latestPower_;
    JetsonNanoInfo latestSoC_;
    std::string csvPath_, jsonPath_;
    std::chrono::seconds retentionWindow_, pruneMaxAge_;
    size_t maxPendingFrames_, maxHistorySize_;
    //AggregatorConfig config_;
    std::ofstream ofs_, jofs_;  // Persistent streams for CSV and JSON
    bool csvHeaderWritten_;     // Member to track header state
    std::atomic<bool> stopping_{false};  // Shutdown flag
    int mergeWaitMs_;  // Configurable wait time in ms
    size_t flushCount_;  // Counter for periodic JSON flush
    size_t flushThreshold_;  // Configurable threshold for JSON flush
    std::deque<PendingOverlay> overlay_q_; // Queue for overlay stats
    static constexpr size_t BATCH_SIZE_LIMIT = 200;

    // add flush_period_ms
    size_t flushPeriodMs_;

    // Compatibility flags
    bool dropEmptyCompat_{true};
    bool dropEmptyOnFlush_{true};
    
    json config_;
    AggregatorConfig aggConfig_;

    // Add to private members
    std::atomic<bool> flushing_{false};
    std::thread flushThread_;
   


    static AggregatorConfig parseConfig(const json& config) {
    AggregatorConfig cfg;
    auto aggCfg = config.contains("Aggregator") && !config["Aggregator"].is_null() && config["Aggregator"].is_object()
                  ? config["Aggregator"]
                  : json{};  // Safe: Empty object if null or missing

    // Null-safe access for each key
    cfg.expectsCamera = aggCfg.contains("expectsCamera") && !aggCfg["expectsCamera"].is_null() && aggCfg["expectsCamera"].is_boolean()
                        ? aggCfg["expectsCamera"].get<bool>()
                        : true;
    cfg.expectsAlgorithm = aggCfg.contains("expectsAlgorithm") && !aggCfg["expectsAlgorithm"].is_null() && aggCfg["expectsAlgorithm"].is_boolean()
                          ? aggCfg["expectsAlgorithm"].get<bool>()
                          : true;
    cfg.expectsDisplay = aggCfg.contains("expectsDisplay") && !aggCfg["expectsDisplay"].is_null() && aggCfg["expectsDisplay"].is_boolean()
                        ? aggCfg["expectsDisplay"].get<bool>()
                        : true;
    cfg.expectsPower = aggCfg.contains("expectsPower") && !aggCfg["expectsPower"].is_null() && aggCfg["expectsPower"].is_boolean()
                       ? aggCfg["expectsPower"].get<bool>()
                       : true;
    cfg.expectsSoC = aggCfg.contains("expectsSoC") && !aggCfg["expectsSoC"].is_null() && aggCfg["expectsSoC"].is_boolean()
                     ? aggCfg["expectsSoC"].get<bool>()
                     : true;
    return cfg;
}



// Inside SystemMetricsAggregatorConcreteV3_2 (private:)
JetsonNanoInfo integrateSoCInWindow(
    std::chrono::system_clock::time_point start,
    std::chrono::system_clock::time_point end)
{
    JetsonNanoInfo avgStats(start);

    if (end <= start) return avgStats;

    // Copy relevant history under lock (C++11 safe)
    std::deque<JetsonNanoInfo> histCopy;
    {
        // take the lock here (same as SoC integrator)
        std::lock_guard<std::mutex> lk(asyncDataMutex_);
        if (socHistory_.empty()) return avgStats;
        histCopy = socHistory_;
    }

    // Gather samples: one before, all inside, one after (C++11 style)
    std::vector<JetsonNanoInfo> samples;
    samples.reserve(histCopy.size());

    // Find closest sample <= start
    const JetsonNanoInfo* before = NULL;
    for ( std::deque<JetsonNanoInfo>::const_reverse_iterator rit = histCopy.rbegin();
         rit != histCopy.rend(); ++rit) {
        if (rit->timestamp <= start) { before = &(*rit); break; }
    }
    if (before) samples.push_back(*before);

    // All within [start, end]
    for ( std::deque<JetsonNanoInfo>::const_iterator it = histCopy.begin();
         it != histCopy.end(); ++it) {
        if (it->timestamp >= start && it->timestamp <= end) samples.push_back(*it);
    }

    // One after end
    const JetsonNanoInfo* after = NULL;
    for ( std::deque<JetsonNanoInfo>::const_iterator it = histCopy.begin();
         it != histCopy.end(); ++it) {
        if (it->timestamp > end) { after = &(*it); break; }
    }
    if (after) samples.push_back(*after);

    // Fallbacks
    if (samples.empty()) {
        // no in-window nor boundary neighbors; pick the latest we have
        return histCopy.back();
    }
    if (samples.size() == 1) {
        // cannot form a trapezoid; best we can do
        return samples.front();
    }

    // Trapezoidal time-weighted average (clip each segment to [start, end])
    double cpu1_val_sec = 0.0, cpu2_val_sec = 0.0, cpu3_val_sec = 0.0, cpu4_val_sec = 0.0;
    double ram_val_sec  = 0.0;
    double total_time_sec = 0.0;

    for (size_t i = 0; i + 1 < samples.size(); ++i) {
        const JetsonNanoInfo& s1 = samples[i];
        const JetsonNanoInfo& s2 = samples[i + 1];

        // clip
        std::chrono::system_clock::time_point t1 = (s1.timestamp < start) ? start : s1.timestamp;
        if (t1 > end) t1 = end;
        std::chrono::system_clock::time_point t2 = (s2.timestamp > end) ? end : s2.timestamp;
        if (t2 < start) t2 = start;

        double dt_sec = std::chrono::duration<double>(t2 - t1).count();
        if (dt_sec <= 0.0) continue;

        total_time_sec += dt_sec;

        cpu1_val_sec += (s1.CPU1_Utilization_Percent + s2.CPU1_Utilization_Percent) * 0.5 * dt_sec;
        cpu2_val_sec += (s1.CPU2_Utilization_Percent + s2.CPU2_Utilization_Percent) * 0.5 * dt_sec;
        cpu3_val_sec += (s1.CPU3_Utilization_Percent + s2.CPU3_Utilization_Percent) * 0.5 * dt_sec;
        cpu4_val_sec += (s1.CPU4_Utilization_Percent + s2.CPU4_Utilization_Percent) * 0.5 * dt_sec;
        ram_val_sec  += (s1.RAM_In_Use_MB              + s2.RAM_In_Use_MB)              * 0.5 * dt_sec;
    }

    if (total_time_sec > 0.0) {
        avgStats.CPU1_Utilization_Percent = cpu1_val_sec / total_time_sec;
        avgStats.CPU2_Utilization_Percent = cpu2_val_sec / total_time_sec;
        avgStats.CPU3_Utilization_Percent = cpu3_val_sec / total_time_sec;
        avgStats.CPU4_Utilization_Percent = cpu4_val_sec / total_time_sec;
        avgStats.RAM_In_Use_MB            = ram_val_sec  / total_time_sec;
    } else {
        // degenerate window after clipping; return nearest sample
        avgStats = samples.front();
    }

    // carry static fields from the last sample we used
    avgStats.Total_RAM_MB = samples.back().Total_RAM_MB;
    return avgStats;
}


// PRECONDITION: caller holds asyncDataMutex_ before calling this function.
// (mergeDisplay does this correctly.)
PowerStats integratePowerInWindow(std::chrono::system_clock::time_point start,
                                  std::chrono::system_clock::time_point end)
{
    PowerStats avgStats(start);

     std::deque<PowerStats> histCopy;
    {   // take the lock here (same as SoC integrator)
        std::lock_guard<std::mutex> lk(asyncDataMutex_);
        if (powerHistory_.empty() || end <= start) return avgStats;
        histCopy = powerHistory_;
    }
    // ... then operate on histCopy (unchanged below this)

    // Validate inputs / availability
    if (histCopy.empty() || end <= start) {
        return avgStats; // default-constructed (zero) stats at 'start'
    }

    // Build a chronologically ordered list of reference samples that
    // bracket [start, end]: one <= start, all inside, and one >= end.
    std::vector<std::reference_wrapper<const PowerStats> > samples;
    samples.reserve(histCopy.size() + 2);

    // (1) last sample <= start
    const PowerStats* s_before = NULL;
    for (std::deque<PowerStats>::const_reverse_iterator it = histCopy.rbegin();
         it != histCopy.rend(); ++it) {
        if (it->timestamp <= start) { s_before = &(*it); break; }
    }
    if (s_before) samples.push_back(std::cref(*s_before));

    // (2) all samples inside [start, end]
    for (std::deque<PowerStats>::const_iterator it = histCopy.begin();
         it != histCopy.end(); ++it) {
        if (it->timestamp >= start && it->timestamp <= end) {
            samples.push_back(std::cref(*it));
        }
    }

    // (3) first sample >= end (so we can close the last trapezoid)
    const PowerStats* s_after = NULL;
    for (std::deque<PowerStats>::const_iterator it = histCopy.begin();
         it != histCopy.end(); ++it) {
        if (it->timestamp >= end) { s_after = &(*it); break; }
    }
    if (s_after) {
        // Avoid duplicating the exact same pointer if it was already pushed as "inside"
        if (samples.empty() || &samples.back().get() != s_after) {
            samples.push_back(std::cref(*s_after));
        }
    }

    // If we still have <2 samples, fall back to a best-effort snapshot
    if (samples.size() < 2u) {
        // If we have at least one, return it; else return the latest seen
        if (!samples.empty()) return samples.front().get();
        return histCopy.back();
    }

    // Decide sensor count from the first sample
    const size_t sensor_count =
        samples.front().get().sensorCount(); // your PowerStats API

    // Integrate energy per sensor over [start, end] using clipped trapezoids
    std::vector<double> energy_joules(sensor_count, 0.0);

    // Window duration (seconds)  best denominator for a time-averaged power over the window
    const double window_duration_sec =
        std::chrono::duration<double>(end - start).count();

    for (size_t i = 0; i + 1 < samples.size(); ++i) {
        const PowerStats& s1 = samples[i].get();
        const PowerStats& s2 = samples[i + 1].get();

        // Clip segment to the window
        std::chrono::system_clock::time_point t1 =
            (s1.timestamp > start) ? s1.timestamp : start;
        std::chrono::system_clock::time_point t2 =
            (s2.timestamp < end) ? s2.timestamp : end;

        const double dt_sec =
            std::chrono::duration<double>(t2 - t1).count();
        if (dt_sec <= 0.0) continue;

        // Trapezoid per sensor: ((P1 + P2)/2) * dt
        for (size_t j = 0; j < sensor_count; ++j) {
            const double p1 = s1.sensorPower(j);
            const double p2 = s2.sensorPower(j);
            energy_joules[j] += (p1 + p2) * 0.5 * dt_sec;
        }
    }

    // Convert energy -> average power for the window
    if (window_duration_sec > 0.0) {
        // Write per-sensor averages. Your PowerStats appears to expose a public
        // array/vector named 'power'. If it's std::array<4,double>, guard on size.
        if (avgStats.power.size() < sensor_count) {
            avgStats.power.resize(sensor_count, 0.0); // if 'power' is a std::vector
        }
        for (size_t j = 0; j < sensor_count; ++j) {
            avgStats.power[j] = energy_joules[j] / window_duration_sec;
        }
        // Recompute derived totals/averages
        avgStats.updateDerivedMetrics();
    } else {
        // Degenerate zero-duration window: return the closest sample
        avgStats = samples.front().get();
    }

    // Timestamp the aggregated result at 'end' (or 'start'); either is fine if consistent
    avgStats.timestamp = end;
    return avgStats;
}



    }; // end of class SystemMetricsAggregatorConcreteV3_2
