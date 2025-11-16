//allModulesStatcs.h
#pragma once

#include <chrono>
#include <vector>
#include <cstdint>
#include <string>
#include <algorithm> // For std::min
#include "../Includes/lynsyn.h" // Lynsyn library for power measurements
#include "../nlohmann/json.hpp" // JSON serialization
#include <spdlog/spdlog.h> // For logging
#include "../Others/utils.h"


using json = nlohmann::json;

// SAMPLING RATE = The frequency corresponding to a time period of \(500\text{\ milliseconds}\) is \(2\text{\ Hz}\).

// Custom definitions for LynsynSample flags (pending confirmation from usbprotocol.h)
#define SAMPLE_FLAG_VOLTAGE_VALID 0x0004 // Bit 2
#define SAMPLE_FLAG_CURRENT_VALID 0x0008 // Bit 3



/**
 * @struct PowerStats
 * @brief Represents power measurements across multiple sensors at a single point in time.
 */
struct PowerStats {
    PowerStats() = default;
    explicit PowerStats(std::chrono::system_clock::time_point ts) : timestamp(ts) {}
    std::chrono::system_clock::time_point timestamp;
    std::vector<double> voltages{4, 0.0};
    std::vector<double> currents{4, 0.0};
    std::vector<double> power{4, 0.0};

    double totalPower_;  // Added as a member
    double averagePower_; // Added as a member

    double sensorPower(size_t i) const { return i < power.size() ? power[i] : 0.0; }
    std::vector<double> powerPerSensor() const { return power; }
    double totalPower() const { return std::accumulate(power.begin(), power.end(), 0.0); }
    double averagePower() const { return sensorCount() > 0 ? totalPower() / sensorCount() : 0.0; }
    size_t sensorCount() const { return std::min(voltages.size(), currents.size()); }
    void setSensorData(size_t i, double voltage, double current) {
        if (i < voltages.size()) {
            voltages[i] = voltage;
            currents[i] = current;
            power[i] = voltage * current;
            updateDerivedMetrics(); // Update total and average
        }
    }
    void setTotalPower(double value) { totalPower_ = value; }
    void setAveragePower(double value) { averagePower_ = value; }
    
    void updateDerivedMetrics() {
        totalPower_ = std::accumulate(power.begin(), power.end(), 0.0);
        averagePower_ = sensorCount() > 0 ? totalPower_ / sensorCount() : 0.0;
    }

    /**
     * @brief Checks if the PowerStats object is valid.
     * @return True if valid, false otherwise.
     */

    bool isValid() const {
            size_t count = std::min(voltages.size(), currents.size());
            return timestamp != std::chrono::system_clock::time_point{} && count > 0 &&
                std::all_of(voltages.begin(), voltages.begin() + count, [](double v) { return v >= 0.0; }) &&
                std::all_of(currents.begin(), currents.begin() + count, [](double c) { return c >= 0.0; });
        }
        // ... (other methods unchanged)

    /**
     * @brief Converts to JSON for serialization.
     * @return JSON object.
     */
    
    json toJson() const {
        json j;
        j["timestamp"] = utils::formatTimestamp(timestamp);
        j["total_w"] = totalPower();
        j["average_w"] = averagePower();
        for (size_t i = 0; i < sensorCount(); ++i) {
            j["sensor" + std::to_string(i)] = {
                {"power_w", sensorPower(i)},
                {"voltage_v", voltages[i]},
                {"current_a", currents[i]}
            };
        }
        return j;
    }
};




/**
 * @brief Convert raw LynsynSample to PowerStats structure.
 * @param s LynsynSample containing voltage, current, and time data.
 * @return PowerStats with populated fields.
 */

inline PowerStats convertToPowerStats(const LynsynSample& s) {
    PowerStats stats;
    double seconds = lynsyn_cyclesToSeconds(s.time);
    auto duration = std::chrono::duration<double>(seconds);
    stats.timestamp = std::chrono::system_clock::time_point(
        std::chrono::duration_cast<std::chrono::system_clock::duration>(duration));

    constexpr size_t max_sensors = std::min(static_cast<size_t>(LYNSYN_MAX_SENSORS), static_cast<size_t>(4));
    stats.voltages.resize(max_sensors, 0.0); // Pre-size with default
    stats.currents.resize(max_sensors, 0.0);

    for (size_t i = 0; i < max_sensors; ++i) {
        if (s.flags & SAMPLE_FLAG_VOLTAGE_VALID) {
            stats.voltages[i] = s.voltage[i];
        }
        if (s.flags & SAMPLE_FLAG_CURRENT_VALID) {
            stats.currents[i] = s.current[i];
        }
    }

    spdlog::debug("[convertToPowerStats] Populated {} voltages, {} currents (flags: {:04x})", 
                  stats.voltages.size(), stats.currents.size(), s.flags);
    return stats;
}

/**
 * @struct CameraStats
 * @brief Metrics for camera performance.
 */
struct CameraStats {
    std::chrono::system_clock::time_point timestamp;
    uint64_t frameNumber = 0; // Frame sequence number
    double fps = 0.0;         // Frames per second
    uint32_t frameWidth = 0;  // Frame width in pixels
    uint32_t frameHeight = 0; // Frame height in pixels
    uint64_t frameSize = 0;   // Frame size in bytes

    CameraStats() = default;
    CameraStats(std::chrono::system_clock::time_point ts)
        : timestamp(ts), frameNumber(0), fps(0.0), frameWidth(0), frameHeight(0), frameSize(0) {}
    CameraStats(std::chrono::system_clock::time_point ts, uint64_t fn, double f, uint32_t w, uint32_t h, uint64_t s)
        : timestamp(ts), frameNumber(fn), fps(f), frameWidth(w), frameHeight(h), frameSize(s) {}

    // Camera stats validation
    /**
     * @brief Validates the camera stats.
     * @return True if valid, false otherwise.
     */
    bool isValid() const {
        return timestamp != std::chrono::system_clock::time_point{} &&
            frameWidth > 0 && frameHeight > 0 && frameSize > 0;
    }
    

    /**
     * @brief Converts to JSON for serialization.
     * @return JSON object.
     */
    json toJson() const {
        return {
            {"timestamp", utils::formatTimestamp(timestamp)},
            {"frame_number", frameNumber},
            {"fps", fps},
            {"width", frameWidth},
            {"height", frameHeight},
            {"frame_size", frameSize}
        };
    }
};

/**
 * @struct AlgorithmStats
 * @brief Metrics for algorithm processing (e.g., inference).
 */
struct AlgorithmStats {

    std::chrono::system_clock::time_point timestamp; // This will represent the END time
    std::chrono::system_clock::time_point startTime; // ADD THIS FIELD
    
    double inferenceTimeMs = 0.0;     // Inference time per frame (ms)
    double confidenceScore = 0.0;     // Algorithm confidence (0-1)
    double fps = 0.0;                 // Algorithm processing FPS
    double avgProcTimeMs = 0.0;       // Average processing time per frame (ms)
    double totalProcTimeMs = 0.0;     // Total processing time (ms)
    uint64_t framesCount = 0;         // Number of processed frames
    uint64_t gpuFreeMemory = 0;       // GPU free memory (bytes)
    uint64_t gpuTotalMemory = 0;      // GPU total memory (bytes)
    double cudaKernelTimeMs = 0.0;    // CUDA kernel execution time (ms)
    uint32_t droppedFrames = 0;       // Number of dropped frames

    AlgorithmStats() = default;
    AlgorithmStats(std::chrono::system_clock::time_point ts)
        : timestamp(ts), inferenceTimeMs(0.0), confidenceScore(0.0), fps(0.0), avgProcTimeMs(0.0),
          totalProcTimeMs(0.0), framesCount(0), gpuFreeMemory(0), gpuTotalMemory(0), cudaKernelTimeMs(0.0) {}
    AlgorithmStats(std::chrono::system_clock::time_point ts, double inf, double conf, double f, double avg, double total, uint64_t fc)
        : timestamp(ts), inferenceTimeMs(inf), confidenceScore(conf), fps(f), avgProcTimeMs(avg), totalProcTimeMs(total), framesCount(fc),
          gpuFreeMemory(0), gpuTotalMemory(0), cudaKernelTimeMs(0.0) {}

    // // OLD CONSTRUCTOR (keep for compatibility or remove if unused)
    // explicit AlgorithmStats(std::chrono::system_clock::time_point ts)
    //     : timestamp(ts), startTime(ts) {} // Default startTime to endTime

    // --- ADD THIS NEW CONSTRUCTOR ---
    /**
     * @brief Construct AlgorithmStats with a defined start and end time.
     */
    AlgorithmStats(std::chrono::system_clock::time_point ts_start, 
                   std::chrono::system_clock::time_point ts_end)
        : timestamp(ts_end), startTime(ts_start) {}


    
    // If you want to allow first frame (framesCount = 0):
    bool isValid() const {
    return timestamp != std::chrono::system_clock::time_point{} &&
           inferenceTimeMs >= 0.0 &&
           fps >= 0.01 && fps < 1000.0;
        }


    // ... (other methods unchanged)

    /**
     * @brief Converts to JSON for serialization.
     * @return JSON object.
     */
    json toJson() const {
        return {
            {"timestamp", utils::formatTimestamp(timestamp)},
            {"inference_time_ms", inferenceTimeMs},
            {"confidence", confidenceScore},
            {"fps", fps},
            {"avg_proc_time_ms", avgProcTimeMs},
            {"total_proc_time_ms", totalProcTimeMs},
            {"frames_count", framesCount},
            {"gpu_free_memory_mb", gpuFreeMemory / (1024.0 * 1024.0)},
            {"gpu_total_memory_mb", gpuTotalMemory / (1024.0 * 1024.0)},
            {"cuda_kernel_time_ms", cudaKernelTimeMs}
        };
    }
};

/**
 * @struct DisplayStats
 * @brief Metrics for display rendering.
 */
struct DisplayStats {
    std::chrono::system_clock::time_point timestamp;
    double latencyMs = 0.0;      // Display latency (ms)
    uint32_t droppedFrames = 0;  // Number of dropped frames
    double renderTimeMs = 0.0;   // Rendering time per frame (ms)

    DisplayStats() = default;
    DisplayStats(std::chrono::system_clock::time_point ts)
        : timestamp(ts), latencyMs(0.0), droppedFrames(0), renderTimeMs(0.0) {}
    DisplayStats(std::chrono::system_clock::time_point ts, double lat, uint32_t df, double rt)
        : timestamp(ts), latencyMs(lat), droppedFrames(df), renderTimeMs(rt) {}

    /**
     * @brief Validates the display stats.
     * @return True if valid, false otherwise.
     */
    bool isValid() const {
        return timestamp != std::chrono::system_clock::time_point{} &&
               //latencyMs >= 0.0 && 
               renderTimeMs > 0.0;
    }

    /**
     * @brief Converts to JSON for serialization.
     * @return JSON object.
     */
    json toJson() const {
        return {
            {"timestamp", utils::formatTimestamp(timestamp)},
            {"latency_ms", latencyMs},
            {"dropped_frames", droppedFrames},
            {"render_time_ms", renderTimeMs}
        };
    }
};

/**
 * @struct JetsonNanoInfo
 * @brief System-on-Chip metrics for Jetson Nano.
 */
struct JetsonNanoInfo {
    std::chrono::system_clock::time_point timestamp;
    // Memory metrics (MB)
    double RAM_In_Use_MB = 0.0;      // RAM currently in use
    double Total_RAM_MB = 0.0;       // Total available RAM
    double LFB_Size_MB = 0.0;        // Largest free block size
    double Block_Max_MB = 0.0;       // Maximum block size
    double SWAP_In_Use_MB = 0.0;     // SWAP currently in use
    double Total_SWAP_MB = 0.0;      // Total available SWAP
    double Cached_MB = 0.0;          // Cached memory
    // IRAM metrics (kB)
    double used_IRAM_kB = 0.0;       // Internal RAM used
    double total_IRAM_kB = 0.0;      // Total internal RAM
    double lfb_kB = 0.0;             // Largest free block
    // CPU metrics (4 cores)
    double CPU1_Utilization_Percent = 0.0;
    double CPU1_Frequency_MHz = 0.0;
    double CPU2_Utilization_Percent = 0.0;
    double CPU2_Frequency_MHz = 0.0;
    double CPU3_Utilization_Percent = 0.0;
    double CPU3_Frequency_MHz = 0.0;
    double CPU4_Utilization_Percent = 0.0;
    double CPU4_Frequency_MHz = 0.0;
    // Memory and GPU frequency
    double EMC_Frequency_Percent = 0.0; // External Memory Controller
    double GR3D_Frequency_Percent = 0.0; // GPU frequency
    // Temperature sensors (°C)
    double PLL_Temperature_C = 0.0;
    double CPU_Temperature_C = 0.0;
    double PMIC_Temperature_C = 0.0;
    double GPU_Temperature_C = 0.0;
    double AO_Temperature_C = 0.0; // Always-On sensor
    double Thermal_Temperature_C = 0.0;

    JetsonNanoInfo() = default;
    JetsonNanoInfo(std::chrono::system_clock::time_point ts)
        : timestamp(ts),
          RAM_In_Use_MB(0.0), Total_RAM_MB(0.0), LFB_Size_MB(0.0), Block_Max_MB(0.0),
          SWAP_In_Use_MB(0.0), Total_SWAP_MB(0.0), Cached_MB(0.0),
          used_IRAM_kB(0.0), total_IRAM_kB(0.0), lfb_kB(0.0),
          CPU1_Utilization_Percent(0.0), CPU1_Frequency_MHz(0.0),
          CPU2_Utilization_Percent(0.0), CPU2_Frequency_MHz(0.0),
          CPU3_Utilization_Percent(0.0), CPU3_Frequency_MHz(0.0),
          CPU4_Utilization_Percent(0.0), CPU4_Frequency_MHz(0.0),
          EMC_Frequency_Percent(0.0), GR3D_Frequency_Percent(0.0),
          PLL_Temperature_C(0.0), CPU_Temperature_C(0.0), PMIC_Temperature_C(0.0),
          GPU_Temperature_C(0.0), AO_Temperature_C(0.0), Thermal_Temperature_C(0.0) {}

    /**
     * @brief Validates the SoC stats.
     * @return True if valid, false otherwise.
     */
    bool isValid() const {
        return timestamp != std::chrono::system_clock::time_point{} &&
               //RAM_In_Use_MB >= 0.0 &&
               Total_RAM_MB > 0.0 ;//&&
               //CPU1_Utilization_Percent >= 0.0 && 
               //CPU1_Frequency_MHz >= 0.0;
    }

    /**
     * @brief Converts to JSON for serialization.
     * @return JSON object.
     */
    json toJson() const {
        return {
            {"timestamp", utils::formatTimestamp(timestamp)},
            {"ram_in_use_mb", RAM_In_Use_MB},
            {"total_ram_mb", Total_RAM_MB},
            {"lfb_size_mb", LFB_Size_MB},
            {"block_max_mb", Block_Max_MB},
            {"swap_in_use_mb", SWAP_In_Use_MB},
            {"total_swap_mb", Total_SWAP_MB},
            {"cached_mb", Cached_MB},
            {"used_iram_kb", used_IRAM_kB},
            {"total_iram_kb", total_IRAM_kB},
            {"lfb_kb", lfb_kB},
            {"cpu1_utilization_percent", CPU1_Utilization_Percent},
            {"cpu1_frequency_mhz", CPU1_Frequency_MHz},
            {"cpu2_utilization_percent", CPU2_Utilization_Percent},
            {"cpu2_frequency_mhz", CPU2_Frequency_MHz},
            {"cpu3_utilization_percent", CPU3_Utilization_Percent},
            {"cpu3_frequency_mhz", CPU3_Frequency_MHz},
            {"cpu4_utilization_percent", CPU4_Utilization_Percent},
            {"cpu4_frequency_mhz", CPU4_Frequency_MHz},
            {"emc_frequency_percent", EMC_Frequency_Percent},
            {"gr3d_frequency_percent", GR3D_Frequency_Percent},
            {"pll_temperature_c", PLL_Temperature_C},
            {"cpu_temperature_c", CPU_Temperature_C},
            {"pmic_temperature_c", PMIC_Temperature_C},
            {"gpu_temperature_c", GPU_Temperature_C},
            {"ao_temperature_c", AO_Temperature_C},
            {"thermal_temperature_c", Thermal_Temperature_C}
        };
    }
};



/**
 * @struct SystemMetricsSnapshot
 * @brief Aggregated snapshot of all system metrics for a single frame at a point in time.
 */
struct SystemMetricsSnapshot {
    std::chrono::system_clock::time_point timestamp;
    uint64_t frameId = 0;                    // Frame number for traceability
    CameraStats cameraStats;
    AlgorithmStats algorithmStats;
    DisplayStats displayStats;
    JetsonNanoInfo socInfo;
    PowerStats powerStats;
    double joulesPerFrame = 0.0;             // Energy per frame (joules)
    double endToEndLatencyMs = 0.0;          // End-to-end latency (ms)
    double processingLatencyMs = 0.0;        // Algorithm processing latency (ms)
    double displayLatencyMs = 0.0;           // Display rendering latency (ms)

    SystemMetricsSnapshot() = default;
    explicit SystemMetricsSnapshot(std::chrono::system_clock::time_point ts)
        : timestamp(ts), frameId(0), cameraStats(ts), algorithmStats(ts), displayStats(ts), socInfo(ts), powerStats(ts) {}

    /**
     * @brief Validates the snapshot for completeness.
     * @return True if all required fields are populated, false otherwise.
     */
    // ... (fields unchanged)
    bool isValid() const {
        return timestamp != std::chrono::system_clock::time_point{} &&
               (cameraStats.isValid() || algorithmStats.isValid() ||
                displayStats.isValid() || socInfo.isValid() || powerStats.isValid());
    }
    // ... (other methods unchanged)

    /**
     * @brief Converts the snapshot to JSON for serialization.
     * @return JSON object representing the snapshot.
     */
    json toJson() const {
        json j;
        j["timestamp"] = utils::formatTimestamp(timestamp);
        j["frame_id"] = frameId;
        j["camera"] = cameraStats.toJson();
        j["algorithm"] = algorithmStats.toJson();
        j["display"] = displayStats.toJson();
        j["soc"] = socInfo.toJson();
        j["power"] = powerStats.toJson();
        j["derived"] = {
            {"end_to_end_latency_ms", endToEndLatencyMs},
            {"processing_latency_ms", processingLatencyMs},
            {"display_latency_ms", displayLatencyMs},
            {"joules_per_frame", joulesPerFrame}
        };
        return j;
    }
};


//#endif // ALL_MODULES_STATCS_H