
// LynsynMonitorConcrete_new.h
//* Concrete implementation of ILynsynMonitor using liblynsyn

#pragma once

// Project interfaces
#include "../Interfaces/ILynsynMonitor.h"
#include "../Interfaces/ISystemMetricsAggregator.h"
#include "../SharedStructures/ThreadManager.h"
#include "../SharedStructures/allModulesStatcs.h"
#include "../SharedStructures/LynsynMonitorConfig.h"

// STL / system
#include <atomic>
#include <chrono>
#include <cstdint>
#include <fstream>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include <iostream>

#include <iomanip>     // for std::fixed, std::setprecision
//#include <fmt/core.h>  // if using fmt::format (or use std::format in C++20)

// Logging
#include <spdlog/spdlog.h>

// Lynsyn C library
extern "C" {
  // If you use a different include path, update this line accordingly.
  #include "lynsyn.h"
  // Some trees' lynsyn.h don't declare this; add a safe forward decl so C++ sees it.
  int lynsyn_reset_device(void);
}

/**
 * @class LynsynMonitorConcrete
 * @brief Concrete implementation of ILynsynMonitor that drives the Lynsyn board
 *        using liblynsyn. Designed to be robust against device glitches without
 *        resetting the USB hub/port (so UVC cameras stay up).
 */
class LynsynMonitorConcrete final : public ILynsynMonitor {
public:
    explicit LynsynMonitorConcrete(std::shared_ptr<ISystemMetricsAggregator> aggregator,
                                   ThreadManager& threadManager)
        : initialized_(false),
          running_(false),
          threadManager_(threadManager),
          threadShouldExit_(false),
          metricAggregator_(std::move(aggregator)) {
        spdlog::debug("[LynsynMonitorConcrete] ctor");
    }

    ~LynsynMonitorConcrete() override {
        stop();
    }

    // Small accessor used by your app:
    bool isRunning() const noexcept { return running_.load(); }

    // Make UsbGuard public so external code can use it (your app does).
    struct UsbGuard {
        using Fn = void(*)();
        Fn onExit;
        explicit UsbGuard(Fn f) : onExit(f) {}
        ~UsbGuard() { if (onExit) onExit(); }
    };


    // --- ILynsynMonitor API -------------------------------------------------

    bool configure(const LynsynMonitorConfig& config) override {
        config_ = config;
        const bool ok = config_.validate();
        if (!ok) {
            reportError("[LynsynMonitorConcrete] Invalid LynsynMonitorConfig");
        }
        return ok;
    }

    bool initialize() override {
        std::lock_guard<std::mutex> g(lifecycleMutex_);
        if (initialized_) {
            spdlog::warn("[LynsynMonitorConcrete] initialize() called but already initialized");
            return true;
        }

        spdlog::info("[LynsynMonitorConcrete] Initializing Lynsyn library...");
        constexpr int MAX_ATTEMPTS = 5;
        constexpr std::chrono::milliseconds DELAY(800);

        for (int attempt = 1; attempt <= MAX_ATTEMPTS; ++attempt) {
            if (lynsyn_init()) {
                initialized_ = true;
                spdlog::info("[LynsynMonitorConcrete] Lynsyn initialized (attempt {}/{})", attempt, MAX_ATTEMPTS);
                return true;
            }
            spdlog::warn("[LynsynMonitorConcrete] lynsyn_init() failed (attempt {}/{}). Retrying in {} ms...",
                         attempt, MAX_ATTEMPTS, DELAY.count());
            std::this_thread::sleep_for(DELAY);
            // Best-effort: release any partial state before retrying
            lynsyn_release();
        }

        reportError("[LynsynMonitorConcrete] Failed to initialize Lynsyn after retries");
        return false;
    }

    void startMonitoring() override {
        std::lock_guard<std::mutex> g(lifecycleMutex_);
        if (!initialized_) {
            reportError("[LynsynMonitorConcrete] startMonitoring() called before initialize()");
            return;
        }
        if (running_) {
            spdlog::warn("[LynsynMonitorConcrete] startMonitoring() called but already running");
            return;
        }
        
        // === CSV Setup ===
        // Open CSV if configured
        if (!config_.outputCSV.empty()) {

            fs::path p(config_.outputCSV);
            fs::path parent = p.parent_path();

            if (!parent.empty() && !fs::exists(parent)) {
                try {
                    fs::create_directories(parent);
                    spdlog::info("Created directory: {}", parent.string());
                } catch (const std::exception& e) {
                    reportError(fmt::format("Failed to create directory {}: {}", parent.string(), e.what()));
                    return;
                }
            }

            outputFile_.open(config_.outputCSV, std::ios::out | std::ios::trunc);
            if (!outputFile_.is_open()) {
                reportError(std::string("[LynsynMonitorConcrete] Failed to open CSV file: ") + config_.outputCSV);
                return;
            } else {
                // Minimal header (you can extend this to include totals/avg power if desired)
                // outputFile_ << "time_s";
                // outputFile_ << ",pc0,pc1,pc2,pc3";
                // // We don't know sensor count ahead of time (depends on HW version). We'll write as we go.
                // outputFile_ << ",i0,v0,i1,v1,i2,v2";
                // outputFile_ << "\n";
                
                // Write header
                outputFile_ << "time_s,pc0,pc1,pc2,pc3,"
                            << "i0,v0,i1,v1,i2,v2,i3,v3,i4,v4,i5,v5,i6,v6\n";  // Max 7 sensors
                outputFile_.flush();  // Ensure header is written
                spdlog::info("CSV logging enabled: {}", config_.outputCSV);
            
            }
        }

        // Arm device

        // Arm sampling mode
       // armSamplingMode_();


        // *** Arm with the desired period if supported, otherwise legacy arming. ***
        armSamplingModeWithPeriodMs_(config_.sampleRateMs);

        // Start worker thread
        threadShouldExit_ = false;
        threadManager_.addThread(Component::Lynsyn,
            std::thread(&LynsynMonitorConcrete::monitoringThreadLoop_, this));
        running_ = true;
        spdlog::info("[LynsynMonitorConcrete] Monitoring started");
        spdlog::info("[Lynsyn] Monitoring started (target {} ms / {} Hz)",
                 config_.sampleRateMs, 1000.0 / std::max(1, config_.sampleRateMs));
    }

    void stop() override {
        threadShouldExit_ = true;

        {
            std::lock_guard<std::mutex> g(lifecycleMutex_);
            if (running_) {
                threadManager_.joinThreadsFor(Component::Lynsyn);
                running_ = false;
            }
        }
        // Final flush before close
        if (outputFile_.is_open()) {
            outputFile_.flush();
            outputFile_.close();
            spdlog::info("CSV file closed and flushed");
        }

        {
            std::lock_guard<std::mutex> g(lifecycleMutex_);
            if (initialized_) {
                // Let the library clean only Lynsyn device (no hub reset).
                lynsyn_release();
                initialized_ = false;
            }
        }


        spdlog::info("[LynsynMonitorConcrete] Monitoring stopped");
    }

    void setSampleCallback(std::function<void(const LynsynSample&)> cb) override {
        sampleCallback_ = std::move(cb);
    }

    void setErrorCallback(std::function<void(const std::string&)> cb) override {
        errorCallback_ = std::move(cb);
    }

private:
    // --- Worker loop --------------------------------------------------------


//     void monitoringThreadLoop_() {
//     spdlog::debug("[LynsynMonitorConcrete] monitoring thread started");

//     // ---- Single source of truth for rate ----
//     //const int64_t sr_ms = std::max<int64_t>(1000, config_.sampleRateMs); // clamp to sane minimum

//     const int64_t sr_ms = std::max<int64_t>(50, config_.sampleRateMs);  // Min 50ms
//     const auto sampleInterval = std::chrono::milliseconds(sr_ms);

//     // Ensure config_.sampleRateMs is set to ~100ms via config.json or default
//     //const int64_t sr_ms = std::max<int64_t>(100, config_.sampleRateMs); // Target ~100ms

//     // spdlog::info("[Lynsyn] Monitoring started (target {} ms / {} Hz)", sr_ms, 1000.0 / sr_ms);
    
//     // const auto sampleInterval = std::chrono::milliseconds(sr_ms);

//     // Warn if period_ms is inconsistent
//     if (config_.periodSampling && config_.period_ms != sr_ms) {
//         spdlog::warn("[Lynsyn] config.period_ms ({}) != sampleRateMs ({}); using sampleRateMs", 
//                      config_.period_ms, sr_ms);
//     } 

//     // spdlog::warn("[Lynsyn] period_ms ({}) != sampleRateMs ({}). Using sr_ms.",
//     //                      config_.period_ms, sr_ms);

//     // // If device uses a separate period, ensure it's aligned (warn if not).
//     // if (config_.periodSampling) {
//     //     if (config_.period_ms != sr_ms) {
//     //         spdlog::warn("[Lynsyn] period_ms ({}) != sampleRateMs ({}). Using {} for device arming.",
//     //                      config_.period_ms, sr_ms, sr_ms);
//     //     }
//     // }

//     // Arm device with the chosen interval
//     armSamplingModeWithPeriodMs_(static_cast<int>(sr_ms)); // make this call period_ms = sampleRateMs

//     constexpr int MAX_CONSECUTIVE_FAILS = 10;
//     constexpr std::chrono::milliseconds BASE_BACKOFF{200};

//     int consecutiveFails = 0;
//     static std::atomic<size_t> sampleCount{0};


//     // Scheduling anchors
//    // using clock = std::chrono::steady_clock;
   
//     using clock = std::chrono::steady_clock;
//     auto nextSampleTime = clock::now() + sampleInterval;  // first read scheduled 1 interval from now

//      auto now = clock::now();
//     // auto nextSampleTime = now + sampleInterval;   // first read scheduled 1 interval from now
//     auto lastPublish = now - sampleInterval;      // for safety


//     while (!threadShouldExit_) {
//         // === Wait for next sample slot ===

//         std::this_thread::sleep_until(nextSampleTime);
//         auto now = clock::now();
//         nextSampleTime += sampleInterval;

        
//         // If we are behind, skip missed slots instead of bursting
//         if (now >= nextSampleTime) {
//             // Jump forward by the exact number of intervals missed
//             auto missed = (now - nextSampleTime) / sampleInterval;
//             nextSampleTime += (missed + 1) * sampleInterval;
//         } else {
//             std::this_thread::sleep_until(nextSampleTime);
//         }

//         LynsynSample raw{};
//         const bool got = lynsyn_getNextSample(&raw);

//         // schedule next slot immediately and guard against going backwards
//         nextSampleTime += sampleInterval;

//         if (!got) {
//             consecutiveFails++;
//             const auto backoff = BASE_BACKOFF * (1 << std::min(consecutiveFails - 1, 5)); // capped
//             spdlog::warn("[Lynsyn] Failed to get sample (#{})  backoff {} ms",
//                          consecutiveFails, backoff.count());

//             if (consecutiveFails == 2 || (consecutiveFails > 2 && consecutiveFails % 3 == 0)) {
//                 spdlog::warn("[Lynsyn] Attempting device-only reset + re-arm");
//                 tryDeviceOnlyResetAndRearm_();
//                 // After resets/backoffs, realign schedule from 'now'
//                 now = clock::now();
//                 nextSampleTime = now + sampleInterval;
//             }

//             if (consecutiveFails >= MAX_CONSECUTIVE_FAILS) {
//                 reportError("[Lynsyn] Too many consecutive failures; stopping monitor");
//                 break;
//             }

//             std::this_thread::sleep_for(backoff);
//             // Ensure we never burst after a long sleep
//             now = clock::now();
//             if (nextSampleTime <= now) {
//                 auto missed = (now - nextSampleTime) / sampleInterval;
//                 nextSampleTime = now + sampleInterval; // snap forward
//             }
//             continue;
//         }

//         // Success
//         consecutiveFails = 0;
//         sampleCount++;

//         // Device signalled halt?
//         // === Handle HALT ===
//         if (raw.flags & SAMPLE_FLAG_HALTED) {
//             spdlog::info("[Lynsyn] Device halted sampling");
//             if (config_.periodSampling) {
//                 armSamplingModeWithPeriodMs_(static_cast<int>(sr_ms));
//                 // Reset schedule so we don't burst after a halt
//                // now = clock::now();
//                 nextSampleTime = clock::now() + sampleInterval;
//                 //continue;
//             } else {
//                 break;
//             }
//             continue;
//         }
        
//         // === ALWAYS LOG TO CSV (if open) ===
//         // Convert & publish at most once per slot
//         now = clock::now();
//         const bool slotReady = (now - lastPublish) >= sampleInterval * 9 / 10; // small guard against jitter
//         if (slotReady) {

//             // === Publish to aggregator / callback ===
//             PowerStats stats = convertLynsynSampleToPowerStats_(raw);
//             if (metricAggregator_) {
//                 metricAggregator_->pushPowerStats(stats);
//             }
//             if (sampleCallback_) {
//                 sampleCallback_(raw);
//             }

//             // CSV
//             if (outputFile_.is_open()) {
//                 const double seconds = lynsyn_cyclesToSeconds(raw.time);
//                 outputFile_ << seconds
//                             << "," << raw.pc[0] << "," << raw.pc[1] << "," << raw.pc[2] << "," << raw.pc[3]
//                             << "," << raw.current[0] << "," << raw.voltage[0]
//                             << "," << raw.current[1] << "," << raw.voltage[1]
//                             << "," << raw.current[2] << "," << raw.voltage[2]
//                             << "\n" << std::flush; // <-- Add this to force the write
//             }

//             lastPublish = now;

//             if ((sampleCount % 1000) == 0) {
//                 spdlog::info("[Lynsyn] {} samples processed", sampleCount.load());
//             }

//             // if ((++sampleCount % 2000) == 0) {
//             //     spdlog::debug("[Lynsyn] processed {} samples", sampleCount.load());
//             // }
//         }
//         // Final flush
//          if (outputFile_.is_open()) {
//             outputFile_.flush();
//             }
//     }

//     spdlog::info("[Lynsyn] monitoring thread exiting");
// }

//==========================================================================================
void monitoringThreadLoop_() {
    spdlog::debug("[LynsynMonitorConcrete] monitoring thread started");

    // --------------------------------------------------------------------
    // 1. Single source of truth for the sampling period
    // --------------------------------------------------------------------
    const int64_t sr_ms = std::max<int64_t>(50, config_.sampleRateMs);   // hardware minimum
    const auto sampleInterval = std::chrono::milliseconds(sr_ms);

    // Warn if the old `period_ms` field disagrees with the real rate
    if (config_.periodSampling && config_.period_ms != static_cast<int>(sr_ms)) {
        spdlog::warn("[Lynsyn] config.period_ms ({}) != sampleRateMs ({}); using sampleRateMs",
                     config_.period_ms, sr_ms);
    }

    // --------------------------------------------------------------------
    // 2. Arm the device **once** with the chosen period
    // --------------------------------------------------------------------
    armSamplingModeWithPeriodMs_(static_cast<int>(sr_ms));

    // --------------------------------------------------------------------
    // 3. Failure-handling constants
    // --------------------------------------------------------------------
    constexpr int MAX_CONSECUTIVE_FAILS = 10;
    constexpr std::chrono::milliseconds BASE_BACKOFF{200};

    int consecutiveFails = 0;
    static std::atomic<size_t> sampleCount{0};

    // --------------------------------------------------------------------
    // 4. Timing anchors (steady_clock = monotonic, immune to NTP jumps)
    // --------------------------------------------------------------------
    using clock = std::chrono::steady_clock;
    auto nextSampleTime = clock::now() + sampleInterval;   // first sample after one interval

    // --------------------------------------------------------------------
    // 5. Main loop
    // --------------------------------------------------------------------
    while (!threadShouldExit_) {

        /* -----------------------------------------------------------
         * 5.1 Sleep exactly until the next scheduled slot
         * ----------------------------------------------------------- */
        std::this_thread::sleep_until(nextSampleTime);
        nextSampleTime += sampleInterval;               // schedule the *next* slot **now**

        /* -----------------------------------------------------------
         * 5.2 Pull a sample from the Lynsyn board
         * ----------------------------------------------------------- */
        LynsynSample raw{};
        const bool got = lynsyn_getNextSample(&raw);

        if (!got) {
            // ----- failure path -----
            ++consecutiveFails;
            const auto backoff = BASE_BACKOFF * (1 << std::min(consecutiveFails - 1, 5));
            spdlog::warn("[Lynsyn] getNextSample failed (#{})  back-off {} ms",
                         consecutiveFails, backoff.count());

            if (consecutiveFails == 2 || (consecutiveFails > 2 && consecutiveFails % 3 == 0)) {
                spdlog::warn("[Lynsyn] Trying device-only reset + re-arm");
                tryDeviceOnlyResetAndRearm_();
                // Re-sync the schedule after a reset
                nextSampleTime = clock::now() + sampleInterval;
            }

            if (consecutiveFails >= MAX_CONSECUTIVE_FAILS) {
                reportError("[Lynsyn] Too many consecutive failures  stopping monitor");
                break;
            }

            std::this_thread::sleep_for(backoff);
            continue;   // go to the top of the loop, keep the schedule intact
        }

        // ----- success path -----
        consecutiveFails = 0;
        ++sampleCount;

        /* -----------------------------------------------------------
         * 5.3 Device halted (e.g. end of breakpoint run)
         * ----------------------------------------------------------- */
        if (raw.flags & SAMPLE_FLAG_HALTED) {
            spdlog::info("[Lynsyn] Device reported HALT");
            if (config_.periodSampling) {
                armSamplingModeWithPeriodMs_(static_cast<int>(sr_ms));
                nextSampleTime = clock::now() + sampleInterval;   // avoid burst after re-arm
            } else {
                break;   // breakpoint mode finished
            }
            continue;
        }

        /* -----------------------------------------------------------
         * 5.4 **ALWAYS** write to CSV (if a file was requested)
         * ----------------------------------------------------------- */
        if (outputFile_.is_open()) {
            const double seconds = lynsyn_cyclesToSeconds(raw.time);

            outputFile_ << std::fixed << std::setprecision(6)
                        << seconds;
            for (int i = 0; i < 4; ++i)                     // 4 PCs
                outputFile_ << ',' << raw.pc[i];
            for (int i = 0; i < 7; ++i)                     // up to 7 sensors (future-proof)
                outputFile_ << ',' << raw.current[i] << ',' << raw.voltage[i];
            outputFile_ << '\n';

            // Flush every ~10 samples (1 second at 100 ms rate)  cheap & safe
            if ((sampleCount % 10) == 0) {
                outputFile_.flush();
            }
        }

        /* -----------------------------------------------------------
         * 5.5 Publish to the aggregator / user callback
         * ----------------------------------------------------------- */
        PowerStats stats = convertLynsynSampleToPowerStats_(raw);
        if (metricAggregator_) {
            metricAggregator_->pushPowerStats(stats);
        }
        if (sampleCallback_) {
            sampleCallback_(raw);
        }

        /* -----------------------------------------------------------
         * 5.6 Periodic progress log
         * ----------------------------------------------------------- */
        if ((sampleCount % 1000) == 0) {
            spdlog::info("[Lynsyn] {} samples processed (rate {} ms)", sampleCount.load(), sr_ms);
        }
    }

    // ----------------------------------------------------------------
    // 6. Final flush before the thread dies
    // ----------------------------------------------------------------
    if (outputFile_.is_open()) {
        outputFile_.flush();
    }

    spdlog::info("[Lynsyn] monitoring thread exiting");
}
//===========================================================================================


    // --- Helpers ------------------------------------------------------------
    // Define this in your build if your lib exposes a "set period" call.
// e.g., add -DLYN_SUPPORTS_SET_PERIOD_MS to your target if available.
// #define LYN_SUPPORTS_SET_PERIOD_MS

void armSamplingModeWithPeriodMs_(int period_ms) {
    const int ms = std::max(1, period_ms);

#ifdef LYN_SUPPORTS_SET_PERIOD_MS
    // Replace these with the actual lib calls your Lynsyn SDK provides.
    // Examples (pseudo; adjust to your API):
    //   lynsyn_set_period_ms(ms);
    //   lynsyn_set_core_mask(config_.coreMask);
    //   lynsyn_arm(config_.durationSec);
    try {
        lynsyn_set_period_ms(ms);
        lynsyn_set_core_mask(config_.coreMask);
        lynsyn_arm(config_.durationSec);
        spdlog::info("[Lynsyn] Armed device for periodic sampling: {} ms, coreMask={}, durationSec={}",
                     ms, config_.coreMask, config_.durationSec);
    } catch (...) {
        spdlog::warn("[Lynsyn] Period-API arming failed; falling back to legacy arming.");
        armSamplingMode_();
    }
#else
    spdlog::info("[Lynsyn] Library has no period_ms API; using legacy arming + host 1Hz scheduler.");
    armSamplingMode_();
#endif
}




    void armSamplingMode_() {
        // Decide and (re)start sampling according to config
        if (config_.periodSampling) {
            lynsyn_startPeriodSampling(config_.durationSec, config_.coreMask);
        } else if (config_.startBreakpoint != 0 && config_.endBreakpoint != 0) {
            lynsyn_startBpSampling(config_.startBreakpoint, config_.endBreakpoint, config_.coreMask);
        } else {
            // Fallback to periodic if config is incomplete
            lynsyn_startPeriodSampling(config_.durationSec, config_.coreMask);
        }
    }

    /**
     * Attempt a **device-only** reset + reopen using liblynsyn, then re-arm sampling.
     * Requires your liblynsyn to implement a robust lynsyn_reset_device() that:
     *  - releases interface,
     *  - does libusb_reset_device on Lynsyn,
     *  - closes/reopens the handle and reclaims the interface,
     *  - refreshes endpoints,
     *  - does NOT reset the hub/port.
     */
    void tryDeviceOnlyResetAndRearm_() {
        // We guard this whole sequence to avoid concurrent resets if you ever add more threads.
        std::lock_guard<std::mutex> g(lifecycleMutex_);

        // If someone requested shutdown, bail quickly.
        if (threadShouldExit_) return;

        // Ask lib to reset the device (device-only).
        const int ret = lynsyn_reset_device();
        if (ret != 0) {
            spdlog::error("[LynsynMonitorConcrete] lynsyn_reset_device failed: {}", ret);
            // Fall through to full re-init path below.
        }

        // Reinitialize Lynsyn lib (idempotent if your patched lib handles inited flags)
        // We do a cautious sequence: release -> small delay -> init.
        lynsyn_release();
        std::this_thread::sleep_for(std::chrono::milliseconds(200));

        if (!lynsyn_init()) {
            spdlog::error("[LynsynMonitorConcrete] lynsyn_init() failed after device-only reset");
            return;
        }

        // Re-arm sampling mode
        armSamplingMode_();
        spdlog::info("[LynsynMonitorConcrete] Lynsyn re-armed after device-only reset");
    }

    void reportError(const std::string& msg) {
        if (errorCallback_) {
            errorCallback_(msg);
        } else {
            spdlog::error("{}", msg);
        }
    }

    // Convert LynsynSample -> PowerStats (lightweight; assumes SI units from lib)
    PowerStats convertLynsynSampleToPowerStats_(const LynsynSample& s) {
        PowerStats stats;
        stats.timestamp = std::chrono::system_clock::now();

        // currents/voltages are doubles already (amps/volts) as produced by liblynsyns conversion
        const unsigned maxSensors = 3; // HW >= 3.0 has 3 sensors; others may map to 7 with zeros
        stats.currents.reserve(maxSensors);
        stats.voltages.reserve(maxSensors);
        for (unsigned i = 0; i < maxSensors; ++i) {
            stats.currents.push_back(static_cast<double>(s.current[i]));
            stats.voltages.push_back(static_cast<double>(s.voltage[i]));
        }
        return stats;
    }

   

private:
    // Config + callbacks
    LynsynMonitorConfig config_;
    std::function<void(const LynsynSample&)>   sampleCallback_;
    std::function<void(const std::string&)>    errorCallback_;

    // State
    std::atomic<bool> initialized_;
    std::atomic<bool> running_;
    ThreadManager&     threadManager_;
    std::atomic<bool>  threadShouldExit_;
    std::ofstream      outputFile_;
    std::shared_ptr<ISystemMetricsAggregator> metricAggregator_;

    // Guards
    std::mutex lifecycleMutex_;

    

};
