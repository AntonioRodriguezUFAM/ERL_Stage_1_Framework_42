// Modules.hpp

// ========================================================================================
// Modules.hpp  (all modules implementing IModule)
// ========================================================================================
#pragma once

#include <memory>
#include <string>
#include <thread>
#include <atomic>
#include <map>
#include <functional>
#include <stdexcept>
#include <unistd.h>                // access()
#include <spdlog/spdlog.h>

// -- TOP OF FILE (after #includes) -------------------------------------
#include <experimental/filesystem>   // <-- ADD
namespace fs = std::experimental::filesystem;   // <-- ADD


#include "../nlohmann/json.hpp"


#include "IModule.h"
#include "ModuleFactory.hpp"

class ThreadManager;  // forward-declare


//========================================================================================
// CORE: Context & Base Module Interface
//========================================================================================
// Context with ThreadManager and a global shutdown flag.
struct Context {
    std::atomic<bool> shutdown_flag{false};
    std::shared_ptr<ThreadManager> tm;
};



// ----------------------------- includes from your project ------------------------------
#include "../Stage_01/SharedStructures/SharedQueue.h"

#include "../Stage_01/SharedStructures/ZeroCopyFrameData.h"

#include "../Stage_01/SharedStructures/CameraConfig.h"
#include "../Stage_01/SharedStructures/DisplayConfig.h"
#include "../Stage_01/SharedStructures/AlgorithmConfig.h"
#include "../Stage_01/SharedStructures/LynsynMonitorConfig.h"
#include "../Stage_01/SharedStructures/allModulesStatcs.h"

#include "../Stage_01/Concretes/DataConcrete_new.h"
#include "../Stage_01/Concretes/AlgorithmConcrete_new.h"
#include "../Stage_01/Concretes/SdlDisplayConcrete_new.h"
#include "../Stage_01/Concretes/SoCConcrete_new.h"
#include "../Stage_01/Concretes/LynsynMonitorConcrete_new.h"
#include "../Stage_01/Concretes/SystemMetricsAggregatorConcrete_v3_2.h"
// ---------------------------------------------------------------------------------------

using json = nlohmann::json;

// ========================================================================================
// Small helpers (string -> enums) as used in your v11 main
// ========================================================================================
inline PixelFormat stringToPixelFormat(const std::string& format) {
    if (format == "YUYV") return PixelFormat::YUYV;
    if (format == "MJPG") return PixelFormat::MJPG;
    throw std::invalid_argument("Unsupported pixel format: " + format);
}

inline AlgorithmType stringToAlgorithmType(const std::string& type) {
    static const std::map<std::string, AlgorithmType> kMap = {
        {"Invert", AlgorithmType::Invert},
        {"Grayscale", AlgorithmType::Grayscale},
        {"EdgeDetection", AlgorithmType::EdgeDetection},
        {"PasswordHash", AlgorithmType::PasswordHash},
        {"MultiPipeline", AlgorithmType::MultiPipeline},
        {"MultiThreadedInvert", AlgorithmType::MultiThreadedInvert},
        {"GaussianBlur", AlgorithmType::GaussianBlur},
        {"OpticalFlow_LucasKanade", AlgorithmType::OpticalFlow_LucasKanade},
        {"MatrixMultiply", AlgorithmType::MatrixMultiply},
        {"Mandelbrot", AlgorithmType::Mandelbrot},
        {"GPUMatrixMultiply", AlgorithmType::GPUMatrixMultiply},
        {"SobelEdge", AlgorithmType::SobelEdge},
        {"MedianFilter", AlgorithmType::MedianFilter},
        {"HistogramEqualization", AlgorithmType::HistogramEqualization},
        {"HeterogeneousGaussianBlur", AlgorithmType::HeterogeneousGaussianBlur}
    };
    auto it = kMap.find(type);
    if (it == kMap.end()) throw std::invalid_argument("Unsupported algorithm type: " + type);
    return it->second;
}

// ===== BEGIN: disable placeholder registry =====
#if 0 
class ModuleRegistry {
public:
    static void registerModule(const std::string& name, std::function<std::unique_ptr<IModule>(json, Context&)> factory) {
        registry_[name] = factory;
    }
    static std::unique_ptr<IModule> create(const std::string& name, json cfg, Context& ctx) {
        auto it = registry_.find(name);
        if (it == registry_.end()) throw std::runtime_error("Unknown module: " + name);
        return it->second(cfg, ctx);
    }
private:
    static std::map<std::string, std::function<std::unique_ptr<IModule>(json, Context&)>> registry_;
};


//=========================================================================================
static bool cameraRegistered = (ModuleRegistry::registerModule("Camera", [](json cfg, Context& ctx) {
    return ModuleFactory::create<CameraModule>(cfg, ctx, ...);
}), true);
//=========================================================================================

#endif
// ===== END: disable placeholder registry =====

// ========================================================================================
// AggregatorModule
//  - Owns SystemMetricsAggregatorConcreteV3_2
//  - Provides accessor for other modules to share the same aggregator instance
// ========================================================================================
class AggregatorModule : public IModule {
public:
    explicit AggregatorModule(const json& cfg)
        : cfg_(cfg) {}

    bool validate() override {
        // Minimal sanity checks
        if (!cfg_.contains("metrics_csv") || !cfg_["metrics_csv"].is_string())
            cfg_["metrics_csv"] = "/tmp/build/realtime_metrics.csv";
        if (!cfg_.contains("metrics_json") || !cfg_["metrics_json"].is_string())
            cfg_["metrics_json"] = "/tmp/build/realtime_metrics.ndjson";
        if (!cfg_.contains("retention_window_sec") || !cfg_["retention_window_sec"].is_number_integer())
            cfg_["retention_window_sec"] = 2000;
        if (!cfg_.contains("merge_wait_ms") || !cfg_["merge_wait_ms"].is_number_integer())
            cfg_["merge_wait_ms"] = 50;
        if (!cfg_.contains("flush_period_ms") || !cfg_["flush_period_ms"].is_number_integer())
            cfg_["flush_period_ms"] = 1000;
        if (!cfg_.contains("json_flush_threshold") || !cfg_["json_flush_threshold"].is_number_integer())
            cfg_["json_flush_threshold"] = 2000;

        return true;
    }

    void start() override {
        spdlog::info("[AggregatorModule] Starting...");
        aggregator_ = std::make_shared<SystemMetricsAggregatorConcreteV3_2>(cfg_);
        spdlog::info("[AggregatorModule] Ready.");
    }

    void stop() override {
        spdlog::info("[AggregatorModule] Stopping...");
        if (aggregator_) {
            aggregator_->stop();
            aggregator_->forceFlushBatch();
        }
        aggregator_.reset();
        spdlog::info("[AggregatorModule] Stopped.");
    }

    std::shared_ptr<SystemMetricsAggregatorConcreteV3_2> get() const { return aggregator_; }

private:
    json cfg_;
    std::shared_ptr<SystemMetricsAggregatorConcreteV3_2> aggregator_;
};

// ========================================================================================
// CameraModule (wraps DataConcrete)
// ========================================================================================
class CameraModule : public IModule {
public:
    CameraModule(const json& cfg,
                 Context& ctx,
                 std::shared_ptr<SharedQueue<std::shared_ptr<ZeroCopyFrameData>>> cam2Alg,
                 std::shared_ptr<SharedQueue<std::shared_ptr<ZeroCopyFrameData>>> cam2Disp,
                 std::shared_ptr<SystemMetricsAggregatorConcreteV3_2> aggregator)
        : cfg_(cfg), ctx_(ctx),
          cam2Alg_(std::move(cam2Alg)), cam2Disp_(std::move(cam2Disp)),
          aggregator_(std::move(aggregator)) {}

    bool validate() override {
        spdlog::info("[CameraModule] Validating configuration...");
        if (!cfg_.contains("width")  || !cfg_["width"].is_number_integer())  cfg_["width"]  = 640;
        if (!cfg_.contains("height") || !cfg_["height"].is_number_integer()) cfg_["height"] = 480;
        if (!cfg_.contains("fps")    || !cfg_["fps"].is_number_integer())    cfg_["fps"]    = 30;
        if (!cfg_.contains("pixelFormat") || !cfg_["pixelFormat"].is_string())
            cfg_["pixelFormat"] = "YUYV";
        
        int fps = cfg_.value("fps", 30);
        fps = std::max(1, std::min(fps, 120));   // 1-120 FPS
        cfg_["fps"] = fps;

        if (access("/dev/video0", F_OK) != 0) {
            spdlog::error("[CameraModule] /dev/video0 not found");
            return false;
        }
        return true;
    }

//================================================================
//================= CANERAMODULE START ===========================
// Changes:

// Added camCfg to pass the CameraConfig object.
// Added ctx_.tm to provide the ThreadManager instance (assuming Context holds a valid tm).
// Reordered arguments to match the constructor signature.
//================================================================
    void start() override {
        spdlog::info("[CameraModule] Starting...");

        CameraConfig camCfg{
            cfg_.value("width", 640),
            cfg_.value("height", 480),
            cfg_.value("fps", 30),
            stringToPixelFormat(cfg_.value("pixelFormat", "YUYV"))
        };
        camera_ = std::make_unique<DataConcrete>(camCfg, ctx_.tm, cam2Alg_, cam2Disp_, aggregator_);

        // == ctx_.tm is a std::shared_ptr<ThreadManager> ? you must dereference it:
        //camera_ = std::make_unique<DataConcrete>(camCfg, *ctx_.tm, cam2Alg_, cam2Disp_, aggregator_);
        
        if (!camera_->openDevice("/dev/video0") || !camera_->configure(camCfg)) {
            throw std::runtime_error("[CameraModule] Failed to initialize camera");
        }
        camera_->startCapture();
        spdlog::info("[CameraModule] Streaming started.");
    }


//================================================================
//================= CANERAMODULE END =============================
//================================================================
    void stop() override {
        spdlog::info("[CameraModule] Stopping...");
        if (camera_) {
            camera_->stopCapture();

            camera_.reset();
        }
        spdlog::info("[CameraModule] Stopped.");
    }

private:
    json cfg_;
    Context& ctx_;
    std::unique_ptr<DataConcrete> camera_;
    std::shared_ptr<SharedQueue<std::shared_ptr<ZeroCopyFrameData>>> cam2Alg_, cam2Disp_;
    std::shared_ptr<SystemMetricsAggregatorConcreteV3_2> aggregator_;
};

// ========================================================================================
// AlgorithmModule (wraps AlgorithmConcrete)
// ========================================================================================
class AlgorithmModule : public IModule {
public:
    AlgorithmModule(const json& cfg,
                    Context& ctx,
                    std::shared_ptr<SharedQueue<std::shared_ptr<ZeroCopyFrameData>>> cam2Alg,
                    std::shared_ptr<SharedQueue<std::shared_ptr<ZeroCopyFrameData>>> alg2Disp,
                    ThreadManager& tm,
                    std::shared_ptr<SystemMetricsAggregatorConcreteV3_2> aggregator)
        : cfg_(cfg), ctx_(ctx), tm_(tm),
          cam2Alg_(std::move(cam2Alg)), alg2Disp_(std::move(alg2Disp)),
          aggregator_(std::move(aggregator)) {}

    bool validate() override {
        spdlog::info("[AlgorithmModule] Validating configuration...");
        if (!cfg_.contains("algorithmType") || !cfg_["algorithmType"].is_string())
            cfg_["algorithmType"] = "Invert";
        if (!cfg_.contains("concurrencyLevel") || !cfg_["concurrencyLevel"].is_number_integer())
            cfg_["concurrencyLevel"] = 4;
        if (!cfg_.contains("blurRadius") || !cfg_["blurRadius"].is_number_integer())
            cfg_["blurRadius"] = 5;
        if (!cfg_.contains("medianWindowSize") || !cfg_["medianWindowSize"].is_number_integer())
            cfg_["medianWindowSize"] = 5;
        if (!cfg_.contains("matrixSize") || !cfg_["matrixSize"].is_number_integer())
            cfg_["matrixSize"] = 512;
        if (!cfg_.contains("mandelbrotIter") || !cfg_["mandelbrotIter"].is_number_integer())
            cfg_["mandelbrotIter"] = 100;
        if (!cfg_.contains("useGPU") || !cfg_["useGPU"].is_boolean())
            cfg_["useGPU"] = false;

        // Validate that algorithmType is supported
        (void)stringToAlgorithmType(cfg_.value("algorithmType", "Invert"));
        return true;
    }

    void start() override {
        spdlog::info("[AlgorithmModule] Starting...");
        AlgorithmConfig algCfg{
            cfg_.value("concurrencyLevel", 4),
            stringToAlgorithmType(cfg_.value("algorithmType", "Invert")),
            cfg_.value("modelPath", ""),
            cfg_.value("matrixSize", 512),
            cfg_.value("mandelbrotIter", 100),
            cfg_.value("blurRadius", 5),
            cfg_.value("medianWindowSize", 5),
            cfg_.value("useGPU", false),
            OpticalFlowConfig{} // default
        };

        algo_ = std::make_unique<AlgorithmConcrete>(cam2Alg_, alg2Disp_, tm_, aggregator_);
        algo_->startAlgorithm();  // AlgorithmConcrete owns its thread(s)
        spdlog::info("[AlgorithmModule] Started.");
    }

    void stop() override {
        spdlog::info("[AlgorithmModule] Stopping...");
        if (algo_) {
            algo_->stopAlgorithm();
            algo_.reset();
        }
        spdlog::info("[AlgorithmModule] Stopped.");
    }

private:
    json cfg_;
    Context& ctx_;
    ThreadManager& tm_;
    std::unique_ptr<AlgorithmConcrete> algo_;
    std::shared_ptr<SharedQueue<std::shared_ptr<ZeroCopyFrameData>>> cam2Alg_, alg2Disp_;
    std::shared_ptr<SystemMetricsAggregatorConcreteV3_2> aggregator_;
};

// ========================================================================================
// DisplayModule (wraps SdlDisplayConcrete)
//  - Manages a small internal thread that repeatedly calls renderAndPollEvents()
// ========================================================================================
class DisplayModule : public IModule {
public:
    DisplayModule(const json& cfg,
                  Context& ctx,
                  std::shared_ptr<SharedQueue<std::shared_ptr<ZeroCopyFrameData>>> cam2Disp,
                  std::shared_ptr<SharedQueue<std::shared_ptr<ZeroCopyFrameData>>> alg2Disp,
                  std::shared_ptr<SystemMetricsAggregatorConcreteV3_2> aggregator)
        : cfg_(cfg), ctx_(ctx),
          cam2Disp_(std::move(cam2Disp)), alg2Disp_(std::move(alg2Disp)),
          aggregator_(std::move(aggregator)) {}

    bool validate() override {
        spdlog::info("[DisplayModule] Validating configuration...");
        if (!cfg_.contains("width")  || !cfg_["width"].is_number_integer())  cfg_["width"]  = 640;
        if (!cfg_.contains("height") || !cfg_["height"].is_number_integer()) cfg_["height"] = 480;
        if (!cfg_.contains("fullscreen") || !cfg_["fullscreen"].is_boolean()) cfg_["fullscreen"] = false;
        if (!cfg_.contains("vsync")      || !cfg_["vsync"].is_boolean())      cfg_["vsync"]      = true;
        return true;
    }

    void start() override {
        spdlog::info("[DisplayModule] Starting...");
        display_ = std::make_unique<SdlDisplayConcrete>(cam2Disp_, alg2Disp_, aggregator_);

        const int w = cfg_.value("width", 640);
        const int h = cfg_.value("height", 480);
        if (!display_->initializeDisplay(w, h)) {
            throw std::runtime_error("[DisplayModule] Failed to initialize display");
        }

        running_.store(true);
        loopThread_ = std::thread([this]() {
            spdlog::info("[DisplayModule] Render loop started.");
            // while (running_.load() && display_ && display_->is_Running() && !ctx_.shutdown_flag.load()) {
            //     display_->renderAndPollEvents();
            //     std::this_thread::sleep_for(std::chrono::milliseconds(1));
            // }
            while (running_.load() && display_ && display_->is_Running() && !ctx_.shutdown_flag.load()) {
                const auto frame_start = std::chrono::steady_clock::now();
                display_->renderAndPollEvents();

                // Target 60 FPS if vsync is off
                if (!cfg_.value("vsync", true)) {
                    const auto target = std::chrono::milliseconds(16); // ~60 Hz
                    const auto elapsed = std::chrono::steady_clock::now() - frame_start;
                    if (elapsed < target)
                        std::this_thread::sleep_for(target - elapsed);
                }
            }


            spdlog::info("[DisplayModule] Render loop finished.");
        });
    }

    void stop() override {
        spdlog::info("[DisplayModule] Stopping...");
        running_.store(false);
        if (loopThread_.joinable()) loopThread_.join();
        if (display_) {
            display_->closeDisplay();
            display_.reset();
        }
        spdlog::info("[DisplayModule] Stopped.");
    }

private:
    json cfg_;
    Context& ctx_;

    std::unique_ptr<SdlDisplayConcrete> display_;
    std::shared_ptr<SharedQueue<std::shared_ptr<ZeroCopyFrameData>>> cam2Disp_, alg2Disp_;
    std::shared_ptr<SystemMetricsAggregatorConcreteV3_2> aggregator_;

    std::atomic<bool> running_{false};
    std::thread loopThread_;
};

// ========================================================================================
// SoCModule (wraps SoCConcrete)
//  - Your SoCConcrete_new in v11 used initializeSoC(); monitoring ran internally
// ========================================================================================
class SoCModule : public IModule {
public:
    SoCModule(const json& /*cfg*/,
              Context& /*ctx*/,
              std::shared_ptr<SystemMetricsAggregatorConcreteV3_2> aggregator)
        : aggregator_(std::move(aggregator)) {}

    bool validate() override {
        // Nothing special here (you can add cfg validation if needed)
        return true;
    }

    void start() override {
        spdlog::info("[SoCModule] Starting...");
        soc_ = std::make_unique<SoCConcrete>(aggregator_);
        if (!soc_->initializeSoC()) {
            throw std::runtime_error("[SoCModule] initializeSoC() failed");
        }
        spdlog::info("[SoCModule] Started.");
    }

    void stop() override {
        spdlog::info("[SoCModule] Stopping...");
        soc_.reset();
        spdlog::info("[SoCModule] Stopped.");
    }

private:
    std::unique_ptr<SoCConcrete> soc_;
    std::shared_ptr<SystemMetricsAggregatorConcreteV3_2> aggregator_;
};

// ========================================================================================
// LynsynModule (wraps LynsynMonitorConcrete)
// ========================================================================================

// ========================================================================================
// LynsynModule (wraps LynsynMonitorConcrete)
// ========================================================================================
class LynsynModule : public IModule {
public:
    LynsynModule(const json& cfg,
                 Context& ctx,
                 ThreadManager& tm,
                 std::shared_ptr<SystemMetricsAggregatorConcreteV3_2> aggregator)
        : cfg_(cfg), ctx_(ctx), tm_(tm), aggregator_(std::move(aggregator)) {}

//====================================================================================================================

// REPLACE THE ENTIRE validate() WITH THIS:
bool validate() override {
    spdlog::info("[LynsynModule] Validating configuration...");

    // [MOD] Sanitize outputCSV early to prevent segfault in std::filesystem::path
    std::string raw_output = cfg_.value("outputCSV", "/tmp/lynsyn_output.csv");
    std::string sanitized_output;

    auto sanitize_path = [](const std::string& in) -> std::string {
        if (in.empty()) {
            spdlog::warn("[LynsynModule] outputCSV is empty, using default: /tmp/lynsyn_output.csv");
            return "/tmp/lynsyn_output.csv";
        }
        // [MOD] Reject control characters to prevent invalid path crashes
        for (char c : in) {
            if (static_cast<unsigned char>(c) < 32) {
                spdlog::warn("[LynsynModule] outputCSV '{}' contains control characters, using default", in);
                return "/tmp/lynsyn_output.csv";
            }
        }
        try {
            fs::path p(in);
            // [MOD] Convert relative to absolute paths
            if (p.is_relative()) {
                std::string abs_path = fs::absolute(p).string();
                spdlog::debug("[LynsynModule] Converted relative path '{}' to absolute: '{}'", in, abs_path);
                return abs_path;
            }
            return in;
        } catch (const fs::filesystem_error& e) {
            spdlog::warn("[LynsynModule] Invalid outputCSV '{}': {}. Using default", in, e.what());
            return "/tmp/lynsyn_output.csv";
        } catch (...) {
            spdlog::warn("[LynsynModule] Unexpected error processing outputCSV '{}'. Using default", in);
            return "/tmp/lynsyn_output.csv";
        }
    };

    sanitized_output = sanitize_path(raw_output);
    cfg_["outputCSV"] = sanitized_output; // [MOD] Update config with sanitized path

    // [MOD] Test file writability
    try {
        std::ofstream test(sanitized_output, std::ios::out | std::ios::app);
        if (!test) {
            spdlog::error("[LynsynModule] Cannot write to '{}', falling back to /tmp/lynsyn_output.csv", sanitized_output);
            sanitized_output = "/tmp/lynsyn_output.csv";
            cfg_["outputCSV"] = sanitized_output;
        }
    } catch (const std::exception& e) {
        spdlog::warn("[LynsynModule] Failed to test writability for '{}': {}. Using default", sanitized_output, e.what());
        sanitized_output = "/tmp/lynsyn_output.csv";
        cfg_["outputCSV"] = sanitized_output;
    }

    // [MOD] Fill other configuration defaults
    if (!cfg_.contains("enabled") || !cfg_["enabled"].is_boolean()) cfg_["enabled"] = true;
    if (!cfg_.contains("coreMask") || !cfg_["coreMask"].is_number_integer()) cfg_["coreMask"] = 15;
    if (!cfg_.contains("periodSampling") || !cfg_["periodSampling"].is_boolean()) cfg_["periodSampling"] = true;
    if (!cfg_.contains("durationSec") || !cfg_["durationSec"].is_number()) cfg_["durationSec"] = 5.0;
    if (!cfg_.contains("sampleRateMs") || !cfg_["sampleRateMs"].is_number_integer()) cfg_["sampleRateMs"] = 1000;
    if (!cfg_.contains("startBreakpoint") || !cfg_["startBreakpoint"].is_number_unsigned()) cfg_["startBreakpoint"] = 0ULL;
    if (!cfg_.contains("endBreakpoint") || !cfg_["endBreakpoint"].is_number_unsigned()) cfg_["endBreakpoint"] = 0ULL;

    // Early exit if module disabled
    if (!cfg_.value("enabled", true)) {
        spdlog::info("[LynsynModule] Disabled by config, validation passed");
        return true;
    }

    // [MOD] Create parent directory for CSV, avoid perm_options
    try {
        fs::path p(sanitized_output);
        fs::path parent = p.parent_path();
        if (!parent.empty()) {
            fs::create_directories(parent);
            // [MOD] Removed fs::perm_options for compatibility with std::experimental::filesystem
            // Default permissions (rwxr-xr-x) set by create_directories are sufficient
            spdlog::info("[LynsynModule] Ensured directory: {}", parent.string());
        }
    } catch (const std::exception& e) {
        spdlog::warn("[LynsynModule] Failed to create CSV directory: {}", e.what());
    }

    // [MOD] Log final resolved path
    spdlog::info("[LynsynModule] Using outputCSV: '{}'", sanitized_output);

    return true;
}
//====================================================================================================================
    void start() override {
        if (!cfg_.value("enabled", true)) {
            spdlog::info("[LynsynModule] Disabled by config; skipping start.");
            return;
        }

        spdlog::info("[LynsynModule] Starting...");
        power_ = std::make_unique<LynsynMonitorConcrete>(aggregator_, tm_);

        // --- Named, type-safe config population ---
        LynsynMonitorConfig lcfg{};
        lcfg.outputCSV       = cfg_.value("outputCSV", std::string("/tmp/lynsyn_output.csv"));
        lcfg.periodSampling  = cfg_.value("periodSampling", true);
        lcfg.durationSec     = cfg_.value("durationSec", 5.0);            // double
        lcfg.coreMask        = static_cast<uint32_t>(cfg_.value("coreMask", 15));
        lcfg.startBreakpoint = cfg_.value("startBreakpoint", 0ULL);
        lcfg.sampleRateMs    = cfg_.value("sampleRateMs", 1000);          // int
        lcfg.endBreakpoint   = cfg_.value("endBreakpoint", 0ULL);

        // Some parts of your monitor reference config_.period_ms; keep it aligned with sampleRateMs.
        // If LynsynMonitorConfig already has period_ms, set it; if not, you can add it to the struct.
        lcfg.period_ms       = lcfg.sampleRateMs;                          // keep device period == host rate

        // Sane clamp for safety (your monitor loop clamps too, but keep it consistent)
        if (lcfg.sampleRateMs < 100)  lcfg.sampleRateMs = 100;
        if (lcfg.period_ms   < 100)   lcfg.period_ms    = 100;

        // --- Configure + Initialize ---
        if (!power_->configure(lcfg) || !power_->initialize()) {
            spdlog::error("[LynsynModule] configure/initialize failed; disabling power expectations.");
            // Avoid aggregator waiting for power rows
            if (aggregator_) {
                auto cfgA = aggregator_->getConfig();
                cfgA.expectsPower = false;
                aggregator_->updateConfig(cfgA);
            }
            power_.reset();
            // Throw if you want to fail the whole pipeline; otherwise return to continue without power.
            // throw std::runtime_error("[LynsynModule] configure/initialize failed");
            return;
        }

        power_->setErrorCallback([](const std::string& msg){
            spdlog::error("[Lynsyn Error] {}", msg);
        });

        power_->startMonitoring();
        spdlog::info("[LynsynModule] Started.");
    }

    void stop() override {
        spdlog::info("[LynsynModule] Stopping...");
        if (power_) {
            power_->stop();
            power_.reset();
        }
        spdlog::info("[LynsynModule] Stopped.");
    }

private:
    json cfg_;
    Context& ctx_;
    ThreadManager& tm_;
    std::unique_ptr<LynsynMonitorConcrete> power_;
    std::shared_ptr<SystemMetricsAggregatorConcreteV3_2> aggregator_;
};
