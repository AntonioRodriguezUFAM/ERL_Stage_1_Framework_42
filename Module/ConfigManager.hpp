//==================================================================================================
// ConfigManager.hpp  Null-safe config loader + pipeline builder
// - Mirrors the embedded class in SampleTestIntegrationCode_v14.cpp exactly
// - Uses std::experimental::filesystem to match Ubuntu 18.04 toolchains
// - Avoids json.value(...) on null by using small helpers
//==================================================================================================

#pragma once

//==================================================================================================
// ConfigManager.hpp  Null-safe config loader + pipeline builder
// - Uses std::experimental::filesystem to match Ubuntu 18.04 toolchains
// - Avoids json.value(...) on null by using small helpers
//==================================================================================================

#include <memory>
#include <vector>
#include <string>
#include <fstream>
#include <stdexcept>
#include <algorithm>
#include <experimental/filesystem>

#include <spdlog/spdlog.h>
//#include <nlohmann/json.hpp>

#include "../nlohmann/json.hpp"

#include "IModule.h"
#include "Modules.hpp"          // CameraModule, AlgorithmModule, DisplayModule, SoCModule, LynsynModule + Context
#include "ModuleFactory.hpp"    // ModuleFactory::create<...>()

#include ".././Stage_01/SharedStructures/SharedQueue.h"
#include ".././Stage_01/SharedStructures/ZeroCopyFrameData.h"
#include ".././Stage_01/SharedStructures/ThreadManager.h"
#include ".././Stage_01/Concretes/SystemMetricsAggregatorConcrete_v3_2.h"

namespace fs = std::experimental::filesystem;
using json  = nlohmann::json;

// ---------- null-safe JSON helpers ----------
template<typename T>
inline T jvalue(const json& j, const char* key, const T& def) {
    if (j.is_object()) {
        auto it = j.find(key);
        if (it != j.end() && !it->is_null()) {
            try { return it->get<T>(); } catch (...) {}
        }
    }
    return def;
}
inline json jobject_or_empty(const json& parent, const char* key) {
    if (parent.is_object()) {
        auto it = parent.find(key);
        if (it != parent.end() && it->is_object()) return *it;
    }
    return json::object();
}

// ---------- ConfigManager ----------
class ConfigManager {
public:
    ConfigManager(const std::string& configPath, Context& ctx)
        : ctx_(ctx)
    {
        if (fs::exists(configPath)) {
            std::ifstream f(configPath);
            try {
                config_ = json::parse(f);
                spdlog::info("Loaded configuration from {}", configPath);
            } catch (const std::exception& e) {
                spdlog::warn("Failed parsing {} ({}). Using defaults.", configPath, e.what());
                config_ = json::object();
            }
        } else {
            spdlog::warn("Config file not found: {}. Using defaults.", configPath);
            config_ = json::object();
        }
        if (!config_.is_object()) {
            spdlog::warn("Top-level config is not an object; resetting to defaults.");
            config_ = json::object();
        }
        sanitizeDefaults(config_);
    }

    ConfigManager(const json& cfg, Context& ctx)
        : ctx_(ctx), config_(cfg)
    {
        if (!config_.is_object()) {
            spdlog::warn("Top-level config is not an object; resetting to defaults.");
            config_ = json::object();
        }
        sanitizeDefaults(config_);
    }

    // void buildPipeline() {
    //     buildQueues_();

    //     const json jCamera = jobject_or_empty(config_, "CameraConfig");
    //     const json jAgg    = jobject_or_empty(config_, "Aggregator");
    //     const json jLynsyn = jobject_or_empty(config_, "LynsynMonitorConfig");

    //     const int cameraFps         = jvalue<int>(jCamera, "fps", 30);
    //     const int defaultMergeWait  = std::max(10, std::min(250, (cameraFps > 0 ? (1000 / cameraFps) / 2 : 33)));
    //     const int mergeWaitMs       = jvalue<int>(jAgg, "merge_wait_ms", defaultMergeWait);

    //     json aggCfg = {
    //         {"metrics_csv",          jvalue<std::string>(config_, "metrics_csv",  "/tmp/build/realtime_metrics_011.csv")},
    //         {"metrics_json",         jvalue<std::string>(config_, "metrics_json", "/tmp/build/realtime_metrics011.ndjson")},
    //         {"retention_window_sec", jvalue<int>(config_, "retention_window_sec", 2000)},
    //         {"expectsPower",         jvalue<bool>(jLynsyn, "enabled", true)},
    //         {"merge_wait_ms",        mergeWaitMs},
    //         {"flush_period_ms",      jvalue<int>(jAgg, "flush_period_ms", 1000)},
    //         {"json_flush_threshold", jvalue<int>(jAgg, "json_flush_threshold", 2000)}
    //     };

    //     aggregator_ = std::make_shared<SystemMetricsAggregatorConcreteV3_2>(aggCfg);
    //     spdlog::info("[ConfigManager] Aggregator initialized: merge_wait_ms={}ms, retention={}s",
    //                  aggCfg["merge_wait_ms"].get<int>(),
    //                  jvalue<int>(config_, "retention_window_sec", 2000));

void buildPipeline() {
    buildQueues_();
    const json jCamera = jobject_or_empty(config_, "CameraConfig");
    const json jAgg = jobject_or_empty(config_, "Aggregator");
    const json jLynsyn = jobject_or_empty(config_, "LynsynMonitorConfig");
    const int cameraFps = jvalue<int>(jCamera, "fps", 30);
    const int defaultMergeWait = std::max(10, std::min(250, (cameraFps > 0 ? (1000 / cameraFps) / 2 : 33)));
    const int mergeWaitMs = jvalue<int>(jAgg, "merge_wait_ms", defaultMergeWait);

    // Include all Aggregator keys, sanitized
    json aggCfg = {
        {"metrics_csv", jvalue<std::string>(config_, "metrics_csv", "/tmp/build/realtime_metrics_011.csv")},
        {"metrics_json", jvalue<std::string>(config_, "metrics_json", "/tmp/build/realtime_metrics011.ndjson")},
        {"retention_window_sec", jvalue<int>(config_, "retention_window_sec", 2000)},
        {"expectsPower", jvalue<bool>(jLynsyn, "enabled", true)},
        {"expectsCamera", jvalue<bool>(jAgg, "expectsCamera", true)},
        {"expectsAlgorithm", jvalue<bool>(jAgg, "expectsAlgorithm", true)},
        {"expectsDisplay", jvalue<bool>(jAgg, "expectsDisplay", true)},
        {"expectsSoC", jvalue<bool>(jAgg, "expectsSoC", true)},
        {"merge_wait_ms", mergeWaitMs},
        {"flush_period_ms", jvalue<int>(jAgg, "flush_period_ms", 1000)},
        {"json_flush_threshold", jvalue<int>(jAgg, "json_flush_threshold", 2000)},
        {"drop_empty_compat_rows", jvalue<bool>(jAgg, "drop_empty_compat_rows", true)},
        {"drop_empty_flush_rows", jvalue<bool>(jAgg, "drop_empty_flush_rows", true)}
    };

    aggregator_ = std::make_shared<SystemMetricsAggregatorConcreteV3_2>(aggCfg);
    spdlog::info("[ConfigManager] Aggregator initialized: merge_wait_ms={}ms, retention={}s",
                 aggCfg["merge_wait_ms"].get<int>(),
                 jvalue<int>(config_, "retention_window_sec", 2000));
    // ... rest unchanged ...
//===============================================================================================
        const json jSoC  = jobject_or_empty(config_, "SoCConfig");
        const json jAlg  = jobject_or_empty(config_, "AlgorithmConfig");
        const json jDisp = jobject_or_empty(config_, "DisplayConfig");

        modules_.push_back(ModuleFactory::create<SoCModule>(jSoC, ctx_, aggregator_));

        try {
            if (jvalue<bool>(jLynsyn, "enabled", true)) {
                modules_.push_back(ModuleFactory::create<LynsynModule>(jLynsyn, ctx_, *ctx_.tm, aggregator_));
                spdlog::info("[ConfigManager] Lynsyn module initialized.");
            } else {
                spdlog::info("[ConfigManager] Lynsyn disabled via config.");
                if (aggregator_) {
                    auto cfg = aggregator_->getConfig();
                    cfg.expectsPower = false;
                    aggregator_->updateConfig(cfg);
                }
            }
        } catch (const std::exception& e) {
            spdlog::error("[ConfigManager] Failed to initialize LynsynModule: {}. Disabling power metrics.", e.what());
            if (aggregator_) {
                auto cfg = aggregator_->getConfig();
                cfg.expectsPower = false;
                aggregator_->updateConfig(cfg);
            }
        }

        modules_.push_back(ModuleFactory::create<CameraModule>(jCamera, ctx_, cam2Alg_, cam2Disp_, aggregator_));
        modules_.push_back(ModuleFactory::create<AlgorithmModule>(jAlg, ctx_, cam2Alg_, alg2Disp_, *ctx_.tm, aggregator_));
        modules_.push_back(ModuleFactory::create<DisplayModule>(jDisp, ctx_, cam2Disp_, alg2Disp_, aggregator_));

        spdlog::info("[ConfigManager] Pipeline built: {} modules.", modules_.size());
    }

    bool validateAll() {
        try {
            for (auto& m : modules_) {
                if (!m->validate()) {
                    spdlog::error("[ConfigManager] Module validation failed.");
                    return false;
                }
            }
        } catch (const std::exception& e) {
            spdlog::error("[ConfigManager] Exception during validateAll(): {}", e.what());
            return false;
        } catch (...) {
            spdlog::error("[ConfigManager] Unknown exception during validateAll().");
            return false;
        }
        spdlog::info("[ConfigManager] All modules validated.");
        return true;
    }

    void startAll() {
        for (auto& m : modules_) m->start();
        spdlog::info("[ConfigManager] All modules started.");
    }

    void stopAll() {
        for (auto it = modules_.rbegin(); it != modules_.rend(); ++it) {
            (*it)->stop();
        }
        spdlog::info("[ConfigManager] All modules stopped.");
    }

    bool runLoop(double runSeconds) {
        auto startTime = std::chrono::steady_clock::now();
        while (!ctx_.shutdown_flag.load()) {
            auto now = std::chrono::steady_clock::now();
            if (std::chrono::duration<double>(now - startTime).count() > runSeconds) {
                spdlog::info("[ConfigManager] Runtime limit reached ({} s).", runSeconds);
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        return true;
    }

    void flushMetrics() {
        if (aggregator_) {
            spdlog::info("[ConfigManager] Flushing metrics...");
            aggregator_->stop();
            aggregator_->forceFlushBatch();
            const json jProf = jobject_or_empty(config_, "Profiling");
            aggregator_->exportToCSV(jvalue<std::string>(jProf, "metricsOutputFile", "/tmp/PerformanceMetrics.csv"));
            spdlog::info("[ConfigManager] Metrics exported successfully.");
        }
    }

    const json& config() const { return config_; }

    // Optional accessors
    std::shared_ptr<SystemMetricsAggregatorConcreteV3_2> aggregator() const { return aggregator_; }
    std::shared_ptr<SharedQueue<std::shared_ptr<ZeroCopyFrameData>>> cam2Alg()  const { return cam2Alg_;  }
    std::shared_ptr<SharedQueue<std::shared_ptr<ZeroCopyFrameData>>> cam2Disp() const { return cam2Disp_; }
    std::shared_ptr<SharedQueue<std::shared_ptr<ZeroCopyFrameData>>> alg2Disp() const { return alg2Disp_; }

private:
    static void sanitizeDefaults(json& cfg) {
        auto ensure_parent_dir = [](const std::string& p) {
            if (p.empty()) return;
            fs::path path(p);
            if (!path.parent_path().empty() && !fs::exists(path.parent_path())) {
                fs::create_directories(path.parent_path());
                spdlog::info("Created directory: {}", path.parent_path().string());
            }
        };

        if (!cfg.contains("metrics_csv")  || !cfg["metrics_csv"].is_string())
            cfg["metrics_csv"]  = "/tmp/build/realtime_metrics_011.csv";
        if (!cfg.contains("metrics_json") || !cfg["metrics_json"].is_string())
            cfg["metrics_json"] = "/tmp/build/realtime_metrics011.ndjson";
        ensure_parent_dir(cfg["metrics_csv"].get<std::string>());
        ensure_parent_dir(cfg["metrics_json"].get<std::string>());

        if (!cfg.contains("retention_window_sec") || !cfg["retention_window_sec"].is_number_integer())
            cfg["retention_window_sec"] = 2000;

        if (!cfg.contains("Profiling")) cfg["Profiling"] = json::object();
        auto& prof = cfg["Profiling"];
        if (!prof.contains("metricsOutputFile") || !prof["metricsOutputFile"].is_string())
            prof["metricsOutputFile"] = "/tmp/PerformanceMetrics.csv";
        if (!prof.contains("samplingIntervalMs") || !prof["samplingIntervalMs"].is_number_integer())
            prof["samplingIntervalMs"] = 1000;
        ensure_parent_dir(prof["metricsOutputFile"].get<std::string>());

        if (!cfg.contains("CameraConfig")) cfg["CameraConfig"] = json::object();
        auto& cam = cfg["CameraConfig"];
        if (!cam.contains("width"))       cam["width"]  = 320;
        if (!cam.contains("height"))      cam["height"] = 240;
        if (!cam.contains("fps"))         cam["fps"]    = 30;
        if (!cam.contains("pixelFormat")) cam["pixelFormat"] = "YUYV";

        if (!cfg.contains("DisplayConfig")) cfg["DisplayConfig"] = json::object();
        auto& dsp = cfg["DisplayConfig"];
        if (!dsp.contains("width"))      dsp["width"]  = cam.value("width", 320);
        if (!dsp.contains("height"))     dsp["height"] = cam.value("height", 240);
        if (!dsp.contains("fullscreen")) dsp["fullscreen"] = false;
        if (!dsp.contains("vsync"))      dsp["vsync"]      = true;

        if (!cfg.contains("AlgorithmConfig")) cfg["AlgorithmConfig"] = json::object();
        auto& alg = cfg["AlgorithmConfig"];
        if (!alg.contains("algorithmType"))     alg["algorithmType"]     = "Invert";
        if (!alg.contains("concurrencyLevel"))  alg["concurrencyLevel"]  = 4;
        if (!alg.contains("blurRadius"))        alg["blurRadius"]        = 5;
        if (!alg.contains("medianWindowSize"))  alg["medianWindowSize"]  = 5;
        if (!alg.contains("matrixSize"))        alg["matrixSize"]        = 512;
        if (!alg.contains("mandelbrotIter"))    alg["mandelbrotIter"]    = 100;
        if (!alg.contains("useGPU"))            alg["useGPU"]            = false;

        if (!cfg.contains("SoCConfig")) cfg["SoCConfig"] = json::object();
        auto& soc = cfg["SoCConfig"];
        if (!soc.contains("pollIntervalMs")) soc["pollIntervalMs"] = 500;
        if (!soc.contains("customCommand"))  soc["customCommand"]  = "tegrastats --interval 500";
        if (!soc.contains("exportCSV"))      soc["exportCSV"]      = false;
        if (!soc.contains("csvPath"))        soc["csvPath"]        = "/tmp/soc_metrics.csv";
        if (soc["csvPath"].is_string() && !soc["csvPath"].get<std::string>().empty())
            ensure_parent_dir(soc["csvPath"].get<std::string>());

        if (!cfg.contains("LynsynMonitorConfig")) cfg["LynsynMonitorConfig"] = json::object();
        auto& lyn = cfg["LynsynMonitorConfig"];
        if (!lyn.contains("enabled"))        lyn["enabled"]        = true;
        if (!lyn.contains("coreMask"))       lyn["coreMask"]       = 15;
        if (!lyn.contains("outputCSV"))      lyn["outputCSV"]      = "/tmp/lynsyn_output.csv";
        if (!lyn.contains("periodSampling")) lyn["periodSampling"] = true;
        if (!lyn.contains("durationSec"))    lyn["durationSec"]    = 5.0;
        if (!lyn.contains("sampleRateMs"))   lyn["sampleRateMs"]   = 1000;
        if (!lyn.contains("startBreakpoint"))lyn["startBreakpoint"]= 0ULL;
        if (!lyn.contains("endBreakpoint"))  lyn["endBreakpoint"]  = 0ULL;
        ensure_parent_dir(lyn["outputCSV"].get<std::string>());

        if (!cfg.contains("SystemBehavior")) cfg["SystemBehavior"] = json::object();
        auto& sysb = cfg["SystemBehavior"];
        if (!sysb.contains("autoRunDurationSec")) sysb["autoRunDurationSec"] = 30;
        if (!sysb.contains("pauseDurationSec"))   sysb["pauseDurationSec"]   = 5;
        if (!sysb.contains("targetFPS"))          sysb["targetFPS"]          = 60;

        // if (!cfg.contains("Aggregator")) cfg["Aggregator"] = json::object();
        // auto& agg = cfg["Aggregator"];
        // if (!agg.contains("flush_period_ms"))        agg["flush_period_ms"]        = 1000;
        // if (!agg.contains("json_flush_threshold"))   agg["json_flush_threshold"]   = 2000;
        // if (!agg.contains("merge_wait_ms"))          agg["merge_wait_ms"]          = 33;
        // if (!agg.contains("drop_empty_compat_rows")) agg["drop_empty_compat_rows"] = true;
        // if (!agg.contains("drop_empty_flush_rows"))  agg["drop_empty_flush_rows"]  = true;
        // if (!agg.contains("expectsPower"))           agg["expectsPower"]           = true;


        // ... existing code ...
        if (!cfg.contains("Aggregator")) cfg["Aggregator"] = json::object();
        auto& agg = cfg["Aggregator"];
        if (!agg.contains("flush_period_ms") || !agg["flush_period_ms"].is_number_integer())
            agg["flush_period_ms"] = 1000;
        if (!agg.contains("json_flush_threshold") || !agg["json_flush_threshold"].is_number_integer())
            agg["json_flush_threshold"] = 2000;
        if (!agg.contains("merge_wait_ms") || !agg["merge_wait_ms"].is_number_integer())
            agg["merge_wait_ms"] = 33;
        if (!agg.contains("drop_empty_compat_rows") || !agg["drop_empty_compat_rows"].is_boolean())
            agg["drop_empty_compat_rows"] = true;
        if (!agg.contains("drop_empty_flush_rows") || !agg["drop_empty_flush_rows"].is_boolean())
            agg["drop_empty_flush_rows"] = true;
        if (!agg.contains("expectsPower") || !agg["expectsPower"].is_boolean())
            agg["expectsPower"] = true;
        if (!agg.contains("expectsCamera") || !agg["expectsCamera"].is_boolean())
            agg["expectsCamera"] = true;
        if (!agg.contains("expectsAlgorithm") || !agg["expectsAlgorithm"].is_boolean())
            agg["expectsAlgorithm"] = true;
        if (!agg.contains("expectsDisplay") || !agg["expectsDisplay"].is_boolean())
            agg["expectsDisplay"] = true;
        if (!agg.contains("expectsSoC") || !agg["expectsSoC"].is_boolean())
            agg["expectsSoC"] = true;
        // ... rest unchanged ...

        }

    void buildQueues_() {
        cam2Alg_  = std::make_shared<SharedQueue<std::shared_ptr<ZeroCopyFrameData>>>(200);
        cam2Disp_ = std::make_shared<SharedQueue<std::shared_ptr<ZeroCopyFrameData>>>(200);
        alg2Disp_ = std::make_shared<SharedQueue<std::shared_ptr<ZeroCopyFrameData>>>(300);

        spdlog::info("[ConfigManager] Queues created: cam2Alg={}, cam2Disp={}, alg2Disp={}",
                     cam2Alg_->capacity(), cam2Disp_->capacity(), alg2Disp_->capacity());
    }

private:
    Context& ctx_;
    json config_;

    std::vector<std::unique_ptr<IModule>> modules_;

    std::shared_ptr<SystemMetricsAggregatorConcreteV3_2> aggregator_;
    std::shared_ptr<SharedQueue<std::shared_ptr<ZeroCopyFrameData>>> cam2Alg_;
    std::shared_ptr<SharedQueue<std::shared_ptr<ZeroCopyFrameData>>> cam2Disp_;
    std::shared_ptr<SharedQueue<std::shared_ptr<ZeroCopyFrameData>>> alg2Disp_;
};