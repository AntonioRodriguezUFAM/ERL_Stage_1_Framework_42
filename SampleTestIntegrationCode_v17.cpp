//========================================================================================
// SampleTestIntegrationCode_v17.cpp
// - Main executable entry point
// - Includes ConfigManager.hpp for pipeline logic
// - Handles signal handling and CLI parsing
// - Optional CLI: MySystem [runSeconds] [configPath]  (first arg treated as config if not numeric)
//========================================================================================

#define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_INFO

#include <spdlog/spdlog.h>
#include <spdlog/fmt/ostr.h>

#include <atomic>
#include <chrono>
#include <csignal>
#include <string>
#include <thread>
#include <memory> // For std::make_shared

// Include the ConfigManager and all its dependencies (Modules, Queues, etc.)
#include "Module/ConfigManager.hpp"

// Forward declaration for Context, which is defined in Modules.hpp (included by ConfigManager.hpp)
struct Context;

extern "C" {
    #include "../../build/lynsyn-host-software/liblynsyn/lynsyn.h"
}

//========================================================================================
// Signal handling + main
//========================================================================================
static std::atomic<bool> g_shutdown_flag{false};
static void signal_handler(int sig) {
    g_shutdown_flag.store(true, std::memory_order_relaxed);
    spdlog::warn("Shutdown signal received: {}", sig);
}

int main(int argc, char* argv[]) {
    // Register signals
    std::signal(SIGINT,  signal_handler);
    std::signal(SIGTERM, signal_handler);

    // Logging
    spdlog::set_level(spdlog::level::info);
    spdlog::set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%^%l%$] %v");
    spdlog::info("Starting Refactored Pipeline...");

    // Context
    Context ctx;
    ctx.tm = std::make_shared<ThreadManager>();

    // CLI: [runSeconds] [configPath]  (first arg treated as config if not numeric)
    double runSeconds = 0.0;
    std::string configPath = "config.json";
    if (argc > 1) {
        try {
            runSeconds = std::stod(argv[1]);
            if (runSeconds <= 0.0) {
                spdlog::warn("Non-positive runtime provided ({}). Falling back to config/defaults.", argv[1]);
                runSeconds = 0.0;
            }
        } catch (...) {
            configPath = argv[1];
            spdlog::info("Interpreting first arg as config path: {}", configPath);
        }
    }
    if (argc > 2) {
        configPath = argv[2];
    }

    try {
        ConfigManager manager(configPath, ctx);
        spdlog::info("Loaded configuration from {}", configPath);
        manager.buildPipeline();

        // If no CLI runSeconds, use config (null safe)
        // jvalue and jobject_or_empty are available via ConfigManager.hpp
        const json jSys = jobject_or_empty(manager.config(), "SystemBehavior");
        if (runSeconds <= 0.0) {
            runSeconds = jvalue<double>(jSys, "autoRunDurationSec", 30.0);
        }
        spdlog::info("Run duration set to {} seconds", runSeconds);

        // Validate
        if (!manager.validateAll()) {
            spdlog::critical("Validation failed.");
            return EXIT_FAILURE;
        }

        // Start
        manager.startAll();

        // Bridge signal flag into context so modules can stop early
        ctx.shutdown_flag.store(false, std::memory_order_relaxed);
        std::thread signalBridge([&](){
            while (!ctx.shutdown_flag.load(std::memory_order_relaxed)) {
                if (g_shutdown_flag.load(std::memory_order_relaxed)) {
                    spdlog::warn("Shutdown signal received  requesting graceful stop.");
                    ctx.shutdown_flag.store(true, std::memory_order_relaxed);
                    break;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            }
        });

        // Main loop (time-bounded unless signal arrives)
        manager.runLoop(runSeconds);

        // Teardown
        ctx.shutdown_flag.store(true, std::memory_order_relaxed);
        if (signalBridge.joinable()) signalBridge.join();

        manager.stopAll();
        manager.flushMetrics();

    } catch (const std::exception& e) {
        spdlog::error("Fatal error: {}", e.what());
        return EXIT_FAILURE;
    }

    spdlog::info("Pipeline finished successfully.");
    return EXIT_SUCCESS;
}