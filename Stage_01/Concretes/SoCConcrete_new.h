// SoCConcrete_new.h

// #pragma once

// #include "../Interfaces/ISoC.h"
// #include <memory>
// #include <thread>
// #include <atomic>
// #include <cstdio>
// #include <array>
// #include <stdexcept>
// #include <mutex>
// #include <fstream>
// #include <iomanip>
// #include <regex>
// #include <sstream>
// #include <string>
// #include <vector>
// #include <chrono>
// #include <functional>
// #include <ostream>

// #include "../SharedStructures/SoCConfig.h"
// #include "../SharedStructures/jetsonNanoInfo.h"

// /**
//  * @class SoCConcrete
//  * @brief Concrete implementation of ISoC that uses 'tegrastats' to retrieve Jetson Nano metrics.
//  */
// class SoCConcrete : public ISoC {
// public:
//     SoCConcrete();
//     ~SoCConcrete() override;

//     // ISoC Interface
//     void initializeSoC() override;
//     std::string getSpecs() override;
//     void pushSoCStats(const JetsonNanoInfo& stats) override;
//     JetsonNanoInfo getPerformance() const override;
//     void stopSoC() override;
//     bool configure(const SoCConfig& config) override;
//     void setErrorCallback(std::function<void(const std::string&)>) override;

// private:
//     void monitorPerformance();
//     std::string getTegraStatsInformation();
//     JetsonNanoInfo parseTegraStats(const std::string& output);
//     std::string formatJetsonNanoInfo(const JetsonNanoInfo& info);
//     void exportToCSV(const JetsonNanoInfo& hd, const std::string& filePath);
//     void reportError(const std::string& msg);

// private:
//     std::unique_ptr<FILE, decltype(&pclose)> pipe_{nullptr, pclose};
//     std::string command_;

//     SoCConfig config_;

//     std::thread monitoringThread_;
//     std::atomic<bool> monitoringActive_{false};

//     mutable std::mutex performanceMutex_;
//     JetsonNanoInfo lastPerformanceData_;

//     std::function<void(const std::string&)> errorCallback_;
// };

// // ---------------- Implementation ---------------- //

// inline SoCConcrete::SoCConcrete()
//     : pipe_(nullptr, pclose) 
// {
//     // If needed, set defaults for config_ here
// }

// inline SoCConcrete::~SoCConcrete() {
//     stopSoC(); // Guarantee thread stops
// }

// inline void SoCConcrete::initializeSoC() {
//     try {
//         command_ = "tegrastats"; // or make configurable
//         pipe_ = std::unique_ptr<FILE, decltype(&pclose)>(
//             popen(command_.c_str(), "r"), 
//             pclose
//         );
//         if (!pipe_) {
//             throw std::runtime_error("[SoCConcrete] Could not popen() command: " + command_);
//         }
//         monitoringActive_ = true;
//         monitoringThread_ = std::thread(&SoCConcrete::monitorPerformance, this);
//     }
//     catch (const std::exception& ex) {
//         monitoringActive_ = false;
//         reportError("[SoCConcrete] Initialization failed: " + std::string(ex.what()));
//     }
// }

// inline std::string SoCConcrete::getSpecs() {
//     // You might gather real specs (GPU/CPU type). 
//     // For now, a static string:
//     return "Jetson Nano SoC: 4x ARM Cortex-A57 @ 1.43 GHz, 128-core Maxwell GPU, 4GB LPDDR4";
// }

// inline void SoCConcrete::pushSoCStats(const JetsonNanoInfo& stats) {
//     std::lock_guard<std::mutex> lock(performanceMutex_);
//     lastPerformanceData_ = stats;
// }

// inline JetsonNanoInfo SoCConcrete::getPerformance() const {
//     std::lock_guard<std::mutex> lock(performanceMutex_);
//     return lastPerformanceData_;
// }

// inline void SoCConcrete::stopSoC() {
//     if (monitoringActive_) {
//         monitoringActive_ = false;
//         if (monitoringThread_.joinable()) {
//             monitoringThread_.join();
//         }
//     }
// }

// inline bool SoCConcrete::configure(const SoCConfig& config) {
//     config_ = config;
//     // If config has a custom command or interval, set them
//     return true;
// }

// inline void SoCConcrete::setErrorCallback(std::function<void(const std::string&)> cb) {
//     errorCallback_ = std::move(cb);
// }

// inline void SoCConcrete::monitorPerformance() {
//     while (monitoringActive_) {
//         std::string data = getTegraStatsInformation();
//         if (!data.empty()) {
//             JetsonNanoInfo info = parseTegraStats(data);

//             // Update shared state
//             {
//                 std::lock_guard<std::mutex> lock(performanceMutex_);
//                 lastPerformanceData_ = info;
//             }

//             // Log or write CSV
//             exportToCSV(info, "jetson_nano_tegrastats.csv");
//         } else {
//             reportError("[SoCConcrete] No data from tegrastats - check if it's installed or working.");
//         }
//         // Sleep
//         std::this_thread::sleep_for(std::chrono::seconds(1));
//     }
// }

// inline std::string SoCConcrete::getTegraStatsInformation() {
//     if (!pipe_) {
//         return {};
//     }
//     std::array<char, 512> buffer;
//     if (fgets(buffer.data(), buffer.size(), pipe_.get()) != nullptr) {
//         return std::string(buffer.data());
//     }
//     return {};
// }

// inline JetsonNanoInfo SoCConcrete::parseTegraStats(const std::string& output) {
//     // Example regex usage
//     std::regex ram_pattern(R"(RAM\s+(\d+)/(\d+)MB)");
//     std::regex cpu_pattern(R"(CPU\s+\[(\d+)%@(\d+),(\d+)%@(\d+),(\d+)%@(\d+),(\d+)%@(\d+)\])");
//     std::regex temp_pattern(R"(PLL@([\d.]+)C\s+CPU@([\d.]+)C\s+PMIC@([\d.]+)C\s+GPU@([\d.]+)C\s+AO@([\d.]+)C\s+thermal@([\d.]+)C)");
    
//     std::smatch matches;
//     JetsonNanoInfo info;

//     // RAM
//     if (std::regex_search(output, matches, ram_pattern) && matches.size() >= 3) {
//         info.RAM_In_Use_MB = std::stoi(matches[1]);
//         info.Total_RAM_MB  = std::stoi(matches[2]);
//     }
//     // CPU
//     if (std::regex_search(output, matches, cpu_pattern) && matches.size() >= 9) {
//         info.CPU1_Utilization_Percent = std::stoi(matches[1]);
//         info.CPU1_Frequency_MHz       = std::stoi(matches[2]);
//         info.CPU2_Utilization_Percent = std::stoi(matches[3]);
//         info.CPU2_Frequency_MHz       = std::stoi(matches[4]);
//         info.CPU3_Utilization_Percent = std::stoi(matches[5]);
//         info.CPU3_Frequency_MHz       = std::stoi(matches[6]);
//         info.CPU4_Utilization_Percent = std::stoi(matches[7]);
//         info.CPU4_Frequency_MHz       = std::stoi(matches[8]);
//     }
//     // Temps
//     if (std::regex_search(output, matches, temp_pattern) && matches.size() >= 7) {
//         info.PLL_Temperature_C     = std::stof(matches[1]);
//         info.CPU_Temperature_C     = std::stof(matches[2]);
//         info.PMIC_Temperature_C    = std::stof(matches[3]);
//         info.GPU_Temperature_C     = std::stof(matches[4]);
//         info.AO_Temperature_C      = std::stof(matches[5]);
//         info.Thermal_Temperature_C = std::stof(matches[6]);
//     }
//     info.timestamp = std::chrono::system_clock::now();
//     return info;
// }

// inline std::string SoCConcrete::formatJetsonNanoInfo(const JetsonNanoInfo& info) {
//     std::ostringstream ss;
//     ss << "RAM: " << info.RAM_In_Use_MB << "/" << info.Total_RAM_MB << " MB | "
//        << "CPU1: " << info.CPU1_Utilization_Percent << "%@" << info.CPU1_Frequency_MHz << "MHz, "
//        << "CPU2: " << info.CPU2_Utilization_Percent << "%@" << info.CPU2_Frequency_MHz << "MHz, "
//        << "Temp CPU: " << info.CPU_Temperature_C << "C, GPU: " << info.GPU_Temperature_C << "C";
//     return ss.str();
// }

// inline void SoCConcrete::exportToCSV(const JetsonNanoInfo& info, const std::string& filePath) {
//     try {
//         std::ofstream file(filePath, std::ios::app);
//         auto now_c = std::chrono::system_clock::to_time_t(info.timestamp);

//         // Write CSV header once if file is empty (optional check)
//         // if (file.tellp() == 0) {
//         //     file << "Timestamp,RAM_Used,Total_RAM,"
//         //          << "CPU1_Util,CPU1_Freq,CPU2_Util,CPU2_Freq,"
//         //          << "CPU_Temp,GPU_Temp\n";
//         // }

//         file << std::put_time(std::localtime(&now_c), "%Y-%m-%d %H:%M:%S") << ","
//              << info.RAM_In_Use_MB << "," << info.Total_RAM_MB << ","
//              << info.CPU1_Utilization_Percent << "," << info.CPU1_Frequency_MHz << ","
//              << info.CPU2_Utilization_Percent << "," << info.CPU2_Frequency_MHz << ","
//              << info.CPU_Temperature_C << "," << info.GPU_Temperature_C << "\n";
//     } catch (...) {
//         reportError("[SoCConcrete] Failed to write CSV data");
//     }
// }

// inline void SoCConcrete::reportError(const std::string& msg) {
//     if (errorCallback_) {
//         errorCallback_(msg);
//     } else {
//         // Fallback to std::cerr
//         //std::cerr << msg << std::endl;
//     }
// }


// //=============================================================
// // SoCConcrete_new.h

// #pragma once

// #include "../Interfaces/ISoC.h"
// #include "../SharedStructures/SoCConfig.h"
// //#include "../SharedStructures/jetsonNanoInfo.h"


// #include "../Interfaces/ISystemMetricsAggregator.h"
// #include "../SharedStructures/allModulesStatcs.h"               // The structure shown above

// #include <memory>
// #include <thread>
// #include <atomic>
// #include <iostream>
// #include <cstdio>
// #include <array>
// #include <stdexcept>
// #include <mutex>
// #include <fstream>
// #include <iomanip>
// #include <regex>
// #include <sstream>
// #include <string>
// #include <vector>
// #include <chrono>
// #include <functional>
// #include <spdlog/spdlog.h>

// /**
//  * @class SoCConcrete
//  * @brief Concrete implementation of ISoC that uses a custom command (by default "tegrastats")
//  *        to retrieve SoC metrics. Supports push-based stats, configurable poll intervals,
//  *        partial parsing, CSV export, and basic "pause/resume" control.
//  */
// class SoCConcrete : public ISoC {
// public:
//     SoCConcrete(std::shared_ptr<ISystemMetricsAggregator> aggregator);
//     ~SoCConcrete() override;

//     // ----------------- ISoC Interface -----------------
//     bool initializeSoC() override;
//     std::string getSpecs() override;
//     JetsonNanoInfo getPerformance() const override;
//     void stopSoC() override;
//     bool configure(const SoCConfig& config) override;
    

//       // Make sure there's a matching definition for this
//     void setErrorCallback(std::function<void(const std::string&)>) override;

//     void pushSoCStats(const JetsonNanoInfo& stats) override;

//     // ----------------- Additional Features -----------------
//     /**
//      * @brief Pause SoC monitoring (halts the reading loop without fully stopping).
//      *        You can call resumeSoC() to continue.
//      */
//     void pauseSoC();

//     /**
//      * @brief Resume SoC monitoring if it was paused.
//      */
//     void resumeSoC();

// private:
//     // Main worker thread that reads and parses SoC data
//     void monitorPerformance();

//     // Read one line from the command pipe
//     std::string getSoCInformationLine();

//     // Parse the line into a JetsonNanoInfo
//     // (Extend or modify for different boards if config_.boardType != JetsonNano)
//     JetsonNanoInfo parseTegraStats(const std::string& output);

//     // Example separate parser for TX2 or Xavier (placeholder)
//     JetsonNanoInfo parseTx2Stats(const std::string& output);
//     JetsonNanoInfo parseXavierStats(const std::string& output);
//     JetsonNanoInfo parseRpiStats(const std::string& output);

//     // Optional partial parse logging
//     void logPartialParse(const std::string& msg);

//     // CSV export if config_.exportCSV == true
//     void exportToCSV(const JetsonNanoInfo& info, const std::string& filePath);

//     // Log or callback for error
//     void reportError(const std::string& msg);

// private:
//     // Popen-based command pipe
//     std::unique_ptr<FILE, decltype(&pclose)> pipe_{nullptr, pclose};
//     std::string command_;

//     SoCConfig config_;

//     std::thread monitoringThread_;
//     std::atomic<bool> monitoringActive_{false};

//     /**
//      * @brief If true, the thread is active but we skip reading lines (a "soft" pause).
//      */
//     std::atomic<bool> monitoringPaused_{false};

//     // Last known performance data
//     mutable std::mutex performanceMutex_;
//     JetsonNanoInfo lastPerformanceData_;

//     // Optional error callback
//     std::function<void(const std::string&)> errorCallback_;

    
//     // DataConcrete to support SystemMetricsAggregatorImpl injection and push camera metrics in real-time using the aggregator
//     std::shared_ptr<ISystemMetricsAggregator> metricAggregator_;

// };

// // ---------------- Implementation ---------------- //

// // In SoCConcrete.cpp
// inline SoCConcrete::SoCConcrete(std::shared_ptr<ISystemMetricsAggregator> aggregator)
//     : pipe_(nullptr, pclose) 
//     ,metricAggregator_(std::move(aggregator)) 
//     {
//     // Constructor implementation
//     // You can set default config here if needed
//     }

// inline SoCConcrete::~SoCConcrete() {
//     stopSoC(); // Ensure we stop the thread on destruction
// }

// inline bool SoCConcrete::configure(const SoCConfig& config) {
//     config_ = config;
//     // If no custom command specified, default to "tegrastats"
//     if (config_.customCommand.empty()) {
//         config_.customCommand = "tegrastats";
//     }
//     spdlog::info("[SoCConcrete] Configured SoC with command='{}', pollIntervalMs={}, exportCSV={}, boardType={}",
//                  config_.customCommand, config_.pollIntervalMs, config_.exportCSV, 
//                  static_cast<int>(config_.boardType));
//     return true;
// }

// inline bool SoCConcrete::initializeSoC() {
//     if (monitoringActive_) {
//         spdlog::warn("[SoCConcrete] SoC is already initialized and running.");
//         return true;
//     }

//     try {
//         command_ = config_.customCommand; // e.g. "tegrastats --interval 1000"
//         pipe_ = std::unique_ptr<FILE, decltype(&pclose)>(
//             popen(command_.c_str(), "r"),
//             pclose
//         );
//         if (!pipe_) {
//             throw std::runtime_error("[SoCConcrete] Could not popen() command: " + command_);
//         }

//         monitoringActive_ = true;
//         monitoringPaused_ = false;
//         monitoringThread_ = std::thread(&SoCConcrete::monitorPerformance, this);

//         spdlog::info("[SoCConcrete] SoC monitoring started with command '{}'.", command_);
//     }
//     catch (const std::exception& ex) {
//         monitoringActive_ = false;
//         pipe_.reset(); // Ensure Close the pipe
//         reportError("[SoCConcrete] Initialization failed: " + std::string(ex.what()));
//         return false;
//     }
// }

// inline void SoCConcrete::stopSoC() {
//     if (monitoringActive_) {
//         monitoringActive_ = false;
//         if (monitoringThread_.joinable()) {
//             monitoringThread_.join();
//         }
//         spdlog::info("[SoCConcrete] SoC monitoring stopped.");
//     }
// }

// inline void SoCConcrete::pauseSoC() {
//     if (monitoringActive_) {
//         monitoringPaused_ = true;
//         spdlog::info("[SoCConcrete] SoC monitoring is now paused.");
//     }
// }

// inline void SoCConcrete::resumeSoC() {
//     if (monitoringActive_) {
//         monitoringPaused_ = false;
//         spdlog::info("[SoCConcrete] SoC monitoring is now resumed.");
//     }
// }

// inline std::string SoCConcrete::getSpecs() {
//     // This can differ per board if config_.boardType != JetsonNano
//     // For now, a static example:
//     switch (config_.boardType) {
//         case BoardType::JetsonNano:
//             return "Jetson Nano: Quad-core ARM Cortex-A57 @ 1.43 GHz, 128-core Maxwell GPU, 4GB LPDDR4";
//         case BoardType::JetsonTX2:
//             return "Jetson TX2: NVIDIA Denver2 + ARM A57 CPU, 256-core Pascal GPU, 8GB LPDDR4";
//         case BoardType::JetsonXavier:
//             return "Jetson Xavier: 8-core ARM v8.2 64-bit, 512-core Volta GPU, 16GB LPDDR4";
//         case BoardType::RaspberryPi:
//             return "Raspberry Pi: e.g., 4-core ARM Cortex-A72, VideoCore VI GPU, up to 4GB LPDDR4";
//         default:
//             return "Unknown board type: limited specs available.";
//     }
// }

// inline JetsonNanoInfo SoCConcrete::getPerformance() const {
//     std::lock_guard<std::mutex> lock(performanceMutex_);
//     return lastPerformanceData_;
// }


// inline void SoCConcrete::pushSoCStats(const JetsonNanoInfo& stats) {
//     // In a push-based design, you might dispatch these stats to a logger or aggregator
//     // e.g. PerformanceLogger::getInstance().pushSoCStats(...);
//     // For demonstration, we just store them as "lastPerformanceData_"
//     std::lock_guard<std::mutex> lock(performanceMutex_);
//     lastPerformanceData_ = stats;

//     // Push to aggregator
//     if (metricAggregator_) {
//         metricAggregator_->pushMetrics(stats.timestamp, [&](SystemMetricsSnapshot& snap) {
//             snap.socInfo = stats;
//         });
//     }
// }



// // The crucial method. Ensure it's defined:
// inline void SoCConcrete::setErrorCallback(std::function<void(const std::string&)> cb) {
//     errorCallback_ = std::move(cb);
// }

// // =============== Private methods =============== //

// inline void SoCConcrete::monitorPerformance() {
//     spdlog::debug("[SoCConcrete] monitorPerformance() thread started.");

//     while (monitoringActive_) {
//         if (monitoringPaused_) {
//             // If paused, just sleep a bit
//             std::this_thread::sleep_for(std::chrono::milliseconds(500));
//             continue;
//         }

//         // Read one line
//         std::string line = getSoCInformationLine();

//        // std::cout <<"Line  : "<<line<< std::endl; // For testing output string

//         if (!line.empty()) {
//             JetsonNanoInfo info;
//             switch (config_.boardType) {
//                 case BoardType::JetsonNano:
//                     info = parseTegraStats(line);
//                     break;
//                 case BoardType::JetsonTX2:
//                     info = parseTx2Stats(line); 
//                     break;
//                 case BoardType::JetsonXavier:
//                     info = parseXavierStats(line); 
//                     break;
//                 case BoardType::RaspberryPi:
//                     info = parseRpiStats(line); 
//                     break;
//                 default:
//                     // If unknown, attempt Jetson parse as fallback
//                     info = parseTegraStats(line); 
//                     break;
//             }

//             // *Push* the stats using pushSoCStats
//             pushSoCStats(info);

//             // CSV export if enabled
//             if (config_.exportCSV) {
//                 exportToCSV(info, config_.csvPath);
//             }
//         } else {
//             // If no line read, log or skip
//             reportError("[SoCConcrete] No data from SoC command. Possibly tegrastats ended?");
//         }

//         // Sleep the poll interval
//         std::this_thread::sleep_for(std::chrono::milliseconds(config_.pollIntervalMs));
//     }
//     spdlog::debug("[SoCConcrete] monitorPerformance() thread exiting.");
// }

// inline std::string SoCConcrete::getSoCInformationLine() {
//     if (!pipe_) {
//         return {};
//     }
//     std::array<char, 512> buffer;
//     if (fgets(buffer.data(), buffer.size(), pipe_.get()) != nullptr) {
//         return std::string(buffer.data());
//     }
//     return {};
// }

// //
// // For each parse method, we do partial parse if config_.partialParseAllowed
// // If partial parse fails for some fields, we log but keep partial data
// //

// inline JetsonNanoInfo SoCConcrete::parseTegraStats(const std::string& output) {
//     JetsonNanoInfo info;
//     info.timestamp = std::chrono::system_clock::now();

//     // Example patterns
//      // Example patterns
//     // - The class uses std::regex which can be performance-intensive. Maybe precompiling regex patterns would help.
//     // Regex Performance:
//     //  Precompile regex patterns as static const:

//     static const std::regex ram_pattern(R"(RAM\s+(\d+)/(\d+)MB)");
//     static const std::regex lfb_pattern("lfb (\\d+)x(\\d+)MB");
//     static const std::regex swap_pattern("SWAP (\\d+)/(\\d+)MB");
//     static const std::regex cached_pattern("cached (\\d+)MB");
//     static const std::regex iram_pattern("IRAM (\\d+)/([\\d]+)kB\\(lfb ([\\d]+)kB\\)");
    
//     static const std::regex cpu_pattern(
//     R"(CPU\s+\[((\d+)%@(\d+)|off),((\d+)%@(\d+)|off),((\d+)%@(\d+)|off),((\d+)%@(\d+)|off)\])");

//     //static const std::regex cpu_pattern("CPU \\[(\\d+)%@(\\d+),(\\d+)%@(\\d+),(\\d+)%@(\\d+),(\\d+)%@(\\d+)\\]");


//     static const std::regex emcgr3d_freq_pattern("EMC_FREQ (\\d+)% GR3D_FREQ (\\d+)%");
//     static const std::regex temp_pattern(R"(PLL@([\d.]+)C\s+CPU@([\d.]+)C\s+PMIC@([\d.]+)C\s+GPU@([\d.]+)C\s+AO@([\d.]+)C\s+thermal@([\d.]+)C)");

//     std::smatch matches;
//     bool foundSomething = false;

//     // 1. RAM
//     if (std::regex_search(output, matches, ram_pattern) && matches.size() >= 3) {
//         info.RAM_In_Use_MB = std::stoi(matches[1]);
//         info.Total_RAM_MB  = std::stoi(matches[2]);
//         foundSomething = true;
//     } else if (config_.partialParseAllowed) {
//         logPartialParse("[SoCConcrete] Could not parse RAM from line: " + output);
//     }

//     if (std::regex_search(output, matches, lfb_pattern) && matches.size() >= 3) {
//         info.LFB_Size_MB = std::stoi(matches[1]);
//         info.Block_Max_MB = std::stoi(matches[2]);
//     } else if (config_.partialParseAllowed) {
//         logPartialParse("[SoCConcrete] Could not parse LFB from line: " + output);
//     }

//     if (std::regex_search(output, matches, swap_pattern) && matches.size() >= 3) {
//         info.SWAP_In_Use_MB = std::stoi(matches[1]);
//         info.Total_SWAP_MB = std::stoi(matches[2]);
//     } else if (config_.partialParseAllowed) {
//         logPartialParse("[SoCConcrete] Could not parse SWAP from line: " + output);
//     }

//     if (std::regex_search(output, matches, cached_pattern) && matches.size() >= 2) {
//         info.Cached_MB = std::stoi(matches[1]);
//     } else if (config_.partialParseAllowed) {
//         logPartialParse("[SoCConcrete] Could not parse CACHE from line: " + output);
//     }

//     if (std::regex_search(output, matches, iram_pattern) && matches.size() == 4) {
//         info.used_IRAM_kB = std::stoi(matches[1].str());
//         info.total_IRAM_kB = std::stoi(matches[2].str());
//         info.lfb_kB = std::stoi(matches[3].str());
//     } else if (config_.partialParseAllowed) {
//         logPartialParse("[SoCConcrete] Could not parse IRAM from line: " + output);
//     }
    

//     //2. CPU Patterns
//     if (std::regex_search(output, matches, cpu_pattern) && matches.size() == 13) {
//         auto parseCpu = [](const std::string& utilStr, const std::string& freqStr, double& util, double& freq) {
//             if (utilStr == "off" || freqStr == "off") {
//                 util = 0;
//                 freq = 0;
//             } else {
//                 util = std::stoi(utilStr);
//                 freq = std::stoi(freqStr);
//             }
//         };

//         parseCpu(matches[2], matches[3], info.CPU1_Utilization_Percent, info.CPU1_Frequency_MHz);
//         parseCpu(matches[5], matches[6], info.CPU2_Utilization_Percent, info.CPU2_Frequency_MHz);
//         parseCpu(matches[8], matches[9], info.CPU3_Utilization_Percent, info.CPU3_Frequency_MHz);
//         parseCpu(matches[11], matches[12], info.CPU4_Utilization_Percent, info.CPU4_Frequency_MHz);
//         foundSomething = true;
//     } else if (config_.partialParseAllowed) {
//         logPartialParse("[SoCConcrete] Could not parse CPU usage from line: " + output);
//     }


//     if (std::regex_search(output, matches, emcgr3d_freq_pattern) && matches.size() >= 3) {
//         info.EMC_Frequency_Percent = std::stoi(matches[1]);
//         info.GR3D_Frequency_Percent = std::stoi(matches[2]);
//     } else if (config_.partialParseAllowed) {
//         logPartialParse("[SoCConcrete] Could not parse CPU usage from line: " + output);
//     }

//     // 3. Temps
//     if (std::regex_search(output, matches, temp_pattern) && matches.size() >= 7) {
//         info.PLL_Temperature_C     = std::stof(matches[1]);
//         info.CPU_Temperature_C     = std::stof(matches[2]);
//         info.PMIC_Temperature_C    = std::stof(matches[3]);
//         info.GPU_Temperature_C     = std::stof(matches[4]);
//         info.AO_Temperature_C      = std::stof(matches[5]);
//         info.Thermal_Temperature_C = std::stof(matches[6]);
//         foundSomething = true;
//     } else if (config_.partialParseAllowed) {
//         logPartialParse("[SoCConcrete] Could not parse temperatures from line: " + output);
//     }

//     if (!foundSomething && !config_.partialParseAllowed) {
//         reportError("[SoCConcrete] parseTegraStats: no fields parsed, check tegrastats format or partialParseAllowed");
//     }

//     return info;
// }

// inline JetsonNanoInfo SoCConcrete::parseTx2Stats(const std::string& output) {
//     // Placeholder parse logic for Jetson TX2
//     // You might have different regex or format
//     if (config_.partialParseAllowed) {
//         logPartialParse("[SoCConcrete] parseTx2Stats not implemented, returning partial info");
//     }
//     // Return minimal or parse similarly
//     return parseTegraStats(output);
// }

// inline JetsonNanoInfo SoCConcrete::parseXavierStats(const std::string& output) {
//     // Placeholder parse logic for Jetson Xavier
//     if (config_.partialParseAllowed) {
//         logPartialParse("[SoCConcrete] parseXavierStats not implemented, returning partial info");
//     }
//     return parseTegraStats(output);
// }

// inline JetsonNanoInfo SoCConcrete::parseRpiStats(const std::string& output) {
//     // Placeholder parse logic for Raspberry Pi
//     if (config_.partialParseAllowed) {
//         logPartialParse("[SoCConcrete] parseRpiStats not implemented, returning partial info");
//     }
//     return parseTegraStats(output);
// }

// inline void SoCConcrete::logPartialParse(const std::string& msg) {
//     spdlog::debug(msg);
// }

// // inline void SoCConcrete::exportToCSV(const JetsonNanoInfo& info, const std::string& filePath) {
// //     try {
// //         std::ofstream file(filePath, std::ios::app);
// //         auto now_c = std::chrono::system_clock::to_time_t(info.timestamp);

// //         if (file.tellp() == 0) {
// //             file << "Timestamp,RAM_Used_MB,Total_RAM_MB,"
// //                 << "LFB_Size,Block_Max_MB,"
// //                 << "SWAP_In_Use_MB,Total_SWAP_MB,Cached_MB,"
// //                 << "used_IRAM_kB,total_IRAM_kB, lfb_kB,"
// //                  << "CPU1_Util,CPU1_Freq_MHZ,CPU2_Util,CPU2_Freq_MHZ,"
// //                  << "CPU3_Util,CPU3_Freq_MHZ,CPU4_Util,CPU4_Freq_MHZ,"
// //                  << "EMC_Frequency_Percent,GR3D_Frequency_Percent,"
// //                  << "PLL_Temperature_C,CPU_Temperature_C,PMIC_Temperature_C,GPU_Temperature_C,AO_Temperature_C,Thermal_Temperature_C\n";
// //         }

// //         // Write data with timestamp
// //         file << std::put_time(std::localtime(&now_c), "%Y-%m-%d %H:%M:%S") << ","
// //              << info.RAM_In_Use_MB << "," << info.Total_RAM_MB << ","
// //             << info.LFB_Size << "," << info.Block_Max_MB << ","
// //             << info.SWAP_In_Use_MB << "," << info.Total_SWAP_MB << "," << info.Cached_MB << ","
// //             << info.used_IRAM_kB << "," << info.total_IRAM_kB << "," << info.lfb_kB << ","
// //             << info.CPU1_Utilization_Percent << "," << info.CPU1_Frequency_MHz << ","
// //             << info.CPU2_Utilization_Percent << "," << info.CPU2_Frequency_MHz << ","
// //             << info.CPU3_Utilization_Percent << "," << info.CPU3_Frequency_MHz << ","
// //             << info.CPU4_Utilization_Percent << "," << info.CPU4_Frequency_MHz << ","
// //             << info.EMC_Frequency_Percent << "," << info.GR3D_Frequency_Percent << ","
// //             << info.PLL_Temperature_C << "," << info.CPU_Temperature_C << ","
// //             << info.PMIC_Temperature_C << "," << info.GPU_Temperature_C << ","
// //             << info.AO_Temperature_C << "," << info.Thermal_Temperature_C << "\n";

// //     } catch (...) {
// //         reportError("[SoCConcrete] Failed to write CSV data");
// //     }
// // }

// inline void SoCConcrete::exportToCSV(const JetsonNanoInfo& info, const std::string& filePath) {
//     try {
//         std::ofstream file(filePath, std::ios::app);
//         auto now_c = std::chrono::system_clock::to_time_t(info.timestamp);

//         if (file.tellp() == 0) {
//             file << "Timestamp,RAM_Used_MB,Total_RAM_MB,"
//                  << "LFB_Size,Block_Max_MB,"
//                  << "SWAP_In_Use_MB,Total_SWAP_MB,Cached_MB,"
//                  << "used_IRAM_kB,total_IRAM_kB,lfb_kB,"
//                  << "CPU1_Util,CPU1_Freq_MHZ,CPU2_Util,CPU2_Freq_MHZ,"
//                  << "CPU3_Util,CPU3_Freq_MHZ,CPU4_Util,CPU4_Freq_MHZ,"
//                  << "EMC_Frequency_Percent,GR3D_Frequency_Percent,"
//                  << "PLL_Temperature_C,CPU_Temperature_C,PMIC_Temperature_C,"
//                  << "GPU_Temperature_C,AO_Temperature_C,Thermal_Temperature_C\n";
//         }

//         // Helper to format CPU values or "off"
//         auto formatCPU = [](int util, int freq) -> std::string {
//             return (util == 0 && freq == 0) ? "off,off" : std::to_string(util) + "," + std::to_string(freq);
//         };

//         file << std::put_time(std::localtime(&now_c), "%Y-%m-%d %H:%M:%S") << ","
//              << info.RAM_In_Use_MB << "," << info.Total_RAM_MB << ","
//              << info.LFB_Size_MB << "," << info.Block_Max_MB << ","
//              << info.SWAP_In_Use_MB << "," << info.Total_SWAP_MB << "," << info.Cached_MB << ","
//              << info.used_IRAM_kB << "," << info.total_IRAM_kB << "," << info.lfb_kB << ","
//              << formatCPU(info.CPU1_Utilization_Percent, info.CPU1_Frequency_MHz) << ","
//              << formatCPU(info.CPU2_Utilization_Percent, info.CPU2_Frequency_MHz) << ","
//              << formatCPU(info.CPU3_Utilization_Percent, info.CPU3_Frequency_MHz) << ","
//              << formatCPU(info.CPU4_Utilization_Percent, info.CPU4_Frequency_MHz) << ","
//              << info.EMC_Frequency_Percent << "," << info.GR3D_Frequency_Percent << ","
//              << info.PLL_Temperature_C << "," << info.CPU_Temperature_C << ","
//              << info.PMIC_Temperature_C << "," << info.GPU_Temperature_C << ","
//              << info.AO_Temperature_C << "," << info.Thermal_Temperature_C << "\n";
//     } catch (...) {
//         reportError("[SoCConcrete] Failed to write CSV data");
//     }
// }


// inline void SoCConcrete::reportError(const std::string& msg) {
//     if (errorCallback_) {
//         errorCallback_(msg);
//     } else {
//         spdlog::error("[SoCConcrete] {}", msg);
//     }
// }

// SoCConcrete_new.h
/// ==================================25-05-2025=====================================

#pragma once

#include "../Interfaces/ISoC.h"
#include "../SharedStructures/SoCConfig.h"
#include "../Interfaces/ISystemMetricsAggregator.h"
#include "../SharedStructures/allModulesStatcs.h"

#include <memory>
#include <thread>
#include <atomic>
#include <mutex>
#include <string>
#include <vector>
#include <chrono>
#include <functional>
#include <spdlog/spdlog.h>
#include <cstdio>
#include <regex>
#include <fstream>
#include <iomanip>


// Add this header for the simulation logic
#include <random> 

class SoCConcrete : public ISoC {
public:
    SoCConcrete(std::shared_ptr<ISystemMetricsAggregator> aggregator);
    ~SoCConcrete() override;

    bool initializeSoC() override;
    std::string getSpecs() override;
    JetsonNanoInfo getPerformance() const override;
    void stopSoC() override;

    
    bool configure(const SoCConfig& config) override;
    void setErrorCallback(std::function<void(const std::string&)> cb) override;
    void pushSoCStats(const JetsonNanoInfo& stats) override;
    void pauseSoC();
    void resumeSoC();
    bool isMonitoringActive() const; // New method to check if monitoring is active

private:
    void monitorPerformance();
    std::string getSoCInformationLine();
    JetsonNanoInfo parseTegraStats(const std::string& output);
    JetsonNanoInfo parseTx2Stats(const std::string& output);
    JetsonNanoInfo parseXavierStats(const std::string& output);
    JetsonNanoInfo parseRpiStats(const std::string& output);
    void logPartialParse(const std::string& msg);
    void exportToCSV(const JetsonNanoInfo& info, const std::string& filePath);
    void reportError(const std::string& msg);
    bool verifyTegraStatsOutput(); // New method to verify tegrastats

    std::unique_ptr<FILE, decltype(&pclose)> pipe_{nullptr, pclose};
    std::string command_;
    SoCConfig config_;
    std::thread monitoringThread_;
    std::atomic<bool> monitoringActive_{false};
    std::atomic<bool> monitoringPaused_{false};
    mutable std::mutex performanceMutex_;
    JetsonNanoInfo lastPerformanceData_;
    std::function<void(const std::string&)> errorCallback_;
    std::shared_ptr<ISystemMetricsAggregator> metricAggregator_;



};

inline SoCConcrete::SoCConcrete(std::shared_ptr<ISystemMetricsAggregator> aggregator)
    : pipe_(nullptr, pclose), metricAggregator_(std::move(aggregator)) {}

inline SoCConcrete::~SoCConcrete() {
    stopSoC();
}

inline bool SoCConcrete::configure(const SoCConfig& config) {
    config_ = config;
    if (config_.customCommand.empty()) {
        //config_.customCommand = "tegrastats --interval 330";
        // Change from 500 to 100 (or similar)
        config_.customCommand = "tegrastats --interval 100";
    }
    spdlog::info("[SoCConcrete] Configured SoC with command='{}', pollIntervalMs={}, exportCSV={}, boardType={}",
                 config_.customCommand, config_.pollIntervalMs, config_.exportCSV,
                 static_cast<int>(config_.boardType));
    return true;
}

inline bool SoCConcrete::initializeSoC() {
    if (monitoringActive_) {
        spdlog::warn("[SoCConcrete] SoC is already initialized and running.");
        return true;
    }

    try {
        command_ = config_.customCommand;
        pipe_ = std::unique_ptr<FILE, decltype(&pclose)>(
            popen(command_.c_str(), "r"),
            pclose
        );
        if (!pipe_) {
            throw std::runtime_error("Could not popen() command: " + command_);
        }

        // Verify tegrastats produces valid output
        if (!verifyTegraStatsOutput()) {
            pipe_.reset();
            throw std::runtime_error("tegrastats command failed to produce valid output");
        }

        monitoringActive_ = true;
        monitoringPaused_ = false;
        monitoringThread_ = std::thread(&SoCConcrete::monitorPerformance, this);

        // Wait briefly to ensure thread starts and reads data
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        if (!monitoringActive_) {
            pipe_.reset();
            monitoringThread_.join();
            throw std::runtime_error("Monitoring thread failed to start");
        }

        spdlog::info("[SoCConcrete] SoC monitoring started with command '{}'.", command_);
        return true;
    }
    catch (const std::exception& ex) {
        monitoringActive_ = false;
        pipe_.reset();
        if (monitoringThread_.joinable()) {
            monitoringThread_.join();
        }
        reportError("[SoCConcrete] Initialization failed: " + std::string(ex.what()));
        return false;
    }
}

inline bool SoCConcrete::verifyTegraStatsOutput() {
    // Try reading one line from the pipe
    std::string line = getSoCInformationLine();
    if (line.empty()) {
        reportError("[SoCConcrete] No data from tegrastats command");
        return false;
    }

    // Basic validation: check if line contains expected tegrastats patterns
    static const std::regex ram_pattern(R"(RAM\s+\d+/\d+MB)");
    std::smatch matches;
    if (!std::regex_search(line, matches, ram_pattern)) {
        reportError("[SoCConcrete] tegrastats output invalid: missing RAM pattern");
        return false;
    }

    return true;
}

inline void SoCConcrete::stopSoC() {
    if (monitoringActive_) {
        monitoringActive_ = false;
        monitoringPaused_ = false;
        if (monitoringThread_.joinable()) {
            monitoringThread_.join();
        }
        pipe_.reset();
        spdlog::info("[SoCConcrete] SoC monitoring stopped.");
    }
}

inline void SoCConcrete::pauseSoC() {
    if (monitoringActive_) {
        monitoringPaused_ = true;
        spdlog::info("[SoCConcrete] SoC monitoring is now paused.");
    }
}

inline void SoCConcrete::resumeSoC() {
    if (monitoringActive_) {
        monitoringPaused_ = false;
        spdlog::info("[SoCConcrete] SoC monitoring is now resumed.");
    }
}

inline std::string SoCConcrete::getSpecs() {
    switch (config_.boardType) {
        case BoardType::JetsonNano:
            return "Jetson Nano: Quad-core ARM Cortex-A57 @ 1.43 GHz, 128-core Maxwell GPU, 4GB LPDDR4";
        case BoardType::JetsonTX2:
            return "Jetson TX2: NVIDIA Denver2 + ARM A57 CPU, 256-core Pascal GPU, 8GB LPDDR4";
        case BoardType::JetsonXavier:
            return "Jetson Xavier: 8-core ARM v8.2 64-bit, 512-core Volta GPU, 16GB LPDDR4";
        case BoardType::RaspberryPi:
            return "Raspberry Pi: e.g., 4-core ARM Cortex-A72, VideoCore VI GPU, up to 4GB LPDDR4";
        default:
            return "Unknown board type: limited specs available.";
    }
}

inline JetsonNanoInfo SoCConcrete::getPerformance() const {
    std::lock_guard<std::mutex> lock(performanceMutex_);
    return lastPerformanceData_;
}

// inline void SoCConcrete::pushSoCStats(const JetsonNanoInfo& stats) {
//     std::lock_guard<std::mutex> lock(performanceMutex_);
//     lastPerformanceData_ = stats;
//     if (metricAggregator_) {
//         metricAggregator_->pushMetrics(stats.timestamp, [&](SystemMetricsSnapshot& snap) {
//             snap.socInfo = stats;
//         });
//     }
// }


 inline void SoCConcrete::pushSoCStats(const JetsonNanoInfo& stats) {
    // In a push-based design, you might dispatch these stats to a logger or aggregator
    // e.g. PerformanceLogger::getInstance().pushSoCStats(...);
    // For demonstration, we just store them as "lastPerformanceData_"
    std::lock_guard<std::mutex> lock(performanceMutex_);
    lastPerformanceData_ = stats;

    // Push to aggregator
    if (metricAggregator_) {
        metricAggregator_->pushMetrics(stats.timestamp, [&](SystemMetricsSnapshot& snap) {
            snap.socInfo = stats;
        });
    }
}

inline void SoCConcrete::setErrorCallback(std::function<void(const std::string&)> cb) {
    errorCallback_ = std::move(cb);
}


inline bool SoCConcrete::isMonitoringActive() const {
    return monitoringActive_.load();
}

inline void SoCConcrete::monitorPerformance() {
    spdlog::debug("[SoCConcrete] monitorPerformance() thread started.");

    while (monitoringActive_) {
        if (monitoringPaused_) {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            continue;
        }

        std::string line = getSoCInformationLine();
        if (!line.empty()) {
            JetsonNanoInfo info;
            switch (config_.boardType) {
                case BoardType::JetsonNano:
                    info = parseTegraStats(line);
                    break;
                case BoardType::JetsonTX2:
                    info = parseTx2Stats(line);
                    break;
                case BoardType::JetsonXavier:
                    info = parseXavierStats(line);
                    break;
                case BoardType::RaspberryPi:
                    info = parseRpiStats(line);
                    break;
                default:
                    info = parseTegraStats(line);
                    break;
            }
            pushSoCStats(info);
            //metricAggregator_->pushSocStats(info); // Push to aggregator
            if (config_.exportCSV) {
                exportToCSV(info, config_.csvPath);
            }
        } else {
            reportError("[SoCConcrete] No data from SoC command. Retrying...");
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            continue; // Retry instead of exiting
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(config_.pollIntervalMs));
    }
    pipe_.reset(); // Ensure pipe is closed
    spdlog::debug("[SoCConcrete] monitorPerformance() thread exiting.");
}

inline std::string SoCConcrete::getSoCInformationLine() {
    if (!pipe_) {
        return {};
    }
    std::array<char, 512> buffer;
    if (fgets(buffer.data(), buffer.size(), pipe_.get()) != nullptr) {
        return std::string(buffer.data());
    }
    return {};
}

inline JetsonNanoInfo SoCConcrete::parseTegraStats(const std::string& output) {
    JetsonNanoInfo info;
    info.timestamp = std::chrono::system_clock::now();

    static const std::regex ram_pattern(R"(RAM\s+(\d+)/(\d+)MB)");
    static const std::regex lfb_pattern(R"(lfb\s+(\d+)x(\d+)MB)");
    static const std::regex swap_pattern(R"(SWAP\s+(\d+)/(\d+)MB)");
    static const std::regex cached_pattern(R"(cached\s+(\d+)MB)");
    static const std::regex iram_pattern(R"(IRAM\s+(\d+)/(\d+)kB\s*\(lfb\s+(\d+)kB\))");
    static const std::regex cpu_pattern(
        R"(CPU\s+\[(?:(\d+)%@(\d+)|off),(?:(\d+)%@(\d+)|off),(?:(\d+)%@(\d+)|off),(?:(\d+)%@(\d+)|off)\])");
   // static const std::regex emcgr3d_freq_pattern(R"(EMC_FREQ\s+(\d+)%\s+GR3D_FREQ\s+(\d+)%)");
    //static const std::regex emcgr3d_freq_pattern(R"(EMC_FREQ\s+(\d+)%\s*GR3D_FREQ\s+(\d+)%)");
    static const std::regex emcgr3d_freq_pattern(R"(EMC_FREQ\s+(\d+)%@[\d]+\s*GR3D_FREQ\s+(\d+)%@[\d]+)");
    // try {
    // static const std::regex emcgr3d_freq_pattern(R"(EMC_FREQ\s+(\d+)%@[\d]+\s*GR3D_FREQ\s+(\d+)%@[\d]+)");
    // } catch (const std::regex_error& e) {
    //     spdlog::error("[SoCConcrete] Regex error: {}", e.what());
    // }

    static const std::regex temp_pattern(R"(PLL@([\d.]+)C\s+CPU@([\d.]+)C\s+PMIC@([\d.]+)C\s+GPU@([\d.]+)C\s+AO@([\d.]+)C\s+thermal@([\d.]+)C)");

    std::smatch matches;
    bool foundSomething = false;

    if (std::regex_search(output, matches, ram_pattern) && matches.size() >= 3) {
        info.RAM_In_Use_MB = std::stoi(matches[1]);
        info.Total_RAM_MB = std::stoi(matches[2]);
        foundSomething = true;
    } else if (config_.partialParseAllowed) {
        logPartialParse("[SoCConcrete] Could not parse RAM from line: " + output);
    }

    if (std::regex_search(output, matches, lfb_pattern) && matches.size() >= 3) {
        info.LFB_Size_MB = std::stoi(matches[1]);
        info.Block_Max_MB = std::stoi(matches[2]);
    } else if (config_.partialParseAllowed) {
        logPartialParse("[SoCConcrete] Could not parse LFB from line: " + output);
    }

    if (std::regex_search(output, matches, swap_pattern) && matches.size() >= 3) {
        info.SWAP_In_Use_MB = std::stoi(matches[1]);
        info.Total_SWAP_MB = std::stoi(matches[2]);
    } else if (config_.partialParseAllowed) {
        logPartialParse("[SoCConcrete] Could not parse SWAP from line: " + output);
    }

    if (std::regex_search(output, matches, cached_pattern) && matches.size() >= 2) {
        info.Cached_MB = std::stoi(matches[1]);
    } else if (config_.partialParseAllowed) {
        logPartialParse("[SoCConcrete] Could not parse CACHE from line: " + output);
    }

    if (std::regex_search(output, matches, iram_pattern) && matches.size() >= 4) {
        info.used_IRAM_kB = std::stoi(matches[1]);
        info.total_IRAM_kB = std::stoi(matches[2]);
        info.lfb_kB = std::stoi(matches[3]);
    } else if (config_.partialParseAllowed) {
        logPartialParse("[SoCConcrete] Could not parse IRAM from line: " + output);
    }

    if (std::regex_search(output, matches, cpu_pattern) && matches.size() >= 9) {
        auto parseCpu = [](const std::string& utilStr, const std::string& freqStr, double& util, double& freq) {
            if (utilStr.empty() || freqStr.empty()) {
                util = 0;
                freq = 0;
            } else {
                util = std::stoi(utilStr);
                freq = std::stoi(freqStr);
            }
        };
        parseCpu(matches[1], matches[2], info.CPU1_Utilization_Percent, info.CPU1_Frequency_MHz);
        parseCpu(matches[3], matches[4], info.CPU2_Utilization_Percent, info.CPU2_Frequency_MHz);
        parseCpu(matches[5], matches[6], info.CPU3_Utilization_Percent, info.CPU3_Frequency_MHz);
        parseCpu(matches[7], matches[8], info.CPU4_Utilization_Percent, info.CPU4_Frequency_MHz);
        foundSomething = true;
    } else if (config_.partialParseAllowed) {
        logPartialParse("[SoCConcrete] Could not parse CPU usage from line: " + output);
    }

    if (std::regex_search(output, matches, emcgr3d_freq_pattern) && matches.size() >= 3) {
        info.EMC_Frequency_Percent = std::stoi(matches[1]);
        info.GR3D_Frequency_Percent = std::stoi(matches[2]);
    } else if (config_.partialParseAllowed) {
        logPartialParse("[SoCConcrete] Could not parse EMC/GR3D from line: " + output);
    }
    
    // if (std::regex_search(output, matches, emcgr3d_freq_pattern) && matches.size() > 2) {
    // info.EMC_Frequency_Percent = std::stoi(matches[1].str());
    // info.GR3D_Frequency_Percent = std::stoi(matches[2].str());
    // spdlog::debug("[SoCConcrete] Parsed EMC: {}, GR3D: {}", matches[1], matches[2]);
    // } else if (config_.partialParseAllowed) {
    //     logPartialParse("[SoCConcrete] Could not parse EMC/GR3D from line: " + output);
    // }

    if (std::regex_search(output, matches, temp_pattern) && matches.size() >= 7) {
        info.PLL_Temperature_C = std::stof(matches[1]);
        info.CPU_Temperature_C = std::stof(matches[2]);
        info.PMIC_Temperature_C = std::stof(matches[3]);
        info.GPU_Temperature_C = std::stof(matches[4]);
        info.AO_Temperature_C = std::stof(matches[5]);
        info.Thermal_Temperature_C = std::stof(matches[6]);
        foundSomething = true;
    } else if (config_.partialParseAllowed) {
        logPartialParse("[SoCConcrete] Could not parse temperatures from line: " + output);
    }

    if (!foundSomething && !config_.partialParseAllowed) {
        reportError("[SoCConcrete] parseTegraStats: no fields parsed, check tegrastats format or partialParseAllowed");
    }

    return info;
}

inline JetsonNanoInfo SoCConcrete::parseTx2Stats(const std::string& output) {
    if (config_.partialParseAllowed) {
        logPartialParse("[SoCConcrete] parseTx2Stats not implemented, returning partial info");
    }
    return parseTegraStats(output);
}

inline JetsonNanoInfo SoCConcrete::parseXavierStats(const std::string& output) {
    if (config_.partialParseAllowed) {
        logPartialParse("[SoCConcrete] parseXavierStats not implemented, returning partial info");
    }
    return parseTegraStats(output);
}

inline JetsonNanoInfo SoCConcrete::parseRpiStats(const std::string& output) {
    if (config_.partialParseAllowed) {
        logPartialParse("[SoCConcrete] parseRpiStats not implemented, returning partial info");
    }
    return parseTegraStats(output);
}

inline void SoCConcrete::logPartialParse(const std::string& msg) {
    spdlog::debug(msg);
}

inline void SoCConcrete::exportToCSV(const JetsonNanoInfo& info, const std::string& filePath) {
    try {
        std::ofstream file(filePath, std::ios::app);
        auto now_c = std::chrono::system_clock::to_time_t(info.timestamp);

        if (file.tellp() == 0) {
            file << "Timestamp,RAM_Used_MB,Total_RAM_MB,"
                 << "LFB_Size_MB,Block_Max_MB,"
                 << "SWAP_In_Use_MB,Total_SWAP_MB,Cached_MB,"
                 << "used_IRAM_kB,total_IRAM_kB,lfb_kB,"
                 << "CPU1_Util,CPU1_Freq_MHZ,CPU2_Util,CPU2_Freq_MHZ,"
                 << "CPU3_Util,CPU3_Freq_MHZ,CPU4_Util,CPU4_Freq_MHZ,"
                 << "EMC_Frequency_Percent,GR3D_Frequency_Percent,"
                 << "PLL_Temperature_C,CPU_Temperature_C,PMIC_Temperature_C,"
                 << "GPU_Temperature_C,AO_Temperature_C,Thermal_Temperature_C\n";
        }

        auto formatCPU = [](int util, int freq) -> std::string {
            return (util == 0 && freq == 0) ? "off,off" : std::to_string(util) + "," + std::to_string(freq);
        };

        file << std::put_time(std::localtime(&now_c), "%Y-%m-%d %H:%M:%S") << ","
             << info.RAM_In_Use_MB << "," << info.Total_RAM_MB << ","
             << info.LFB_Size_MB << "," << info.Block_Max_MB << ","
             << info.SWAP_In_Use_MB << "," << info.Total_SWAP_MB << "," << info.Cached_MB << ","
             << info.used_IRAM_kB << "," << info.total_IRAM_kB << "," << info.lfb_kB << ","
             << formatCPU(info.CPU1_Utilization_Percent, info.CPU1_Frequency_MHz) << ","
             << formatCPU(info.CPU2_Utilization_Percent, info.CPU2_Frequency_MHz) << ","
             << formatCPU(info.CPU3_Utilization_Percent, info.CPU3_Frequency_MHz) << ","
             << formatCPU(info.CPU4_Utilization_Percent, info.CPU4_Frequency_MHz) << ","
             << info.EMC_Frequency_Percent << "," << info.GR3D_Frequency_Percent << ","
             << info.PLL_Temperature_C << "," << info.CPU_Temperature_C << ","
             << info.PMIC_Temperature_C << "," << info.GPU_Temperature_C << ","
             << info.AO_Temperature_C << "," << info.Thermal_Temperature_C << "\n";
    } catch (...) {
        reportError("[SoCConcrete] Failed to write CSV data");
    }
}

inline void SoCConcrete::reportError(const std::string& msg) {
    if (errorCallback_) {
        errorCallback_(msg);
    } else {
        spdlog::error("[SoCConcrete] {}", msg);
    }
}