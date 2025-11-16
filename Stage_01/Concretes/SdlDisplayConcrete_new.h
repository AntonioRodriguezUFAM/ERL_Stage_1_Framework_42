

// ///=====================TESTING  code Final Version ==============================

// // ====================================WORKING ========================================


// [MOD 09-22-2025] This header provides a clean interface, with implementation moved to the .cpp file.
// [MOD 09-22-2025] Removed unused members and corrected function signatures.
// [MOD 09-22-2025] Added missing <stdexcept> for std::runtime_error.
#ifndef SDL_DISPLAY_CONCRETE_NEW_H
#define SDL_DISPLAY_CONCRETE_NEW_H

#define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_INFO

#include "../Interfaces/IDisplay.h"
#include "../SharedStructures/DisplayConfig.h"
#include "../SharedStructures/ZeroCopyFrameData.h"
#include "../SharedStructures/SharedQueue.h"
#include "../Interfaces/ISystemMetricsAggregator.h"
#include "../SharedStructures/allModulesStatcs.h"

#include <SDL2/SDL.h>
#include <atomic>
#include <mutex>
#include <memory>
#include <chrono>
#include <functional>
#include <vector>
#include <spdlog/spdlog.h>
#include <stdexcept>



#include "../Others/utils.h"


// =======================================================================

class SdlDisplayConcrete : public IDisplay {
public:
    /**
     * @brief Constructor for the SDL Display module.
     * @param origQueue A shared queue for receiving original, unprocessed frames.
     * @param procQueue A shared queue for receiving algorithm-processed frames.
     * @param aggregator The system-wide metrics aggregator.
     */
    SdlDisplayConcrete(
        std::shared_ptr<SharedQueue<std::shared_ptr<ZeroCopyFrameData>>> origQueue,
        std::shared_ptr<SharedQueue<std::shared_ptr<ZeroCopyFrameData>>> procQueue,
        std::shared_ptr<ISystemMetricsAggregator> aggregator
    );

    /**
     * @brief Destructor that ensures all SDL resources are released.
     */
    ~SdlDisplayConcrete() override;

    // --- IDisplay Interface Implementation ---
    bool configure(const DisplayConfig& config) override;
    bool initializeDisplay(int width, int height) override;
    void updateOriginalFrame(const uint8_t* rgbData, int width, int height) override;
    void updateProcessedFrame(const uint8_t* rgbData, int width, int height) override;
    void renderAndPollEvents() override;
    void setErrorCallback(std::function<void(const std::string&)>) override;
    void closeDisplay() override;
    bool is_Running() override;

    /**
     * @brief A static utility to convert a YUYV format image buffer to RGB.
     */
    static void yuyvToRGB(const uint8_t* yuyv, uint8_t* rgb, int width, int height);

private:
    // --- Private Member Variables ---
    std::shared_ptr<ISystemMetricsAggregator> metricAggregator_;

    // Custom deleter for safely managing SDL resources with std::unique_ptr
    struct SDL_Deleter {
        void operator()(SDL_Window* w) const;
        void operator()(SDL_Renderer* r) const;
        void operator()(SDL_Texture* t) const;
    };

    // SDL resources managed by smart pointers
    std::unique_ptr<SDL_Window, SDL_Deleter>   window_;
    std::unique_ptr<SDL_Renderer, SDL_Deleter> renderer_;
    std::unique_ptr<SDL_Texture, SDL_Deleter>  texOrig_;
    std::unique_ptr<SDL_Texture, SDL_Deleter>  texProc_;

    // Thread-safe state flags
    std::atomic<bool> running_{false};
    std::atomic<bool> initialized_{false};
    mutable std::mutex stateMutex_;

    // Configuration and queues
    DisplayConfig config_;
    int displayWidth_{0};
    int displayHeight_{0};
    std::shared_ptr<SharedQueue<std::shared_ptr<ZeroCopyFrameData>>> originalQueue_;
    std::shared_ptr<SharedQueue<std::shared_ptr<ZeroCopyFrameData>>> processedQueue_;

    // Buffers for holding the most recent frame data
    std::shared_ptr<ZeroCopyFrameData> lastOriginal_;
    std::shared_ptr<ZeroCopyFrameData> lastProcessed_;
    std::vector<uint8_t> origRgbBuffer_;
    std::vector<uint8_t> procRgbBuffer_;
    
    std::function<void(const std::string&)> errorCallback_;

    // --- Private Helper Methods ---
    void reportError(const std::string& msg);
    void initializeTextures(int width, int height);
};




// [MOD 09-22-2025] New file created to separate implementation from header.
// #include "SdlDisplayConcrete_new.h"
// #include "../Others/utils.h"

// =======================================================================
// Constructor / Destructor / Deleters
// =======================================================================

// [MOD 09-22-2025] Upgraded Constructor:
// - Removed the creation of a "dummy" ZeroCopyFrameData object.
// - This fixes compilation errors caused by outdated constructor calls.
// - Initializes lastOriginal_ and lastProcessed_ to nullptr for a more robust startup.
//   The render loop now gracefully handles the initial state before any frames are received.
SdlDisplayConcrete::SdlDisplayConcrete(
    std::shared_ptr<SharedQueue<std::shared_ptr<ZeroCopyFrameData>>> origQueue,
    std::shared_ptr<SharedQueue<std::shared_ptr<ZeroCopyFrameData>>> procQueue,
    std::shared_ptr<ISystemMetricsAggregator> aggregator
) : metricAggregator_(std::move(aggregator)),
    originalQueue_(std::move(origQueue)),
    processedQueue_(std::move(procQueue)),
    running_(false),
    initialized_(false),
    lastOriginal_(nullptr),
    lastProcessed_(nullptr) {
    spdlog::debug("[SdlDisplayConcrete] Constructor complete. Frame pointers initialized to null.");
}

SdlDisplayConcrete::~SdlDisplayConcrete() {
    try {
        closeDisplay();
    } catch (...) {
        // Destructors should not throw exceptions.
    }
}

void SdlDisplayConcrete::SDL_Deleter::operator()(SDL_Window* w) const { if (w) SDL_DestroyWindow(w); }
void SdlDisplayConcrete::SDL_Deleter::operator()(SDL_Renderer* r) const { if (r) SDL_DestroyRenderer(r); }
void SdlDisplayConcrete::SDL_Deleter::operator()(SDL_Texture* t) const { if (t) SDL_DestroyTexture(t); }


// =======================================================================
// IDisplay Interface Implementation
// =======================================================================

bool SdlDisplayConcrete::configure(const DisplayConfig& config) {
    std::lock_guard<std::mutex> lock(stateMutex_);
    if (config.width <= 0 || config.height <= 0) {
        reportError("Invalid display configuration (width/height <= 0).");
        return false;
    }
    config_ = config;
    return true;
}

bool SdlDisplayConcrete::initializeDisplay(int width, int height) {
    if (initialized_) {
        return true;
    }
    if (SDL_Init(SDL_INIT_VIDEO) < 0) {
        reportError(std::string("SDL_Init failed: ") + SDL_GetError());
        return false;
    }
    Uint32 windowFlags = SDL_WINDOW_SHOWN | SDL_WINDOW_RESIZABLE;
    if (config_.fullScreen) windowFlags |= SDL_WINDOW_FULLSCREEN_DESKTOP;
    
    const char* title = config_.windowTitle.empty() ? "ERL Framework Display" : config_.windowTitle.c_str();
    window_.reset(SDL_CreateWindow(title, SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, width, height, windowFlags));
    if (!window_) {
        reportError(std::string("SDL_CreateWindow failed: ") + SDL_GetError());
        SDL_Quit();
        return false;
    }
    renderer_.reset(SDL_CreateRenderer(window_.get(), -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC));
    if (!renderer_) {
        reportError(std::string("SDL_CreateRenderer failed: ") + SDL_GetError());
        SDL_Quit();
        return false;
    }
    initializeTextures(width / 2, height);
    initialized_ = true;
    running_ = true;
    displayWidth_ = width;
    displayHeight_ = height;
    spdlog::info("[SdlDisplayConcrete] SDL display initialized: {}x{}", width, height);
    return true;
}

void SdlDisplayConcrete::renderAndPollEvents() {
    if (!running_) return;

    SDL_Event e;
    while (SDL_PollEvent(&e)) {
        if (e.type == SDL_QUIT || (e.type == SDL_KEYDOWN && e.key.keysym.sym == SDLK_ESCAPE)) {
            running_ = false;
            return;
        }
    }

    auto frameStartTime = std::chrono::steady_clock::now();

    auto drain_latest = [](auto& q, std::shared_ptr<ZeroCopyFrameData>& out) {
        if (!q) return false;
        std::shared_ptr<ZeroCopyFrameData> tmp;
        bool got_new = false;
        while (q->try_pop(tmp)) {
            out = std::move(tmp);
            got_new = true;
        }
        return got_new;
    };

    drain_latest(originalQueue_, lastOriginal_);
    drain_latest(processedQueue_, lastProcessed_);
    
    if (lastOriginal_ && lastOriginal_->isValid()) {
        origRgbBuffer_.resize(lastOriginal_->width * lastOriginal_->height * 3);
        yuyvToRGB(static_cast<const uint8_t*>(lastOriginal_->dataPtr), origRgbBuffer_.data(), lastOriginal_->width, lastOriginal_->height);
        updateOriginalFrame(origRgbBuffer_.data(), lastOriginal_->width, lastOriginal_->height);
    }

    if (lastProcessed_ && lastProcessed_->isValid()) {
        procRgbBuffer_.resize(lastProcessed_->width * lastProcessed_->height * 3);
        yuyvToRGB(static_cast<const uint8_t*>(lastProcessed_->dataPtr), procRgbBuffer_.data(), lastProcessed_->width, lastProcessed_->height);
        updateProcessedFrame(procRgbBuffer_.data(), lastProcessed_->width, lastProcessed_->height);
    }
    
    {
        std::lock_guard<std::mutex> lock(stateMutex_);
        if (!renderer_) return;
        SDL_SetRenderDrawColor(renderer_.get(), 0, 0, 0, 255);
        SDL_RenderClear(renderer_.get());
        if (texOrig_) {
            SDL_Rect rectLeft = {0, 0, displayWidth_ / 2, displayHeight_};
            SDL_RenderCopy(renderer_.get(), texOrig_.get(), nullptr, &rectLeft);
        }
        if (texProc_) {
            SDL_Rect rectRight = {displayWidth_ / 2, 0, displayWidth_ / 2, displayHeight_};
            SDL_RenderCopy(renderer_.get(), texProc_.get(), nullptr, &rectRight);
        }
        SDL_RenderPresent(renderer_.get());
    }

    const auto shownFrame = lastProcessed_ ? lastProcessed_ : lastOriginal_;
    if (metricAggregator_ && shownFrame && shownFrame->isValid()) {
        auto frameEndSteady = std::chrono::steady_clock::now();
        DisplayStats stats{};
        stats.timestamp = std::chrono::system_clock::now();
        stats.renderTimeMs = std::chrono::duration<double, std::milli>(frameEndSteady - frameStartTime).count();
        if (shownFrame->hasSteadyTimestamp()) {
            stats.latencyMs = std::chrono::duration<double, std::milli>(frameEndSteady - shownFrame->captureSteady).count();
        }
        
    
        spdlog ::info ("[DebugID] mergeDisplay frameId={}",shownFrame->frameNumber);
        metricAggregator_->mergeDisplay(shownFrame->frameNumber, stats);
    }
}

void SdlDisplayConcrete::closeDisplay() {
    if (!running_.exchange(false)) return;
    initialized_ = false;
    {
        std::lock_guard<std::mutex> lock(stateMutex_);
        texOrig_.reset();
        texProc_.reset();
        renderer_.reset();
        window_.reset();
    }
    SDL_Quit();
    spdlog::info("[SdlDisplayConcrete] Display closed.");
}

bool SdlDisplayConcrete::is_Running() {
    return running_;
}

void SdlDisplayConcrete::setErrorCallback(std::function<void(const std::string&)> callback) {
    errorCallback_ = std::move(callback);
}

// =======================================================================
// Helper Methods
// =======================================================================

void SdlDisplayConcrete::initializeTextures(int width, int height) {
    if(!renderer_) throw std::runtime_error("Renderer is null during texture initialization.");
    
    texOrig_.reset(SDL_CreateTexture(renderer_.get(), SDL_PIXELFORMAT_RGB24, SDL_TEXTUREACCESS_STREAMING, width, height));
    texProc_.reset(SDL_CreateTexture(renderer_.get(), SDL_PIXELFORMAT_RGB24, SDL_TEXTUREACCESS_STREAMING, width, height));

    if (!texOrig_ || !texProc_) {
        throw std::runtime_error(std::string("Failed to create SDL textures: ") + SDL_GetError());
    }
}

void SdlDisplayConcrete::updateOriginalFrame(const uint8_t* rgbData, int width, int /*height*/) {
    std::lock_guard<std::mutex> lock(stateMutex_);
    if (!texOrig_ || !rgbData) return;
    SDL_UpdateTexture(texOrig_.get(), nullptr, rgbData, width * 3);
}

void SdlDisplayConcrete::updateProcessedFrame(const uint8_t* rgbData, int width, int /*height*/) {
    std::lock_guard<std::mutex> lock(stateMutex_);
    if (!texProc_ || !rgbData) return;
    SDL_UpdateTexture(texProc_.get(), nullptr, rgbData, width * 3);
}

void SdlDisplayConcrete::reportError(const std::string& msg) {
    if (errorCallback_) {
        errorCallback_(msg);
    } else {
        spdlog::error("[SdlDisplayConcrete] {}", msg);
    }
}

void SdlDisplayConcrete::yuyvToRGB(const uint8_t* yuyv, uint8_t* rgb, int width, int height) {
    auto clamp = [](int x) { return (x < 0) ? 0 : (x > 255 ? 255 : x); };
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; x += 2) {
            int yuyv_idx = (y * width + x) * 2;
            int rgb_idx = (y * width + x) * 3;

            int y0 = yuyv[yuyv_idx + 0];
            int u = yuyv[yuyv_idx + 1];
            int y1 = yuyv[yuyv_idx + 2];
            int v = yuyv[yuyv_idx + 3];

            int c = y0 - 16;
            int d = u - 128;
            int e = v - 128;
            rgb[rgb_idx + 0] = clamp((298 * c + 409 * e + 128) >> 8);
            rgb[rgb_idx + 1] = clamp((298 * c - 100 * d - 208 * e + 128) >> 8);
            rgb[rgb_idx + 2] = clamp((298 * c + 516 * d + 128) >> 8);

            c = y1 - 16;
            rgb[rgb_idx + 3] = clamp((298 * c + 409 * e + 128) >> 8);
            rgb[rgb_idx + 4] = clamp((298 * c - 100 * d - 208 * e + 128) >> 8);
            rgb[rgb_idx + 5] = clamp((298 * c + 516 * d + 128) >> 8);
        }
    }
}


#endif // SDL_DISPLAY_CONCRETE_NEW_H