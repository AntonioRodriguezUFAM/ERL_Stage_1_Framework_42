
//======================================================================================================================================================
// CameraConfig.h
// Date: 09-20-2025
// [MOD 09-20-2025] Added numBuffers field with validation for V4L2 buffer configuration.
//======================================================================================================================================================

#pragma once
#include <string>
#include <cstdint>       // for uint32_t
#ifdef __linux__
#include <linux/videodev2.h>
#else
// Define fallback V4L2 constants for non-Linux systems
#define V4L2_PIX_FMT_YUYV 0x56595559 // 'YUYV'
#define V4L2_PIX_FMT_MJPEG 0x47504A4D // 'MJPG'
#endif
#include <spdlog/spdlog.h> // For logging

/**
 * @brief Enum that wraps V4L2 pixel format codes
 */
enum class PixelFormat : uint32_t {
    YUYV = V4L2_PIX_FMT_YUYV, 
    MJPG = V4L2_PIX_FMT_MJPEG,
    // Add more as needed...
};

/**
 * @brief Configuration structure for a camera device.
 */
struct CameraConfig {
    int width = 640;          ///< Desired capture width (pixels)
    int height = 480;         ///< Desired capture height (pixels)
    int fps = 30;             ///< Desired frames per second
    PixelFormat pixelFormat = PixelFormat::YUYV; ///< Pixel format (e.g., YUYV, MJPEG)
    int numBuffers = 4;       ///< [MOD 09-20-2025] Number of V4L2 MMAP buffers (2–8 recommended)

    /**
     * @brief Constructor for CameraConfig with default values.
     * @param w Width in pixels.
     * @param h Height in pixels.
     * @param f Frames per second.
     * @param pf Pixel format (V4L2 code).
     * @param nb Number of buffers (default 4).
     * [MOD 09-20-2025] Added numBuffers parameter.
     */
    CameraConfig(int w = 640, int h = 480, int f = 30, PixelFormat pf = PixelFormat::YUYV, int nb = 4)
        : width(w), height(h), fps(f), pixelFormat(pf), numBuffers(nb) {
        // [MOD 09-20-2025] Validate numBuffers in constructor
        if (!validate()) {
            spdlog::error("[CameraConfig] Invalid configuration: width={}, height={}, fps={}, numBuffers={}",
                          width, height, fps, numBuffers);
            throw std::invalid_argument("Invalid CameraConfig parameters");
        }
    }

    /**
     * @brief Validates the configuration.
     * @return True if valid, false otherwise.
     * [MOD 09-20-2025] Added numBuffers validation (2–8).
     */
    bool validate() const {
        if (width <= 0 || height <= 0) {
            spdlog::error("[CameraConfig] Invalid dimensions: width={}, height={}", width, height);
            return false;
        }
        if (fps <= 0) {
            spdlog::error("[CameraConfig] Invalid FPS: {}", fps);
            return false;
        }
        // [MOD 09-20-2025] Ensure numBuffers is in a reasonable range
        if (numBuffers < 2 || numBuffers > 8) {
            spdlog::error("[CameraConfig] numBuffers={} is out of range (2–8)", numBuffers);
            return false;
        }
        return true;
    }
};
//======================================================================================================================================================