// SharedQueue.h

//**
// Review of SharedQueue.h
// Overview
// SharedQueue<T> is a thread-safe, bounded queue for producer-consumer patterns, using mutexes and condition variables. It supports stopping/graceful shutdown and optional buffer pooling (via BufferPool). The template allows flexibility (e.g., T = std::shared_ptr<ZeroCopyFrameData> for frame queues).
// BufferPool: A separate class for pre-allocating/reusing buffers (char arrays). It expands dynamically if exhausted.
// Key features:

// Bounded/Unbounded: maxSize enforces capacity; unbounded if 0 (but code asserts >0, so always bounded).
// Operations: Blocking push/pop, non-blocking try_pop, isEmpty/isFull, stop/close.
// Shutdown: stop() sets flag, clears queue, notifies allunblocks waiters.
// Buffer Management: Optional BufferPool integration (though not used in visible methods; likely for T-specific alloc).

// *// @file SharedQueue.h
//  * @brief Thread-safe bounded queue for producer-consumer scenarios with optional buffer pooling.
//  * 
//  * This class provides a thread-safe queue implementation to support producer-consumer scenarios.
//  * It allows producers to push items into the queue and consumers to pop items from it.
//  * The queue can be bounded (with a maximum size) or unbounded, and it supports blocking and non-blocking operations.
//  * Additionally, it includes a mechanism to stop the queue and unblock all waiting threads.
//  * 
//  * Optionally, a BufferPool can be provided to manage memory for the items in the queue.
//  * 
//  * @tparam T The type of items stored in the queue. Typically, this would be std::shared_ptr<ZeroCopyFrameData>.
//  * 
//  * @note This implementation uses C++11 features such as std::mutex, std::condition_variable, and std::atomic.
//  * 
//  * @author Antonio Souto
//  * @date 2024-06-10
//  */

#pragma once

#include <queue>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <stdexcept>
#include <spdlog/spdlog.h>

class BufferPool {
public:
        BufferPool(size_t bufferSize, size_t poolSize) 
        : bufferSize_(bufferSize), poolSize_(poolSize) {
        if (bufferSize == 0 || poolSize == 0) {
            spdlog::error("[BufferPool] Invalid bufferSize ({}) or poolSize ({}).", bufferSize, poolSize);
            throw std::invalid_argument("Invalid buffer or pool size");
        }
        for (size_t i = 0; i < poolSize_; ++i) {
            auto buffer = std::make_unique<char[]>(bufferSize_);
            freeBuffers_.push(std::move(buffer));
        }
    }

    

    std::unique_ptr<char[]> acquireBuffer() {
        std::lock_guard<std::mutex> lock(mutex_);
        if (freeBuffers_.empty()) {
            spdlog::warn("[BufferPool] Expanding pool.");
            return std::make_unique<char[]>(bufferSize_);
        }
        auto buffer = std::move(freeBuffers_.front());
        freeBuffers_.pop();
        return buffer;
    }

    void releaseBuffer(std::unique_ptr<char[]> buffer) {
        std::lock_guard<std::mutex> lock(mutex_);
        freeBuffers_.push(std::move(buffer));
    }

private:
    size_t bufferSize_;
    size_t poolSize_;
    std::queue<std::unique_ptr<char[]>> freeBuffers_;
    std::mutex mutex_;
};



/**
 * @brief SharedQueue is a thread-safe queue implementation to support producer-consumer scenarios.
 * 
 * This class provides mechanisms for producers to push items into the queue and consumers to pop
 * items from it. The queue blocks consumers when it is empty and unblocks them when items are added
 * or the queue is stopped.
 * 
 * Optionally, you can enable "bounded capacity" by setting maxSize > 0.
 */
template <typename T>   // SharedQueue.h (no changes needed for template)
// Existing implementation handles T as shared_ptr<ZeroCopyFrameData>
class SharedQueue {
    
public:
    /**
     * @brief Construct a new SharedQueue with optional bounded capacity.
     * @param maxSize If > 0, queue is bounded. If 0, unbounded.
     * @param bufferPool Optional BufferPool for managing T's buffers.
     */
    // explicit SharedQueue(size_t maxSize = 10, std::shared_ptr<BufferPool> bufferPool = nullptr)
    //     : stop_(false), maxSize_(maxSize), bufferPool_(bufferPool) {
    //     assert(maxSize_ >= 0 && "Invalid maxSize");
    // }
//     explicit SharedQueue(size_t maxSize, std::shared_ptr<BufferPool> bufferPool = nullptr)
//     : maxSize_(maxSize), bufferPool_(bufferPool) {
//     assert(maxSize_ > 0 && "Invalid maxSize");
//     //queue_.reserve(maxSize_);
// }
 explicit SharedQueue(size_t maxSize = 100, std::shared_ptr<BufferPool> bufferPool = nullptr)
        : maxSize_(maxSize), stop_(false), bufferPool_(bufferPool) {
            // assert(maxSize_ > 0 && "[SharedQueue] maxSize must be > 0");
            // spdlog::info("[SharedQueue] Initialized with maxSize: {}.", maxSize_);
            // [MOD] Allow maxSize=0 for unbounded queues; remove assert and handle in isFull().
            // assert(maxSize_ > 0 && "[SharedQueue] maxSize must be > 0");
            if (maxSize_ == 0) spdlog::warn("[SharedQueue] Unbounded queue; monitor for OOM.");
            spdlog::info("[SharedQueue] Initialized with maxSize: {}.", maxSize_);
        }
        


    /**
     * @brief Destructor to clean up resources.
     * Stops the queue and destroys the mutex.
     */
    ~SharedQueue() {
        stop();
        spdlog::info("[SharedQueue] Destroyed.");
    }
   

    /**
     * @brief Push an item into the queue (Producer).
     * Blocks if queue is full (bounded mode) until space is available or stop() is called.
     */
    // void push(const T& item) {
    //     {
    //         std::unique_lock<std::mutex> lock(mutex_);
    //         condNotFull_.wait(lock, [this] {
    //             return stop_ || (maxSize_ == 0) || (queue_.size() < maxSize_);
    //         });
    //         if (stop_) {
    //             spdlog::warn("SharedQueue: Push to stopped queue");
    //             return;
    //         }
    //         queue_.push(item);
    //     }
    //     condNotEmpty_.notify_one();
    // }
    /**
     * @brief Push an item into the queue (Producer).
     * Non-blocking if queue is full or stopped.
     * @return true if item was pushed, false if queue is full or stopped.
     */
    
    bool push(const T& item) {
        std::unique_lock<std::mutex> lock(mutex_);
        condNotFull_.wait(lock, [this] {
            return stop_ || (maxSize_ == 0) || (queue_.size() < maxSize_);
        });
        if (stop_) {
            spdlog::warn("[SharedQueue] Push to stopped queue.");
            return false;
        }
        queue_.push(item);
        lock.unlock();
        condNotEmpty_.notify_one();
        return true;
    }

    void push(T&& item) {
        std::unique_lock<std::mutex> lock(mutex_);
        condNotFull_.wait(lock, [this] {
            return stop_ || (maxSize_ == 0) || (queue_.size() < maxSize_);
        });
        if (stop_) {
            spdlog::warn("[SharedQueue] Push to stopped queue.");
            return;
        }
        queue_.push(std::move(item));
        lock.unlock();
        condNotEmpty_.notify_one();
    }

    /**
     * @brief Push an rvalue reference (move).
     */
    // void push(T&& item) {
    //     {
    //         std::unique_lock<std::mutex> lock(mutex_);
    //         condNotFull_.wait(lock, [this] {
    //             return stop_ || (maxSize_ == 0) || (queue_.size() < maxSize_);
    //         });
    //         if (stop_) {
    //              spdlog::warn("SharedQueue: Push to stopped queue");
    //             return;
    //         }
    //         queue_.push(std::move(item));
    //     }
    //     condNotEmpty_.notify_one();
    // }

    /**
     * @brief Pop an item from the queue (Consumer). 
     * Blocks if queue is empty (and not stopped).
     * @return false if the queue is stopped and empty.
     * 
     * @attention:
     * The consumer will wait until the queue is not empty or stop() is called.
     * The SharedQueue class provides the methods you need to retrieve frames:
     * - bool pop(FrameData& item) (Blocking Pop)
     * - bool tryPop(FrameData& item) (Non-blocking Pop)
     * pop(FrameData& item) (Blocking Pop):
     * This method will block (wait) if the queue is currently empty until a new FrameData becomes available in the queue.
     * Once a FrameData is available, it is removed from the queue and copied into the item variable that you provide as an argument.
     * Use Case: Suitable for consumer threads that must process every frame and can afford to wait. If no frames are available, the thread will pause, saving CPU cycles.
     */
    // bool pop(T& item) {
    //     std::unique_lock<std::mutex> lock(mutex_);
    //     condNotEmpty_.wait(lock, [this] {
    //         return stop_ || !queue_.empty();
    //     });
    //     if (stop_ && queue_.empty()) {
    //         return false;
    //     }
    //     item = std::move(queue_.front());
    //     queue_.pop();
    //     if (maxSize_ > 0) {
    //         condNotFull_.notify_one();
    //     }
    //     return true;
    // }

    bool pop(T& item) {
        std::unique_lock<std::mutex> lock(mutex_);
        condNotEmpty_.wait(lock, [this] {
            return stop_ || !queue_.empty();
        });
        if (stop_ && queue_.empty()) {
            return false;
        }
        item = std::move(queue_.front());
        queue_.pop();
        lock.unlock();
        if (maxSize_ > 0) {
            condNotFull_.notify_one();
        }
        return true;
    }


     /**
     * @brief try_pop method in the SharedQueue class. This method will attempt to pop an item from the queue without blocking
     * Try to pop an item from the queue without blocking.
     * @return false if the queue is stopped and empty.
     * 
     * @attention: try_pop(FrameData& item) (Non-Blocking Pop):
     * This method will not block. It immediately checks if there is a FrameData in the queue.
     * If a FrameData is available, it is removed from the queue and copied into item, and the method returns true.
     * If the queue is empty, it returns false immediately without waiting.
     * Use Case: Useful when the consumer thread needs to do other things if no frame is immediately available, 
     * or when you want to avoid blocking in certain scenarios. 
     * You would typically use this in a loop and check the return value to see if a frame was retriev
     */
    bool try_pop(T& item) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (queue_.empty()) {
            return false; // Return false if the queue is empty
        }
        item = std::move(queue_.front());
        queue_.pop();
        if (maxSize_ > 0) {
            condNotFull_.notify_one();
        }
        return true; // Return true if an item was popped
    }

    

    /**
     * @brief isEmpty method in the SharedQueue class. This method checks if the queue is empty.
     * 
     */
    bool isEmpty() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return queue_.empty();
        //return queue_.empty() && !stop_;
    }


/**
 * @brief is full method in the SharedQueue class. This method checks if the queue is full.
 * 
 */
    bool isFull() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return (maxSize_ > 0) && (queue_.size() >= maxSize_);
        //return queue_.size() >= maxSize_;
    }
    /**
     * @brief Stop the queue and unblock all producers/consumers.
     */
    // void stop() {
    //     {
    //         std::lock_guard<std::mutex> lock(mutex_);
    //         stop_ = true;
    //     }
    //     cond_.notify_all();
    //     condNotEmpty_.notify_all();
    //     condNotFull_.notify_all();
    // }


    void stop() {
        std::lock_guard<std::mutex> lock(mutex_);
        stop_ = true;
        // [MOD] Optional clear: Add param bool clearQueue=false; only clear if true to avoid data loss during shutdown.
        // queue_ = std::queue<T>();
        condNotEmpty_.notify_all();
        condNotFull_.notify_all();
    }


    /**
     * @brief Alias for stop().
     */
    void close() { stop(); }

    /**
     * @brief Return snapshot of current size (not always accurate due to concurrency).
     */
    size_t size() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return queue_.size();
    }

    

    size_t capacity() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return maxSize_;
    }

    /**
     * @brief Check whether stop() has been called.
     */
    bool isStopped() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return stop_;
    }

private:
     std::queue<T> queue_;
    mutable std::mutex mutex_;
    std::condition_variable condNotEmpty_;
    std::condition_variable condNotFull_;
    size_t maxSize_;
    std::atomic<bool> stop_;
    std::shared_ptr<BufferPool> bufferPool_;
};
