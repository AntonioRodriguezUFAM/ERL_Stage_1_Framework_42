// RuntimeControls.hpp
#pragma once
#include <atomic>
#include <cstdint>

namespace hrl {

// Thread-safe knobs your runtime (pipeline) will read.
struct RuntimeControls {
  // How many worker threads/operators to run in parallel (algorithm-level).
  std::atomic<int>  concurrency_level;     // >= 1
  // Whether to allow GPU kernels (your algorithm can check this).
  std::atomic<bool> enable_gpu;
  // Optional hint you can map to governor/profile (0=default, 1=perf, 2=powersave).
  std::atomic<int>  governor_hint;

  RuntimeControls()
  : concurrency_level(1)
  , enable_gpu(true)
  , governor_hint(0) {}
};

} // namespace hrl
