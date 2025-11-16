// Scheduler.hpp  C++11 compatible, header-only
#pragma once
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <functional>
#include <string>
#include <thread>
#include <vector>
#include <algorithm>   // for std::min/std::max
#include <sched.h>
#include <unistd.h>    // access(), getpid()

#include "spdlog/spdlog.h"

namespace hrl {

// ====== Pequenos utilitários tipo-optional (C++11) ===========================
template<typename T>
struct Maybe {
  bool has;
  T    value;
  Maybe() : has(false), value() {}
  explicit Maybe(const T& v) : has(true), value(v) {}
};
struct MaybeBool  { bool has; bool value;  MaybeBool():has(false),value(false){} explicit MaybeBool(bool v):has(true),value(v){} };
struct MaybeDouble{ bool has; double value; MaybeDouble():has(false),value(0){}  explicit MaybeDouble(double v):has(true),value(v){} };

// ====== High-level policy (from RL agent) ===================================
enum class PolicyMode { MAX_PERFORMANCE, LOW_POWER, BALANCED };

struct RLAction {
  PolicyMode   mode;                    // default: BALANCED
  MaybeDouble  target_fps;              // ex: MaybeDouble(30.0) ou vazio
  MaybeDouble  power_budget_watts;      // ex: MaybeDouble(5.0)
  MaybeBool    prefer_gpu;              // ex: MaybeBool(true)

  RLAction()
  : mode(PolicyMode::BALANCED), target_fps(), power_budget_watts(), prefer_gpu() {}
  RLAction(PolicyMode m, MaybeDouble tfps, MaybeDouble pwr, MaybeBool pgpu)
  : mode(m), target_fps(tfps), power_budget_watts(pwr), prefer_gpu(pgpu) {}
};

// ====== Observação/Estado (do Aggregator) ===================================
struct MetricsSnapshot {
  double fps = 0.0;
  double latency_ms = 0.0;
  double energy_j_alg = 0.0;
  double avg_power_w_alg = 0.0;
  double cpu_util_avg = 0.0;   // 0..100
  double gpu_util_avg = 0.0;   // 0..100
  double cpu_temp_c = 0.0;
  double gpu_temp_c = 0.0;
};

// ====== Sinalizadores de controle de runtime (aplicados no seu pipeline) ====
struct RuntimeControls {
  // Integre estes campos ao seu Config/Algorithm/ThreadManager
  std::atomic<int>  concurrency_level; // threads do workload (CPU)
  std::atomic<bool> enable_gpu;        // alterna caminho GPU/CPU
  // Afinidade (packing/spreading)
  enum class Affinity { Spread, Pack };
  std::atomic<Affinity> affinity;

  RuntimeControls()
  : concurrency_level(2),
    enable_gpu(true),
    affinity(Affinity::Spread) {}
};

// ====== Utilitários DVFS / sysfs (sem <filesystem>) ==========================
struct Sysfs {
  static bool exists(const std::string& path) {
    return ::access(path.c_str(), F_OK) == 0;
  }
  static bool writeStr(const std::string& path, const std::string& val) {
    if (!exists(path)) return false;
    std::ofstream f(path.c_str());
    if (!f.good()) return false;
    f << val;
    return f.good();
  }
  static bool writeInt(const std::string& path, long v) {
    return writeStr(path, toString(v));
  }
  static std::string toString(long v) {
    // std::to_string is fine in C++11, but keeping a wrapper anyway
    return std::to_string(v);
  }
  static bool cpuSetGovernorAll(const std::string& gov) {
    bool ok = true;
    for (int cpu = 0; ; ++cpu) {
      std::string base = "/sys/devices/system/cpu/cpu" + std::to_string(cpu);
      if (!exists(base)) break;
      std::string p = base + "/cpufreq/scaling_governor";
      ok = writeStr(p, gov) && ok;
    }
    return ok;
  }
  static bool cpuSetOnlineCores(int online) {
    bool ok = true;
    int count = 0;
    for (int cpu = 0; ; ++cpu) {
      std::string base = "/sys/devices/system/cpu/cpu" + std::to_string(cpu);
      if (!exists(base)) break;
      ++count;
    }
    for (int cpu = 0; cpu < count; ++cpu) {
      std::string p = "/sys/devices/system/cpu/cpu" + std::to_string(cpu) + "/online";
      ok = writeInt(p, (cpu < online ? 1 : 0)) && ok;
    }
    return ok;
  }
  static bool cpuSetFreqKHzAll(long min_khz, long max_khz) {
    bool ok = true;
    for (int cpu = 0; ; ++cpu) {
      std::string base = "/sys/devices/system/cpu/cpu" + std::to_string(cpu);
      if (!exists(base)) break;
      ok = writeInt(base + "/cpufreq/scaling_min_freq", min_khz) && ok;
      ok = writeInt(base + "/cpufreq/scaling_max_freq", max_khz) && ok;
    }
    return ok;
  }
  static bool gpuSetFreqKHz(long min_khz, long max_khz) {
    // Nano (devfreq path pode variar por BSP)
    const std::string base = "/sys/devices/57000000.gpu/devfreq/57000000.gpu";
    bool ok = true;
    ok = writeInt(base + "/min_freq", min_khz) && ok;
    ok = writeInt(base + "/max_freq", max_khz) && ok;
    return ok;
  }
  static bool nvpmodel(int mode) {
    // requer sudoers; fallback silencioso se não disponível
    std::string cmd = "nvpmodel -m " + std::to_string(mode) + " 2>/dev/null";
    int rc = std::system(cmd.c_str());
    return (rc == 0);
  }
  static bool jetsonClocks(bool enable) {
    int rc = std::system(enable ? "jetson_clocks 2>/dev/null"
                                : "jetson_clocks --restore 2>/dev/null");
    return (rc == 0);
  }
};

// ====== Afinidade de CPU para threads do processo ============================
inline bool setThreadAffinity(const std::vector<int>& cpus) {
  cpu_set_t set; CPU_ZERO(&set);
  for (size_t i = 0; i < cpus.size(); ++i) CPU_SET(cpus[i], &set);
  // 0 => processo atual; se quiser o processo, pode usar getpid()
  return ::sched_setaffinity(0, sizeof(set), &set) == 0;
}

// Estratégias simples de seleção de CPUs (pack/spread) dado N on-line
inline std::vector<int> chooseCpus(int online, RuntimeControls::Affinity af) {
  std::vector<int> v;
  if (online < 1) return v;
  // Simples, mas mantém semântica para expandir no futuro (big.LITTLE etc.)
  if (af == RuntimeControls::Affinity::Pack) {
    for (int i = 0; i < online; ++i) v.push_back(i);
  } else {
    for (int i = 0; i < online; ++i) v.push_back(i);
  }
  return v;
}

// ====== Scheduler ============================================================
class Scheduler {
public:
  struct Limits {
    int  max_cores;    // Jetson Nano quad-core
    long cpu_min_khz;  // ajuste aos valores do /cpufreq/scaling_available_frequencies
    long cpu_max_khz;
    long gpu_min_khz;  // valores típicos (confirme via sysfs)
    long gpu_max_khz;

    Limits()
    : max_cores(4),
      cpu_min_khz(102000),
      cpu_max_khz(1479000),
      gpu_min_khz(76800),
      gpu_max_khz(921600) {}
  };

  explicit Scheduler(const Limits& lim = Limits()) : lim_(lim) {}

  // Aplica decisão completa com base na política (RLAction) + métricas atuais
  void apply(const RLAction& action, const MetricsSnapshot& m, RuntimeControls& rt) {
    spdlog::info("[Scheduler] policy={}, fps={:.2f} cpu={:.1f}% gpu={:.1f}% P={:.2f}W",
                 policyName(action.mode), m.fps, m.cpu_util_avg, m.gpu_util_avg, m.avg_power_w_alg);

    switch (action.mode) {
      case PolicyMode::MAX_PERFORMANCE: applyMaxPerf(action, m, rt); break;
      case PolicyMode::LOW_POWER:       applyLowPower(action, m, rt); break;
      case PolicyMode::BALANCED:        applyBalanced(action, m, rt); break;
    }
  }

private:
  Limits lim_;

  static const char* policyName(PolicyMode p) {
    switch (p) {
      case PolicyMode::MAX_PERFORMANCE: return "MAX_PERFORMANCE";
      case PolicyMode::LOW_POWER:       return "LOW_POWER";
      case PolicyMode::BALANCED:        return "BALANCED";
    }
    return "UNKNOWN";
  }

  // --- Perf preset -----------------------------------------------------------
  void applyMaxPerf(const RLAction& a, const MetricsSnapshot& /*m*/, RuntimeControls& rt) {
    const int cores = std::max(lim_.max_cores, 1);
    rt.concurrency_level.store(cores);
    rt.affinity.store(RuntimeControls::Affinity::Spread);
    (void)setThreadAffinity(chooseCpus(cores, rt.affinity.load()));

    // GPU on por padrão, a menos que especificado
    bool gpu_pref = a.prefer_gpu.has ? a.prefer_gpu.value : true;
    rt.enable_gpu.store(gpu_pref);

    // DVFS / presets
    Sysfs::nvpmodel(0);        // modo destravado (ajuste conforme seu nvpmodel.conf)
    Sysfs::jetsonClocks(true); // trava clocks no máximo (se permitido)
    Sysfs::cpuSetGovernorAll("performance");
    Sysfs::cpuSetOnlineCores(cores);
    Sysfs::cpuSetFreqKHzAll(lim_.cpu_max_khz, lim_.cpu_max_khz);
    Sysfs::gpuSetFreqKHz(lim_.gpu_max_khz, lim_.gpu_max_khz);

    spdlog::info("[Scheduler] MAX_PERFORMANCE: {} cores, GPU={}, governor=performance",
                 cores, rt.enable_gpu.load());
  }

  // --- Low power preset ------------------------------------------------------
  void applyLowPower(const RLAction& a, const MetricsSnapshot& /*m*/, RuntimeControls& rt) {
    const double budgetW = a.power_budget_watts.has ? a.power_budget_watts.value : 4.5; // Nano ~5W
    int cores = (budgetW <= 3.0 ? 1 : 2);
    cores = std::max(1, std::min(cores, lim_.max_cores));
    rt.concurrency_level.store(cores);
    rt.affinity.store(RuntimeControls::Affinity::Pack);
    (void)setThreadAffinity(chooseCpus(cores, rt.affinity.load()));

    bool gpu = a.prefer_gpu.has ? a.prefer_gpu.value : false;
    rt.enable_gpu.store(gpu);

    Sysfs::jetsonClocks(false);          // libera DVFS
    Sysfs::nvpmodel(1);                  // perfil econômico, ajuste ao seu board
    Sysfs::cpuSetGovernorAll("powersave");

    // Frequências moderadas (min=~102MHz, max=~1.0GHz  ajuste à sua SKU)
    long cpu_mid = 1020000L;
    cpu_mid = std::max(lim_.cpu_min_khz, std::min(cpu_mid, lim_.cpu_max_khz));

    Sysfs::cpuSetOnlineCores(cores);
    Sysfs::cpuSetFreqKHzAll(lim_.cpu_min_khz, cpu_mid);

    // GPU mínima (ou desligar via rt.enable_gpu=false e ramo CPU-only no app)
    long gmax = gpu ? (lim_.gpu_min_khz * 2) : lim_.gpu_min_khz;
    gmax = std::max(lim_.gpu_min_khz, std::min(gmax, lim_.gpu_max_khz));
    Sysfs::gpuSetFreqKHz(lim_.gpu_min_khz, gmax);

    spdlog::info("[Scheduler] LOW_POWER: {} cores, GPU={}, governor=powersave, budget~{:.1f}W",
                 cores, rt.enable_gpu.load(), budgetW);
  }

  // --- Balanced (PID-lite em cima de FPS/Power) ------------------------------
  void applyBalanced(const RLAction& a, const MetricsSnapshot& m, RuntimeControls& rt) {
    const double targetFps = a.target_fps.has ? a.target_fps.value : 30.0;
    const double err = targetFps - m.fps; // >0 falta desempenho, <0 sobra

    // Heurística: ajuste pequeno no #cores
    int curC = rt.concurrency_level.load();
    int delta = (err >  2.0 ? +1 : (err < -5.0 ? -1 : 0));
    int newC = curC + delta;
    newC = std::max(1, std::min(newC, lim_.max_cores));
    rt.concurrency_level.store(newC);

    // Afinidade: se CPU > 85% e fps baixo -> Spread; se fps ok -> Pack
    if (m.cpu_util_avg > 85.0 && m.fps < targetFps) {
      rt.affinity.store(RuntimeControls::Affinity::Spread);
    } else if (m.fps >= targetFps) {
      rt.affinity.store(RuntimeControls::Affinity::Pack);
    }
    (void)setThreadAffinity(chooseCpus(newC, rt.affinity.load()));

    // GPU: se GPU ociosa mas fps baixo -> ligar; se fps de sobra e power alto -> desligar
    bool gpu = rt.enable_gpu.load();
    if (m.fps < targetFps - 2.0 && m.gpu_util_avg < 60.0) gpu = true;
    if (m.fps > targetFps + 5.0 && m.avg_power_w_alg > 5.0 && m.gpu_util_avg > 80.0) gpu = false;
    if (a.prefer_gpu.has) gpu = a.prefer_gpu.value; // RL override
    rt.enable_gpu.store(gpu);

    // DVFS intermediário: governor ondemand/schedutil + teto adaptativo
    Sysfs::jetsonClocks(false);
    Sysfs::nvpmodel(1);
    Sysfs::cpuSetGovernorAll("schedutil");

    // Ajuste teto CPU (eleva se err>0 e CPU>70; reduz se err<0 e Power alto)
    long max_khz;
    if (err > 0.0 && m.cpu_util_avg > 70.0) {
      max_khz = lim_.cpu_max_khz;
    } else {
      long mid = static_cast<long>((lim_.cpu_min_khz + lim_.cpu_max_khz) * 0.7);
      max_khz = mid;
    }
    max_khz = std::max(lim_.cpu_min_khz, std::min(max_khz, lim_.cpu_max_khz));

    Sysfs::cpuSetOnlineCores(newC);
    Sysfs::cpuSetFreqKHzAll(lim_.cpu_min_khz, max_khz);

    // GPU teto moderado
    long gmax;
    if (gpu) {
      long span = lim_.gpu_max_khz - lim_.gpu_min_khz;
      gmax = lim_.gpu_min_khz + static_cast<long>(span * 0.7);
    } else {
      gmax = lim_.gpu_min_khz;
    }
    gmax = std::max(lim_.gpu_min_khz, std::min(gmax, lim_.gpu_max_khz));
    Sysfs::gpuSetFreqKHz(lim_.gpu_min_khz, gmax);

    spdlog::info("[Scheduler] BALANCED: cores={} ({}), gpu={}, cpu_max={} kHz, errFPS={:+.1f}",
                 newC,
                 (rt.affinity.load()==RuntimeControls::Affinity::Spread ? "spread":"pack"),
                 rt.enable_gpu.load(), max_khz, err);
  }
};

} // namespace hrl

// /* ================== EXEMPLO DE USO (C++11) ==================

// #include "Scheduler.hpp"
// using namespace hrl;

// int main() {
//   spdlog::set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%^%l%$] %v");

//   Scheduler::Limits lim;
//   Scheduler sched(lim);

//   RuntimeControls rt;

//   // Sem std::optional: use MaybeDouble/MaybeBool
//   RLAction action(PolicyMode::BALANCED,
//                   MaybeDouble(30.0),     // target_fps
//                   MaybeDouble(),         // power_budget (vazio)
//                   MaybeBool(true));      // prefer_gpu

//   MetricsSnapshot m;
//   m.fps=24.0; m.cpu_util_avg=82; m.gpu_util_avg=35; m.avg_power_w_alg=4.8;

//   sched.apply(action, m, rt);

//   spdlog::info("concurrency_level={} enable_gpu={}",
//                rt.concurrency_level.load(),
//                rt.enable_gpu.load());
//   return 0;
// }

//====================================================== */

