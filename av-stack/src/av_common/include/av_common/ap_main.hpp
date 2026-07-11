// av_common/ap_main.hpp — shared main() helpers for Adaptive Applications.
// Handles ara::core init, ara::exec running/terminating, and a SIGINT-driven run loop.
#ifndef AV_COMMON_AP_MAIN_HPP
#define AV_COMMON_AP_MAIN_HPP

#include <atomic>
#include <chrono>
#include <csignal>
#include <functional>
#include <string>
#include <thread>

#include "av_ap/ap.hpp"

namespace av {
namespace ap {

inline std::atomic<bool>& run_flag() {
  static std::atomic<bool> g{true};
  return g;
}
inline void install_sigint() {
  std::signal(SIGINT, [](int) { run_flag() = false; });
  std::signal(SIGTERM, [](int) { run_flag() = false; });
}

// Run an Adaptive Application: init AP, report running, tick `on_period` every
// `period` (use a no-op for purely event-driven apps), until SIGINT/SIGTERM.
// on_start (optional): the app's transition into the RUNNING state — e.g. subscribing to
// SOME/IP events. It runs AFTER ReportRunning and a short settle, so the binding registration
// has completed before any receive handler can fire. Apps whose subscribed data is already
// live at startup (localization's IMU feed) MUST defer their Subscribe here; subscribing in the
// ctor lets the event flood race an incomplete registration -> SIGSEGV.
inline int run_app(const std::function<void()>& on_period,
                   std::chrono::milliseconds period,
                   const std::function<void()>& on_start = nullptr) {
  Initialize();
  install_sigint();
  ExecutionClient exec;
  exec.ReportRunning();  // ara::exec: process is up (Execution Management) -> enter RUNNING
  if (on_start) {
    std::this_thread::sleep_for(std::chrono::milliseconds(300));  // let registration settle
    on_start();
  }
  while (run_flag()) {
    if (on_period) on_period();
    std::this_thread::sleep_for(period);
  }
  exec.ReportTerminating();
  Deinitialize();
  return 0;
}

// Startup phase, selected by the Execution Manifest via StartupConfig.processArgument.
// Two Processes share one executable: an "init" Process (phase=init) and a "run"
// Process (phase=run); see av_execution_manifest.arxml.
enum class Phase { kInit, kRun };

// Parse the phase from the process arguments; defaults to kRun when unspecified so a
// plainly-launched binary still behaves as before.
inline Phase phase_from_args(int argc, char** argv) {
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    if (a == "--phase=init" || a == "init") return Phase::kInit;
    if (a == "--phase=run" || a == "run") return Phase::kRun;
  }
  return Phase::kRun;
}

// AUTOSAR AP two-phase startup — the INIT half of the barrier (see Figure 8.9 of the
// Manifest Specification). The app object is already constructed by the caller, so the
// one-time initialization (the ctor: register + offer, plus any optional on_init) has
// run. We report RUNNING so Execution Management observes the Process, hold briefly, then
// self-terminate. The manifest marks this Process processIsSelfTerminating; the companion
// "run" Process is held by an ExecutionDependency until every AA's init Process reaches
// the Terminated state.
inline int run_init(std::chrono::milliseconds hold = std::chrono::seconds(2),
                    const std::function<void()>& on_init = nullptr) {
  Initialize();  // idempotent (see run_app)
  install_sigint();
  ExecutionClient exec;
  exec.ReportRunning();
  if (on_init) on_init();
  // "run for a while": let offered services register / one-time init settle, but cut short
  // on SIGINT/SIGTERM so Execution Management can stop us promptly.
  const auto deadline = std::chrono::steady_clock::now() + hold;
  while (run_flag() && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  exec.ReportTerminating();  // self-terminate -> EM marks this Process Terminated
  Deinitialize();
  return 0;
}

}  // namespace ap
}  // namespace av
#endif  // AV_COMMON_AP_MAIN_HPP
