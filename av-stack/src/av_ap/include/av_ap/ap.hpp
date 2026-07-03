// av_ap/ap.hpp — AUTOSAR Adaptive Platform Functional-Cluster adaptation layer.
//
// Thin wrappers over the AP Functional Clusters, matching the arc42 architecture:
//   ara::exec  → ExecutionClient   (report process lifecycle to Execution Management)
//   ara::log   → Logger            (structured Log & Trace)
//   ara::phm   → SupervisedEntity   (alive/deadline supervision by Platform Health Mgmt)
//   (system state = ara::sm Machine States + Function Groups; see the Machine/Execution Manifests)
//
// Build modes:
//   -DAP_HAVE_ARA  → real ara::* APIs (Adaptive-AUTOSAR runtime on the Jetson).
//   (default)      → host shim (stderr) so the stack builds/runs on a plain host (Path A).
#ifndef AV_AP_AP_HPP
#define AV_AP_AP_HPP

#include <cstdint>
#include <memory>
#include <string>

#if defined(AP_HAVE_ARA)
#include "ara/core/initialization.h"
#include "ara/core/instance_specifier.h"
#include "ara/exec/execution_client.h"
#include "ara/log/logging.h"
#include "ara/phm/supervised_entity.h"
#else
#include <iostream>
#endif

namespace av {
namespace ap {

// ---------------------------------------------------------------------------
// Runtime init/deinit — ara::core::Initialize / Deinitialize on the target.
// ---------------------------------------------------------------------------
inline void Initialize() {
#if defined(AP_HAVE_ARA)
  ara::core::Initialize();
#endif
}
inline void Deinitialize() {
#if defined(AP_HAVE_ARA)
  ara::core::Deinitialize();
#endif
}

// ---------------------------------------------------------------------------
// ara::exec — Execution Management client.
//   NOTE: ara::exec::ExecutionClient must be constructed with the platform's SOME/IP
//   RpcClient, which is provided by a running Execution Manager. In this manual
//   bring-up (run_ap.sh stands in for EM) there is no EM daemon to report to, so these
//   calls are no-ops. Under a real EM, construct an ara::exec::ExecutionClient here and
//   forward ReportExecutionState(kRunning/kTerminating).
// ---------------------------------------------------------------------------
class ExecutionClient {
 public:
  ExecutionClient() = default;
  void ReportRunning() {}
  void ReportTerminating() {}
};

// ---------------------------------------------------------------------------
// ara::log — Log & Trace. Severity-tagged logger.
// ---------------------------------------------------------------------------
class Logger {
 public:
  explicit Logger(const std::string& ctx_id, const std::string& ctx_desc = "")
#if defined(AP_HAVE_ARA)
      : logger_(ara::log::CreateLogger(ctx_id, ctx_desc)) {}
#else
      : ctx_(ctx_id) { (void)ctx_desc; }
#endif

  void Info(const std::string& m) {
#if defined(AP_HAVE_ARA)
    logger_.LogInfo() << m;
#else
    std::cerr << "[INFO][" << ctx_ << "] " << m << "\n";
#endif
  }
  void Warn(const std::string& m) {
#if defined(AP_HAVE_ARA)
    logger_.LogWarn() << m;
#else
    std::cerr << "[WARN][" << ctx_ << "] " << m << "\n";
#endif
  }
  void Error(const std::string& m) {
#if defined(AP_HAVE_ARA)
    logger_.LogError() << m;
#else
    std::cerr << "[ERROR][" << ctx_ << "] " << m << "\n";
#endif
  }

 private:
#if defined(AP_HAVE_ARA)
  ara::log::Logger logger_;   // ara::log::CreateLogger returns by value
#else
  std::string ctx_;
#endif
};

// ---------------------------------------------------------------------------
// ara::phm — Platform Health Management. Report a checkpoint each control cycle so
// PHM alive/deadline supervision can detect a stalled / overrunning loop.
// ---------------------------------------------------------------------------
class SupervisedEntity {
 public:
  explicit SupervisedEntity(std::uint32_t se_id)
      : id_(se_id)
#if defined(AP_HAVE_ARA)
        , spec_("av/SupervisedEntity_" + std::to_string(se_id))
#endif
  {
#if defined(AP_HAVE_ARA)
    auto r = ara::phm::SupervisedEntity::Create(spec_);
    if (r.HasValue()) {
      entity_ = std::make_unique<ara::phm::SupervisedEntity>(std::move(r).Value());
    }
#endif
  }

  void ReportCheckpoint(std::uint32_t checkpoint_id) {
#if defined(AP_HAVE_ARA)
    // ara::phm::SupervisedEntity::ReportCheckpoint requires a uint32-backed enum id.
    enum class Checkpoint : std::uint32_t {};
    if (entity_) {
      (void)entity_->ReportCheckpoint(static_cast<Checkpoint>(checkpoint_id));
    }
#else
    (void)checkpoint_id;  // host: supervision is a no-op
#endif
  }
  std::uint32_t id() const { return id_; }

 private:
  std::uint32_t id_;
#if defined(AP_HAVE_ARA)
  ara::core::InstanceSpecifier spec_;                       // must outlive entity_
  std::unique_ptr<ara::phm::SupervisedEntity> entity_;      // null if PHM unavailable
#endif
};

}  // namespace ap
}  // namespace av
#endif  // AV_AP_AP_HPP
