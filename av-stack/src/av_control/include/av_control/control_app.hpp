// Control Adaptive Application (conformant ara::com).
//   requires: VehicleStateService, TrajectoryService   [Proxy]
//   provides: ControlService (command)                  [Skeleton]
// Runs ONLY while the DrivingFG function group is in state "Active" — started and
// stopped by Execution Management as State Management changes the machine mode /
// function-group state. There is no in-app operating-state gating.
#ifndef AV_CONTROL_APP_HPP
#define AV_CONTROL_APP_HPP
#include <string>
#include "av_ap/ap.hpp"
#include "av_ap/av_services.hpp"
#include "av_control/controller_core.hpp"
namespace av {
class ControlApp {
 public:
  explicit ControlApp(const std::string& instance, const ControlParams& p = {})
      : core_(p),
        state_(services::VehicleStateServiceProxy::FindService(instance).front()),
        traj_(services::TrajectoryServiceProxy::FindService(instance).front()),
        out_(instance), log_("control"), se_(1) {
    out_.OfferService();
    state_.state.Subscribe(1); traj_.trajectory.Subscribe(1);
    log_.Info("ControlService offered; VehicleState + Trajectory subscribed");
  }
  void step(double dt) {
    se_.ReportCheckpoint(10);                                   // ara::phm
    state_.state.GetNewSamples([this](ara::com::SamplePtr<services::VehicleStateSample> s) {
      ego_ = *s; have_ego_ = true;
    });
    traj_.trajectory.GetNewSamples(
        [this](ara::com::SamplePtr<services::TrajectorySample> t) { traj_cache_ = t->points; });
    if (!have_ego_) return;
    ControlCommand c = core_.computeCommand(ego_, traj_cache_, dt);
    c.stamp = ego_.stamp;
    out_.command.Send(c);
  }
 private:
  ControllerCore core_;
  services::VehicleStateServiceProxy state_;
  services::TrajectoryServiceProxy traj_;
  services::ControlServiceSkeleton out_;
  ap::Logger log_;
  ap::SupervisedEntity se_;
  VehicleState ego_{};
  Trajectory traj_cache_;
  bool have_ego_{false};
};
}  // namespace av
#endif  // AV_CONTROL_APP_HPP
