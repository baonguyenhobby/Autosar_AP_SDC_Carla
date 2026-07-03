// Planning Adaptive Application (conformant ara::com).
//   requires: VehicleStateService (state), ObjectsService (objects)  [Proxy]
//   provides: TrajectoryService (trajectory)                          [Skeleton]
#ifndef AV_PLANNING_APP_HPP
#define AV_PLANNING_APP_HPP
#include <string>
#include <vector>
#include "av_ap/ap.hpp"
#include "av_ap/av_services.hpp"
#include "av_planning/planner_core.hpp"
namespace av {
class PlanningApp {
 public:
  PlanningApp(const std::string& instance, const PlanningParams& p,
             const std::vector<Point3>& centerline)
      : core_(p),
        state_(services::VehicleStateServiceProxy::FindService(instance).front()),
        objects_(services::ObjectsServiceProxy::FindService(instance).front()),
        out_(instance), log_("planning") {
    core_.setGlobalPath(centerline);
    out_.OfferService();
    state_.state.Subscribe(1); objects_.objects.Subscribe(1);
    log_.Info("TrajectoryService offered; VehicleStateService + ObjectsService subscribed");
  }
  void step() {
    state_.state.GetNewSamples([this](ara::com::SamplePtr<services::VehicleStateSample> s) {
      ego_ = *s; have_ego_ = true;
    });
    objects_.objects.GetNewSamples(
        [this](ara::com::SamplePtr<services::ObjectsSample> o) { objects_cache_ = o->objects; });
    if (!have_ego_) return;
    Trajectory traj = core_.plan(ego_, objects_cache_);
    services::TrajectorySample t; t.stamp = ego_.stamp; t.points = traj;
    out_.trajectory.Send(t);
  }
 private:
  PlannerCore core_;
  services::VehicleStateServiceProxy state_;
  services::ObjectsServiceProxy objects_;
  services::TrajectoryServiceSkeleton out_;
  ap::Logger log_;
  VehicleState ego_{};
  DetectedObjectArray objects_cache_;
  bool have_ego_{false};
};
}  // namespace av
#endif  // AV_PLANNING_APP_HPP
