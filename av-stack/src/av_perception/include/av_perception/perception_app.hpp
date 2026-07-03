// Perception Adaptive Application (conformant ara::com).
//   requires: SensorService (lidar), VehicleStateService (ego pose)  [Proxy]
//   provides: ObjectsService (objects)                                [Skeleton]
#ifndef AV_PERCEPTION_APP_HPP
#define AV_PERCEPTION_APP_HPP
#include <string>
#include "av_ap/ap.hpp"
#include "av_ap/av_services.hpp"
#include "av_perception/perception_core.hpp"
namespace av {
class PerceptionApp {
 public:
  explicit PerceptionApp(const std::string& instance, const PerceptionParams& p = {})
      : core_(p),
        sensors_(services::SensorServiceProxy::FindService(instance).front()),
        state_(services::VehicleStateServiceProxy::FindService(instance).front()),
        out_(instance), log_("perception") {
    out_.OfferService();
    state_.state.Subscribe(1); sensors_.lidar.Subscribe(1);
    state_.state.SetReceiveHandler([this] {
      state_.state.GetNewSamples(
          [this](ara::com::SamplePtr<services::VehicleStateSample> s) { ego_ = *s; });
    });
    sensors_.lidar.SetReceiveHandler([this] { onLidar(); });
    log_.Info("ObjectsService offered; SensorService.lidar + VehicleStateService subscribed");
  }
  int lastCount() const { return last_count_; }
 private:
  void onLidar() {
    sensors_.lidar.GetNewSamples([this](ara::com::SamplePtr<services::LidarSample> s) {
      Pose2D ego{ego_.x, ego_.y, ego_.yaw};
      DetectedObjectArray objs = core_.detect(s->points, ego);
      last_count_ = static_cast<int>(objs.size());
      services::ObjectsSample o; o.stamp = s->stamp; o.objects = objs;
      out_.objects.Send(o);
    });
  }
  PerceptionCore core_;
  services::SensorServiceProxy sensors_;
  services::VehicleStateServiceProxy state_;
  services::ObjectsServiceSkeleton out_;
  ap::Logger log_;
  VehicleState ego_{};
  int last_count_{0};
};
}  // namespace av
#endif  // AV_PERCEPTION_APP_HPP
