// Localization Adaptive Application (conformant ara::com).
//   requires: SensorService (imu, gps, speed)   [Proxy]
//   provides: VehicleStateService (state)        [Skeleton]
#ifndef AV_LOCALIZATION_APP_HPP
#define AV_LOCALIZATION_APP_HPP
#include <string>
#include "av_ap/ap.hpp"
#include "av_ap/av_services.hpp"
#include "av_localization/ekf_localizer.hpp"
namespace av {
class LocalizationApp {
 public:
  explicit LocalizationApp(const std::string& instance, const EkfParams& p = {})
      : ekf_(p),
        sensors_(services::SensorServiceProxy::FindService(instance).front()),
        out_(instance), log_("localization") {
    ekf_.initialize(0.0, 0.0, 0.0, 0.0, 0.0);
    out_.OfferService();
    sensors_.imu.Subscribe(1); sensors_.gps.Subscribe(1); sensors_.speed.Subscribe(1);
    sensors_.imu.SetReceiveHandler([this] { onImu(); });
    sensors_.gps.SetReceiveHandler([this] { onGps(); });
    sensors_.speed.SetReceiveHandler([this] { onSpeed(); });
    log_.Info("VehicleStateService offered; SensorService subscribed");
  }
  const VehicleState& lastState() const { return last_; }
 private:
  void onImu() {
    sensors_.imu.GetNewSamples([this](ara::com::SamplePtr<services::ImuSample> s) {
      if (last_imu_t_ > 0.0) ekf_.predict(s->stamp - last_imu_t_, s->yaw_rate);
      last_imu_t_ = s->stamp; ekf_.setStamp(s->stamp);
      last_ = ekf_.state(); out_.state.Send(last_);
    });
  }
  void onGps() {
    sensors_.gps.GetNewSamples([this](ara::com::SamplePtr<services::GpsSample> s) {
      ekf_.updateGps({s->stamp, s->x, s->y}); ekf_.updateYaw(s->yaw);
    });
  }
  void onSpeed() {
    sensors_.speed.GetNewSamples(
        [this](ara::com::SamplePtr<services::SpeedSample> s) { ekf_.updateSpeed(s->v); });
  }
  EkfLocalizer ekf_;
  services::SensorServiceProxy sensors_;
  services::VehicleStateServiceSkeleton out_;
  ap::Logger log_;
  VehicleState last_{};
  double last_imu_t_{-1.0};
};
}  // namespace av
#endif  // AV_LOCALIZATION_APP_HPP
