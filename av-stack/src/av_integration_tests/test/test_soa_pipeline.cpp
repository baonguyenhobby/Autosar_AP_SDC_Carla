// test_soa_pipeline.cpp — end-to-end test of the SOA pipeline over the ara::com API.
//
// The four driving Adaptive Applications (Localization, Perception, Planning, Control)
// are wired ONLY through ara::com services (host in-process binding stands in for
// SOME/IP). A synthetic gateway/sim offers SensorService and consumes ControlService.
// Control here represents the DrivingFG = "Active" state (on the target, EM starts it
// only in that state per the Execution Manifest; there is no in-app state gating).
// Verifies the loop cruises, detects the obstacle, and stops before it.
#include <cassert>
#include <cmath>
#include <cstdio>
#include <vector>

#include "av_ap/av_services.hpp"
#include "av_common/math.hpp"
#include "av_control/control_app.hpp"
#include "av_localization/localization_app.hpp"
#include "av_perception/perception_app.hpp"
#include "av_planning/planning_app.hpp"

using namespace av;
namespace svc = av::services;

int main() {
  const std::string kInst = "av";
  svc::SensorServiceSkeleton sensors(kInst);
  sensors.OfferService();

  LocalizationApp localization(kInst, EkfParams{});
  PerceptionApp   perception(kInst, PerceptionParams{});
  PlanningApp     planning(kInst, PlanningParams{}, [] {
    std::vector<Point3> lane;
    for (double x = 0.0; x <= 200.0; x += 1.0) lane.push_back({x, 0.0, 0.0});
    return lane;
  }());
  ControlApp      control(kInst, ControlParams{});   // DrivingFG = Active
  localization.Start();   // RUNNING: subscribe SensorService (deferred from INIT)

  svc::ControlServiceProxy actuator(svc::ControlServiceProxy::FindService(kInst).front());
  ControlCommand cmd{};
  actuator.command.Subscribe();
  actuator.command.SetReceiveHandler([&] {
    actuator.command.GetNewSamples([&](ara::com::SamplePtr<ControlCommand> c) { cmd = *c; });
  });

  const double obstacle_x = 100.0, obstacle_y = 0.0;
  const double dt = 0.02, wheelbase = 2.7, max_accel = 2.0, max_decel = 4.0;
  double x = 0, y = 0, yaw = 0, v = 0, t = 0;
  bool nan_seen = false;
  int detections = 0;

  for (int k = 0; k < 2000; ++k) {
    const double accel = cmd.throttle * max_accel - cmd.brake * max_decel;
    v = std::max(0.0, v + accel * dt);
    const double yaw_rate = (v / wheelbase) * std::tan(cmd.steer);
    yaw = av::normalizeAngle(yaw + yaw_rate * dt);
    x += v * std::cos(yaw) * dt;
    y += v * std::sin(yaw) * dt;
    t += dt;
    if (std::isnan(x) || std::isnan(v)) nan_seen = true;

    sensors.imu.Send({t, yaw_rate, yaw});
    sensors.speed.Send({t, v});
    if (k % 25 == 0) sensors.gps.Send({t, x, y, yaw});
    if (k % 5 == 0) {
      svc::LidarSample scan; scan.stamp = t;
      const double c = std::cos(-yaw), s = std::sin(-yaw);
      for (double ox = -1.0; ox <= 1.0; ox += 0.25)
        for (double oy = -1.0; oy <= 1.0; oy += 0.25)
          for (double oz = -1.0; oz <= 0.5; oz += 0.5) {
            const double rx = (obstacle_x + ox) - x, ry = (obstacle_y + oy) - y;
            scan.points.push_back({rx * c - ry * s, rx * s + ry * c, oz});
          }
      sensors.lidar.Send(scan);
      detections += (perception.lastCount() > 0) ? 1 : 0;
    }
    if (k % 5 == 0) planning.step();
    control.step(dt);
  }

  std::printf("final: x=%.2f m, v=%.3f m/s, detections=%d\n", x, v, detections);
  assert(!nan_seen);
  assert(detections > 0);
  assert(x > 40.0 && x < obstacle_x);
  assert(v < 0.5);
  assert(obstacle_x - x >= 2.0);
  std::printf("PASS test_soa_pipeline: SOA loop over ara::com (DrivingFG=Active) stopped %.1f m "
              "before obstacle\n", obstacle_x - x);
  return 0;
}
