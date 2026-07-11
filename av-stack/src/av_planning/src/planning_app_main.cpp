// Planning Adaptive Application entry point. Periodic at ~10 Hz.
#include <vector>
#include "av_common/ap_main.hpp"
#include "av_planning/planning_app.hpp"

int main(int argc, char** argv) {
  av::ap::Initialize();   // init the ara runtime before the app ctor subscribes (see localization main)
  std::vector<av::Point3> lane;
  for (double x = 0.0; x <= 200.0; x += 1.0) lane.push_back({x, 0.0, 0.0});
  av::PlanningApp app("av", av::PlanningParams{}, lane);
  if (av::ap::phase_from_args(argc, argv) == av::ap::Phase::kInit)
    return av::ap::run_init();   // init barrier: hold, then self-terminate
  return av::ap::run_app([&] { app.step(); }, std::chrono::milliseconds(100));
}
