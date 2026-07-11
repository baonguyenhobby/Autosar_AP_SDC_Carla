// Control Adaptive Application entry point. Periodic at ~50 Hz.
#include "av_common/ap_main.hpp"
#include "av_control/control_app.hpp"

int main(int argc, char** argv) {
  av::ap::Initialize();   // init the ara runtime before the app ctor subscribes (see localization main)
  av::ControlApp app("av");
  if (av::ap::phase_from_args(argc, argv) == av::ap::Phase::kInit)
    return av::ap::run_init();   // init barrier: hold, then self-terminate
  const double dt = 0.02;
  return av::ap::run_app([&] { app.step(dt); }, std::chrono::milliseconds(20));
}
