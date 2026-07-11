// Perception Adaptive Application entry point.
// Event-driven: publishes ObjectsService on each new LiDAR sample.
#include "av_common/ap_main.hpp"
#include "av_perception/perception_app.hpp"

int main(int argc, char** argv) {
  av::ap::Initialize();   // init the ara runtime before the app ctor subscribes (see localization main)
  av::PerceptionApp app("av");
  if (av::ap::phase_from_args(argc, argv) == av::ap::Phase::kInit)
    return av::ap::run_init();   // init barrier: hold, then self-terminate
  return av::ap::run_app(nullptr, std::chrono::milliseconds(10));
}
