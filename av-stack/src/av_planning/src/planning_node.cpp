// av_planning/planning_node.cpp
// lwrcl (rclcpp-compatible) node wrapping PlannerCore. CycloneDDS backend;
// method-style message accessors.
//   in : av_msgs/VehicleState        on input/vehicle_state
//        av_msgs/DetectedObjectArray on input/objects
//   out: av_msgs/Trajectory          on output/trajectory
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "av_msgs/msg/vehicle_state.hpp"
#include "av_msgs/msg/detected_object_array.hpp"
#include "av_msgs/msg/trajectory.hpp"

#include "av_planning/planner_core.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class PlanningNode : public rclcpp::Node {
 public:
  PlanningNode() : rclcpp::Node("planning_node") {
    av::PlanningParams p;
    p.cruise_speed = declare_parameter("cruise_speed", p.cruise_speed);
    p.horizon_points = declare_parameter("horizon_points", p.horizon_points);
    p.stop_margin = declare_parameter("stop_margin", p.stop_margin);
    p.lane_half_width = declare_parameter("lane_half_width", p.lane_half_width);
    const double lane_len = declare_parameter("lane_length", 200.0);
    const double spacing = declare_parameter("lane_spacing", 1.0);
    core_ = std::make_unique<av::PlannerCore>(p);

    // Default global reference path: a straight lane along +x.
    std::vector<av::Point3> centerline;
    for (double x = 0.0; x <= lane_len; x += spacing) centerline.push_back({x, 0.0, 0.0});
    core_->setGlobalPath(centerline);

    state_sub_ = create_subscription<av_msgs::msg::VehicleState>(
        "/av/vehicle_state", 10,
        [this](av_msgs::msg::VehicleState::SharedPtr m) { state_ = *m; have_state_ = true; });
    obj_sub_ = create_subscription<av_msgs::msg::DetectedObjectArray>(
        "/av/objects", 10,
        [this](av_msgs::msg::DetectedObjectArray::SharedPtr m) { objects_ = *m; });

    pub_ = create_publisher<av_msgs::msg::Trajectory>("/av/trajectory", 10);
    timer_ = create_wall_timer(100ms, std::bind(&PlanningNode::onTimer, this));
    RCLCPP_INFO(get_logger(), "planning_node ready");
  }

 private:
  void onTimer() {
    if (!have_state_) return;
    av::VehicleState ego;
    ego.x = state_.x(); ego.y = state_.y(); ego.yaw = state_.yaw(); ego.v = state_.v();

    av::DetectedObjectArray objs;
    for (const auto& d : objects_.objects()) {
      av::DetectedObject o;
      o.id = d.id();
      o.centroid = {d.centroid().x(), d.centroid().y(), d.centroid().z()};
      o.size_x = d.size_x(); o.size_y = d.size_y(); o.size_z = d.size_z();
      o.yaw = d.yaw(); o.num_points = d.num_points();
      objs.push_back(o);
    }

    av::Trajectory traj = core_->plan(ego, objs);
    av_msgs::msg::Trajectory out;
    out.header().stamp() = now();
    out.header().frame_id() = "map";
    for (const auto& tp : traj) {
      av_msgs::msg::TrajectoryPoint p;
      p.x() = tp.x; p.y() = tp.y; p.yaw() = tp.yaw; p.v() = tp.v;
      out.points().push_back(p);
    }
    pub_->publish(out);
  }

  std::unique_ptr<av::PlannerCore> core_;
  av_msgs::msg::VehicleState state_;
  av_msgs::msg::DetectedObjectArray objects_;
  bool have_state_{false};
  rclcpp::Subscription<av_msgs::msg::VehicleState>::SharedPtr state_sub_;
  rclcpp::Subscription<av_msgs::msg::DetectedObjectArray>::SharedPtr obj_sub_;
  rclcpp::Publisher<av_msgs::msg::Trajectory>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlanningNode>());
  rclcpp::shutdown();
  return 0;
}
