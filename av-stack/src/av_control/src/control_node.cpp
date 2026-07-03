// av_control/control_node.cpp
// lwrcl (rclcpp-compatible) node wrapping ControllerCore. Built against the lwrcl
// CycloneDDS backend; uses method-style message accessors (m->x(), out.x() = ...).
//   in : av_msgs/VehicleState  on input/vehicle_state
//        av_msgs/Trajectory    on input/trajectory
//   out: av_msgs/ControlCommand on output/command  (fixed-rate)
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "av_msgs/msg/vehicle_state.hpp"
#include "av_msgs/msg/trajectory.hpp"
#include "av_msgs/msg/control_command.hpp"

#include "av_control/controller_core.hpp"

using namespace std::chrono_literals;

class ControlNode : public rclcpp::Node {
 public:
  ControlNode() : rclcpp::Node("control_node") {
    av::ControlParams p;
    p.wheelbase = declare_parameter("wheelbase", p.wheelbase);
    p.lookahead_gain = declare_parameter("lookahead_gain", p.lookahead_gain);
    p.min_lookahead = declare_parameter("min_lookahead", p.min_lookahead);
    p.kp = declare_parameter("kp", p.kp);
    p.ki = declare_parameter("ki", p.ki);
    p.kd = declare_parameter("kd", p.kd);
    core_ = std::make_unique<av::ControllerCore>(p);

    state_sub_ = create_subscription<av_msgs::msg::VehicleState>(
        "/av/vehicle_state", 10,
        [this](av_msgs::msg::VehicleState::SharedPtr m) { state_ = *m; have_state_ = true; });
    traj_sub_ = create_subscription<av_msgs::msg::Trajectory>(
        "/av/trajectory", 10,
        [this](av_msgs::msg::Trajectory::SharedPtr m) { traj_ = *m; });

    pub_ = create_publisher<av_msgs::msg::ControlCommand>("/av/command", 10);
    const double rate_hz = declare_parameter("rate_hz", 50.0);
    period_ = 1.0 / rate_hz;
    timer_ = create_wall_timer(
        std::chrono::duration<double>(period_), std::bind(&ControlNode::onTimer, this));
    RCLCPP_INFO(get_logger(), "control_node ready");
  }

 private:
  void onTimer() {
    if (!have_state_) return;
    av::VehicleState ego;
    ego.stamp = now().seconds();
    ego.x = state_.x(); ego.y = state_.y(); ego.yaw = state_.yaw(); ego.v = state_.v();

    av::Trajectory traj;
    for (const auto& p : traj_.points())
      traj.push_back({p.x(), p.y(), p.yaw(), p.v()});

    av::ControlCommand c = core_->computeCommand(ego, traj, period_);
    av_msgs::msg::ControlCommand out;
    out.header().stamp() = now();
    out.header().frame_id() = "base_link";
    out.throttle() = c.throttle; out.brake() = c.brake; out.steer() = c.steer;
    pub_->publish(out);
  }

  std::unique_ptr<av::ControllerCore> core_;
  av_msgs::msg::VehicleState state_;
  av_msgs::msg::Trajectory traj_;
  bool have_state_{false};
  double period_{0.02};
  rclcpp::Subscription<av_msgs::msg::VehicleState>::SharedPtr state_sub_;
  rclcpp::Subscription<av_msgs::msg::Trajectory>::SharedPtr traj_sub_;
  rclcpp::Publisher<av_msgs::msg::ControlCommand>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
