// av_localization/localization_node.cpp
// lwrcl (rclcpp-compatible) node wrapping EkfLocalizer. CycloneDDS backend;
// method-style message accessors.
//   in : sensor_msgs/Imu           on input/imu
//        geometry_msgs/PoseStamped on input/gps
//        std_msgs/Float64          on input/wheel_speed
//   out: av_msgs/VehicleState      on output/vehicle_state
//        nav_msgs/Odometry         on output/odometry
#include <cmath>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/float64.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "av_msgs/msg/vehicle_state.hpp"
#include "av_localization/ekf_localizer.hpp"

using std::placeholders::_1;

namespace {
double yawFromQuat(double x, double y, double z, double w) {
  return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}
}  // namespace

class LocalizationNode : public rclcpp::Node {
 public:
  LocalizationNode() : rclcpp::Node("localization_node") {
    av::EkfParams p;
    p.r_gps = declare_parameter("r_gps", p.r_gps);
    p.r_speed = declare_parameter("r_speed", p.r_speed);
    p.r_yaw = declare_parameter("r_yaw", p.r_yaw);
    ekf_ = std::make_unique<av::EkfLocalizer>(p);
    ekf_->initialize(0.0, 0.0, 0.0, 0.0, now().seconds());

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        "/av/imu", 50, std::bind(&LocalizationNode::onImu, this, _1));
    gps_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "/av/gps", 10, std::bind(&LocalizationNode::onGps, this, _1));
    spd_sub_ = create_subscription<std_msgs::msg::Float64>(
        "/av/wheel_speed", 10,
        [this](std_msgs::msg::Float64::SharedPtr m) { ekf_->updateSpeed(m->data()); });

    state_pub_ = create_publisher<av_msgs::msg::VehicleState>("/av/vehicle_state", 10);
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/av/odometry", 10);
    RCLCPP_INFO(get_logger(), "localization_node ready");
  }

 private:
  void onImu(sensor_msgs::msg::Imu::SharedPtr msg) {
    const double t = rclcpp::Time(msg->header().stamp()).seconds();
    if (last_imu_t_ > 0.0) {
      const double dt = t - last_imu_t_;
      ekf_->predict(dt, msg->angular_velocity().z());
    }
    last_imu_t_ = t;
    ekf_->setStamp(t);
    publish();
  }

  void onGps(geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    av::GpsMeasurement g;
    g.stamp = rclcpp::Time(msg->header().stamp()).seconds();
    g.x = msg->pose().position().x();
    g.y = msg->pose().position().y();
    ekf_->updateGps(g);
    const auto& q = msg->pose().orientation();
    ekf_->updateYaw(yawFromQuat(q.x(), q.y(), q.z(), q.w()));
  }

  void publish() {
    const av::VehicleState s = ekf_->state();
    av_msgs::msg::VehicleState vs;
    vs.header().stamp() = now();
    vs.header().frame_id() = "map";
    vs.x() = s.x; vs.y() = s.y; vs.yaw() = s.yaw; vs.v() = s.v;
    state_pub_->publish(vs);

    nav_msgs::msg::Odometry odom;
    odom.header() = vs.header();
    odom.child_frame_id() = "base_link";
    odom.pose().pose().position().x() = s.x;
    odom.pose().pose().position().y() = s.y;
    odom.pose().pose().orientation().z() = std::sin(s.yaw * 0.5);
    odom.pose().pose().orientation().w() = std::cos(s.yaw * 0.5);
    odom.twist().twist().linear().x() = s.v;
    odom_pub_->publish(odom);
  }

  std::unique_ptr<av::EkfLocalizer> ekf_;
  double last_imu_t_{-1.0};
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr gps_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr spd_sub_;
  rclcpp::Publisher<av_msgs::msg::VehicleState>::SharedPtr state_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LocalizationNode>());
  rclcpp::shutdown();
  return 0;
}
