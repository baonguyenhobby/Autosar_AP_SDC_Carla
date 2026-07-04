// av_gateway/carla_gateway_node.cpp
// ROS 2 / lwrcl (CycloneDDS) side of the CARLA gateway. Subscribes the CARLA ros-bridge
// topics and forwards them to the ara::com side via the plain CarlaAraBridge interface;
// publishes the ara::com control command back to CARLA. This TU includes ONLY ROS/CycloneDDS
// headers (never ara::com) — see carla_bridge.hpp for why.
#include <cmath>
#include <cstdint>
#include <cstring>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "carla_msgs/msg/carla_ego_vehicle_control.hpp"

#include "av_gateway/carla_bridge.hpp"

namespace {
double yawFromQuat(double x, double y, double z, double w) {
  return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}
double clampd(double v, double lo, double hi) { return v < lo ? lo : (v > hi ? hi : v); }
}  // namespace

class CarlaGateway : public rclcpp::Node {
 public:
  CarlaGateway() : rclcpp::Node("carla_gateway") {
    // lwrcl's declare_parameter is non-templated and returns void; read the
    // value back with get_parameter(name).as_*().
    declare_parameter("role_name", std::string("ego_vehicle"));
    role_ = get_parameter("role_name").as_string();
    declare_parameter("max_steer_rad", 0.6);
    max_steer_ = get_parameter("max_steer_rad").as_double();
    const std::string ns = "/carla/" + role_;

    ctrl_pub_ = create_publisher<carla_msgs::msg::CarlaEgoVehicleControl>(
        ns + "/vehicle_control_cmd", 10);
    bridge_.setControlHandler(
        [this](const av::ControlCommand& c) { publishControl(c); });

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        ns + "/imu", rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::Imu::SharedPtr m) {
          const auto& st = m->header.stamp;
          const double t = static_cast<double>(st.sec) + static_cast<double>(st.nanosec) * 1e-9;
          const auto& q = m->orientation;
          bridge_.pushImu(t, m->angular_velocity.z,
                          yawFromQuat(q.x, q.y, q.z, q.w));
        });

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        ns + "/odometry", 10,
        [this](nav_msgs::msg::Odometry::SharedPtr m) {
          const auto& st = m->header.stamp;
          const double t = static_cast<double>(st.sec) + static_cast<double>(st.nanosec) * 1e-9;
          const auto& p = m->pose.pose.position;
          const auto& q = m->pose.pose.orientation;
          const double yaw = yawFromQuat(q.x, q.y, q.z, q.w);
          bridge_.pushGps(t, p.x, p.y, yaw);
          bridge_.pushSpeed(t, m->twist.twist.linear.x);
        });

    lidar_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        ns + "/lidar", rclcpp::SensorDataQoS(),
        std::bind(&CarlaGateway::onCloud, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "carla_gateway (ROS side) bridging %s <-> ara::com", ns.c_str());
  }

 private:
  void onCloud(sensor_msgs::msg::PointCloud2::SharedPtr m) {
    const auto& st = m->header.stamp;
    const double stamp = static_cast<double>(st.sec) + static_cast<double>(st.nanosec) * 1e-9;
    int ox = -1, oy = -1, oz = -1;
    for (const auto& f : m->fields) {
      if (f.name == "x") ox = static_cast<int>(f.offset);
      else if (f.name == "y") oy = static_cast<int>(f.offset);
      else if (f.name == "z") oz = static_cast<int>(f.offset);
    }
    av::PointCloud cloud;
    const std::size_t step = m->point_step;
    const auto& data = m->data;
    if (ox >= 0 && oy >= 0 && oz >= 0 && step > 0) {
      const std::size_t n = data.size() / step;
      for (std::size_t i = 0; i < n; ++i) {
        const std::uint8_t* p = data.data() + i * step;
        float x, y, z;
        std::memcpy(&x, p + ox, sizeof(float));
        std::memcpy(&y, p + oy, sizeof(float));
        std::memcpy(&z, p + oz, sizeof(float));
        cloud.push_back({static_cast<double>(x), static_cast<double>(y),
                         static_cast<double>(z)});
      }
    }
    bridge_.pushLidar(stamp, cloud);
  }

  void publishControl(const av::ControlCommand& c) {
    carla_msgs::msg::CarlaEgoVehicleControl out;
    const auto tnow = now();
    out.header.stamp.sec = static_cast<int32_t>(tnow.nanoseconds() / 1000000000LL);
    out.header.stamp.nanosec = static_cast<uint32_t>(tnow.nanoseconds() % 1000000000LL);
    out.throttle = static_cast<float>(clampd(c.throttle, 0.0, 1.0));
    out.brake = static_cast<float>(clampd(c.brake, 0.0, 1.0));
    out.steer = static_cast<float>(clampd(c.steer / max_steer_, -1.0, 1.0));
    ctrl_pub_->publish(out);
  }

  std::string role_;
  double max_steer_{0.6};
  av::CarlaAraBridge bridge_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
  rclcpp::Publisher<carla_msgs::msg::CarlaEgoVehicleControl>::SharedPtr ctrl_pub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CarlaGateway>());
  rclcpp::shutdown();
  return 0;
}
