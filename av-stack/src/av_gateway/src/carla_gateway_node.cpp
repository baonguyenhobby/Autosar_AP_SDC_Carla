// av_gateway/carla_gateway_node.cpp
// The ONE place rclcpp/lwrcl is used: the CARLA boundary. It is the ROS 2 <-> ara::com
// bridge — CARLA ros-bridge topics come in as ROS 2 DDS (via lwrcl), and are offered to
// the Adaptive Applications as ara::com SensorService events (SOME/IP). The AAs' control
// command (ara::com ControlService) is converted back to a CARLA control topic.
//
//   CARLA (ROS 2)  --lwrcl-->  SensorService   (ara::com / SOME/IP)  -->  AAs
//   AAs  -->  ControlService (ara::com)  --lwrcl-->  /carla/<role>/vehicle_control_cmd
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

#include "av_ap/ap.hpp"
#include "av_ap/av_services.hpp"

namespace {
double yawFromQuat(double x, double y, double z, double w) {
  return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}
double clampd(double v, double lo, double hi) { return v < lo ? lo : (v > hi ? hi : v); }
}  // namespace

class CarlaGateway : public rclcpp::Node {
 public:
  CarlaGateway()
      : rclcpp::Node("carla_gateway"),
        sensors_("av"),
        control_(av::services::ControlServiceProxy::FindService("av").front()) {
    role_ = declare_parameter<std::string>("role_name", "ego_vehicle");
    max_steer_ = declare_parameter<double>("max_steer_rad", 0.6);
    const std::string ns = "/carla/" + role_;

    sensors_.OfferService();  // offer SensorService to the AAs (ara::com)

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        ns + "/imu", rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::Imu::SharedPtr m) {
          const double t = rclcpp::Time(m->header().stamp()).seconds();
          const auto& q = m->orientation();
          sensors_.imu.Send({t, m->angular_velocity().z(),
                             yawFromQuat(q.x(), q.y(), q.z(), q.w())});
        });

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        ns + "/odometry", 10,
        [this](nav_msgs::msg::Odometry::SharedPtr m) {
          const double t = rclcpp::Time(m->header().stamp()).seconds();
          const auto& p = m->pose().pose().position();
          const auto& q = m->pose().pose().orientation();
          const double yaw = yawFromQuat(q.x(), q.y(), q.z(), q.w());
          sensors_.gps.Send({t, p.x(), p.y(), yaw});
          sensors_.speed.Send({t, m->twist().twist().linear().x()});
        });

    lidar_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        ns + "/lidar", rclcpp::SensorDataQoS(),
        std::bind(&CarlaGateway::onCloud, this, std::placeholders::_1));

    ctrl_pub_ = create_publisher<carla_msgs::msg::CarlaEgoVehicleControl>(
        ns + "/vehicle_control_cmd", 10);

    // ControlService (ara::com) -> CARLA control topic (lwrcl).
    control_.command.Subscribe(1);
    control_.command.SetReceiveHandler([this] {
      control_.command.GetNewSamples(
          [this](ara::com::SamplePtr<av::services::ControlSample> c) { publishControl(*c); });
    });

    exec_.ReportRunning();
    log_.Info("carla_gateway ready: SensorService offered, ControlService bridged to " + ns);
  }

  ~CarlaGateway() override { exec_.ReportTerminating(); }

 private:
  void onCloud(sensor_msgs::msg::PointCloud2::SharedPtr m) {
    // lwrcl doesn't ship sensor_msgs/point_cloud2_iterator.hpp, so read the raw buffer
    // directly using the x/y/z field offsets (float32 points).
    av::services::LidarSample scan;
    scan.stamp = rclcpp::Time(m->header().stamp()).seconds();

    int ox = -1, oy = -1, oz = -1;
    for (const auto& f : m->fields()) {
      if (f.name() == "x") ox = static_cast<int>(f.offset());
      else if (f.name() == "y") oy = static_cast<int>(f.offset());
      else if (f.name() == "z") oz = static_cast<int>(f.offset());
    }
    const std::size_t step = m->point_step();
    const auto& data = m->data();
    if (ox >= 0 && oy >= 0 && oz >= 0 && step > 0) {
      const std::size_t n = data.size() / step;
      for (std::size_t i = 0; i < n; ++i) {
        const std::uint8_t* p = data.data() + i * step;
        float x, y, z;
        std::memcpy(&x, p + ox, sizeof(float));
        std::memcpy(&y, p + oy, sizeof(float));
        std::memcpy(&z, p + oz, sizeof(float));
        scan.points.push_back({static_cast<double>(x), static_cast<double>(y),
                               static_cast<double>(z)});
      }
    }
    sensors_.lidar.Send(scan);
  }

  void publishControl(const av::ControlCommand& c) {
    carla_msgs::msg::CarlaEgoVehicleControl out;
    out.header().stamp() = now();
    out.throttle() = static_cast<float>(clampd(c.throttle, 0.0, 1.0));
    out.brake() = static_cast<float>(clampd(c.brake, 0.0, 1.0));
    out.steer() = static_cast<float>(clampd(c.steer / max_steer_, -1.0, 1.0));
    ctrl_pub_->publish(out);
  }

  std::string role_;
  double max_steer_{0.6};
  av::services::SensorServiceSkeleton sensors_;
  av::services::ControlServiceProxy control_;
  av::ap::ExecutionClient exec_;
  av::ap::Logger log_{"gateway"};
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
  rclcpp::Publisher<carla_msgs::msg::CarlaEgoVehicleControl>::SharedPtr ctrl_pub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  av::ap::Initialize();
  rclcpp::spin(std::make_shared<CarlaGateway>());
  av::ap::Deinitialize();
  rclcpp::shutdown();
  return 0;
}
