// av_perception/perception_node.cpp
// lwrcl (rclcpp-compatible) node wrapping PerceptionCore. CycloneDDS backend;
// method-style message accessors.
//   in : sensor_msgs/PointCloud2  on input/points
//        av_msgs/VehicleState     on input/vehicle_state
//   out: av_msgs/DetectedObjectArray on output/objects
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

#include "av_msgs/msg/vehicle_state.hpp"
#include "av_msgs/msg/detected_object_array.hpp"

#include "av_perception/perception_core.hpp"

using std::placeholders::_1;

class PerceptionNode : public rclcpp::Node {
 public:
  PerceptionNode() : rclcpp::Node("perception_node") {
    av::PerceptionParams p;
    p.ground_z_max = declare_parameter("ground_z_max", p.ground_z_max);
    p.roi_range = declare_parameter("roi_range", p.roi_range);
    p.cell_size = declare_parameter("cell_size", p.cell_size);
    p.min_points = declare_parameter("min_points", p.min_points);
    core_ = std::make_unique<av::PerceptionCore>(p);

    state_sub_ = create_subscription<av_msgs::msg::VehicleState>(
        "/av/vehicle_state", 10,
        [this](av_msgs::msg::VehicleState::SharedPtr msg) { ego_ = *msg; });

    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "/av/points", rclcpp::SensorDataQoS(),
        std::bind(&PerceptionNode::onCloud, this, _1));

    pub_ = create_publisher<av_msgs::msg::DetectedObjectArray>("/av/objects", 10);
    RCLCPP_INFO(get_logger(), "perception_node ready");
  }

 private:
  void onCloud(sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    av::PointCloud cloud;
    cloud.reserve(static_cast<std::size_t>(msg->width()) * msg->height());
    sensor_msgs::PointCloud2ConstIterator<float> ix(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iy(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iz(*msg, "z");
    for (; ix != ix.end(); ++ix, ++iy, ++iz) {
      cloud.push_back({static_cast<double>(*ix), static_cast<double>(*iy),
                       static_cast<double>(*iz)});
    }

    av::Pose2D ego{ego_.x(), ego_.y(), ego_.yaw()};
    av::DetectedObjectArray objs = core_->detect(cloud, ego);

    av_msgs::msg::DetectedObjectArray out;
    out.header() = msg->header();
    out.header().frame_id() = "map";
    for (const auto& o : objs) {
      av_msgs::msg::DetectedObject d;
      d.id() = o.id;
      d.centroid().x() = o.centroid.x;
      d.centroid().y() = o.centroid.y;
      d.centroid().z() = o.centroid.z;
      d.size_x() = o.size_x;
      d.size_y() = o.size_y;
      d.size_z() = o.size_z;
      d.yaw() = o.yaw;
      d.num_points() = o.num_points;
      out.objects().push_back(d);
    }
    pub_->publish(out);
  }

  std::unique_ptr<av::PerceptionCore> core_;
  av_msgs::msg::VehicleState ego_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<av_msgs::msg::VehicleState>::SharedPtr state_sub_;
  rclcpp::Publisher<av_msgs::msg::DetectedObjectArray>::SharedPtr pub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PerceptionNode>());
  rclcpp::shutdown();
  return 0;
}
