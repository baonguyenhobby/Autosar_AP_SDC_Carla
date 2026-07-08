#!/usr/bin/env python3
"""Keeps zenoh-bridge-ros2dds subscriber routes active for the lwrcl carla_gateway.

Why this exists (open issue O3 in TEST_REPORT_2026-07-07.md):
zenoh-bridge-ros2dds only activates a subscriber route when it can attribute a
local DDS reader to a ROS 2 node, which it learns from the `ros_discovery_info`
topic that every real RMW publishes. lwrcl is a lightweight rclcpp on raw
CycloneDDS and does NOT publish `ros_discovery_info`, so the bridge sees the
gateway's readers but leaves the routes inactive (`is_active: false,
local_nodes: []`) and never writes any data.

This node is a real-RMW stand-in: it subscribes to the same topics (and drops
every message). The bridge activates the routes for it, and since DDS delivers
to ALL matched readers on a topic, the lwrcl gateway receives the data too.

Run it inside the autoware-dev container, in the same DDS environment as the
bridge (see BUILD_AND_RUN.md Part 5):

    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_DOMAIN_ID=0
    export CYCLONEDDS_URI="file:///home/nguyennqb/av-stack-config/cyclonedds-local.xml"
    python3 route_keeper.py &

Proper fixes tracked in O3: patch lwrcl to publish ros_discovery_info, or
switch to zenoh-bridge-dds (raw DDS bridging, no graph attribution needed).
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, PointCloud2
from nav_msgs.msg import Odometry

rclpy.init()
n = Node("av_stack_route_keeper")
noop = lambda m: None
subs = [
    n.create_subscription(Imu, "/carla/ego_vehicle/imu", noop, qos_profile_sensor_data),
    n.create_subscription(Odometry, "/carla/ego_vehicle/odometry", noop, qos_profile_sensor_data),
    n.create_subscription(PointCloud2, "/carla/ego_vehicle/lidar", noop, qos_profile_sensor_data),
]
n.get_logger().info("route keeper up: imu / odometry / lidar")
rclpy.spin(n)
