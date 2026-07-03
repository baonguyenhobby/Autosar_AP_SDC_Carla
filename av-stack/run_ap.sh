#!/usr/bin/env bash
# Launch the av-stack on the Jetson and bridge to CARLA.
# Prereqs: ./build_ap.sh adaptive-autosar ; CARLA (Windows) + autoware_carla_launch
# (zenoh_carla_bridge on WSL2, and zenoh-bridge-ros2dds running on THIS Jetson, domain 0).
#
# Transports:
#   * internal AA<->AA and gateway<->AA  = ara::com over SOME/IP (vsomeip).
#   * gateway<->CARLA (rclcpp/lwrcl side) = CycloneDDS on ROS_DOMAIN_ID=0 (loopback), which
#     zenoh-bridge-ros2dds republishes to/from the WSL zenoh bridge.
#
# NOTE: this script is a manual stand-in for Execution Management (which, on a full AP
# platform, starts these processes from the Machine/Execution manifests when the AvStack
# function group enters the Driving state). safe_stop_app is NOT started here — it belongs
# to the DrivingFG=SafeStop state.
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
BIN="${HERE}/build_ap"

# --- ara::com internal binding = SOME/IP (vsomeip) ---
export ARA_COM_EVENT_BINDING="${ARA_COM_EVENT_BINDING:-vsomeip}"
export VSOMEIP_CONFIGURATION="${VSOMEIP_CONFIGURATION:-${HERE}/config/vsomeip-av.json}"

# --- gateway's rclcpp/CARLA side = CycloneDDS, domain 0, loopback (see cyclonedds-local.xml) ---
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export CYCLONEDDS_URI="file://${HERE}/config/cyclonedds-local.xml"
export LD_LIBRARY_PATH="/opt/vsomeip/lib:/opt/autosar-ap/lib:/opt/autosar-ap-libs/lib:/opt/cyclonedds-libs/lib:/opt/cyclonedds/lib:${LD_LIBRARY_PATH:-}"

# vsomeip routing manager (SOME/IP) for the local ara::com services.
if [ -x /opt/autosar-ap/bin/autosar_vsomeip_routing_manager ]; then
  /opt/autosar-ap/bin/autosar_vsomeip_routing_manager & RM=$!; sleep 1
fi

# Manual bring-up: gateway first (offers SensorService), then the four AAs.
"${BIN}/carla_gateway"    & G=$!;  sleep 0.5
"${BIN}/localization_app" & L=$!;  sleep 0.2
"${BIN}/perception_app"   & P=$!;  sleep 0.2
"${BIN}/planning_app"     & N=$!;  sleep 0.2
"${BIN}/control_app"      & C=$!

trap 'kill $G $L $P $N $C ${RM:-} 2>/dev/null || true' INT TERM
wait
