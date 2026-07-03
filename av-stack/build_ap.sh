#!/usr/bin/env bash
# Build the av-stack for the AP target or for host testing.
#   ./build_ap.sh host             # cores + host tests on the ara::com shim (no ara/ROS)
#   ./build_ap.sh adaptive-autosar # AAs on real ara::com (SOME/IP) + gateway ROS2<->ara::com
set -euo pipefail

BACKEND="${1:-host}"
BUILD_DIR="build_ap"

case "$BACKEND" in
  host)
    cmake -S . -B "$BUILD_DIR" -DAP_BACKEND=off ;;
  adaptive-autosar)
    # AAs use real ara::com -> need the Adaptive-AUTOSAR runtime (/opt/autosar-ap,
    # exporting AdaptiveAutosarAP::ara_*). The gateway also needs the lwrcl cyclonedds
    # backend + ROS message types under /opt/cyclonedds-libs and CycloneDDS under
    # /opt/cyclonedds (./build_libraries.sh cyclonedds install; ./build_data_types.sh
    #  cyclonedds install; ./build_lwrcl.sh cyclonedds install).
    export CMAKE_PREFIX_PATH="/opt/autosar-ap:/opt/autosar-ap-libs:/opt/vsomeip:/opt/cyclonedds-libs:/opt/cyclonedds:${CMAKE_PREFIX_PATH:-}"
    cmake -S . -B "$BUILD_DIR" -DAP_BACKEND=adaptive-autosar ;;
  *)
    echo "usage: $0 [host|adaptive-autosar]"; exit 1 ;;
esac

cmake --build "$BUILD_DIR" -j"$(nproc)"
echo "--- ctest ---"
ctest --test-dir "$BUILD_DIR" --output-on-failure
