# av-stack — Build & Run Guide

Two paths:

- **Path A — Host verification** (no hardware, ~2 min): build the ROS-free cores + the
  `ara::com` SOA wiring and run the closed-loop test. This is what proves the logic.
- **Path B — Full demo** on a Jetson Orin Nano driving CARLA (Windows) via lwrcl.

> Status: Path A is verified. Path B is the correct procedure but depends on the
> lwrcl / AUTOSAR-AP toolchain + CARLA hardware; validate each step on your machines.

---

## Path A — Host verification (Linux/WSL2, no lwrcl, no ROS)

```bash
cd git_repos/av-stack
./build_ap.sh host          # cmake build + ctest  → "100% tests passed"
# or direct g++:
g++ -std=c++17 -O2 -Isrc/av_ap/include -Isrc/av_common/include \
  -Isrc/av_perception/include -Isrc/av_localization/include \
  -Isrc/av_planning/include -Isrc/av_control/include \
  src/av_integration_tests/test/test_soa_pipeline.cpp src/av_perception/src/perception_core.cpp src/av_localization/src/ekf_localizer.cpp \
  src/av_planning/src/planner_core.cpp src/av_control/src/controller_core.cpp \
  -o test_soa && ./test_soa
```
Expected:
```
final: x=96.62 m, v=0.000 m/s, detections=331
PASS test_soa_pipeline: SOA loop over ara::com (DrivingFG=Active) stopped 3.4 m before obstacle
```
The four Adaptive Applications are wired through `ara::com` (an in-process broker stands
in for the SOME/IP binding); `control_app` represents the `DrivingFG=Active` state.

---

## Path B — Jetson Orin Nano ⇄ CARLA (Windows) via lwrcl

Topology (two transports):
```
CARLA 0.9.15 (Windows) <> WSL carla-ros-bridge ──DDS (local)──▶ zenoh-bridge-ros2dds (WSL)
                                                                        │ Zenoh tcp/7447
                                                                        ▼
                                                       zenoh-bridge-ros2dds (Jetson)
                                                   │  CycloneDDS, ROS_DOMAIN_ID=0, localhost
                                                   ▼
                                          lwrcl carla_gateway (CycloneDDS)
                                                   │ ara::com / SOME/IP
                                          localization → perception → planning → control							  
```
- **Internal AA↔AA** = `ara::com` over **SOME/IP (vsomeip)**, `ARA_COM_EVENT_BINDING=vsomeip`.
- **Gateway ⇄ CARLA bridge** = **DDS** (the gateway's rclcpp/lwrcl side over CycloneDDS).

The cross-machine hop is **Zenoh only** (`tcp/7447`). On the Jetson everything is local:
CycloneDDS loopback between `zenoh-bridge-ros2dds` and the gateway, vsomeip between the
gateway and the four AAs. The CARLA side runs the **official carla-ros-bridge** (which
publishes the `/carla/ego_vehicle/*` topics and `CarlaEgoVehicleControl` type the gateway
was written against) plus a WSL-side `zenoh-bridge-ros2dds` that routes exactly those
topics over Zenoh. Do **NOT** use `zenoh_carla_bridge` from `autoware_carla_launch` here —
it publishes Autoware-style topics/types (`v1/sensing/...`, Autoware control messages)
that the gateway cannot consume.

### Network assumptions (this setup)

- Jetson ⇄ Windows over a **direct Ethernet cable**, `192.168.100.0/24`:
  Jetson `192.168.100.1`, Windows NIC `192.168.100.2` (set statically — see network setup).
- WSL2 runs in **mirrored networking** mode, so WSL shares the Windows host IPs
  (`192.168.100.2` for the cable) and CARLA is reachable from WSL at `127.0.0.1`.

### Part 1 — Windows host: CARLA
1. Launch **CARLA 0.9.15**: `CarlaUE4.exe` (add `-quality-level=Low` to spare the Orin).
2. CARLA listens on `0.0.0.0:2000`; from mirrored WSL2 it's reachable at `127.0.0.1`.
cd C:\AUTONOMOUS_DRIVING\CARLA\CARLA_0.9.15\WindowsNoEditor
.\CarlaUE4.exe -quality-level=Low -world-port=2000

### Part 2 — WSL2: carla-ros-bridge + Zenoh bridge (CARLA → ROS 2 → Zenoh)
Two processes on the WSL2 side: the official **carla-ros-bridge** turns CARLA into the
`/carla/ego_vehicle/*` ROS 2 topics (spawning the ego + sensors from the av-stack's
`objects.json`, whose sensor ids map 1:1 to the topics the gateway subscribes), and a
WSL-side **zenoh-bridge-ros2dds** routes exactly those topics onto Zenoh, listening on
`tcp/7447` for the Jetson. Copy `av-stack/config/carla/objects.json` and
`av-stack/config/zenoh-bridge-ros2dds-carla-wsl.json5` to the WSL2 machine.

Note: copy files into docker 
	cd /mnt/c/Claude_wsp/Wiki_Self-Driving-Car/git_repos/Autosar_AP_SDC_Carla/av-stack/config/carla
    docker cp ./objects.json carla-dev:/home/nguyennqb/av-stack-config/

###Raise Kernel UDP buffer limit, execute it outside the docker###
# raise kernel UDP buffer limits
sudo sysctl -w net.core.rmem_max=16777216 net.core.rmem_default=16777216
sudo sysctl -w net.core.wmem_max=16777216 net.core.wmem_default=16777216

# make it survive WSL restarts
echo -e "net.core.rmem_max=16777216\nnet.core.rmem_default=16777216\nnet.core.wmem_max=16777216\nnet.core.wmem_default=16777216" | sudo tee /etc/sysctl.d/99-dds-buffers.conf

###INSTALL carla-ros-bridge inside docker carla-dev###
# 0) prerequisites (once)
source /opt/ros/humble/setup.bash
sudo apt update
sudo apt install -y python3-rosdep python3-colcon-common-extensions \
                    ros-humble-rmw-cyclonedds-cpp
pip3 install carla==0.9.15 pygame transforms3d   # CARLA client must match the 0.9.15 server

# 1) get the source (the ROS 2 support lives on master; --recurse-submodules is required,
#    it pulls in ros-carla-msgs = the carla_msgs package your gateway's types must match)
mkdir -p ~/carla-ros-bridge && cd ~/carla-ros-bridge
git clone --recurse-submodules https://github.com/carla-simulator/ros-bridge.git src/ros-bridge

# 2) resolve ROS dependencies
sudo rosdep init 2>/dev/null || true
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# 3) build (only what the demo needs — avoids the packages with Humble quirks)
colcon build --symlink-install \
  --packages-up-to carla_ros_bridge carla_spawn_objects carla_manual_control

# 4) use it
source ~/carla-ros-bridge/install/setup.bash

cd ~/carla-ros-bridge
echo "0.9.15" > src/ros-bridge/carla_ros_bridge/src/carla_ros_bridge/CARLA_VERSION
colcon build --symlink-install   # rebuild so the installed copy updates
source install/setup.bash

cd ~/carla-ros-bridge
touch src/ros-bridge/pcl_recorder/COLCON_IGNORE
touch src/ros-bridge/rviz_carla_plugin/COLCON_IGNORE
colcon build --symlink-install
source install/setup.bash

###INSTALL zenoh-bridge-ros2dds###
echo "deb [trusted=yes] https://download.eclipse.org/zenoh/debian-repo/ /" \
  | sudo tee /etc/apt/sources.list.d/zenoh.list
sudo apt update
sudo apt install zenoh-bridge-ros2dds #/usr/bin/zenoh-bridge-ros2dds
	
```bash
# shell 1 — carla-ros-bridge (spawns ego 'ego_vehicle' + the av-stack sensors). #export ROS_LOCALHOST_ONLY=1
source /opt/ros/humble/setup.bash
source ~/carla-ros-bridge/install/setup.bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp     # match the rest of the pipeline
ros2 launch carla_ros_bridge carla_ros_bridge_with_example_ego_vehicle.launch.py \
    host:=127.0.0.1 fixed_delta_seconds:=0.1 \
    objects_definition_file:=$HOME/av-stack-config/objects.json
# fixed_delta_seconds must match the camera/lidar sensor_tick in objects.json (0.1 = 10 Hz).
# confirm the flag took
ros2 param get /carla_ros_bridge fixed_delta_seconds   # expect 0.1

# shell 2 — Zenoh bridge (listens tcp/0.0.0.0:7447; allow-list = the gateway's topics)
#export PATH="$HOME/zenoh-plugin-ros2dds/target/release:$PATH"
export ROS_DOMAIN_ID=0
zenoh-bridge-ros2dds -c $HOME/av-stack-config/zenoh-bridge-ros2dds-carla-wsl.json5
```

The ego role name is `ego_vehicle` (objects.json), which matches the gateway's
`role_name` default — there is **no** namespace prefix on the Zenoh keys in this
topology.

Confirm `/carla/ego_vehicle/{imu,odometry,lidar}` appear in `ros2 topic list` on WSL2 and
the Zenoh bridge is listening on `7447`.

**Container networking (must expose 7447 to the Jetson).** `run-bridge-docker.sh` has to
either use `--network host` or publish the port (`-p 7447:7447`) so the LAN can reach the
Zenoh listener, and Windows must allow inbound TCP 7447 (Defender **and**, under mirrored
mode, the Hyper-V firewall — see Part 3). Verify from the Jetson: `nc -vz 192.168.100.2 7447`.

### Part 3 — Networking
The only cross-machine port is **Zenoh TCP 7447**. DDS never leaves the Jetson loopback,
so no DDS peers or UDP 7400–7500 rules are needed.
1. WSL **mirrored networking** — `%UserProfile%\.wslconfig`: `[wsl2]` /
   `networkingMode=mirrored`, then `wsl --shutdown`. WSL then shares `192.168.100.2`.
2. Allow inbound **TCP 7447** on Windows Defender:
   `New-NetFirewallRule -DisplayName "Zenoh 7447" -Direction Inbound -Action Allow -Protocol TCP -LocalPort 7447`
3. Mirrored mode also has a Hyper-V firewall layer; if the Jetson still can't reach 7447:
   `Set-NetFirewallHyperVVMSetting -Name '{40E0AC32-46A5-438A-A0B2-2B479E8F2E90}' -DefaultInboundAction Allow`

### Part 4 — Jetson Orin Nano: build runtime + stack (aarch64 / JetPack)
1. **CycloneDDS** (gateway's DDS transport to CARLA):
   ```bash
   cd ~/Autosar_AP_SDC_Carla/lwrcl && ./scripts/install_cyclonedds.sh && source ~/Autosar_AP_SDC_Carla/.bashrc
   ```
2. **vsomeip** (SOME/IP runtime for the internal `ara::com`):
   ```bash
   cd ~/Autosar_AP_SDC_Carla/lwrcl && ./scripts/install_vsomeip.sh      # installs to /opt/vsomeip
   ```
3. **AUTOSAR-AP runtime + codegen** — build Adaptive-AUTOSAR and install the runtime +
   `ara_com_codegen` tools to `/opt/autosar-ap` (adds `autosar-generate-comm-manifest`,
   `autosar-generate-proxy-skeleton` to PATH):
   ```bash
   cd ~/Autosar_AP_SDC_Carla/Adaptive-AUTOSAR && cmake -DCMAKE_BUILD_TYPE=Release -S . -B build && cmake --build build
   # then install per the project's install step
   ```bash
   cd ~/Autosar_AP_SDC_Carla/Autosar_AP_SDC_Carla/Adaptive-AUTOSAR
   sudo ./scripts/build_and_install_autosar_ap.sh --prefix /opt/autosar-ap
4. **lwrcl (CycloneDDS backend — for the gateway's ROS 2 side only)**:
   ```bash
   cd ~/Autosar_AP_SDC_Carla/lwrcl
   ./build_libraries.sh  cyclonedds install
   ./build_data_types.sh cyclonedds install   # ROS 2 msg types: sensor_msgs, nav_msgs, ...
   ./build_lwrcl.sh      cyclonedds install    # -> /opt/cyclonedds-libs
   ```
   > The gateway speaks **ROS 2 DDS** to `zenoh-bridge-ros2dds`, so it needs the
   > **cyclonedds** lwrcl (ROS 2 wire-compatible). Do **not** use the `adaptive-autosar`
   > lwrcl for the gateway — its ara::com/AUTOSAR topic mapping won't interoperate with the
   > ROS 2 bridge. The four Adaptive Applications don't use lwrcl at all: they link
   > Adaptive-AUTOSAR's `ara::com` (SOME/IP) from step 3 (`AdaptiveAutosarAP::ara_*`).
5. **carla_msgs IDL** — add `carla_msgs` (+`ackermann_msgs` if used) to lwrcl's
   **cyclonedds** `data_types` and regenerate, so the gateway can (de)serialize
   `CarlaEgoVehicleControl` / `CarlaEgoVehicleStatus` (not in the stock ROS set).
6. **Build av-stack**:
   ```bash
   cd ~/Autosar_AP_SDC_Carla/av-stack && ./build_ap.sh adaptive-autosar
   ```
   Builds `localization_app perception_app planning_app control_app safe_stop_app
   carla_gateway`. The AAs use the hand-written manifests in `config/manifests/`
   (their `ara::com` layer isn't scanned by lwrcl's auto-generator).

   **Integration note (gateway transport):** the gateway is **dual-stack** — its
   CARLA-facing **rclcpp** topics ride **CycloneDDS** (set `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`,
   `ROS_DOMAIN_ID`, and `CYCLONEDDS_URI` to match the bridge), while its ara::com service
   side and the internal apps stay on **vsomeip**. These are independent stacks in one
   binary, so do **NOT** set `ARA_COM_EVENT_BINDING=dds` for the gateway — that would break
   its service link to the AAs. See `run_ap.sh` env.

### Part 5 — Run (machine mode Driving + DrivingFG Active)

Prerequisite: CARLA + the WSL2 Zenoh bridge are up (Parts 1 + 2A), and on the Jetson the
local **`zenoh-bridge-ros2dds`** is running — the Zenoh↔DDS half that connects out to the
WSL Zenoh listener and republishes the CARLA topics onto local CycloneDDS domain 0 (Zenoh
carries the LAN hop; DDS stays on loopback):

###Raise Kernel UDP buffer limit on the Jetson host too (outside the docker)###
sudo sysctl -w net.core.rmem_max=16777216 net.core.rmem_default=16777216
sudo sysctl -w net.core.wmem_max=16777216 net.core.wmem_default=16777216
echo -e "net.core.rmem_max=16777216\nnet.core.rmem_default=16777216\nnet.core.wmem_max=16777216\nnet.core.wmem_default=16777216" | sudo tee /etc/sysctl.d/99-dds-buffers.conf

nc -vz 192.168.100.2 7447  #expected successful

###################################################JETSON#############################################
###To Kill them####
   pkill -f zenoh-bridge-ros2dds; pkill -f run_ap.sh; pkill -f autosar_vsomeip_routing_manager
   pkill -f '_app$'; pkill -f carla_gateway

cd ~/autoware_carla_launch
./container/run-autoware-docker.sh

#Inside the docker autoware-dev
mkdir ~/av-stack-config
exit

###Copy files into docker
   cd ~/Autosar_AP_SDC_Carla/av-stack/config/
   docker cp ./zenoh-bridge-ros2dds-carla-jetson.json5 autoware-dev:/home/nguyennqb/av-stack-config/
   docker cp ./cyclonedds-local.xml               autoware-dev:/home/nguyennqb/av-stack-config/

docker start -ai autoware-dev
   
###Inside docker autoware-dev###
export PATH="$HOME/autoware_carla_launch/external/zenoh-plugin-ros2dds/target/release:$PATH"
export CYCLONEDDS_URI="file://$HOME/av-stack-config/cyclonedds-local.xml"
zenoh-bridge-ros2dds \
  -c ~/av-stack-config/zenoh-bridge-ros2dds-carla-jetson.json5 \
  -e tcp/192.168.100.2:7447

# other terminal:
cd ~/Autosar_AP_SDC_Carla/av-stack && ./run_ap.sh

# verify (needs the same DDS env as the gateway):
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=0
export CYCLONEDDS_URI="file:///home/nguyennqb/av-stack-config/cyclonedds-local.xml"

ros2 daemon stop        # kill the cached daemon that was started with the wrong RMW
ros2 topic list
```
- `192.168.100.2:7447` is the mirrored-WSL2 host on the direct Ethernet link (Part 3).
- **No `-n` namespace** in this topology — the WSL2 side publishes unprefixed
  `/carla/...` keys. A stray `-n /v1` breaks every route.
- The zenoh-bridge-ros2dds routes appear **on demand**: the sensor topics materialize
  once `carla_gateway` (run_ap.sh, next step) subscribes to them.

Then start the stack:

```bash
cd ~/Autosar_AP_SDC_Carla/av-stack        # or .../Autosar_AP_SDC_Carla/av-stack
./run_ap.sh
```

You do **not** need to export anything by hand — `run_ap.sh` already sets it:
`ARA_COM_EVENT_BINDING=vsomeip` + `VSOMEIP_CONFIGURATION` (internal AAs + gateway service
side), and `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`, `ROS_DOMAIN_ID=0`,
`CYCLONEDDS_URI=config/cyclonedds-local.xml` (gateway's rclcpp side → the local
`zenoh-bridge-ros2dds` on loopback). It then starts the vsomeip **routing manager**, the
gateway, and the four driving apps — the manual stand-in for **Execution Management** in
machine mode `Driving` with `DrivingFG = Active` (on a full platform, EM starts these from
the Machine/Execution Manifests). `safe_stop_app` is **not** started here — it runs only in
`DrivingFG=SafeStop`.

### Endpoint summary

| Piece | Machine | Endpoint / config |
|---|---|---|
| CARLA 0.9.15 | Windows | listens `0.0.0.0:2000` |
| `carla-ros-bridge` | WSL2 | CARLA `127.0.0.1:2000` → `/carla/ego_vehicle/*` on local DDS (domain 0) |
| `zenoh-bridge-ros2dds` | WSL2 | `zenoh-bridge-ros2dds-carla-wsl.json5` → Zenoh listen `tcp/0.0.0.0:7447` |
| `zenoh-bridge-ros2dds` | Jetson | `zenoh-bridge-ros2dds-carla-jetson.json5` + `CYCLONEDDS_URI=cyclonedds-local.xml`, `-e tcp/192.168.100.2:7447` |
| `carla_gateway` + 4 AAs | Jetson | `cyclonedds-local.xml` (loopback, domain 0) + vsomeip (`run_ap.sh`) |

Start order: CARLA → `carla-ros-bridge` (WSL2) → `zenoh-bridge-ros2dds` (WSL2) →
`zenoh-bridge-ros2dds` (Jetson) → `run_ap.sh`.

### Part 6 — Verify
- Ego vehicle drives the lane in CARLA and **stops before the obstacle**, then resumes
  (UC-2 / UC-3, AC-01 / AC-02).
- Echo the control command on the **Jetson** (`ROS_DOMAIN_ID=0`,
  `CYCLONEDDS_URI=config/cyclonedds-local.xml`):
  `ros2 topic echo /carla/ego_vehicle/vehicle_control_cmd`.
- **Safe-stop (UC-4)** is an EM/SM behavior: a PHM supervision failure makes State
  Management switch `DrivingFG Active → SafeStop`; Execution Management stops `control_app`
  and starts `safe_stop_app` (which commands brake). To exercise it without a real fault,
  request the `SafeStop` function-group state via the platform's SM tooling.

---

## Troubleshooting

| Symptom | Likely cause | Fix |
|---|---|---|
| Jetson `nc -vz 192.168.100.2 7447` fails | WSL bridge not listening, or firewall | start the WSL `zenoh-bridge-ros2dds` (listen `tcp/0.0.0.0:7447`); allow TCP 7447 (Defender + Hyper-V, Part 3) |
| `carla-ros-bridge` can't reach CARLA | wrong CARLA host / CARLA down | mirrored WSL2 → host `127.0.0.1`; confirm CARLA is running on `:2000` |
| Bridge connects but no topics on Jetson | namespace/allow-list mismatch, or nothing subscribed yet | no `-n` flag in this topology; use the `-carla-jetson.json5` config; routes appear once `carla_gateway` runs |
| Gateway topics invisible to `ros2` CLI / bridge on Jetson | CycloneDDS on `lo` without the unicast-peer config (Linux `lo` has no multicast → discovery dead) | every local participant (bridge, CLI, gateway) needs `CYCLONEDDS_URI=file://.../cyclonedds-local.xml` + `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` |
| Gateway ↔ CARLA silent, apps idle | gateway's rclcpp RMW/domain not matching the bridge | set `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`, `ROS_DOMAIN_ID`, `CYCLONEDDS_URI` (Part 4 note) |
| Internal apps don't discover each other | no vsomeip routing manager | start it (run_ap.sh) / check `VSOMEIP_CONFIGURATION` |
| Gateway can't (de)serialize control | `carla_msgs` IDL missing in lwrcl | add carla_msgs to `data_types` (Part 4.5) |
| Subscriptions never match | QoS mismatch with bridge | `SensorDataQoS` for camera/lidar; reliable for control |
| Car twitches / wrong steer | steer unit mismatch | gateway converts rad→normalized via `max_steer_rad` |
| False safe-stops | clock skew WSL2↔Jetson | common time base; widen freshness threshold |

## See also
`README.md` (architecture) and the wiki pages `ap-sdc-implementation`,
`arc42-deployment-view`, `ap-sdc-constraints-assumptions`.
