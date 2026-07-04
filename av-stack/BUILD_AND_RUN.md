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
CARLA 0.9.14 (Windows) <> WSL zenoh_carla_bridge ──Zenoh tcp/7447──▶ zenoh-bridge-ros2dds (Jetson)
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
gateway and the four AAs. Two bridge processes, one per machine, each doing one
translation: `zenoh_carla_bridge` (CARLA→Zenoh, WSL2) and `zenoh-bridge-ros2dds`
(Zenoh→DDS, Jetson).

### Network assumptions (this setup)

- Jetson ⇄ Windows over a **direct Ethernet cable**, `192.168.100.0/24`:
  Jetson `192.168.100.1`, Windows NIC `192.168.100.2` (set statically — see network setup).
- WSL2 runs in **mirrored networking** mode, so WSL shares the Windows host IPs
  (`192.168.100.2` for the cable) and CARLA is reachable from WSL at `127.0.0.1`.

### Part 1 — Windows host: CARLA
1. Launch **CARLA 0.9.14**: `CarlaUE4.exe` (add `-quality-level=Low` to spare the Orin).
2. CARLA listens on `0.0.0.0:2000`; from mirrored WSL2 it's reachable at `127.0.0.1`.

### Part 2 — WSL2: Zenoh bridge (CARLA → Zenoh)
Runs `zenoh_carla_bridge` from `autoware_carla_launch` (humble branch) **inside its Docker
container** (ROS Humble + the bridge live in the image; WSL2 is only the Docker host).

Start the container, then run the bridge **inside** it:
```bash
# WSL2 host: start/enter the bridge container
cd ~/autoware_carla_launch
./container/run-bridge-docker.sh

# inside the container:
cd ~/autoware_carla_launch
source env.sh                              # sets CARLA_SIMULATOR_IP=127.0.0.1, VEHICLE_NAME=v1, ROS_DOMAIN_ID=0
export CARLA_SIMULATOR_IP=127.0.0.1
./script/bridge_ros2dds/run-bridge.sh      # starts zenoh_carla_bridge (listen tcp/0.0.0.0:7447) + carla_agent (spawns ego)
```
`run-bridge.sh` runs two things in parallel: the `zenoh_carla_bridge` **and** the Python
`carla_agent` that spawns the ego + sensors — so no separate ego-spawn step is needed.

> **Namespace:** `env.sh` sets `VEHICLE_NAME=v1`, so the CARLA topics arrive over Zenoh
> under the **`v1`** namespace and the ego rolename is `v1`. The gateway's `role_name`
> param must match — set `role_name:=v1` (or change its default from `ego_vehicle`), or
> override `VEHICLE_NAME` before launching. See Part 5.

Confirm the container logs a CARLA connection and is listening on `7447`.

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

```bash
export ROS_DOMAIN_ID=0
zenoh-bridge-ros2dds -d 0 -e tcp/192.168.100.2:7447   # 192.168.100.2 = WSL2 host over the direct cable
ros2 topic list | grep /carla/ego_vehicle    # imu / odometry / lidar should appear
ros2 topic list                                       # confirm /carla/ego_vehicle/{imu,odometry,lidar} appear
```
- `192.168.100.2:7447` is the mirrored-WSL2 host on the direct Ethernet link (Part 3).
- If topics don't appear or don't match, reconcile the bridge **namespace** (`-n`) with the
  names `carla_gateway` subscribes (`/carla/ego_vehicle/...`); a stray `/v1` prefix is the
  usual culprit.

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
| CARLA 0.9.14 | Windows | listens `0.0.0.0:2000` |
| `zenoh_carla_bridge` | WSL2 (Docker) | CARLA `127.0.0.1:2000` → Zenoh listen `tcp/0.0.0.0:7447` |
| `zenoh-bridge-ros2dds` | Jetson | `-d 0 -e tcp/192.168.100.2:7447` → local CycloneDDS |
| `carla_gateway` + 4 AAs | Jetson | `cyclonedds-local.xml` (loopback, domain 0) + vsomeip (`run_ap.sh`) |

Start order: CARLA → `zenoh_carla_bridge` (WSL2) → `zenoh-bridge-ros2dds` (Jetson) → `run_ap.sh`.

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
| Jetson `nc -vz 192.168.100.2 7447` fails | container not exposing 7447, or firewall | `--network host`/`-p 7447:7447` in `run-bridge-docker.sh`; allow TCP 7447 (Defender + Hyper-V, Part 3) |
| `zenoh_carla_bridge` can't reach CARLA | wrong CARLA host / CARLA down | mirrored WSL2 → host `127.0.0.1`; confirm CARLA is running on `:2000` |
| Bridge connects but no topics on Jetson | Zenoh namespace mismatch | match `zenoh-bridge-ros2dds -n` to the gateway's `/carla/ego_vehicle/...` names |
| Gateway ↔ CARLA silent, apps idle | gateway's rclcpp RMW/domain not matching the bridge | set `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`, `ROS_DOMAIN_ID`, `CYCLONEDDS_URI` (Part 4 note) |
| Internal apps don't discover each other | no vsomeip routing manager | start it (run_ap.sh) / check `VSOMEIP_CONFIGURATION` |
| Gateway can't (de)serialize control | `carla_msgs` IDL missing in lwrcl | add carla_msgs to `data_types` (Part 4.5) |
| Subscriptions never match | QoS mismatch with bridge | `SensorDataQoS` for camera/lidar; reliable for control |
| Car twitches / wrong steer | steer unit mismatch | gateway converts rad→normalized via `max_steer_rad` |
| False safe-stops | clock skew WSL2↔Jetson | common time base; widen freshness threshold |

## See also
`README.md` (architecture) and the wiki pages `ap-sdc-implementation`,
`arc42-deployment-view`, `ap-sdc-constraints-assumptions`.
