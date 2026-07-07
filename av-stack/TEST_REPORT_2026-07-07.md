# av-stack Integration Test Report — CARLA ⇄ Jetson demo (Path B)

**Date:** 2026-07-06 … 2026-07-07 (integration session)
**Scope:** First end-to-end bring-up of the av-stack Adaptive AUTOSAR demo:
CARLA 0.9.15 (Windows) → carla-ros-bridge (WSL2) → Zenoh → Jetson Orin Nano
(zenoh-bridge-ros2dds → lwrcl `carla_gateway` → 4 Adaptive Applications over ara::com/SOME/IP).

## Environment

| Item | Version / detail |
|---|---|
| Jetson Orin Nano | JetPack 6.2.1, kernel 5.15.148-tegra, Ubuntu 22.04 |
| SOME/IP runtime | vsomeip 3.4.10 (`/opt/vsomeip`) |
| DDS runtime | CycloneDDS 0.10.5 (`/opt/cyclonedds`), lwrcl cyclonedds backend (`/opt/cyclonedds-libs`) |
| Bridges | zenoh-bridge-ros2dds v1.7.2 (both machines), Zenoh over TCP 7447 |
| CARLA | 0.9.15 (Windows host), carla-ros-bridge master (WSL2, ROS 2 Humble, `carla==0.9.15` client) |
| Link | Direct Ethernet Jetson `192.168.100.1` ⇄ Windows/WSL2-mirrored `192.168.100.2` |

## Results summary

| # | Feature | Verdict | Evidence |
|---|---|---|---|
| 1 | **SOME/IP binding (ara::com over vsomeip)** — routing manager host, app registration, service discovery | **PASS** | RM starts as Host (`/tmp/vsomeip-0`, client 0x1000); all 5 apps register (0x101…0x10a) |
| 2 | SOME/IP SOA cascade — offer/subscribe of all 5 services | **PASS** | vsomeip log: OFFER + SUBSCRIBE ACK for 0x1001 (Sensor, 4 events), 0x1002 (VehicleState), 0x1003 (Objects), 0x1004 (Trajectory), 0x1005 (Control) |
| 3 | **DDS binding (gateway lwrcl / CycloneDDS)** — node, pubs/subs on loopback domain 0 | **PASS** (after fixes) | Gateway stable; `/carla/ego_vehicle/{imu,odometry,lidar,vehicle_control_cmd}` visible; imu received at ~40 Hz |
| 4 | Zenoh LAN transport + bridge routing (both directions) | **PASS** | Bridge-to-bridge discovery; 5 routes created; sensor routes active; `vehicle_control_cmd` pub route established |
| 5 | Sensor path end-to-end, small topics (imu, odometry, clock) | **PASS** | ~39–60 Hz measured on Zenoh wire and on Jetson loopback |
| 6 | Sensor path end-to-end, **lidar** (PointCloud2, ~228 KB, 13 979 pts) | **PARTIAL / OPEN** | Full-size clouds delivered to Jetson, but at ~1.0 Hz instead of 10 Hz — loss on WSL2-local DDS hop (see Open issues) |
| 7 | **Closed control loop** (lidar → perception → planning → control → CARLA) | **NOT TESTED (blocked by #6)** | `vehicle_control_cmd` carries no data yet; expected once lidar reaches nominal rate |
| 8 | Host verification (Path A, in-process ara::com pipeline) | **PASS** (pre-existing) | `test_soa_pipeline`: stopped 3.4 m before obstacle |

## Functional Clusters (FCs) exercised

| FC | Usage in demo | Verdict |
|---|---|---|
| **ara::com (Communication Management)** | Skeleton/Proxy, events, `FindService`, `Subscribe`, receive handlers over the vsomeip SOME/IP binding (gateway + 4 AAs) | **PASS** — full 5-service event cascade verified |
| **Execution Management (EM)** | `run_ap.sh` as manual EM stand-in (machine mode `Driving`); apps call `ExecutionClient::ReportRunning` | **PASS** (stand-in) — all processes report running and stay up |
| **State Management (SM)** | `DrivingFG = Active` represented by `control_app` running | **PASS** (implicit) — Active state only; **SafeStop transition NOT TESTED** (`safe_stop_app` never started, UC-4 open) |
| **PHM (Platform Health Management)** | Supervision-failure → SafeStop scenario | **NOT TESTED** — no fault injection performed |
| **ara::log (Logging)** | Gateway/app loggers | PASS (informal) — log output present |

## Defects found and fixed during the session

| # | Defect | Root cause | Fix (committed to repo) |
|---|---|---|---|
| D1 | All vsomeip apps stuck as routing Proxy, endless `Couldn't connect to /tmp/vsomeip-0` | `vsomeip-av.json` declared routing host `av_routing_manager`, binary registers as `autosar_vsomeip_routing_manager`; unicast was `192.168.1.50` (no such interface) | Renamed routing host in config; unicast → `192.168.100.1` |
| D2 | `carla_gateway` failed to start: missing `libiceoryx_binding_c.so` | `/opt/iceoryx/lib` missing from `LD_LIBRARY_PATH` in `run_ap.sh` | Added to `LD_LIBRARY_PATH` |
| D3 | `carla_gateway` SIGSEGV in `dds::pub::TPublisher` ctor | Two incompatible `liblwrcl.so` builds installed; loader picked the **adaptive-autosar** backend (no CycloneDDS symbols) instead of the **cyclonedds** backend → ABI mismatch | Reordered `LD_LIBRARY_PATH` (`/opt/cyclonedds-libs` before `/opt/autosar-ap-libs`) + comment |
| D4 | No DDS discovery on Jetson loopback (topics invisible to bridge/CLI) | Linux `lo` has no MULTICAST flag → CycloneDDS silently disables SPDP multicast | `cyclonedds-local.xml`: unicast peer discovery on `127.0.0.1` (`ParticipantIndex auto`, `MaxAutoParticipantIndex 32`) |
| D5 | Zenoh bridge created routes but left them **inactive** (`local_nodes: []`) | **lwrcl does not publish `ros_discovery_info`**, so zenoh-bridge-ros2dds cannot attribute the gateway's DDS readers to a ROS node | Workaround: `route_keeper.py` (rclpy no-op subscriber) keeps routes active; DDS then also delivers to the gateway readers |
| D6 | No CARLA data at all with original topology | `zenoh_carla_bridge` (autoware_carla_launch) publishes Autoware topics/types (`v1/sensing/...`), incompatible with the gateway's carla-ros-bridge topics/types | Topology changed to carla-ros-bridge + zenoh-bridge-ros2dds on WSL2; new allow-list configs `zenoh-bridge-ros2dds-carla-{wsl,jetson}.json5`; docs updated |
| D7 | vsomeip config/docs assorted | `-n "v1"` namespace must be `/`-prefixed and is not needed in the new topology; CARLA client wheels: 0.9.14 has no cp310/aarch64 wheels | Docs updated (0.9.15 alignment); no `-n` in new topology |

## Open issues (for next session)

| # | Issue | Suspected cause | Planned action |
|---|---|---|---|
| O1 | **Lidar arrives at ~1 Hz instead of 10 Hz** (full 228 KB clouds, steady trickle; imu/odometry unaffected) | Loss of large fragmented UDP samples on the **WSL2-local DDS hop** (carla-ros-bridge → WSL zenoh bridge), likely the WSL2 mirrored-network stack; reliable-QoS retransmission yields the 1 Hz trickle | Pin both WSL2 processes' CycloneDDS to loopback (`cyclonedds-local.xml`, same fix as Jetson D4); verify kernel UDP buffers applied + processes restarted; relaunch with `fixed_delta_seconds:=0.1` + 10 Hz `objects.json` (sim currently runs old async config — clock ticks at ~39 Hz) |
| O2 | Closed loop / `vehicle_control_cmd` unverified | Blocked by O1 (perception starved) | Re-verify after O1; expect ego to stop 3.4 m before obstacle (AC-01/02) |
| O3 | `route_keeper.py` is a workaround, not a fix | lwrcl lacks ROS 2 graph announcement (`ros_discovery_info`) | Either patch lwrcl to publish graph info, or run keeper as a supervised service; alternatively evaluate zenoh-bridge-**dds** (raw DDS bridging, no graph needed) |
| O4 | Latent startup race in `carla_bridge.cpp:13` | `ControlServiceProxy::FindService("av").front()` on an empty container is UB if the gateway starts before `control_app` offers 0x1005 (caused one SIGSEGV; currently survives due to retry timing) | Guard: poll `FindService` until non-empty before constructing the proxy |
| O5 | SafeStop / PHM path untested (UC-4) | Not reached in this session | Exercise `DrivingFG Active → SafeStop` via SM tooling once loop closes |

## Measured data rates (final state of session)

| Topic | Zenoh wire (Jetson↔WSL2) | Jetson loopback (at gateway) | Target |
|---|---|---|---|
| `/carla/ego_vehicle/imu` | 38.7 Hz | ~40 Hz | ~50 Hz (async sim) ✔ |
| `/carla/ego_vehicle/odometry` | 38.7 Hz | ~40 Hz | tick rate ✔ |
| `/clock` | 38.7 Hz | routed | tick rate ✔ |
| `/carla/ego_vehicle/lidar` | **1.0 Hz** (228 012 B/msg) | ~1 Hz | **10 Hz** ✘ (O1) |
| `/carla/ego_vehicle/vehicle_control_cmd` | 0 (no data yet) | 0 | ~tick rate ✘ (O2) |
