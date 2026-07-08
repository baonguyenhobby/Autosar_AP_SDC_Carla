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
| 6 | Sensor path end-to-end, **lidar** (PointCloud2, ~228 KB, 13 979 pts) | **PASS 2026-07-08** (intermittent) | After WSL2 relaunch with `fixed_delta_seconds:=0.1` + 10 Hz `objects.json`: 13.5 Hz full-size clouds on the Zenoh wire, ~19 Hz bursts at the gateway. Regressed to 0 later in the session (WSL2 side stopped) — see O1 |
| 7 | **Closed control loop** (sensors → localization → control → CARLA) | **PASS 2026-07-08** | After fixing D8: `control_app` stable, `vehicle_control_cmd` at **49.6 Hz on the Zenoh wire** back to carla-ros-bridge. Obstacle-stop scenario (perception/planning via lidar) still pending a stable lidar feed |
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
| D7 | vsomeip config/docs assorted | `-n "v1"` namespace must be `/`-prefixed and is not needed in the new topology; CARLA client wheels: 0.9.15 has no cp310/aarch64 wheels | Docs updated (0.9.15 alignment); no `-n` in new topology |
| D8 | **`control_app` terminated with uncaught `std::runtime_error: "Optional contains no value"`**; `perception_app` threw the same on every lidar sample (caught by vsomeip's handler wrapper — alive but blind, `planning_app` starved) | **Use-after-move in Adaptive-AUTOSAR** `Serializer<std::vector<T>>::DeserializeAt()` (serialization.h): `std::move(elemResult).Value().first` disengages the Result's Optional (`Optional::Value()&&` nulls the value ptr), then `elemResult.Value().second` throws. Hit ALL non-trivial vector payloads (lidar, objects, trajectory); flat structs (imu/gps/speed/state/command) use the memcpy path and were unaffected. The map serializer had the correct order — only the vector specialization was reversed | Fixed in `Adaptive-AUTOSAR/src/ara/com/serialization.h` (take the pair out once), synced to `/opt/autosar-ap/include`, av-stack rebuilt. Verified: 0 exceptions, `control_app` stable, control commands at 49.6 Hz on the wire. Upstream repo is third-party → fix carried as **`av-stack/patches/0001-adaptive-autosar-fix-vector-deserialize-use-after-move.patch`** (apply before building Adaptive-AUTOSAR, see BUILD_AND_RUN.md Part 4 step 3) |

## Open issues (for next session)

| # | Issue | Suspected cause | Planned action |
|---|---|---|---|
| O1 | **Lidar feed unstable on WSL2 side** — reached 13.5 Hz after the 2026-07-08 relaunch (`fixed_delta_seconds:=0.1`, 10 Hz `objects.json`), then dropped to 0 later the same session while imu/odometry kept flowing | WSL2-side process state (carla-ros-bridge lidar publisher stops or loses the loopback `CYCLONEDDS_URI` on restart); earlier 1 Hz trickle was large-UDP loss on the WSL2-local DDS hop | Make the WSL2 launch environment persistent (exports in shell profile / launch script); re-check `ros2 topic hz /carla/ego_vehicle/lidar` locally on WSL2 after every restart |
| O2 | Closed loop / `vehicle_control_cmd` | **Control path VERIFIED 2026-07-08** (49.6 Hz on the wire after D8 fix) | Remaining: obstacle-stop scenario (UC-2/3, AC-01/02) — needs stable lidar (O1) so perception → planning produce an obstacle-aware trajectory |
| O3 | `route_keeper.py` is a workaround, not a fix | lwrcl lacks ROS 2 graph announcement (`ros_discovery_info`) | **Mitigated 2026-07-08:** keeper added to repo (`tools/route_keeper.py`) and wired into the Part 5 bridge start procedure (BUILD_AND_RUN.md). Proper fix still open: patch lwrcl to publish graph info, or evaluate zenoh-bridge-**dds** (raw DDS bridging, no graph needed) |
| O4 | Latent startup race in `carla_bridge.cpp:13` | `ControlServiceProxy::FindService("av").front()` on an empty container is UB if the gateway starts before `control_app` offers 0x1005 (caused one SIGSEGV; currently survives due to retry timing) | **RESOLVED 2026-07-08:** `services::WaitForService<ProxyT>()` (av_services.hpp) polls until a handle exists; `carla_bridge.cpp` offers SensorService first, then waits. Verified: gateway runs 8 s+ without `control_app`, no crash |
| O5 | SafeStop / PHM path untested (UC-4) | Not reached in this session | Exercise `DrivingFG Active → SafeStop` via SM tooling once loop closes |

## Measured data rates (final state of session)

| Topic | Zenoh wire (Jetson↔WSL2) | Jetson loopback (at gateway) | Target |
|---|---|---|---|
| `/carla/ego_vehicle/imu` | 38.7 Hz | ~40 Hz | ~50 Hz (async sim) ✔ |
| `/carla/ego_vehicle/odometry` | 38.7 Hz | ~40 Hz | tick rate ✔ |
| `/clock` | 38.7 Hz | routed | tick rate ✔ |
| `/carla/ego_vehicle/lidar` | **1.0 Hz** (228 012 B/msg) | ~1 Hz | **10 Hz** ✘ (O1) |
| `/carla/ego_vehicle/vehicle_control_cmd` | 0 (no data yet) | 0 | ~tick rate ✘ (O2) |
