# Autosar_AP_SDC_Carla

A simple self-driving-car demonstrator built on the **AUTOSAR Adaptive Platform (AP)**.
The four pipeline stages (localization, perception, planning, control) run as AP
**Adaptive Applications** communicating service-orientedly over `ara::com`, on a
**Jetson Orin Nano**, driving **CARLA** on a Windows host through the **lwrcl** bridge.

The AP stack, architecture notes, and build/run guide live under [`av-stack/`](av-stack/)
(see `av-stack/README.md` and `av-stack/BUILD_AND_RUN.md`).

## Repository layout

| Path | Contents |
|---|---|
| `av-stack/` | The AP Adaptive Applications, `ara::com` services, gateway, manifests, tests, docs |
| `Adaptive-AUTOSAR/` | AUTOSAR AP runtime (git submodule) |
| `lwrcl/` | LightWeight Rclcpp Compatible Library — SOME/IP ⇄ CycloneDDS bridge (git submodule) |
| `LICENSE` | MIT (this repo) + third-party acknowledgements |

Clone with submodules:

```bash
git clone --recursive https://github.com/<you>/Autosar_AP_SDC_Carla.git
# or, if already cloned:
git submodule update --init --recursive
```

### Apply the carla_msgs patch (required for the gateway)

The `CarlaEgoVehicleControl` message the gateway publishes is **not** in the stock ROS
set. It lives in `patches/add-carla_msgs.patch`, which adds the `carla_msgs` package to the
`ros-data-types-cyclonedds` **nested submodule** (that submodule can't carry the change
directly). After the recursive clone/update above, apply it:

```bash
cd lwrcl/data_types/src/ros-data-types-cyclonedds
git apply --check ../../../../patches/add-carla_msgs.patch   # dry run
git apply         ../../../../patches/add-carla_msgs.patch
cd -                                                          # back to repo root
```

Then (re)build the lwrcl CycloneDDS data types so `carla_msgs` is generated:

```bash
cd lwrcl && ./build_data_types.sh cyclonedds install
```

Skip this only if you have already repointed the submodules at forks that include
`carla_msgs`.

## Dependencies

### Windows host (CARLA simulator + bridge)

The vehicle is simulated on the Windows host; the CARLA ⇄ ROS 2 bridge runs in WSL2.

| Dependency | Version / branch | Purpose | Link |
|---|---|---|---|
| **CARLA Simulator** | **0.9.14** | Native GPU driving simulator (UE4); serves the world and the ego vehicle. Binds its RPC server on `:2000`. | https://carla.org/ |
| **autoware_carla_launch** | **`humble` branch** | CARLA ⇄ ROS 2 bridge stack. Runs the Carla agent plus `zenoh_carla_bridge` + `zenoh-bridge-ros2dds`, exposing CARLA sensors/actuators as ROS 2 (DDS) topics that the av-stack gateway consumes. Runs in WSL2 / Ubuntu. | https://github.com/evshary/autoware_carla_launch/tree/humble |

Notes:
- CARLA 0.9.14 must match the bridge's expected CARLA API version.
- The `humble` branch targets ROS 2 **Humble** and bridges over **Zenoh**
  (`zenoh_carla_bridge` on the CARLA side, `zenoh-bridge-ros2dds` on the ROS 2 side).
  On the Jetson, the av-stack `carla_gateway` subscribes the ROS 2 / DDS topics that
  `zenoh-bridge-ros2dds` republishes locally.
- Setup and launch instructions for the bridge are in its own
  [documentation](https://autoware-carla-launch.readthedocs.io/en/latest/).

### Jetson Orin Nano (AP target)

Built from the submodules + this repo — see `av-stack/BUILD_AND_RUN.md` for the full
step list. In brief: CycloneDDS, vsomeip (SOME/IP), the AUTOSAR-AP runtime + codegen
(from `Adaptive-AUTOSAR/`), and lwrcl (adaptive-autosar backend, from `lwrcl/`), then
`./build_ap.sh adaptive-autosar` and `./run_ap.sh`.

## License

MIT for this repository's original code — see [`LICENSE`](LICENSE). The submodules keep
their own upstream licenses (Adaptive-AUTOSAR: MIT; lwrcl: Apache 2.0). Algorithm cores
were developed with reference to the Udacity self-driving-car course repositories
(https://github.com/udacity); see the acknowledgements in `LICENSE`.
