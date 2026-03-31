# Progress Report — UAV Swarm Research Project

**Author:** AI Engineer — Computer Vision & UAV Platforms
**Started:** 2026-03-30
**Last updated:** 2026-03-31

---

## Summary

Two days of environment setup and simulation bring-up completed. A 3-drone PX4 SITL
swarm is running in Gazebo Harmonic with downward-facing cameras, ROS 2 integration
via Micro XRCE-DDS, and a Foxglove Studio visualization pipeline. Camera image
bridging is configured but image data flow is pending final verification.

---

## Day 1 — 2026-03-30

**Goal:** Single drone running in simulation, controllable via ROS 2.

### Completed

| Task | Status | Notes |
|------|--------|-------|
| PX4-Autopilot cloned as submodule | ✅ | `firmware/` directory |
| Single drone launched in Gazebo Harmonic | ✅ | `make px4_sitl gz_x500` |
| Micro XRCE-DDS Agent installed and tested | ✅ | `MicroXRCEAgent udp4 -p 8888` |
| ROS 2 Humble verified (`ros2 topic list`) | ✅ | PX4 topics visible under `/fmu/` |
| MAVROS2 investigated | ✅ | Chose Micro XRCE-DDS as primary bridge (newer standard) |

### Issues encountered

- **Accel #0 TIMEOUT on first multi-drone attempt** — investigated, root cause was
  sensor timing during multi-vehicle SITL; resolved in Day 2.

---

## Day 2 — 2026-03-31

**Goal:** 3 drones spawned with cameras, MAVSDK control script, camera pipeline to Foxglove.

### Completed

| Task | Status | Notes |
|------|--------|-------|
| Multi-drone launch script written | ✅ | `scripts/launch_multi_drone.sh` |
| 3x `x500_mono_cam_down` drones spawned in Gazebo | ✅ | Downward-facing cameras |
| MAVSDK Python waypoint script | ✅ | `scripts/multi_drone_waypoints.py` |
| Camera model identified and documented | ✅ | `x500_mono_cam_down` — no SDF editing needed |
| `ros_gz_bridge` installed and configured | ✅ | YAML config approach |
| Gazebo camera topics verified publishing | ✅ | `gz topic -e` confirms frames at 30 Hz |
| ROS 2 camera topics created | ✅ | `/drone1/camera/image_raw` etc. visible |
| Foxglove bridge installed | ✅ | Connects on `ws://localhost:8765` |
| Foxglove subscribes to camera topics | ✅ | Subscriptions confirmed in bridge logs |
| Camera images visible in Foxglove | ⚠️ | Topics exist, bridge subscribed, image data not yet confirmed end-to-end |

### Issues encountered and resolved

#### Multi-drone sensor failures (`Accel #0 TIMEOUT`, sensors missing)

- **Root cause:** Using `PX4_GZ_MODEL` instead of `PX4_SIM_MODEL=gz_<model>` (the
  official env var), combined with running PX4 from the wrong working directory so
  `gz_env.sh` was never sourced, leaving `GZ_SIM_SYSTEM_PLUGIN_PATH` unset.
- **Fix:** Rewrote `launch_multi_drone.sh` following the official PX4 multi-vehicle
  docs. Drone 1 uses `PX4_SIM_MODEL=gz_x500` without `PX4_GZ_STANDALONE` (starts
  Gazebo). Drones 2+ use `PX4_GZ_STANDALONE=1` and connect to the running world.
- **Reference:** https://docs.px4.io/main/en/sim_gazebo_gz/multi_vehicle_simulation

#### `[Err] Unable to find uri[file:///x500/model.sdf]`

- **Root cause:** `GZ_SIM_RESOURCE_PATH` not set when Gazebo was launched standalone.
- **Fix:** The official `PX4_SIM_MODEL` approach handles resource paths internally via
  `gz_env.sh`; no manual path export needed.

#### `MotorFailurePlugin` shared library error

- **Status:** Harmless — optional plugin not built by default. Ignored.

#### `vehicle_command_ack lost` MAVLink errors

- **Root cause:** QGroundControl not connected (no GCS). Expected in pure ROS 2 setup.
- **Status:** Harmless warning. Ignored.

#### Camera images not visible in Foxglove

- **Diagnosed:** Gazebo publishes frames (`gz topic -e` confirmed). Bridge creates ROS 2
  topics. Foxglove subscribes successfully. End-to-end image flow not yet confirmed.
- **Likely cause:** `lazy: false` was missing from YAML config (bridge was not eagerly
  subscribing to Gazebo); or `__file__` path resolution issue when launching without
  a built ROS 2 package.
- **Fix applied 2026-03-31:** Added `lazy: false` to all YAML entries; switched
  `os.path.dirname(__file__)` to `os.path.realpath(__file__)` for reliable path
  resolution; added `use_sim_time: true` parameter.
- **Pending:** Relaunch and verify with `ros2 topic hz /drone1/camera/image_raw`.

---

## Files Created

| File | Purpose |
|------|---------|
| `scripts/launch_multi_drone.sh` | Launches N drones in separate terminals |
| `scripts/multi_drone_waypoints.py` | MAVSDK script: arm, takeoff, waypoint, RTL |
| `ros2_ws/src/swarm_bringup/launch/camera_bridge.launch.py` | Bridges Gazebo→ROS 2 cameras |
| `ros2_ws/src/swarm_bringup/config/camera_bridge.yaml` | Topic mapping for all 3 drones |
| `docs/px4_sitl_performance.md` | GPU setup, headless mode, CPU starvation fixes |
| `docs/drone_cameras.md` | Camera attachment, bridging, Foxglove, RViz2 |
| `docs/progress_report.md` | This file |
| `quick_start_guide.md` | Background reading on CV+MARL+swarm concepts |

---

## Pending (Start of Day 3)

- [ ] Verify camera image data flows end-to-end:
  `ros2 topic hz /drone1/camera/image_raw` should show ~30 Hz
- [ ] Run `multi_drone_waypoints.py` with all 3 drones active and confirm takeoff
- [ ] Test Foxglove image panel with live camera feed
- [ ] Begin Day 3: YOLOv8 inference on ROS 2 camera topic stream

---

## Software Versions

| Software | Version |
|----------|---------|
| Ubuntu | 22.04 LTS |
| ROS 2 | Humble Hawksbill |
| Gazebo | Harmonic (gz-sim 8.x) |
| PX4-Autopilot | main branch (as of 2026-03-30) |
| NVIDIA Driver | 580.126.09 |
| CUDA | 13.0 |
| GPU | RTX 4060 Laptop (8 GB VRAM) |
| Python | 3.10 |
| MAVSDK-Python | latest pip |
| ros-humble-ros-gz-bridge | apt |
| ros-humble-foxglove-bridge | apt |
