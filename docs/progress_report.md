# Progress Report — UAV Swarm Research Project

**Author:** AI Engineer — Computer Vision & UAV Platforms
**Started:** 2026-03-30
**Last updated:** 2026-04-05
**Chosen Topic:** **A — Vision-Guided Adaptive Area Monitoring with Drone Swarms**
**Target Venue:** MDPI Drones (open access)
**Timeline:** 5 weeks (2–3 hrs/day), ending ~2026-05-05

---

## Summary

**Week 1 of the adaptive area monitoring project is complete.** The full system is
implemented: 3-drone PX4 SITL simulation with downward cameras, YOLOv8 detection
pipeline publishing ROS 2 messages, MAVSDK waypoint executor, Voronoi-based adaptive
coverage coordinator, data logging, offline metrics computation, three baseline
controllers, and experiment automation scripts. The system is ready for dry runs
and parameter tuning (Week 2).

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
- **Root cause:** ROS 2 Humble with Gazebo Harmonic (`GZ_VERSION=harmonic`) requires
  the matching `ros-humble-ros-gzharmonic` bridge package. The default `ros-humble-ros-gz`
  installs bridges for Gazebo Garden, which are incompatible with Harmonic topics.
- **Fix:**
  1. Remove the previous `gz_sim` / `ros-gz` installation
  2. Install the Harmonic-specific bridge: `sudo apt-get install ros-humble-ros-gzharmonic`
  3. Re-source everything (`source /opt/ros/humble/setup.bash`)
- **Earlier partial fixes (before root cause found):** Added `lazy: false` to YAML
  entries, switched `os.path.dirname(__file__)` to `os.path.realpath(__file__)`,
  added `use_sim_time: true`. These were helpful but not the main fix.
- **Status:** ✅ Resolved. Camera images confirmed flowing end-to-end.

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

## Day 3 — 2026-04-03

**Goal:** YOLOv8 inference on drone camera feeds, target spawning, end-to-end CV demo.

### Completed

| Task | Status | Notes |
|------|--------|-------|
| YOLOv8 detector node | ✅ | `scripts/yolo_detector.py` — subscribes to camera topics, runs inference |
| Target object spawning | ✅ | `scripts/spawn_targets.sh` — 5 colored primitives via `gz service` |
| Full Day 3 runner script | ✅ | `scripts/run_day3.sh` — orchestrates drones → bridge → targets → YOLO |
| Camera feed verification | ✅ | `ros2 topic hz` confirms ~30 Hz on all 3 drone camera topics |

### Issues encountered and resolved

#### `gz service` protobuf multiline string error
- **Root cause:** Inline SDF XML in `--req` string contained newlines; protobuf text format rejects multi-line string literals.
- **Fix:** Write SDF models to temp files and use `sdf_filename` field instead of `sdf` in the EntityFactory request.

---

## Day 4 — 2026-04-04 (Topic A: Adaptive Area Monitoring)

**Goal:** Custom ROS 2 messages, detection publisher with world-position estimation.

### Completed

| Task | Status | Notes |
|------|--------|-------|
| Project topic chosen | ✅ | **Topic A** — Vision-Guided Adaptive Area Monitoring with Drone Swarms |
| 5-week research plan created | ✅ | `adaptive_area_monitoring.md` — daily tasks, weekly deliverables |
| `swarm_msgs` ROS 2 package | ✅ | `Detection.msg` (drone_id, class, confidence, bbox, world_position, stamp) |
| | | `TargetWaypoint.msg` (drone_id, lat, lon, alt, priority) |
| `detection_publisher.py` | ✅ | Refactored from `yolo_detector.py` — publishes Detection messages on `/droneN/detection`, estimates ground position from bbox center + altitude + camera FOV |
| `waypoint_executor.py` | ✅ | MAVSDK subscriber node — listens to `/droneN/target_waypoint`, calls `goto_location()`, publishes GPS position at 2 Hz on `/droneN/position`. Threading: ROS 2 in background, asyncio MAVSDK in main |
| `voronoi_utils.py` | ✅ | Bounded Voronoi cells (Sutherland-Hodgman clipping), Lloyd's relaxation, density-weighted centroids (Monte Carlo), GPS↔XY flat-earth conversion, Gaussian density map with temporal decay |
| `voronoi_coordinator.py` | ✅ | Core algorithm node — subscribes to positions + detections, maintains adaptive density map (uniform + Gaussian bumps at detection sites), computes weighted Voronoi every 2s, publishes TargetWaypoint per drone |
| `swarm` conda env created | ✅ | Python 3.10 (ROS 2 compatible), all deps installed |

### Issues encountered and resolved

#### `conda myenv` Python 3.11 incompatible with ROS 2 Humble
- **Root cause:** ROS 2 Humble's `rclpy` C extensions are compiled for Python 3.10. The `myenv` conda environment has Python 3.11, causing `ModuleNotFoundError: No module named 'rclpy._rclpy_pybind11'`.
- **Fix:** Created a new conda env `swarm` with Python 3.10. All project scripts must run with `conda activate swarm`.

#### `colcon build` missing `empy` and `catkin_pkg`
- **Root cause:** ROS 2 message generation (`rosidl`) requires `empy` and `catkin_pkg` Python packages. These are normally available via system Python but not in a fresh conda env.
- **Fix:** Installed `empy==3.3.4`, `catkin_pkg`, and `lark` in the `swarm` conda env.

#### Disk space exhausted during package installation
- **Root cause:** Only 2.7 GB free; PyTorch + CUDA pip packages require ~6 GB.
- **Fix:** Cleaned pip cache (`pip cache purge`, freed 7 GB) and conda cache (`conda clean --all`, freed 1.4 GB). Total freed: ~8.3 GB.

#### `Node.publishers` attribute conflict
- **Root cause:** Named a dict `self.publishers` which shadows a built-in property on `rclpy.Node`.
- **Fix:** Renamed to `self.det_publishers` in `detection_publisher.py`.

---

## Day 5 — 2026-04-05 (Week 1 continued)

**Goal:** Full pipeline integration, data logging, baselines, experiment automation.

### Completed

| Task | Status | Notes |
|------|--------|-------|
| `run_adaptive.sh` | ✅ | Full pipeline launcher: drones → bridge → targets → executor → detector → coordinator. Supports `--no-sim`, `--drones N`, all algorithm params configurable |
| `data_logger.py` | ✅ | ROS 2 node logging positions, detections, waypoints to CSV files in `data/logs/<experiment_id>/`. Auto-stop via `--duration` |
| `metrics_compute.py` | ✅ | Offline analysis: coverage % (grid + FOV projection), redundancy ratio, energy (total distance), detection latency. Outputs `metrics_summary.json` + `coverage_over_time.csv` |
| `baselines/static_grid.py` | ✅ | Fixed grid positions — divides area into N cells, drones hover at centers |
| `baselines/lawnmower.py` | ✅ | Back-and-forth sweep in assigned vertical strips, loops on completion, advances on arrival |
| `baselines/random_waypoints.py` | ✅ | Random point selection within monitoring area, picks new target on arrival |
| `run_experiment.sh` | ✅ | Single trial runner: `--method {adaptive\|static\|lawnmower\|random} --drones N --targets M --duration S --trial T`. Launches everything, logs, computes metrics, cleans up |
| `run_all_experiments.sh` | ✅ | Batch runner for E1–E4, supports `--exp e1` for single experiment, `--trials N` override |

---

## Pending (Start of Week 2)

- [ ] Day 11: Parameterized target spawning (`--count N`, `--random`, `--seed S`) + 5-drone support
- [ ] Day 12: Edge case handling + parameter tuning + `config/adaptive_params.yaml`
- [ ] Day 13: Dry run E1 (baselines comparison — 2 trials each method)
- [ ] Day 14: Dry run E2 + E3 (scalability + target density)
- [ ] Day 15: Response time instrumentation for E4
- [ ] Days 16–17: Literature review (20 papers, BibTeX file)

---

## System Architecture

```
┌────────────────────────────────────────┐
│        voronoi_coordinator.py          │
│  Sub: /droneN/detection, /droneN/pos   │
│  Pub: /droneN/target_waypoint          │
│  Algorithm: Weighted Voronoi + Lloyd's │
│  Density: uniform + Gaussian bumps     │
│  at detection sites (decaying)         │
└──────────┬──────────────┬──────────────┘
           │              │
           ▼              ▼
┌──────────────────┐  ┌───────────────────┐
│ detection_pub.py │  │ wp_executor.py    │
│ Sub: image_raw   │  │ Sub: target_wp    │
│ Pub: /detection  │  │ Pub: /position    │
│ YOLOv8 + world   │  │ MAVSDK goto_loc() │
│ position estim.  │  │ + telemetry 2 Hz  │
└──────────────────┘  └───────────────────┘

┌──────────────────┐  ┌───────────────────┐
│ data_logger.py   │  │ baselines/        │
│ Sub: all topics  │  │  static_grid.py   │
│ Out: CSV files   │  │  lawnmower.py     │
│                  │  │  random_wps.py    │
└──────────────────┘  └───────────────────┘
```

### ROS 2 Topic Map

| Topic | Publisher | Subscriber | Message Type |
|-------|----------|------------|-------------|
| `/droneN/camera/image_raw` | Gazebo (via bridge) | detection_publisher | sensor_msgs/Image |
| `/droneN/detection` | detection_publisher | voronoi_coordinator | swarm_msgs/Detection |
| `/droneN/position` | waypoint_executor | coordinator, baselines, logger | swarm_msgs/TargetWaypoint |
| `/droneN/target_waypoint` | coordinator / baselines | waypoint_executor, logger | swarm_msgs/TargetWaypoint |

---

## Files Created

| File | Day | Purpose |
|------|-----|---------|
| `scripts/launch_multi_drone.sh` | 2 | Launches N drones in separate terminals |
| `scripts/multi_drone_waypoints.py` | 2 | MAVSDK script: arm, takeoff, waypoint, RTL |
| `scripts/yolo_detector.py` | 3 | YOLOv8 inference on camera feeds (print only) |
| `scripts/spawn_targets.sh` | 3 | Spawn colored target objects in Gazebo |
| `scripts/run_day3.sh` | 3 | Day 3 full pipeline orchestration |
| `scripts/detection_publisher.py` | 4 | YOLOv8 + ROS 2 Detection message publisher |
| `scripts/waypoint_executor.py` | 4 | MAVSDK subscriber-based waypoint control |
| `scripts/voronoi_utils.py` | 4 | Voronoi coverage algorithms and utilities |
| `scripts/voronoi_coordinator.py` | 4 | Core adaptive coverage coordinator node |
| `scripts/run_adaptive.sh` | 5 | Full adaptive pipeline launcher |
| `scripts/data_logger.py` | 5 | Position/detection/waypoint CSV logger |
| `scripts/metrics_compute.py` | 5 | Offline metrics from CSV logs |
| `scripts/baselines/static_grid.py` | 5 | Baseline: fixed grid positions |
| `scripts/baselines/lawnmower.py` | 5 | Baseline: back-and-forth sweep |
| `scripts/baselines/random_waypoints.py` | 5 | Baseline: random waypoint patrol |
| `scripts/run_experiment.sh` | 5 | Single trial experiment runner |
| `scripts/run_all_experiments.sh` | 5 | Batch experiment runner (E1–E4) |
| `ros2_ws/src/swarm_msgs/msg/Detection.msg` | 4 | Custom detection message |
| `ros2_ws/src/swarm_msgs/msg/TargetWaypoint.msg` | 4 | Custom waypoint message |
| `ros2_ws/src/swarm_msgs/package.xml` | 4 | ROS 2 message package manifest |
| `ros2_ws/src/swarm_msgs/CMakeLists.txt` | 4 | Message package build config |
| `ros2_ws/src/swarm_bringup/launch/camera_bridge.launch.py` | 2 | Gazebo→ROS 2 camera bridge |
| `ros2_ws/src/swarm_bringup/config/camera_bridge.yaml` | 2 | Topic mappings (3 drones) |
| `adaptive_area_monitoring.md` | 4 | 5-week research plan with daily tasks |
| `docs/baselines.md` | 5 | Baseline controller documentation |
| `docs/px4_sitl_performance.md` | 1 | GPU setup, headless mode notes |
| `docs/drone_cameras.md` | 2 | Camera bridging documentation |
| `docs/progress_report.md` | 1 | This file |
| `quick_start_guide.md` | 0 | Background reading on CV+MARL+swarm concepts |

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
