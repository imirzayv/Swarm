# Progress Report — UAV Swarm Research Project

**Author:** AI Engineer — Computer Vision & UAV Platforms
**Started:** 2026-03-30
**Last updated:** 2026-04-07
**Chosen Topic:** **A — Vision-Guided Adaptive Area Monitoring with Drone Swarms**
**Target Venue:** MDPI Drones (open access)
**Timeline:** 5 weeks (2–3 hrs/day), ending ~2026-05-05

---

## Summary

**Week 1 complete. Week 2 in progress (Day 12).** The full adaptive coverage
system was implemented in Week 1. In early Week 2, significant improvements
were made to simulation realism (Day 11: shadows, Fuel models, 200m area) and
all three novel elements were implemented (Day 12: confidence-weighted density,
class-specific formations, dual-mode explore/exploit with split-and-reform).
Two ablation baselines (binary_voronoi, all_converge) were added for E5, the
SwarmMode.msg was created, and multi-class target spawning (person, vehicle,
fire) is now supported.

---

## Day 1 — 2026-03-30

**Goal:** Single drone running in simulation, controllable via ROS 2.

### Completed

| Task | Status | Notes |
|------|--------|-------|
| PX4-Autopilot cloned as submodule | Done | `firmware/` directory |
| Single drone launched in Gazebo Harmonic | Done | `make px4_sitl gz_x500` |
| Micro XRCE-DDS Agent installed and tested | Done | `MicroXRCEAgent udp4 -p 8888` |
| ROS 2 Humble verified (`ros2 topic list`) | Done | PX4 topics visible under `/fmu/` |
| MAVROS2 investigated | Done | Chose Micro XRCE-DDS as primary bridge (newer standard) |

### Issues encountered

- **Accel #0 TIMEOUT on first multi-drone attempt** — investigated, root cause was
  sensor timing during multi-vehicle SITL; resolved in Day 2.

---

## Day 2 — 2026-03-31

**Goal:** 3 drones spawned with cameras, MAVSDK control script, camera pipeline to Foxglove.

### Completed

| Task | Status | Notes |
|------|--------|-------|
| Multi-drone launch script written | Done | `scripts/launch_multi_drone.sh` |
| 3x `x500_mono_cam_down` drones spawned in Gazebo | Done | Downward-facing cameras |
| MAVSDK Python waypoint script | Done | `scripts/multi_drone_waypoints.py` |
| Camera model identified and documented | Done | `x500_mono_cam_down` — no SDF editing needed |
| `ros_gz_bridge` installed and configured | Done | YAML config approach |
| Gazebo camera topics verified publishing | Done | `gz topic -e` confirms frames at 30 Hz |
| ROS 2 camera topics created | Done | `/drone1/camera/image_raw` etc. visible |
| Foxglove bridge installed | Done | Connects on `ws://localhost:8765` |

### Issues encountered and resolved

#### Multi-drone sensor failures (`Accel #0 TIMEOUT`, sensors missing)

- **Root cause:** Using `PX4_GZ_MODEL` instead of `PX4_SIM_MODEL=gz_<model>` (the
  official env var), combined with running PX4 from the wrong working directory so
  `gz_env.sh` was never sourced, leaving `GZ_SIM_SYSTEM_PLUGIN_PATH` unset.
- **Fix:** Rewrote `launch_multi_drone.sh` following the official PX4 multi-vehicle
  docs. Drone 1 uses `PX4_SIM_MODEL=gz_x500` without `PX4_GZ_STANDALONE` (starts
  Gazebo). Drones 2+ use `PX4_GZ_STANDALONE=1` and connect to the running world.

#### Camera images not visible in Foxglove

- **Root cause:** ROS 2 Humble with Gazebo Harmonic requires `ros-humble-ros-gzharmonic`
  bridge package. The default `ros-humble-ros-gz` installs bridges for Gazebo Garden.
- **Fix:** `sudo apt-get install ros-humble-ros-gzharmonic`, re-source everything.

---

## Day 3 — 2026-04-03

**Goal:** YOLOv8 inference on drone camera feeds, target spawning, end-to-end CV demo.

### Completed

| Task | Status | Notes |
|------|--------|-------|
| YOLOv8 detector node | Done | `scripts/yolo_detector.py` |
| Target object spawning | Done | `scripts/spawn_targets.sh` |
| Full Day 3 runner script | Done | `scripts/run_day3.sh` |
| Camera feed verification | Done | ~30 Hz on all 3 drone camera topics |

### Issues encountered

#### `gz service` protobuf multiline string error
- **Fix:** Write SDF models to temp files and use `sdf_filename` field instead of
  inline `sdf` in the EntityFactory request.

---

## Day 4 — 2026-04-04 (Topic A: Adaptive Area Monitoring)

**Goal:** Custom ROS 2 messages, detection publisher with world-position estimation.

### Completed

| Task | Status | Notes |
|------|--------|-------|
| Project topic chosen | Done | **Topic A** — Vision-Guided Adaptive Area Monitoring |
| 5-week research plan created | Done | `adaptive_area_monitoring.md` |
| `swarm_msgs` ROS 2 package | Done | `Detection.msg`, `TargetWaypoint.msg` |
| `detection_publisher.py` | Done | YOLOv8 + Detection messages + world position estimation |
| `waypoint_executor.py` | Done | MAVSDK subscriber, goto_location(), position publishing |
| `voronoi_utils.py` | Done | Bounded Voronoi, Lloyd's, density map, GPS conversion |
| `voronoi_coordinator.py` | Done | Adaptive coverage: weighted Voronoi + Gaussian density |
| `swarm` conda env created | Done | Python 3.10 (ROS 2 compatible) |

### Issues encountered and resolved

- **conda Python 3.11 vs ROS 2 Humble (3.10):** Created new `swarm` conda env.
- **colcon build missing `empy` and `catkin_pkg`:** Installed in swarm env.
- **Disk space exhausted:** Cleaned pip (7 GB) and conda (1.4 GB) caches.
- **`Node.publishers` attribute conflict:** Renamed to `self.det_publishers`.

---

## Day 5 — 2026-04-05 (Week 1 completion)

**Goal:** Full pipeline integration, data logging, baselines, experiment automation.

### Completed

| Task | Status | Notes |
|------|--------|-------|
| `run_adaptive.sh` | Done | Full pipeline launcher with all params configurable |
| `data_logger.py` | Done | CSV logging: positions, detections, waypoints |
| `metrics_compute.py` | Done | Coverage %, redundancy, energy, detection latency |
| `baselines/static_grid.py` | Done | Fixed grid positions |
| `baselines/lawnmower.py` | Done | Back-and-forth sweep in vertical strips |
| `baselines/random_waypoints.py` | Done | Random point selection within area |
| `run_experiment.sh` | Done | Single trial runner with all methods |
| `run_all_experiments.sh` | Done | Batch runner for E1–E4 |

---

## Day 6 — 2026-04-06 (Week 2 start)

**Goal:** Simulation realism improvements, detection pipeline fixes, visualization.

### Completed

| Task | Status | Notes |
|------|--------|-------|
| Flight path visualization tool | Done | `scripts/plot_paths.py` — combined + per-drone images, auto-scaled bounds, target markers, legend outside plot. Integrated into `run_experiment.sh` |
| Custom Gazebo world (no shadows) | Done | `worlds/default.sdf` with `<shadows>false</shadows>` and `<cast_shadows>false</cast_shadows>`. Symlinked into PX4 worlds as `swarm_default.sdf` |
| Replaced geometric targets with vehicles | Done | `spawn_targets.sh` rewritten — SUV, Hatchback, Hatchback Red/Blue, Pickup from Gazebo Fuel. COCO-detectable classes (car, truck) |
| Fixed detection coordinate bug | Done | `data_logger.py` was logging drone-relative offsets as world coordinates. Now adds drone position to get global XY |
| Dynamic camera bridge | Done | `run_experiment.sh` and `run_adaptive.sh` generate camera bridge YAML at runtime for N drones (no longer hardcoded to 3) |
| Monitoring area scaled to 200x200m | Done | `AREA_SIZE=200` in `run_experiment.sh`. Targets spread 40-60m apart, 60m+ from origin |
| Fixed Fuel model texture paths | Done | Patched `.mtl` files for hatchback variants — corrected relative paths, copied missing `wheels3.png` |
| Ground-truth targets on plots | Done | `plot_paths.py` shows real target positions as red X markers with labels |

### Issues encountered and resolved

#### Drone shadows causing YOLO false positives
- **Root cause:** Drone shadows on the ground plane were classified as "kite",
  "airplane", etc. by YOLOv8.
- **Fix:** Created custom Gazebo world with shadows disabled. Symlinked into PX4's
  worlds directory to avoid modifying the PX4 submodule.
- **Complication:** PX4's `gz_env.sh` overrides `PX4_GZ_WORLDS` at startup. Setting
  the env var externally has no effect.
- **Solution:** Symlink `worlds/default.sdf` → PX4's worlds dir as `swarm_default.sdf`,
  set `PX4_GZ_WORLD=swarm_default`.
- **Doc:** `docs/gazebo_disable_shadows.md`

#### YOLO cannot detect geometric shapes
- **Root cause:** Colored boxes/cylinders/spheres don't exist in the COCO dataset.
  YOLOv8 guesses the closest match ("kite", "suitcase") with unreliable confidence.
- **Fix:** Replaced all geometric targets with realistic 3D vehicle models from
  Gazebo Fuel (SUV, Hatchback, Pickup). These are COCO classes `car` and `truck`.
- **Doc:** `docs/gazebo_fuel_targets.md`

#### Detection coordinates scattered randomly on plots
- **Root cause:** `data_logger.py` logged `msg.world_position` directly from the
  Detection message, but this field contains drone-relative offsets (e.g., "3.2m to
  the right"), not global coordinates.
- **Fix:** Logger now tracks each drone's last known position and computes
  `global = drone_pos + offset` before writing to CSV.

#### Fuel model textures not loading (black models)
- **Root cause:** `.mtl` files in hatchback models had broken texture paths:
  bare filenames (`hatchback.png`), `model://` URIs, or full Fuel URLs that
  Gazebo Harmonic couldn't resolve.
- **Fix:** Patched all `.mtl` files to use correct relative paths
  (`../materials/textures/filename.png`) and copied missing `wheels3.png`
  from the base hatchback model to red/blue variants.

#### Attempted realistic environment (houses, trees) — reverted
- Downloaded House 1/2/3 and Oak/Pine Tree models from Gazebo Fuel and added
  them to the world SDF.
- **Problem:** House models use Ogre1 `<script>` materials in both SDF and DAE
  files. Gazebo Harmonic (Ogre2) cannot render these — models appeared solid black.
- **Attempted fix:** Injected PBR `<albedo_map>` entries into DAE files.
  Did not resolve the issue (mesh-embedded materials override SDF materials).
- **Decision:** Reverted to plain ground plane. Houses/trees removed from world
  and Fuel cache cleaned. Focus on car targets for detection experiments.

---

## Day 7 — 2026-04-07 (Week 2 continued — Novel Elements)

**Goal:** Implement all 3 novel elements, ablation baselines, multi-class spawning, SwarmMode message.

### Completed

| Task | Status | Notes |
|------|--------|-------|
| `SwarmMode.msg` ROS 2 message | Done | EXPLORE/EXPLOIT mode, exploit_drone_ids, event_class, event_position, event_confidence |
| `config/adaptive_params.yaml` | Done | Class priorities (person:0.9, vehicle:0.6, fire:1.0), formation configs, exploit thresholds |
| `voronoi_utils.py` updated | Done | `DensityMap` supports `confidence_weighted` flag; `DetectionEvent` carries confidence, class_priority, class_name |
| `response_selector.py` created | Done | Class-specific formations: person→cluster(2,3m), vehicle→chain(2,8m), fire→perimeter(3,10m) |
| `voronoi_coordinator.py` rewritten | Done | Dual-mode explore/exploit, ExploitEvent tracking, SwarmMode publishing, ResponseSelector integration |
| `baselines/binary_voronoi.py` created | Done | Ablation: Voronoi with binary density (weight=1.0 regardless of confidence/class) |
| `baselines/all_converge.py` created | Done | Ablation: confidence-weighted Voronoi but all drones converge (no split-and-reform) |
| `spawn_targets.sh` updated | Done | `--classes person,vehicle,fire`, `--count N`, `--random`, `--seed S`; person + fire Gazebo models |
| `data_logger.py` updated | Done | Subscribes to `/swarm/mode`, logs to `swarm_mode.csv` |
| `metrics_compute.py` updated | Done | New `coverage_during_event` metric from `swarm_mode.csv` |
| `run_experiment.sh` updated | Done | Added `binary_voronoi`, `all_converge` methods; `--classes` flag |
| `run_all_experiments.sh` updated | Done | Added E5 ablation study experiment batch |
| `run_adaptive.sh` updated | Done | Defaults updated (area=200m, altitude=40m, sigma=15m); `--classes`/`--targets` flags |
| `CMakeLists.txt` updated | Done | Added `SwarmMode.msg` to swarm_msgs build |
| Documentation updated | Done | `docs/custom_messages.md` (new), `docs/baselines.md`, `docs/progress_report.md`, `docs/week_2_task_report.md`, `docs/week_2_end2end_test_deliverables.md` |

### Novel elements implemented

**Novel Element 1 — Multi-class detection → heterogeneous swarm response:**
- `response_selector.py`: takes (class, confidence, drone_positions) → formation waypoints
- person → tight cluster (2 nearest drones, 3m radius)
- vehicle/car/truck → tracking chain (2 drones, 8m separation)
- fire → wide perimeter (3 drones, 10m radius equilateral triangle)
- Class priorities loaded from `config/adaptive_params.yaml`

**Novel Element 2 — Confidence-weighted Voronoi density function:**
- `density(x) = baseline + Σ (conf_i × priority_i × Gaussian(x, pos_i, σ)) × decay`
- `DensityMap.confidence_weighted` flag: True = weighted, False = binary (ablation)
- Binary mode used by `binary_voronoi.py` ablation baseline

**Novel Element 3 — Dual-mode exploration/exploitation with split-and-reform:**
- On high-confidence detection: K nearest drones → exploit set (fly formation)
- Remaining drones → explore set (continue Voronoi coverage)
- Reform after timeout (30s default) or confidence drop below threshold (0.2)
- `SwarmMode.msg` published on mode transitions
- `all_converge.py` ablation baseline: same weighting, no split

---

## Files Created / Modified

### Week 2 new files
| File | Purpose |
|------|---------|
| `scripts/plot_paths.py` | Flight path visualization with targets and detections |
| `scripts/response_selector.py` | Class-specific formation generator (cluster, chain, perimeter) |
| `scripts/baselines/binary_voronoi.py` | Ablation: Voronoi with binary density (no confidence weighting) |
| `scripts/baselines/all_converge.py` | Ablation: confidence-weighted Voronoi, all drones converge |
| `config/adaptive_params.yaml` | Algorithm parameters, class priorities, formation configs |
| `ros2_ws/src/swarm_msgs/msg/SwarmMode.msg` | ROS 2 message: explore/exploit mode transitions |
| `worlds/default.sdf` | Custom Gazebo world (no shadows, world name: `swarm_default`) |
| `docs/custom_messages.md` | Documentation: all 3 custom ROS 2 messages |
| `docs/gazebo_disable_shadows.md` | Documentation: disabling shadows in Gazebo Harmonic |
| `docs/gazebo_fuel_targets.md` | Documentation: Gazebo Fuel models for YOLO detection |

### Week 2 modified files
| File | Change |
|------|--------|
| `scripts/voronoi_coordinator.py` | Rewritten: dual-mode explore/exploit, SwarmMode, class-specific response |
| `scripts/voronoi_utils.py` | DensityMap: confidence × priority weighting, DetectionEvent expanded |
| `scripts/spawn_targets.sh` | Multi-class spawning: `--classes person,vehicle,fire`, `--random`, `--seed` |
| `scripts/data_logger.py` | Fixed coordinates + SwarmMode logging to `swarm_mode.csv` |
| `scripts/metrics_compute.py` | Added coverage-during-event metric from `swarm_mode.csv` |
| `scripts/run_experiment.sh` | Added: binary_voronoi, all_converge methods; `--classes` flag |
| `scripts/run_all_experiments.sh` | Added: E5 ablation study experiment batch |
| `scripts/run_adaptive.sh` | Updated defaults (200m, 40m, 15m sigma); `--classes`/`--targets` flags |
| `scripts/launch_multi_drone.sh` | Added: custom world symlink + PX4_GZ_WORLD env var |
| `ros2_ws/src/swarm_msgs/CMakeLists.txt` | Added SwarmMode.msg to build |

### All files (cumulative)
| File | Day | Purpose |
|------|-----|---------|
| `scripts/launch_multi_drone.sh` | 2 | Launches N drones + custom world |
| `scripts/multi_drone_waypoints.py` | 2 | MAVSDK: arm, takeoff, waypoint, RTL |
| `scripts/yolo_detector.py` | 3 | YOLOv8 inference (print only, superseded) |
| `scripts/spawn_targets.sh` | 3/6/7 | Multi-class target spawning (person, vehicle, fire) |
| `scripts/run_day3.sh` | 3 | Day 3 pipeline orchestration |
| `scripts/detection_publisher.py` | 4 | YOLOv8 + Detection message publisher |
| `scripts/waypoint_executor.py` | 4 | MAVSDK waypoint control + position pub |
| `scripts/voronoi_utils.py` | 4/7 | Voronoi algorithms + GPS + confidence-weighted density |
| `scripts/voronoi_coordinator.py` | 4/7 | Adaptive coverage: dual-mode explore/exploit + formations |
| `scripts/response_selector.py` | 7 | Class-specific formation generator |
| `scripts/run_adaptive.sh` | 5/6/7 | Full adaptive pipeline launcher |
| `scripts/data_logger.py` | 5/6/7 | CSV logging (positions, detections, waypoints, swarm_mode) |
| `scripts/metrics_compute.py` | 5/7 | Offline metrics + coverage-during-event |
| `scripts/baselines/static_grid.py` | 5 | Baseline: fixed grid |
| `scripts/baselines/lawnmower.py` | 5 | Baseline: sweep pattern |
| `scripts/baselines/random_waypoints.py` | 5 | Baseline: random waypoints |
| `scripts/baselines/binary_voronoi.py` | 7 | Ablation: binary Voronoi (no confidence weighting) |
| `scripts/baselines/all_converge.py` | 7 | Ablation: all-converge (no split-and-reform) |
| `scripts/run_experiment.sh` | 5/6/7 | Single trial runner (6 methods) |
| `scripts/run_all_experiments.sh` | 5/7 | Batch experiment runner (E1–E5) |
| `scripts/plot_paths.py` | 6 | Flight path visualization |
| `config/adaptive_params.yaml` | 7 | Algorithm parameters + class priorities + formations |
| `worlds/default.sdf` | 6 | Custom Gazebo world (no shadows) |
| `ros2_ws/src/swarm_msgs/msg/Detection.msg` | 4 | Custom detection message |
| `ros2_ws/src/swarm_msgs/msg/TargetWaypoint.msg` | 4 | Custom waypoint message |
| `ros2_ws/src/swarm_msgs/msg/SwarmMode.msg` | 7 | Explore/exploit mode transitions |
| `ros2_ws/src/swarm_bringup/launch/camera_bridge.launch.py` | 2 | Gazebo→ROS 2 camera bridge |
| `ros2_ws/src/swarm_bringup/config/camera_bridge.yaml` | 2 | Static topic mappings (3 drones) |
| `adaptive_area_monitoring.md` | 4 | 5-week research plan |
| `docs/baselines.md` | 5/7 | Baseline + ablation controller documentation |
| `docs/custom_messages.md` | 7 | Custom ROS 2 messages documentation |
| `docs/gazebo_disable_shadows.md` | 6 | Shadow removal documentation |
| `docs/gazebo_fuel_targets.md` | 6 | Fuel model targets documentation |
| `docs/progress_report.md` | 1 | This file |
| `docs/week_1_end2end_test_deliverables.md` | 5 | Week 1 E2E test guide |
| `docs/week_2_end2end_test_deliverables.md` | 7 | Week 2 E2E test guide |
| `docs/week_2_task_report.md` | 7 | Week 2 detailed task breakdown |

---

## System Architecture

```
┌─────────────────────────────────────────────────────────┐
│              voronoi_coordinator.py                      │
│                                                         │
│  Sub: /droneN/detection, /droneN/position               │
│  Pub: /droneN/target_waypoint, /swarm/mode              │
│                                                         │
│  Algorithm:                                             │
│    1. Confidence-weighted Voronoi + Lloyd's relax       │
│    2. Density: base + Σ(conf × priority × Gaussian)     │
│    3. Class-specific response (response_selector.py):   │
│       person → cluster, vehicle → chain, fire → perim  │
│    4. Dual-mode: exploit set (formation) +              │
│       explore set (Voronoi coverage)                    │
└──────────┬─────────────────────┬────────────────────────┘
           │                     │
           ▼                     ▼
┌──────────────────┐  ┌───────────────────┐
│ detection_pub.py │  │ wp_executor.py    │
│ Sub: image_raw   │  │ Sub: target_wp    │
│ Pub: /detection  │  │ Pub: /position    │
│ YOLOv8 + class   │  │ MAVSDK goto_loc() │
│ + confidence     │  │ + telemetry 2 Hz  │
│ + world position │  │                   │
└──────────────────┘  └───────────────────┘

┌──────────────────┐  ┌───────────────────┐
│ data_logger.py   │  │ baselines/        │
│ Sub: all topics  │  │  static_grid.py   │
│ + /swarm/mode    │  │  lawnmower.py     │
│ Out: CSV files   │  │  random_wps.py    │
│ + swarm_mode.csv │  │  binary_voronoi.py│ (ablation)
└──────────────────┘  │  all_converge.py  │ (ablation)
                      └───────────────────┘
┌──────────────────┐
│ plot_paths.py    │  config/adaptive_params.yaml
│ In: CSV logs     │  → class priorities, formations,
│ Out: PNG plots   │    exploit thresholds
└──────────────────┘
```

---

## Pending (Week 2, Days 12–17)

- [ ] Day 12: Edge case handling + parameter tuning
- [ ] Day 13: Dry run E1 (baselines) + E5 (ablations) — 2 trials each method
- [ ] Day 14: Dry run E2 + E3 (scalability + target density)
- [ ] Day 15: Response time instrumentation for E4
- [ ] Days 16–17: Literature review (20 papers, BibTeX file)

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
| ros-humble-ros-gzharmonic | apt |
| Ultralytics YOLOv8 | latest pip |
