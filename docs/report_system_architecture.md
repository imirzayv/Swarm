# System Architecture Report
## Vision-Guided Adaptive Area Monitoring — Software Pipeline

---

## 1. Overview

The system is a multi-drone surveillance stack built on ROS 2 Humble. Each
drone runs a downward-facing camera and an individual perception pipeline.
Detection results are published as structured ROS messages and consumed by a
central coordinator that continuously replans drone positions. A separate
executor translates those position commands into flight manoeuvres through
MAVSDK. An experiment harness wraps everything, handles launch sequencing,
and collects data.

---

## 2. Nodes and Their Roles

| Node / Script | Primary Job |
|---|---|
| `detection_publisher.py` | Per-drone perception: image → HSV detection → world-projected `Detection` message |
| `waypoint_executor.py` | MAVSDK flight control: receives `TargetWaypoint` → calls `goto_location()` on real PX4 SITL |
| `voronoi_coordinator.py` | Swarm brain: reads positions + detections → Voronoi partition → publishes `TargetWaypoint` |
| `data_logger.py` | Passive observer: records all topics to CSV for offline analysis |
| `metrics_compute.py` | Offline postprocessor: reads CSV → computes coverage %, energy, latency, etc. |
| `run_experiment.sh` | Experiment executor: sequences every node's launch for a single trial |
| `run_all_experiments.sh` | Experiment controller: iterates over the full experiment battery (S1/S2/D1–D3) |

---

## 3. Drone Perception Pipeline

### 3.1 Camera Feed

Gazebo publishes synthetic camera images from the `x500_mono_cam_down` model
(1280×960 px, 100° HFOV, nadir-pointing). A `ros_gz_bridge` node forwards
these images onto ROS 2 topics:

```
/droneN/camera/image_raw  (sensor_msgs/Image)
```

The bridge config is generated dynamically by `run_experiment.sh` for however
many drones are in the trial.

### 3.2 Attitude and Position Telemetry

`waypoint_executor.py` streams live telemetry from PX4 via MAVSDK and
republishes it on ROS 2:

| Topic | Type | Rate | Content |
|---|---|---|---|
| `/droneN/attitude` | `geometry_msgs/QuaternionStamped` | 30 Hz | body→NED quaternion |
| `/droneN/position` | `swarm_msgs/TargetWaypoint` | 2 Hz | GPS lat/lon + relative AGL altitude |

### 3.3 Detection Publisher (`detection_publisher.py`)

**Why HSV instead of YOLOv8:** Gazebo's flat-shaded synthetic primitives
(solid-color boxes) are outside COCO's training distribution, causing YOLOv8
to hallucinate class names or miss targets entirely. HSV thresholds for the
three simulated target colors (red=person, blue=vehicle, orange=fire) are
100% reliable for these deterministic simulation conditions. Because the
downstream pipeline consumes `swarm_msgs/Detection`, not raw imagery,
switching to a CNN detector later is a one-file change.

**Per-drone architecture:**

```
ROS callback: /droneN/camera/image_raw  ──► image_queue (deque, maxlen=8)
ROS callback: /droneN/attitude           ──► attitude_queue (deque, maxlen=256)
ROS callback: /droneN/position           ──► _altitudes dict (live AGL update)

Background processor thread (one per node):
  1. Pop oldest image from image_queue
  2. Look up attitude that brackets the image timestamp
       → SLERP between before/after samples
       → fallback to nearest sample within 100 ms
  3. Run HSV color segmentation on the frame
  4. For each detected blob:
       a. Compute bbox center (cx, cy) in pixels
       b. Build camera ray:  ray_cam = [(cx - W/2)/fx,  (cy - H/2)/fy,  1]
       c. Rotate into body frame:  ray_body = R_BODY_CAM @ ray_cam
       d. Rotate into NED frame:  ray_ned  = R(q) @ ray_body
       e. Project onto ground plane at live altitude:
              t = altitude / ray_ned[2]
              north_off = t * ray_ned[0]
              east_off  = t * ray_ned[1]
       f. Publish swarm_msgs/Detection with [east_off, north_off, 0] as
          world_position (ENU, drone-relative)
```

**Output topic (one per drone):**

```
/droneN/detection  (swarm_msgs/Detection)
  .drone_id         int
  .class_name       str   ("person" | "vehicle" | "fire")
  .confidence       float  (area-normalised, 0–1)
  .bbox             float[4]  (pixel bounding box)
  .world_position   float[3]  (east_off, north_off, 0 — meters, ENU, drone-relative)
  .stamp            builtin_interfaces/Time
```

---

## 4. How Detection Results Are Used

### 4.1 Received by: Voronoi Coordinator

`voronoi_coordinator.py` subscribes to `/droneN/detection` for every drone.
On each message it:

1. **Converts the drone-relative offset to a global XY coordinate:**
   ```
   global_x = drone_x + detection.world_position[0]   # east
   global_y = drone_y + detection.world_position[1]   # north
   ```
2. **Adds a Gaussian bump to the density map** at `(global_x, global_y)`:
   - Weight = `confidence × class_priority`
   - Class priorities from `config/adaptive_params.yaml`:
     fire=1.0, person=0.9, vehicle=0.6
   - The bump decays exponentially with a 30 s half-life
3. **Checks for exploit trigger:**
   - If `confidence >= 0.4` AND the detection is not within 20 m of an
     already-active exploit, a new exploit event is opened.
   - The coordinator selects the N nearest free drones (N depends on class)
     and computes a class-specific formation around the detection point.
   - A `SwarmMode` message is published to `/swarm/mode` signalling which
     drones are now in EXPLOIT mode.

### 4.2 Received by: Data Logger

`data_logger.py` subscribes to `/droneN/detection` and appends each event
to `detections.csv` with a global XY coordinate computed from the drone's
last known position. This CSV feeds `metrics_compute.py` for offline analysis.

### 4.3 Used by: Density Map → Voronoi Weighted Centroid

In EXPLORE mode, the coordinator's periodic Voronoi update (0.5 Hz) calls
`weighted_centroid()` on each Voronoi cell. This Monte Carlo integrator
samples the density function `DensityMap(x, y)`:

```
density(x,y) = baseline
             + Σ over detections:
                 detection_weight
                 × confidence × class_priority   (novel weighting)
                 × decay(age)
                 × gaussian(dist to detection)
```

The centroid of high-density regions becomes the next waypoint for that
drone's Voronoi cell, attracting exploring drones toward areas of interest
without abandoning overall coverage.

---

## 5. Full Topic Map

```
Gazebo Simulation
  /droneN/camera/image_raw  ──────────────────────────┐
                                                       ▼
                                          detection_publisher.py
  /droneN/attitude  ─────────────────────►  (HSV + ray projection)
  /droneN/position  ─────────────────────►
                                                       │
                                           /droneN/detection
                                                  │            │
                                                  ▼            ▼
                                       voronoi_coordinator  data_logger
                                       .py                   .py
                                          │                   │
                     ┌────────────────────┤                   ▼
                     ▼                    ▼              detections.csv
              /swarm/mode    /droneN/target_waypoint
                  │                   │
                  ▼                   ▼
           data_logger      waypoint_executor.py
              .py                 (MAVSDK)
                  │                   │
           swarm_mode.csv             ▼
                             PX4 SITL → drone moves
                                       │
                             /droneN/position
                             /droneN/attitude
                                       │
                            ┌──────────┴──────────┐
                            ▼                     ▼
                 voronoi_coordinator          detection_publisher
                 (position updates)          (altitude updates)
```

---

## 6. Experiment Controller vs. Experiment Executor

These two terms refer to different layers of the launch hierarchy.

### Experiment Controller (`run_all_experiments.sh`)

The **controller** operates at the battery level. It knows nothing about
individual ROS nodes — it only knows which experiment series to run (S1, S2,
D1–D3), how many drones and targets each uses, how many trials to run, and
whether to resume from previous results.

Responsibilities:
- Iterates over experiments × methods × trials (the outer loop)
- Looks up per-experiment config (drone count, schedule, target classes,
  duration, trial count) from hardcoded `CFG_*` tables
- Calls `run_experiment.sh` once per trial with the right arguments
- Waits 10 seconds between trials ("cooling down")
- Supports `--resume` (skip trials that already have `metrics_summary.json`),
  `--dry-run`, and `--experiments S1 D1 ...` to run a subset

### Experiment Executor (`run_experiment.sh`)

The **executor** operates at the single-trial level. It orchestrates the
full node lifecycle for one trial run.

Responsibilities (sequential steps):

| Step | Action |
|---|---|
| 1 | Launch multi-drone Gazebo simulation via `launch_multi_drone.sh` |
| 2 | Start `ros_gz_bridge` with a dynamically generated camera bridge YAML |
| 3 | Spawn target objects (all at once for static experiments; deferred + timed for dynamic) |
| 4 | Start `waypoint_executor.py` (arms drones, takes off, listens for waypoints) |
| 5 | Start `detection_publisher.py` (begins processing camera feeds) |
| 6 | Start the **controller node** (voronoi/lawnmower/static/etc.) + `data_logger.py` |
| 7 (post) | Run `metrics_compute.py` offline, then `plot_paths.py` |

The executor enforces a strict startup sequence with `sleep` gaps because
MAVSDK drone connections are stateful: the waypoint executor must arm and
take off before the detection publisher tries to use altitude telemetry, and
the coordinator must not publish waypoints before drones are airborne.

### Relationship Summary

```
run_all_experiments.sh  (controller)
    │
    └──► run_experiment.sh  (executor, one per trial)
              │
              ├── launch_multi_drone.sh    (Gazebo + PX4 SITL)
              ├── ros_gz_bridge            (camera bridge)
              ├── spawn_targets.sh         (world population)
              ├── waypoint_executor.py     (flight control)
              ├── detection_publisher.py   (perception)
              ├── [voronoi / lawnmower / static / ...]  (coordinator)
              ├── data_logger.py           (logging, blocks until done)
              ├── metrics_compute.py       (offline analysis)
              └── plot_paths.py            (offline visualization)
```

The controller decides **what** to run and **how many times**.
The executor decides **how** to run it — node ordering, timing, cleanup,
and postprocessing.

---

## 7. Baseline Controllers vs. Adaptive Controller

The executor's `--method` argument selects which coordinator node replaces
`voronoi_coordinator.py`. All baselines publish to the same
`/droneN/target_waypoint` topics and read the same `/droneN/position` topics,
so the rest of the stack (executor, detector, logger) is identical across
methods.

| Method | Script | Behavior |
|---|---|---|
| `adaptive` | `voronoi_coordinator.py` | Weighted Voronoi + EXPLOIT mode (the proposed algorithm) |
| `lawnmower` | `baselines/lawnmower.py` | FOV-tiled boustrophedon sweep per drone strip |
| `static` | `baselines/static_grid.py` | Fixed grid waypoints, no replanning |
| `random` | `baselines/random_waypoints.py` | Random waypoints within bounds |
| `binary_voronoi` | `baselines/binary_voronoi.py` | Unweighted Voronoi (ablation: no confidence weighting) |
| `all_converge` | `baselines/all_converge.py` | All drones converge on every detection |
| `pso` | `baselines/pso_coverage.py` | Particle swarm optimisation coverage |
| `apf` | `baselines/apf_coverage.py` | Artificial potential field coverage |
