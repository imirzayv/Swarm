# Week 1 End-to-End Test Guide

Step-by-step guide to test all Week 1 deliverables. Each test builds on the
previous one. You need **7 terminal windows** for the full adaptive pipeline test.

**Environment:** Ubuntu 22.04, `conda activate swarm` (Python 3.10)

---

## Prerequisites

Before starting, verify:

```bash
# 1. PX4 firmware is built
ls ~/Desktop/Swarm/firmware/build/px4_sitl_default/bin/px4
# Should exist

# 2. Conda swarm env works
conda activate swarm
python3 --version
# Should print: Python 3.10.x

# 3. ROS 2 + workspace sourced
source /opt/ros/humble/setup.bash
source ~/Desktop/Swarm/ros2_ws/install/setup.bash
ros2 interface show swarm_msgs/msg/Detection
# Should print the message definition

# 4. YOLOv8 model available
ls ~/Desktop/Swarm/yolov8n.pt
# Should exist
```

If `swarm_msgs` is not found, rebuild:
```bash
conda activate swarm
source /opt/ros/humble/setup.bash
cd ~/Desktop/Swarm/ros2_ws
colcon build --packages-select swarm_msgs
source install/setup.bash
```

---

## Test 1: Simulation Launch (Terminal 1)

**What it tests:** PX4 SITL multi-drone spawning in Gazebo Harmonic.

```bash
cd ~/Desktop/Swarm
./scripts/launch_multi_drone.sh 3 x500_mono_cam_down
```

**Expected:** 3 gnome-terminal windows open (Drone-1, Drone-2, Drone-3). Gazebo
window appears with 3 quadcopters on the ground.

**Wait:** ~60 seconds for all drones to show `Ready for takeoff!` in their terminals.

**Verify:**
```bash
# In a separate terminal
gz model --list
# Should show: x500_mono_cam_down_1, x500_mono_cam_down_2, x500_mono_cam_down_3
```

---

## Test 2: Micro XRCE-DDS Agent (Terminal 2)

**What it tests:** PX4 ↔ ROS 2 communication bridge.

```bash
MicroXRCEAgent udp4 -p 8888
```

**Expected:** Agent connects to PX4 instances. You should see client connection
messages.

**Verify:**
```bash
# In another terminal
source /opt/ros/humble/setup.bash
ros2 topic list | grep px4
# Should show /px4_1/fmu/*, /px4_2/fmu/*, /px4_3/fmu/* topics
```

---

## Test 3: Camera Bridge (Terminal 3)

**What it tests:** Gazebo camera images bridged to ROS 2 topics.

```bash
conda activate swarm
source /opt/ros/humble/setup.bash
source ~/Desktop/Swarm/ros2_ws/install/setup.bash
ros2 launch ~/Desktop/Swarm/ros2_ws/src/swarm_bringup/launch/camera_bridge.launch.py
```

**Expected:** Bridge node starts, no errors.

**Verify:**
```bash
source /opt/ros/humble/setup.bash
ros2 topic hz /drone1/camera/image_raw
# Should show ~30 Hz
```

---

## Test 4: Target Spawning (Terminal 1, one-shot)

**What it tests:** Dynamic object spawning into Gazebo world.

```bash
cd ~/Desktop/Swarm
./scripts/spawn_targets.sh
```

**Expected:** 5 lines like `[Spawn] target_red_box at (5, 3, 0.4)` with no errors.
Colored objects should appear on the ground plane in Gazebo.

**Verify:** Look in the Gazebo 3D view — you should see red, blue, green, yellow,
and white objects scattered within a ~20m area.

---

## Test 5: Waypoint Executor (Terminal 4)

**What it tests:** MAVSDK drone control + ROS 2 position publishing.

```bash
conda activate swarm
source /opt/ros/humble/setup.bash
source ~/Desktop/Swarm/ros2_ws/install/setup.bash
cd ~/Desktop/Swarm
python3 scripts/waypoint_executor.py --drones 1 2 3 --altitude 30
```

**Expected:**
```
[Drone 1] Connecting on udp://:14541 ...
[Drone 1] Connected
[Drone 1] GPS OK
[Drone 1] Arming...
[Drone 1] Taking off to 30.0m...
[Drone 1] Airborne
```
Repeated for drones 2 and 3. All 3 drones should take off in Gazebo.

**Verify:**
```bash
source /opt/ros/humble/setup.bash
source ~/Desktop/Swarm/ros2_ws/install/setup.bash
ros2 topic echo /drone1/position --once
# Should show: drone_id, latitude, longitude, altitude fields
```

---

## Test 6: Detection Publisher (Terminal 5)

**What it tests:** YOLOv8 inference on drone camera feeds + Detection message publishing.

```bash
conda activate swarm
source /opt/ros/humble/setup.bash
source ~/Desktop/Swarm/ros2_ws/install/setup.bash
cd ~/Desktop/Swarm
python3 scripts/detection_publisher.py --drones 1 2 3 --confidence 0.15 --altitude 30
```

**Expected:**
```
Loading YOLO model: yolov8n.pt
YOLO model loaded
Drone 1: /drone1/camera/image_raw → /drone1/detection
...
Detection publisher ready — 3 drone(s), conf=0.15, altitude=30.0m
```

If drones are flying over targets, you should see detection lines:
```
[Drone 1] Frame #50 — 2 detection(s):
  → suitcase (0.32) bbox=[...] pos=(3.2, -1.5)m
```

If no targets are below the cameras, you will see periodic status messages:
```
[Drone 1] Frame #50 — no detections (total: 0)
```

**Verify:**
```bash
source /opt/ros/humble/setup.bash
source ~/Desktop/Swarm/ros2_ws/install/setup.bash
ros2 topic echo /drone1/detection
# Should show Detection messages when targets are detected
```

---

## Test 7: Voronoi Coordinator (Terminal 6)

**What it tests:** Adaptive Voronoi coverage algorithm.

```bash
conda activate swarm
source /opt/ros/humble/setup.bash
source ~/Desktop/Swarm/ros2_ws/install/setup.bash
cd ~/Desktop/Swarm
python3 scripts/voronoi_coordinator.py --drones 1 2 3 --area-size 40 --altitude 30
```

**Expected:**
```
Voronoi coordinator ready — 3 drones, area=40.0m, update=0.5Hz, ...
```

Once drone positions are received, the coordinator computes Voronoi cells every
2 seconds and publishes waypoints. On a detection event:
```
[Detection] suitcase (0.32) from drone 1 at global (5.2, 3.1)m — active detections: 1
```

**Verify:**
```bash
source /opt/ros/humble/setup.bash
source ~/Desktop/Swarm/ros2_ws/install/setup.bash
ros2 topic echo /drone1/target_waypoint
# Should show waypoints being published every 2 seconds
```

**Expected behavior in Gazebo:** Drones should spread out to cover the area.
When a drone detects a target, nearby drones shift toward it. After ~30 seconds
(decay half-life), they relax back toward uniform coverage.

---

## Test 8: Data Logger (Terminal 7)

**What it tests:** CSV logging of all experiment data.

```bash
conda activate swarm
source /opt/ros/humble/setup.bash
source ~/Desktop/Swarm/ros2_ws/install/setup.bash
cd ~/Desktop/Swarm
python3 scripts/data_logger.py --drones 1 2 3 --experiment-id week1_test --duration 60
```

**Expected:**
```
Data logger ready — 3 drones, output: .../data/logs/week1_test, duration: 60s
[10s] Logged: 60 positions, 3 detections, 15 waypoints
...
[60s] Duration (60s) reached — stopping logger
Final counts: 360 positions, 12 detections, 90 waypoints
Data saved to: .../data/logs/week1_test
```

**Verify:**
```bash
ls ~/Desktop/Swarm/data/logs/week1_test/
# Should contain: positions.csv, detections.csv, waypoints.csv

head -5 ~/Desktop/Swarm/data/logs/week1_test/positions.csv
# Should show CSV header + data rows
```

---

## Test 9: Metrics Computation (after Test 8 completes)

**What it tests:** Offline metric analysis from CSV logs.

```bash
conda activate swarm
cd ~/Desktop/Swarm
python3 scripts/metrics_compute.py --experiment-id week1_test
```

**Expected:**
```
Loading data from: .../data/logs/week1_test
  Positions: 360 rows
  Detections: 12 rows

══════════════════════════════════════
  Experiment: week1_test
  Duration: 58.5s
══════════════════════════════════════
  Coverage:      42.3%
  Redundancy:    0.523
  Total distance:125.4m
  Detections:    12
  First detect:  8.3s
══════════════════════════════════════
  Saved: .../metrics_summary.json
  Saved: .../coverage_over_time.csv
```

(Exact numbers will vary.)

**Verify:**
```bash
cat ~/Desktop/Swarm/data/logs/week1_test/metrics_summary.json
# Should be valid JSON with all metrics
```

---

## Test 10: Baselines (replace coordinator in Test 7)

Stop the Voronoi coordinator (Ctrl+C in Terminal 6), then test each baseline.
Keep all other terminals (4, 5, 7) running.

### 10a: Static Grid

```bash
# Terminal 6
conda activate swarm
source /opt/ros/humble/setup.bash
source ~/Desktop/Swarm/ros2_ws/install/setup.bash
cd ~/Desktop/Swarm
python3 scripts/baselines/static_grid.py --drones 1 2 3 --area-size 40 --altitude 30
```

**Expected:** Drones move to fixed grid positions and hold.

### 10b: Lawnmower

```bash
# Stop static_grid (Ctrl+C), then:
python3 scripts/baselines/lawnmower.py --drones 1 2 3 --area-size 40 --altitude 30
```

**Expected:** Drones sweep back-and-forth in vertical strips. Log messages like:
```
[Drone 1] Arrived at wp 0, advancing to 1 (...)
```

### 10c: Random Waypoints

```bash
# Stop lawnmower (Ctrl+C), then:
python3 scripts/baselines/random_waypoints.py --drones 1 2 3 --area-size 40 --seed 42
```

**Expected:** Drones fly to random positions, pick new ones on arrival:
```
[Drone 2] Arrived, new target: (12, -8)m
```

---

## Test 11: Automated Experiment (all-in-one)

**What it tests:** Full experiment automation — launch, log, compute metrics.

Stop all terminals from previous tests first:
```bash
pkill -f "bin/px4" 2>/dev/null
pkill -f "gz sim" 2>/dev/null
pkill -f waypoint_executor 2>/dev/null
pkill -f detection_publisher 2>/dev/null
pkill -f voronoi_coordinator 2>/dev/null
sleep 3
```

Then run a single automated trial:

```bash
cd ~/Desktop/Swarm
./scripts/run_experiment.sh \
    --method adaptive \
    --drones 3 \
    --targets 5 \
    --duration 120 \
    --trial 1
```

**Expected:** Script launches everything automatically, runs for 120 seconds,
logs data, computes metrics, and cleans up. Final output:

```
══════════════════════════════════════════════════════════════
  Trial complete: adaptive_d3_t5_trial01
  Results: data/logs/adaptive_d3_t5_trial01/metrics_summary.json
══════════════════════════════════════════════════════════════
```

**Verify:**
```bash
cat ~/Desktop/Swarm/data/logs/adaptive_d3_t5_trial01/metrics_summary.json
```

---

## Test 12: Full Pipeline via run_adaptive.sh

Alternative to manual terminal setup. Launches everything in one command:

```bash
cd ~/Desktop/Swarm
./scripts/run_adaptive.sh --drones 3
```

Or if simulation is already running:
```bash
./scripts/run_adaptive.sh --no-sim --drones 3
```

**Expected:** Opens 4 gnome-terminal windows (Camera Bridge, Waypoint Executor,
Detection Publisher, Voronoi Coordinator). Monitor the Voronoi Coordinator
terminal for detection events and waypoint updates.

---

## Quick Shutdown

To kill all project processes:

```bash
pkill -f "bin/px4"
pkill -f "gz sim"
pkill -f waypoint_executor
pkill -f detection_publisher
pkill -f voronoi_coordinator
pkill -f data_logger
pkill -f static_grid
pkill -f lawnmower
pkill -f random_waypoints
pkill -f parameter_bridge
```

---

## Troubleshooting

| Problem | Cause | Fix |
|---------|-------|-----|
| `ModuleNotFoundError: rclpy` | Wrong Python env | `conda activate swarm` (Python 3.10) |
| `ModuleNotFoundError: swarm_msgs` | Workspace not sourced | `source ~/Desktop/Swarm/ros2_ws/install/setup.bash` |
| Drones won't arm | No GPS fix yet | Wait 30+ seconds after PX4 startup |
| No camera data (0 Hz) | Bridge not running or wrong gz package | Verify `ros-humble-ros-gzharmonic` is installed |
| `Accel #0 TIMEOUT` | Drone startup race condition | Wait longer between drone spawns |
| `udp:// is deprecated` | MAVSDK cosmetic warning | Harmless, ignore |
| Waypoint executor hangs at "Connecting" | PX4 not running on that port | Check drone instance is alive |
| No detections | Drones not over targets, or confidence too high | Lower `--confidence` to 0.10, fly drones over target area |
| Metrics show 0% coverage | No position data logged | Ensure waypoint_executor is running and publishing `/droneN/position` |

---

## Summary Checklist

| # | Test | Deliverable Verified |
|---|------|---------------------|
| 1 | Simulation launch | Multi-drone PX4 SITL + Gazebo |
| 2 | XRCE-DDS agent | PX4 ↔ ROS 2 bridge |
| 3 | Camera bridge | Gazebo cameras → ROS 2 topics |
| 4 | Target spawning | Dynamic object placement |
| 5 | Waypoint executor | MAVSDK control + position publishing |
| 6 | Detection publisher | YOLOv8 + Detection messages |
| 7 | Voronoi coordinator | Adaptive coverage algorithm |
| 8 | Data logger | CSV logging |
| 9 | Metrics computation | Offline analysis |
| 10 | Baselines (3x) | Static grid, lawnmower, random |
| 11 | Experiment automation | Single trial runner |
| 12 | Full pipeline | run_adaptive.sh orchestration |
