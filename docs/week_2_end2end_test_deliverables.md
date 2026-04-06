# Week 2 End-to-End Test Guide

Step-by-step guide to verify all Week 2 deliverables. Tests build on the
Week 1 foundation — all Week 1 tests should pass first.

**Environment:** Ubuntu 22.04, `conda activate swarm` (Python 3.10)

---

## Prerequisites

```bash
# 1. All Week 1 prerequisites still hold
ls ~/Desktop/Swarm/firmware/build/px4_sitl_default/bin/px4

# 2. Conda swarm env
conda activate swarm
python3 --version  # Python 3.10.x

# 3. ROS 2 + workspace
source /opt/ros/humble/setup.bash
source ~/Desktop/Swarm/ros2_ws/install/setup.bash
ros2 interface show swarm_msgs/msg/Detection

# 4. YOLOv8 model
ls ~/Desktop/Swarm/yolov8n.pt

# 5. Custom world symlink exists
ls ~/Desktop/Swarm/firmware/Tools/simulation/gz/worlds/swarm_default.sdf
# If missing:
ln -sf ~/Desktop/Swarm/worlds/default.sdf \
    ~/Desktop/Swarm/firmware/Tools/simulation/gz/worlds/swarm_default.sdf
```

---

## Test 1: Custom Gazebo World (No Shadows)

**Deliverable:** `worlds/default.sdf` — shadows disabled, world name `swarm_default`.

### 1a. Verify world file

```bash
grep -c "shadows.*false" ~/Desktop/Swarm/worlds/default.sdf
# Should print: 1

grep -c "cast_shadows.*false" ~/Desktop/Swarm/worlds/default.sdf
# Should print: 1

grep 'world name=' ~/Desktop/Swarm/worlds/default.sdf
# Should show: <world name="swarm_default">
```

### 1b. Verify symlink

```bash
ls -la ~/Desktop/Swarm/firmware/Tools/simulation/gz/worlds/swarm_default.sdf
# Should be a symlink pointing to ~/Desktop/Swarm/worlds/default.sdf
```

### 1c. Launch simulation with custom world

```bash
cd ~/Desktop/Swarm
./scripts/launch_multi_drone.sh 1 x500_mono_cam_down
```

**Expected:**
- Gazebo window opens with a **light grey ground plane**.
- Terminal shows: `Using custom world: swarm_default (shadows disabled)`
- **No shadows visible** under the drone.
- Drone terminal shows `Ready for takeoff!` after ~20s.

**Verify world loaded correctly:**
```bash
gz world --list
# Should show: swarm_default
```

**Cleanup:**
```bash
pkill -f "bin/px4"; pkill -f "gz sim"; sleep 3
```

---

## Test 2: Gazebo Fuel Vehicle Targets

**Deliverable:** `scripts/spawn_targets.sh` — 5 realistic vehicle models from Gazebo Fuel.

### 2a. Launch simulation first (keep running for Tests 2–5)

```bash
cd ~/Desktop/Swarm
./scripts/launch_multi_drone.sh 1 x500_mono_cam_down
# Wait for "Ready for takeoff!" in drone terminal (~25s)
```

### 2b. Spawn targets

```bash
cd ~/Desktop/Swarm
./scripts/spawn_targets.sh swarm_default 5
```

**Expected output:**
```
=== Spawning 5 vehicle targets into Gazebo world: swarm_default ===
Checking Fuel model cache...
Models ready.
[Spawn] target_suv_1 at (70, -60, 0.1) yaw=0
[Spawn] target_hatchback_red_2 at (-80, 70, 0.1) yaw=1.57
[Spawn] target_hatchback_blue_3 at (50, 80, 0.1) yaw=0.78
[Spawn] target_pickup_4 at (-60, -50, 0.1) yaw=2.35
[Spawn] target_hatchback_5 at (85, -30, 0.1) yaw=1.05
=== Done! 5 vehicles spawned ===
```

**Verify in Gazebo:**
- Navigate camera to coordinates (70, -60, 0). You should see the SUV model.
- All 5 vehicles should have **visible textures** (not solid black).
- Models are **car-sized** (not giant or tiny).

**Verify via CLI:**
```bash
gz model --list
# Should include: target_suv_1, target_hatchback_red_2, etc.
```

### 2c. Spawn fewer targets

```bash
./scripts/spawn_targets.sh swarm_default 3
# Should spawn only the first 3 targets (SUV, Hatchback Red, Hatchback Blue)
```

---

## Test 3: Dynamic Camera Bridge (N Drones)

**Deliverable:** Runtime YAML generation for any number of drones.

### 3a. Test with 1 drone (simulation already running from Test 2)

```bash
# Start Micro XRCE-DDS agent
MicroXRCEAgent udp4 -p 8888 &
sleep 3

conda activate swarm
source /opt/ros/humble/setup.bash
source ~/Desktop/Swarm/ros2_ws/install/setup.bash

# Generate YAML for 1 drone
cat > /tmp/test_bridge.yaml <<'YAML'
- ros_topic_name: /drone1/camera/image_raw
  gz_topic_name: /world/swarm_default/model/x500_mono_cam_down_1/link/camera_link/sensor/imager/image
  ros_type_name: sensor_msgs/msg/Image
  gz_type_name: gz.msgs.Image
  direction: GZ_TO_ROS
  lazy: false
YAML

ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=/tmp/test_bridge.yaml &
sleep 3
```

**Verify:**
```bash
ros2 topic hz /drone1/camera/image_raw
# Should show ~30 Hz
```

**Cleanup:**
```bash
pkill -f parameter_bridge
pkill -f MicroXRCEAgent
```

### 3b. Test dynamic generation for 3 drones

Restart simulation with 3 drones:
```bash
pkill -f "bin/px4"; pkill -f "gz sim"; sleep 3
cd ~/Desktop/Swarm
./scripts/launch_multi_drone.sh 3 x500_mono_cam_down
# Wait ~80s for all 3 drones
```

```bash
MicroXRCEAgent udp4 -p 8888 &
sleep 3

# Check the generated YAML
cat ~/Desktop/Swarm/data/camera_bridge_dynamic.yaml 2>/dev/null
# If running run_adaptive.sh, this file will be auto-generated.

# Alternatively, manually verify the script generates correct YAML:
cd ~/Desktop/Swarm
NUM_DRONES=3
GZ_WORLD="swarm_default"
BRIDGE_YAML="/tmp/test_bridge_3.yaml"
> "$BRIDGE_YAML"
for i in $(seq 1 $NUM_DRONES); do
    cat >> "$BRIDGE_YAML" <<YAML
- ros_topic_name: /drone${i}/camera/image_raw
  gz_topic_name: /world/${GZ_WORLD}/model/x500_mono_cam_down_${i}/link/camera_link/sensor/imager/image
  ros_type_name: sensor_msgs/msg/Image
  gz_type_name: gz.msgs.Image
  direction: GZ_TO_ROS
  lazy: false

YAML
done

cat "$BRIDGE_YAML"
# Should show entries for drone1, drone2, drone3
wc -l "$BRIDGE_YAML"
# Should be ~21 lines (7 lines per drone)
```

**Cleanup:**
```bash
pkill -f MicroXRCEAgent
```

---

## Test 4: Detection Coordinate Fix

**Deliverable:** `data_logger.py` computes global detection coordinates
(`drone_pos + offset`) instead of logging raw offsets.

### 4a. Inspect the code fix

```bash
grep -n "_last_pos" ~/Desktop/Swarm/scripts/data_logger.py
# Should show lines where _last_pos dict is defined and used
# Key logic: global_x = drone_x + dx, global_y = drone_y + dy
```

### 4b. Run a short experiment and check detection coordinates

This requires the full pipeline. Use `run_experiment.sh`:

```bash
cd ~/Desktop/Swarm
# Simulation should already be running with 3 drones from Test 3b
./scripts/run_experiment.sh \
    --method adaptive \
    --drones 3 \
    --targets 5 \
    --duration 120 \
    --trial 99 \
    --no-sim
```

**Wait for the trial to complete (~2 minutes + setup time).**

Then verify detection coordinates are near target positions:

```bash
# Check detections CSV
head -20 ~/Desktop/Swarm/data/logs/adaptive_d3_t5_trial99/detections.csv
```

**Expected:** The `world_x` and `world_y` columns should show coordinates
**near the actual target positions** (within ~20m), not random scatter:
- SUV: ~(70, -60)
- Hatchback Red: ~(-80, 70)
- Hatchback Blue: ~(50, 80)
- Pickup: ~(-60, -50)
- Hatchback: ~(85, -30)

**Red flag:** If `world_x` and `world_y` are all small values like (-5, 3) or
scattered randomly between -200 and 200, the coordinate fix is not working.

---

## Test 5: Monitoring Area (200x200m)

**Deliverable:** Area scaled from 40m to 200m in experiment scripts.

### 5a. Verify defaults

```bash
grep "AREA_SIZE=" ~/Desktop/Swarm/scripts/run_experiment.sh
# Should show: AREA_SIZE=200
```

### 5b. Verify target spread

```bash
grep "POS_X=" ~/Desktop/Swarm/scripts/spawn_targets.sh
grep "POS_Y=" ~/Desktop/Swarm/scripts/spawn_targets.sh
# X values: 70, -80, 50, -60, 85 (all within ±100m)
# Y values: -60, 70, 80, -50, -30 (all within ±100m)
```

### 5c. Verify metrics use correct area

If Trial 99 completed from Test 4:

```bash
cat ~/Desktop/Swarm/data/logs/adaptive_d3_t5_trial99/metrics_summary.json | python3 -m json.tool
# Check that coverage % is reasonable for a 200x200m area
# (will be low — ~5-15% for 3 drones in 120s over a large area)
```

---

## Test 6: Flight Path Visualization

**Deliverable:** `scripts/plot_paths.py` — generates path plots with targets and detections.

### 6a. Verify plot generation from Trial 99

If Trial 99 completed:

```bash
ls ~/Desktop/Swarm/data/logs/adaptive_d3_t5_trial99/paths_combined.png
ls ~/Desktop/Swarm/data/logs/adaptive_d3_t5_trial99/path_drone1.png
ls ~/Desktop/Swarm/data/logs/adaptive_d3_t5_trial99/path_drone2.png
ls ~/Desktop/Swarm/data/logs/adaptive_d3_t5_trial99/path_drone3.png
# All 4 files should exist
```

### 6b. Verify plot contents

Open `paths_combined.png`:
```bash
xdg-open ~/Desktop/Swarm/data/logs/adaptive_d3_t5_trial99/paths_combined.png
```

**Check:**
- [ ] Colored flight path lines for each drone (red, blue, green)
- [ ] Start markers (▲ triangles) visible at path beginnings
- [ ] End markers (■ squares) visible at path ends
- [ ] Red ✕ markers at 5 ground-truth target positions
- [ ] Gold ★ markers at detection locations (if any detections occurred)
- [ ] Legend is **outside** the plot (right side), not overlapping data
- [ ] Dashed rectangle showing the 200×200m monitoring area boundary
- [ ] Plot auto-scales to show all data (including start positions outside area)
- [ ] Axis labels: "X (m)" and "Y (m)"

### 6c. Run plot_paths.py standalone

```bash
conda activate swarm
cd ~/Desktop/Swarm
python3 scripts/plot_paths.py \
    --experiment-id adaptive_d3_t5_trial99 \
    --area-size 200 \
    --altitude 40
```

**Expected:**
```
Plotting paths for: adaptive_d3_t5_trial99
  Drones: [1, 2, 3]
  Area: 200.0m x 200.0m
  FOV radius: 36.6m (at 40.0m altitude)
  Targets: 5
  Detections: <number>
  Saved: .../paths_combined.png
  Saved: .../path_drone1.png
  ...
```

### 6d. Test with custom targets file

```bash
# Create a test targets CSV
cat > /tmp/test_targets.csv <<EOF
car_a,10,20
car_b,-30,40
EOF

python3 scripts/plot_paths.py \
    --experiment-id adaptive_d3_t5_trial99 \
    --targets-file /tmp/test_targets.csv
```

**Expected:** Plot shows only 2 target markers (car_a and car_b) instead of 5.

---

## Test 7: Full Automated Experiment (Integration)

**Deliverable:** `run_experiment.sh` orchestrates simulation launch, target
spawning, camera bridge, all nodes, logging, metrics, and plot generation.

### 7a. Clean start

```bash
# Kill everything
pkill -f "bin/px4" 2>/dev/null
pkill -f "gz sim" 2>/dev/null
pkill -f waypoint_executor 2>/dev/null
pkill -f detection_publisher 2>/dev/null
pkill -f voronoi_coordinator 2>/dev/null
pkill -f data_logger 2>/dev/null
pkill -f parameter_bridge 2>/dev/null
pkill -f MicroXRCEAgent 2>/dev/null
sleep 5
```

### 7b. Run an adaptive trial

```bash
cd ~/Desktop/Swarm
./scripts/run_experiment.sh \
    --method adaptive \
    --drones 3 \
    --targets 5 \
    --duration 120 \
    --trial 1
```

**Expected:** Script runs for ~3-4 minutes total:
1. Launches 3 drones in Gazebo with `swarm_default` world (no shadows)
2. Starts dynamic camera bridge for 3 drones
3. Spawns 5 Fuel vehicle targets
4. Starts waypoint executor (drones arm + take off)
5. Starts YOLOv8 detection publisher
6. Starts Voronoi coordinator
7. Logs data for 120 seconds
8. Computes metrics
9. Generates path plots

**Final output:**
```
══════════════════════════════════════════════════════════════
  Trial complete: adaptive_d3_t5_trial01
  Results: data/logs/adaptive_d3_t5_trial01/metrics_summary.json
  Plots:   data/logs/adaptive_d3_t5_trial01/paths_combined.png
══════════════════════════════════════════════════════════════
```

### 7c. Verify all outputs

```bash
ls ~/Desktop/Swarm/data/logs/adaptive_d3_t5_trial01/
```

**Expected files:**
```
positions.csv
detections.csv
waypoints.csv
metrics_summary.json
coverage_over_time.csv
paths_combined.png
path_drone1.png
path_drone2.png
path_drone3.png
```

### 7d. Verify metrics

```bash
cat ~/Desktop/Swarm/data/logs/adaptive_d3_t5_trial01/metrics_summary.json | python3 -m json.tool
```

**Check:**
- `coverage_pct` > 0 (should be 5–20% for 200m area)
- `total_distance_m` > 0
- `num_detections` >= 0
- `duration_s` close to 120

### 7e. Verify Gazebo had no shadows

During the trial, look at the Gazebo window:
- Ground plane should be uniformly lit
- No dark drone shadows on the ground

### 7f. Run a baseline trial (skip sim relaunch)

```bash
cd ~/Desktop/Swarm
./scripts/run_experiment.sh \
    --method lawnmower \
    --drones 3 \
    --targets 5 \
    --duration 120 \
    --trial 1 \
    --no-sim
```

**Expected:** Uses existing simulation. Starts lawnmower controller instead of
Voronoi coordinator. Produces `data/logs/lawnmower_d3_t5_trial01/` with same
file structure.

---

## Test 8: run_adaptive.sh (Interactive Pipeline)

**Deliverable:** Updated `run_adaptive.sh` with dynamic camera bridge and
custom world.

### 8a. Full launch

```bash
# Clean start
pkill -f "bin/px4" 2>/dev/null; pkill -f "gz sim" 2>/dev/null; sleep 5

cd ~/Desktop/Swarm
./scripts/run_adaptive.sh --drones 3 --area-size 200 --altitude 40
```

**Expected:**
- Banner shows parameters: Drones: 3, Altitude: 40m, Area: 200m
- 4 gnome-terminal windows open: Camera Bridge, Waypoint Executor, Detection
  Publisher, Voronoi Coordinator
- Gazebo opens with `swarm_default` world (no shadows)
- 5 vehicle targets spawned

**Verify:**
```bash
source /opt/ros/humble/setup.bash
source ~/Desktop/Swarm/ros2_ws/install/setup.bash

# Camera topics active
ros2 topic hz /drone1/camera/image_raw  # ~30 Hz

# Position being published
ros2 topic echo /drone1/position --once

# Waypoints being published
ros2 topic echo /drone1/target_waypoint --once

# Check for detections (may take time for drones to reach targets)
ros2 topic echo /drone1/detection
```

---

---

## Test 9: SwarmMode Message + Build

**Deliverable:** `SwarmMode.msg` added to `swarm_msgs` package and builds correctly.

### 9a. Rebuild swarm_msgs

```bash
cd ~/Desktop/Swarm/ros2_ws
colcon build --packages-select swarm_msgs
source install/setup.bash
```

**Expected:** Build succeeds without errors.

### 9b. Verify message definition

```bash
ros2 interface show swarm_msgs/msg/SwarmMode
```

**Expected output:**
```
uint8 MODE_EXPLORE=0
uint8 MODE_EXPLOIT=1

uint8 mode
uint8[] exploit_drone_ids
string event_class
float64[3] event_position
float32 event_confidence
builtin_interfaces/Time stamp
```

### 9c. Verify all 3 messages

```bash
ros2 interface show swarm_msgs/msg/Detection
ros2 interface show swarm_msgs/msg/TargetWaypoint
ros2 interface show swarm_msgs/msg/SwarmMode
```

**Expected:** All three display their fields without errors.

---

## Test 10: Multi-Class Target Spawning

**Deliverable:** `spawn_targets.sh` supports `--classes`, `--count`, `--random`, `--seed`.

### 10a. Spawn mixed-class targets

```bash
cd ~/Desktop/Swarm
# Simulation should be running from previous tests
./scripts/spawn_targets.sh --world swarm_default --count 6 --classes person,vehicle,fire
```

**Expected output:**
```
=== Spawning 6 targets into Gazebo world: swarm_default ===
    Classes: person vehicle fire
    Random placement: false (seed=42)
...
[Spawn] target_person_rescue_randy_1 at (70, -60, 0.0) yaw=0
[Spawn] target_vehicle_suv_2 at (-80, 70, 0.1) yaw=1.57
[Spawn] target_fire_patch_3 at (50, 80, 0.01)
[Spawn] target_person_casual_female_4 at (-60, -50, 0.0) yaw=2.35
[Spawn] target_vehicle_hatchback_red_5 at (85, -30, 0.1) yaw=1.05
[Spawn] target_fire_patch_6 at (-75, 20, 0.01)
=== Done! 6 targets spawned (classes: person vehicle fire) ===
```

**Verify:** Round-robin class assignment (person, vehicle, fire, person, vehicle, fire).

### 10b. Spawn with random placement

```bash
./scripts/spawn_targets.sh --world swarm_default --count 3 --classes vehicle --random --seed 99
```

**Expected:** 3 vehicle targets at random (but reproducible) positions.

### 10c. Legacy positional args still work

```bash
./scripts/spawn_targets.sh swarm_default 3
```

**Expected:** Spawns 3 vehicle-only targets at predefined positions (backwards compatible).

---

## Test 11: Ablation Baselines

**Deliverable:** `binary_voronoi.py` and `all_converge.py` run as experiment controllers.

### 11a. Run binary_voronoi trial

```bash
cd ~/Desktop/Swarm
./scripts/run_experiment.sh \
    --method binary_voronoi \
    --drones 3 \
    --targets 5 \
    --duration 120 \
    --trial 99 \
    --no-sim
```

**Expected:**
- Controller terminal shows `Binary Voronoi baseline ready — 3 drones, area=200m (confidence_weighted=False)`
- Detections logged as `[Binary Detection] ... weight=1.0 (binary)`
- Trial produces `data/logs/binary_voronoi_d3_t5_trial99/` with all CSVs

### 11b. Run all_converge trial

```bash
./scripts/run_experiment.sh \
    --method all_converge \
    --drones 3 \
    --targets 5 \
    --duration 120 \
    --trial 99 \
    --no-sim
```

**Expected:**
- Controller terminal shows `All-converge baseline ready — 3 drones, area=200m (confidence_weighted=True, no split-and-reform)`
- Detections logged as `[All-Converge Detection] ... ALL drones respond`
- Trial produces `data/logs/all_converge_d3_t5_trial99/`

### 11c. Run E5 batch (if time permits)

```bash
./scripts/run_all_experiments.sh --exp e5 --trials 2 --duration 120
```

**Expected:** Runs 2 trials each of binary_voronoi and all_converge.

---

## Test 12: Dual-Mode Explore/Exploit

**Deliverable:** Coordinator publishes SwarmMode, drones split into explore/exploit sets.

### 12a. Run adaptive and monitor /swarm/mode

In one terminal:
```bash
cd ~/Desktop/Swarm
./scripts/run_adaptive.sh --drones 3 --no-sim --no-spawn
```

In another terminal:
```bash
source /opt/ros/humble/setup.bash
source ~/Desktop/Swarm/ros2_ws/install/setup.bash
ros2 topic echo /swarm/mode
```

**Expected:** When a detection exceeds confidence 0.4, you should see:
```yaml
mode: 1
exploit_drone_ids: [1, 3]
event_class: car
event_position: [45.2, -30.8, 0.0]
event_confidence: 0.72
```

After ~30 seconds (timeout), you should see:
```yaml
mode: 0
exploit_drone_ids: []
event_class: ''
```

### 12b. Verify swarm_mode.csv logging

After running an adaptive trial:
```bash
cat ~/Desktop/Swarm/data/logs/adaptive_d3_t5_trial01/swarm_mode.csv
```

**Expected columns:** `timestamp, elapsed_s, mode, exploit_drone_ids, event_class, event_x, event_y, event_confidence`

### 12c. Verify coverage-during-event metric

```bash
cat ~/Desktop/Swarm/data/logs/adaptive_d3_t5_trial01/metrics_summary.json | python3 -m json.tool
```

**Check for new fields:**
- `coverage_during_exploit` — coverage % while exploit mode is active
- `coverage_during_explore` — coverage % during explore-only periods
- `exploit_duration_s` — total seconds spent in exploit mode
- `num_exploit_events` — number of exploit mode activations

---

## Test 13: Config File Loading

**Deliverable:** `config/adaptive_params.yaml` is loaded by coordinator and response selector.

### 13a. Verify config exists

```bash
cat ~/Desktop/Swarm/config/adaptive_params.yaml
```

**Check:**
- `class_priorities` section with person, vehicle, car, truck, fire
- `formations` section with cluster, chain, perimeter types
- `exploit_confidence_threshold`, `exploit_timeout_s` values

### 13b. Verify auto-discovery

The coordinator should find the config automatically:
```bash
# In the Voronoi Coordinator terminal (launched by run_adaptive.sh), look for:
# No --config flag needed — it auto-discovers config/adaptive_params.yaml
```

---

## Quick Shutdown

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
pkill -f binary_voronoi
pkill -f all_converge
pkill -f parameter_bridge
pkill -f MicroXRCEAgent
```

---

## Troubleshooting

| Problem | Cause | Fix |
|---------|-------|-----|
| `Waiting for Gazebo world...` hangs | World name mismatch | Verify symlink exists: `ls -la firmware/Tools/simulation/gz/worlds/swarm_default.sdf` |
| Shadows still visible | Old world cached | `pkill -f "gz sim"`, wait 5s, relaunch |
| Targets spawn but appear black | Broken `.mtl` texture paths | See `docs/gazebo_fuel_targets.md` troubleshooting section |
| Targets not visible in Gazebo | Camera too far away | Navigate Gazebo camera to (70, -60, 50) to find SUV |
| Zero detections after 120s | Drones never fly over targets | Check area size matches target positions; reduce altitude |
| Detections have wrong coordinates | Logger not computing global coords | Check `data_logger.py` has `_last_pos` tracking |
| Camera bridge shows 0 Hz | World name wrong in YAML | Verify `GZ_WORLD="swarm_default"` in run scripts |
| Plot doesn't show start markers | Data bounds issue | Plot auto-scales; check `positions.csv` has data |
| `ModuleNotFoundError: rclpy` | Wrong Python env | `conda activate swarm` (Python 3.10) |
| Fuel model download fails | Network issue | Retry; models cached in `~/.gz/fuel/` after first download |
| `run_experiment.sh` hangs at cleanup | Zombie processes | `pkill -9 -f "bin/px4"; pkill -9 -f "gz sim"` |
| `ImportError: swarm_msgs.msg.SwarmMode` | swarm_msgs not rebuilt | `cd ros2_ws && colcon build --packages-select swarm_msgs && source install/setup.bash` |
| No `/swarm/mode` messages | Detections below threshold | Lower `--exploit-threshold` or `--confidence` |
| `binary_voronoi` shows same results as adaptive | Both use Voronoi centroids | Check logs — binary should show `weight=1.0 (binary)` |
| Fire patches invisible from altitude | Orange box too small | Increase fire patch size in `spawn_targets.sh` (currently 3×3m) |
| `config/adaptive_params.yaml` not found | Wrong working directory | Run scripts from `~/Desktop/Swarm/` or use `--config` flag |

---

## Summary Checklist

| # | Test | Deliverable Verified |
|---|------|---------------------|
| 1 | Custom world launch | No-shadow Gazebo world loads correctly |
| 2 | Fuel vehicle targets | 5 textured vehicle models spawn at correct positions |
| 3 | Dynamic camera bridge | YAML auto-generated for N drones, images bridge to ROS 2 |
| 4 | Detection coordinates | Detections logged as global XY (near actual target positions) |
| 5 | 200m monitoring area | Area size, target spread, and metrics use 200m |
| 6 | Path visualization | Combined + per-drone plots with targets, detections, legend |
| 7 | Full experiment automation | run_experiment.sh produces all CSVs, metrics, and plots |
| 8 | Interactive pipeline | run_adaptive.sh launches all nodes with Week 2 improvements |
| 9 | SwarmMode message | SwarmMode.msg builds and shows correct fields |
| 10 | Multi-class spawning | person, vehicle, fire targets spawn with --classes flag |
| 11 | Ablation baselines | binary_voronoi and all_converge run as experiment methods |
| 12 | Dual-mode explore/exploit | /swarm/mode publishes, swarm_mode.csv logged, coverage-during-event computed |
| 13 | Config file loading | adaptive_params.yaml auto-discovered and loaded |
