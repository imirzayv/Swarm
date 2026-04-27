# Adaptive UAV Swarm Coverage for Sustainable Aerial Monitoring

> **⚠ HISTORICAL — pre-energy-scaling pivot (2026-04-25).** The active
> paper is now lawnmower N-scaling (Sw1–Sw5, 27 trials). Read
> [`istras_energy_scaling_guide.md`](istras_energy_scaling_guide.md)
> and
> [`istras_results_discussion_conclusion.md`](istras_results_discussion_conclusion.md)
> first. The adaptive runtime described below is implemented but not
> benchmarked in the current paper.

> Decentralized event-driven coverage framework for a swarm of PX4 UAVs, implemented on ROS 2 + Gazebo with real-time color-based detection.

## Overview

This repository contains the simulation stack, coordination logic, and analysis scripts for a multi-drone adaptive coverage system. A swarm of drones monitors a defined area and dynamically reallocates its Voronoi cells when transport-relevant events (colored target objects) are detected by onboard cameras. The system is designed to minimize total flight distance — and therefore operational energy and CO₂ — while maintaining event responsiveness.

**What's novel:** confidence-weighted Voronoi density, class-specific swarm response, dual-mode exploration/exploitation with split-and-reform behavior.

**What it runs on:** Ubuntu 22.04, ROS 2 Humble, Gazebo Harmonic, PX4-Autopilot SITL, Python 3.10+, OpenCV 4.x.

## Quick Start

```bash
# 1. Launch the 3-drone simulation
./scripts/launch_multi_drone.sh 3

# 2. In a new terminal, spawn colored target objects
./scripts/spawn_targets.sh --count 3 --random --seed 42

# 3. In a new terminal, start the full adaptive pipeline
./scripts/run_adaptive.sh

# 4. In a new terminal, monitor detections live
ros2 topic echo /drone1/detection
```

Drones spread out via decentralized Voronoi at startup. When a detection arrives at the coordinator, nearby drones converge in a class-specific formation; the rest continue baseline coverage. After a configurable timeout or loss of detection confidence, the swarm reforms.

## Repository Structure

```
.
├── README.md
├── ros2_ws/
│   └── src/
│       ├── swarm_msgs/          Custom messages: Detection, TargetWaypoint, SwarmMode
│       └── swarm_bringup/       Launch files, camera bridge configuration
├── scripts/
│   ├── launch_multi_drone.sh    Spawn N PX4 SITL drones + Gazebo world
│   ├── spawn_targets.sh         Spawn colored target objects in Gazebo
│   ├── detection_publisher.py   Per-drone HSV color detection → Detection messages
│   ├── waypoint_executor.py     MAVSDK waypoint follower per drone
│   ├── voronoi_coordinator.py   Decentralized adaptive coverage controller
│   ├── voronoi_utils.py         Lloyd's algorithm, GPS ↔ local conversion
│   ├── response_selector.py     Class-specific formation generator
│   ├── data_logger.py           Records positions, detections, swarm modes → CSV
│   ├── metrics_compute.py       Offline metrics: distance, energy, CO₂, latency
│   ├── plot_results.py          Generate paper figures from aggregated results
│   ├── run_adaptive.sh          Orchestration: launches all nodes in one command
│   ├── run_experiment.sh        Parameterized single-trial runner
│   ├── run_all_experiments.sh   Batch runner across methods and trials
│   └── baselines/
│       ├── static_grid.py       Fixed-grid coverage
│       ├── lawnmower.py         Boustrophedon sweep
│       └── random_waypoints.py  Uniform random patrol
├── config/
│   ├── adaptive_params.yaml     Voronoi, density, class priorities, formations
│   ├── camera_bridge.yaml       Gazebo ↔ ROS 2 camera topic bridges
│   └── experiment_configs/      Per-experiment YAML overrides
├── models/                      Gazebo target object SDFs (colored boxes)
├── worlds/                      Gazebo world files
├── data/
│   ├── logs/                    Raw CSV trial logs (one dir per experiment_id)
│   └── results/                 Aggregated metrics CSVs
├── figures/                     Publication-ready figures
└── paper/                       LaTeX source for the paper
```

## Prerequisites

| Component | Version | Why |
|---|---|---|
| Ubuntu | 22.04 LTS | Required base for ROS 2 Humble |
| ROS 2 | Humble Hawksbill | Middleware; LTS until 2027 |
| Gazebo | Harmonic (or Classic 11) | Physics + sensor simulation |
| PX4-Autopilot | main branch (SITL) | Flight stack in simulation loop |
| Python | 3.10+ | Scripts and ROS 2 nodes |
| OpenCV | 4.6+ | HSV color detection |
| MAVSDK-Python | latest | High-level drone control API |

## Installation

The full installation is described in the PX4 and ROS 2 official docs. Minimum steps:

```bash
# ROS 2 Humble
sudo apt install ros-humble-desktop ros-humble-ros-gz
source /opt/ros/humble/setup.bash

# PX4-Autopilot
git clone --recursive https://github.com/PX4/PX4-Autopilot.git
cd PX4-Autopilot
bash ./Tools/setup/ubuntu.sh
make px4_sitl_default

# Python dependencies for this project
pip install opencv-python numpy scipy matplotlib pandas mavsdk pyyaml

# Build this workspace
cd ros2_ws
colcon build --packages-select swarm_msgs swarm_bringup
source install/setup.bash
```

## Configuration

All tunable parameters live in `config/adaptive_params.yaml`. Key values:

```yaml
area:
  size_m: 40                    # square monitoring zone
  altitude_m: 10                # constant flight altitude
  home_lat: 47.397742           # PX4 default home
  home_lon: 8.545594

voronoi:
  update_rate_hz: 2             # coordinator tick
  lloyd_iterations: 1           # per tick
  density_base: 1.0             # uniform floor
  detection_gaussian_sigma: 5.0 # density bump width (m)
  detection_decay_s: 30.0       # temporal weight decay

class_priorities:
  person:  0.9
  vehicle: 0.6
  fire:    1.0

formations:
  person:  {drones: 2, radius_m: 3.0,  layout: cluster}
  vehicle: {drones: 2, spacing_m: 8.0, layout: chain}
  fire:    {drones: 3, radius_m: 10.0, layout: triangle}

dual_mode:
  exploit_timeout_s: 30.0
  confidence_threshold: 0.4     # below this, return to explore

detection:
  min_pixel_area: 150
  min_confidence: 0.3
```

## Detection Pipeline

The detector uses OpenCV HSV color segmentation — reliable, deterministic, and appropriate for the controlled simulation environment. Each target color maps to a class:

| Class | Gazebo color (RGB) | HSV range (OpenCV: H∈[0,180], S,V∈[0,255]) |
|---|---|---|
| person | (1, 0, 0) pure red | H: 0–10, S: 150–255, V: 100–255 |
| vehicle | (0, 0, 1) pure blue | H: 100–140, S: 150–255, V: 100–255 |
| fire | (1, 0.5, 0) orange | H: 10–25, S: 150–255, V: 150–255 |

Each detection publishes a `swarm_msgs/Detection` message on `/drone{N}/detection`:

```
uint8 drone_id
string class_name
float32 confidence       # area-normalized, clipped to [0,1]
float32[4] bbox          # [x, y, w, h] in image pixels
geometry_msgs/Point world_position   # projected from bbox + drone pose
builtin_interfaces/Time stamp
```

### Why not YOLOv8?

YOLOv8 pretrained on COCO does not reliably recognize Gazebo's synthetic objects — the feature distributions don't match photographic training data. Fine-tuning YOLOv8 on simulated imagery is possible but costs 3–4 days. HSV color detection is the pragmatic choice for isolating coordination performance from perception noise. For real-world deployment, swap `detection_publisher.py` for a YOLOv8 or equivalent CNN detector — the rest of the pipeline is unchanged because it consumes `Detection` messages, not raw images.

## Running a Single Trial

```bash
./scripts/run_experiment.sh \
  --method adaptive_voronoi \
  --drones 3 \
  --targets 3 \
  --duration 120 \
  --trial 1 \
  --seed 42 \
  --output-dir data/logs/istras/
```

Valid methods: `adaptive_voronoi`, `lawnmower`, `static_grid`, `random_waypoints`.

## Running the Full Experiment Set

```bash
./scripts/run_all_experiments.sh \
  --config config/experiment_configs/E1_baselines.yaml \
  --trials 3 \
  --output-dir data/logs/istras/
```

This sequences all method × trial combinations and writes each to a separate log directory. Expected wall-clock time for the minimum set (3 methods × 3 trials × 120 s) is about 18 minutes of simulation plus process-restart overhead (~3 minutes).

## Computing Metrics

After experiments complete:

```bash
python3 scripts/metrics_compute.py \
  --logs-dir data/logs/istras/ \
  --output data/results/istras_all_results.csv
```

This reads every trial's CSVs and produces one row per trial with columns:

- `method`, `trial`, `seed`, `n_drones`, `duration_s`
- `total_distance_m` — sum of drone travel distances
- `energy_wh` — computed from distance using hover rate 150 W
- `co2_azerbaijan_g`, `co2_eu_avg_g` — derived from energy
- `detection_coverage_pct` — fraction of spawned targets detected
- `detection_latency_s` — mean time-to-first-detection per target

### Energy model

Flight energy is estimated as `E_Wh = P_avg × t_hours`, where `P_avg ≈ 150 W` is the hover power of a small quadrotor (anchor: Abeywickrama et al., IEEE Access 2018, reports 142 W for a DJI Matrice 100). Distance traveled is reported as a separate metric for transparency — it is a known energy proxy since forward-flight power at modest speed (< 5 m/s) is dominated by hover power.

### Emission factors

CO₂ equivalent uses two grid factors:
- **Azerbaijan:** 0.44 kg CO₂/kWh (IEA country profile)
- **EU average:** 0.25 kg CO₂/kWh (EEA)

Both are reported so readers in either region can interpret the result.

## Generating Figures

```bash
python3 scripts/plot_results.py \
  --results data/results/istras_all_results.csv \
  --output figures/
```

Produces:
- `fig2_distance_comparison.pdf` — bar chart, mean ± std distance per method
- `fig3_latency_comparison.pdf` — bar chart, mean ± std detection latency per method
- `table1_summary.tex` — LaTeX-formatted summary table ready to `\input{}` in the paper

## Recording a Demo Video

For the paper's supplementary material:

```bash
# Start the pipeline
./scripts/run_adaptive.sh &
sleep 10  # let things stabilize

# Record Gazebo viewport (requires ffmpeg + gst-launch)
ffmpeg -video_size 1920x1080 -framerate 30 -f x11grab -i :0.0 \
  -t 120 -c:v libx264 -preset fast docs/demo.mp4
```

## Troubleshooting

**Gazebo runs slowly with 3 drones and cameras.**
Lower camera resolution in `config/camera_bridge.yaml` (try 320×240 at 15 Hz). Process every other frame in `detection_publisher.py` by skipping odd-numbered callbacks.

**HSV detector misses the targets.**
Save one camera frame to disk and read the HSV values at the target pixel. Adjust `COLOR_CLASSES` in `detection_publisher.py` to widen the hue window by ±15 and the saturation/value windows by ±50 around the observed values.

**Drones don't spread out at startup.**
Check that `voronoi_coordinator.py` is receiving position messages from all drones: `ros2 topic list | grep position`. If some drones are missing, verify MAVSDK connections in `waypoint_executor.py` (each drone uses `udp://:14540+i` and `grpc://localhost:50040+i`).

**"kite" detections appearing.**
You are still running YOLOv8 instead of the HSV detector. Check which script `run_adaptive.sh` is launching. Detection topics should come from `detection_publisher.py` (HSV), not the archived `yolo_detector.py`.

**Experiment trials don't reset cleanly between runs.**
Kill all Gazebo, PX4, and Python processes between trials. `run_experiment.sh` does this with `pkill -f px4 && pkill -f gz && pkill -f python3` before each trial. On slow machines, increase the settling sleep in the loop from 5 s to 10 s.

**Voronoi cells look degenerate (drones too close).**
The adaptive density's Gaussian bumps may be overlapping. Reduce `detection_gaussian_sigma` in `config/adaptive_params.yaml` from 5.0 to 3.0, or increase the spatial spread of target positions.

## Key Design Decisions

**Why decentralized?** Centralized coordination adds a single point of failure and scales poorly. Voronoi partitioning only needs neighbor positions, which fits naturally with real mesh communication.

**Why Voronoi specifically?** It provably minimizes expected distance to the nearest drone for a given density function (Cortés et al. 2004), making it a natural fit for energy-aware coverage. The confidence-weighted density extension lets detections directly drive swarm reallocation without requiring explicit leader election or voting.

**Why MAVSDK over pure MAVROS?** MAVSDK's Python API is higher-level than raw MAVROS, letting you call `goto_location(lat, lon, alt)` directly and await completion. This simplifies the waypoint executor considerably.

**Why HSV detection in simulation?** Pragmatic: it works instantly with colored Gazebo models. The coordination layer does not care how detections are produced — it consumes `Detection` messages. Swapping in a CNN detector later is a 1-file change.

## Citation

If you use this work, please cite:

```
[Placeholder: ISTRAS'26 paper once accepted]
```

## License

[Choose one — MIT or Apache 2.0 are standard for research code.]

## Acknowledgements

Built on PX4-Autopilot, ROS 2 Humble, Gazebo, MAVSDK, OpenCV, and Ultralytics YOLOv8 (retained as a reference implementation in `scripts/_archive/`). See individual package licenses.
