# Scalable Energy-Responsive UAV Swarm Coverage

> **⚠ Paper scope pivoted 2026-04-25.** The active framing is
> **swarm-size energy/time Pareto for lawnmower coverage** — see
> [`istras_energy_scaling_guide.md`](istras_energy_scaling_guide.md)
> for the runbook,
> [`istras_pareto_analysis.md`](istras_pareto_analysis.md) for the
> frontier walk, and
> [`istras_results_discussion_conclusion.md`](istras_results_discussion_conclusion.md)
> for the writeup against the completed Sw1–Sw5 dataset (N ∈ {1..5},
> 27 trials, all 8/8 targets found). The adaptive-vs-baseline / σ /
> dynamic-event framing in the rest of *this* README is **historical**;
> the runtime is implemented but not benchmarked in the current paper.

> A decentralized adaptive coverage framework for PX4 UAV swarms, evaluated across swarm sizes and dynamic event schedules, with explicit accounting of per-mission flight distance, energy consumption, and operational CO₂.

## What This Repo Does

Implements and evaluates a **confidence-weighted Voronoi** coverage controller for a swarm of UAVs monitoring a transport corridor. The controller reacts to onboard detections by dynamically reallocating Voronoi cells toward detection events (class-specific formations: cluster, chain, perimeter) while the remainder of the swarm maintains baseline area coverage. A dual-mode exploration/exploitation mechanism splits and reforms the swarm over time.

**Research questions answered by this work:**

1. At a fixed swarm size, does adaptive coverage save energy versus lawnmower and static-grid baselines at comparable detection quality?
2. How does the adaptive advantage scale as the swarm grows from 3 to 5 drones?
3. When events arrive dynamically during a mission, how quickly does each coordination strategy respond?

**Sustainability framing:** Total flight distance is reported both as a raw metric and as derived energy consumption (Wh) and operational CO₂ emission (g) using published hover-power rates and regional grid emission factors.

## Platform

- Ubuntu 22.04 LTS
- ROS 2 Humble Hawksbill
- Gazebo Harmonic (or Classic 11)
- PX4-Autopilot SITL (main branch)
- Python 3.10+, OpenCV 4.6+
- MAVSDK-Python
- NumPy, SciPy, pandas, matplotlib

## Repository Structure

```
.
├── README.md
├── ros2_ws/
│   └── src/
│       ├── swarm_msgs/          Detection, TargetWaypoint, SwarmMode messages
│       └── swarm_bringup/       Launch files, camera bridge configuration
├── scripts/
│   ├── launch_multi_drone.sh         Spawn N PX4 SITL drones + corridor world
│   ├── spawn_targets.sh              Static or scheduled dynamic spawning
│   ├── detection_publisher.py        HSV color detection → Detection messages
│   ├── waypoint_executor.py          MAVSDK waypoint follower per drone
│   ├── voronoi_coordinator.py        Decentralized adaptive controller
│   ├── voronoi_utils.py              Lloyd's algorithm, GPS↔local conversion
│   ├── response_selector.py          Class-specific formation generator
│   ├── data_logger.py                Per-trial CSV logging
│   ├── metrics_compute.py            Offline metrics aggregator
│   ├── plot_results.py               Publication-quality figure generator
│   ├── run_adaptive.sh               Orchestration for live demos
│   ├── run_experiment.sh             Parameterized single-trial runner
│   ├── run_all_experiments.sh        Batch runner for E1/E2/E3
│   └── baselines/
│       ├── static_grid.py            Fixed-grid coverage
│       ├── lawnmower.py              Boustrophedon sweep
│       └── random_waypoints.py       Uniform random patrol (unused in E1-E3)
├── config/
│   ├── adaptive_params.yaml          Voronoi, density, priorities, formations
│   ├── camera_bridge.yaml            Gazebo ↔ ROS 2 camera topic bridges
│   └── experiment_configs/
│       ├── E1.yaml                   3 methods × 3 drones × 5 trials
│       ├── E2.yaml                   2 methods × 5 drones × 5 trials
│       └── E3.yaml                   2 methods × 3 drones × 6 dynamic targets × 3 trials
├── models/                           Gazebo target SDFs (pure-color boxes)
├── worlds/                           Gazebo worlds: corridor.sdf (40×40m)
├── data/
│   ├── logs/                         Raw per-trial CSV logs
│   └── results/                      Aggregated metric CSVs
├── figures/                          Publication-ready PDFs
└── paper/
    ├── main.tex                      LaTeX source
    └── sections/                     Per-section .tex files
```

## Quick Start

```bash
# Build the ROS 2 workspace
cd ros2_ws && colcon build --packages-select swarm_msgs swarm_bringup
source install/setup.bash

# Interactive demo: 3 drones with adaptive coverage, targets spawn over time
./scripts/launch_multi_drone.sh 3 &
./scripts/spawn_targets.sh --schedule 20,50,80 --classes person,vehicle,fire --seed 42 &
./scripts/run_adaptive.sh

# Batch experiments
bash scripts/run_all_experiments.sh --experiments E1 E2 E3 --trials 5
python scripts/metrics_compute.py --logs-dir data/logs/istras/ --output data/results/all.csv
python scripts/plot_results.py --results data/results/all.csv --output figures/
```

## Experiment Design

Three experiments answer three research questions. All share the same world (40 m × 40 m corridor), altitude (10 m), HSV color detection, and dynamic target schedule (t=20, 50, 80 s for E1/E2; t=15, 30, 45, 60, 90, 120 s for E3).

### E1 — Head-to-head at fixed swarm size (3 drones, 5 trials per method)

| Method | Description |
|---|---|
| `static_grid` | Each drone hovers at a fixed cell center (lower bound) |
| `lawnmower` | Each drone sweeps an assigned strip (strong non-adaptive) |
| `adaptive` | Confidence-weighted Voronoi + class-specific response + dual-mode |

### E2 — Scalability (3 vs 5 drones, 5 trials per method)

Only `lawnmower` vs `adaptive`. Static grid scales trivially (just more hovering) and is cut for brevity.

### E3 — Dynamic event density (3 drones, 6 staggered targets, 3 trials per method)

Only `lawnmower` vs `adaptive`. Dense, staggered events amplify the response-time difference.

### Trial timing

Every trial follows the same three-phase structure:

```
t=0s    ───── 20s ───── 50s ───── 80s ────────── 150s
 spawn           person    vehicle   fire          end
 drones          target    target    target
 (no events)     appears   appears   appears
```

The first 20 s is an event-free exploration period common to all methods. This isolates steady-state coverage behavior from event response.

## Detection Pipeline

HSV color segmentation on downward-facing simulated cameras. Chosen over COCO-pretrained YOLOv8 because Gazebo primitives lack the texture/lighting richness required for photographic-data detectors (YOLOv8 returned spurious "kite" class labels on other drones seen against sky — a known symptom of out-of-distribution inputs).

| Class | Gazebo color (RGB) | HSV range (OpenCV) |
|---|---|---|
| person | pure red (1, 0, 0) | H: 0–10, S: 150–255, V: 100–255 |
| vehicle | pure blue (0, 0, 1) | H: 100–140, S: 150–255, V: 100–255 |
| fire | orange (1, 0.5, 0) | H: 10–25, S: 150–255, V: 150–255 |

Detection message (`swarm_msgs/Detection`):

```
uint8 drone_id
string class_name
float32 confidence       # area-normalized, clipped to [0, 1]
float32[4] bbox          # [x, y, w, h] in image pixels
geometry_msgs/Point world_position   # projected from bbox + drone pose
builtin_interfaces/Time stamp
```

For real-world deployment, swap `detection_publisher.py` for a CNN detector (YOLOv8 fine-tuned on domain data, or any equivalent); downstream nodes consume `Detection` messages and don't care how they were produced.

## Dynamic Target Spawning

`spawn_targets.sh` supports two modes:

**Mode 1 — All at t=0 (original):**
```bash
./scripts/spawn_targets.sh --count 3 --random --seed 42 --classes person,vehicle,fire
```

**Mode 2 — Scheduled dynamic spawning (v2):**
```bash
./scripts/spawn_targets.sh \
  --schedule 20,50,80 \
  --classes person,vehicle,fire \
  --area 40 \
  --seed 42
```

Each `--schedule` entry is the time (in simulation seconds) at which a target of the corresponding class appears. Target positions are drawn from a deterministic RNG seeded by `--seed`, so two runs with the same seed produce the same target locations and spawn times.

For E3's 6-target schedule:
```bash
./scripts/spawn_targets.sh \
  --schedule 15,30,45,60,90,120 \
  --classes person,vehicle,fire,person,vehicle,fire \
  --area 40 \
  --seed 17
```

## Configuration

All runtime parameters live in `config/adaptive_params.yaml`:

```yaml
area:
  size_m: 40
  altitude_m: 10
  home_lat: 47.397742
  home_lon: 8.545594

voronoi:
  update_rate_hz: 2
  lloyd_iterations: 1
  density_base: 1.0
  detection_gaussian_sigma: 5.0
  detection_decay_s: 30.0

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
  confidence_threshold: 0.4

detection:
  min_pixel_area: 150
  min_confidence: 0.3
```

## Metrics

`metrics_compute.py` computes these for each trial and writes them as one row per trial to `data/results/`:

| Metric | Definition |
|---|---|
| `total_distance_m` | Sum of ‖pos(t+1) − pos(t)‖ over all drones, all timesteps |
| `steady_state_distance_m` | Same but restricted to t ∈ [0, 20] s (no events) |
| `event_phase_distance_m` | Same but restricted to t ∈ [20, end] (event response) |
| `energy_wh` | `P_hover × mission_time_s / 3600 × n_drones`, where P_hover = 150 W |
| `co2_azerbaijan_g` | `energy_wh × 0.001 × 0.44 × 1000` |
| `co2_eu_avg_g` | `energy_wh × 0.001 × 0.25 × 1000` |
| `detection_coverage_pct` | Fraction of spawned targets detected during trial |
| `mean_detection_latency_s` | Mean over all events: time from spawn to first detection |
| `per_event_latency_s` | List of per-event latencies (for E3 detailed analysis) |

### Energy model citation

Hover power `P = 150 W` is anchored to Abeywickrama et al. 2018 ("Comprehensive Energy Consumption Model for Unmanned Aerial Vehicles, Based on Empirical Studies of Battery Performance," IEEE Access), which reports 142 W hover for a DJI Matrice 100-class quadrotor. 150 W rounds up slightly for robustness.

### Grid emission factors

- Azerbaijan: 0.44 kg CO₂ / kWh (IEA country profile, 2022)
- EU average: 0.25 kg CO₂ / kWh (EEA, 2022)

Both are reported so readers in either region can interpret the sustainability implications.

## Running Experiments

### Single trial

```bash
./scripts/run_experiment.sh \
  --method adaptive \
  --drones 3 \
  --schedule 20,50,80 \
  --classes person,vehicle,fire \
  --duration 150 \
  --trial 1 \
  --seed 42 \
  --output-dir data/logs/istras/E1/
```

Valid methods: `adaptive`, `lawnmower`, `static_grid`, `random_waypoints`.

### Full experiment batch

```bash
bash scripts/run_all_experiments.sh \
  --experiments E1 E2 E3 \
  --trials 5 \
  --output-dir data/logs/istras/
```

Expected wall-clock time: roughly 90 minutes (31 trials at ~3 min per trial including Gazebo startup/teardown).

### Resume after crash

```bash
bash scripts/run_all_experiments.sh --experiments E1 E2 E3 --trials 5 --resume
```

Skips any trial whose output directory already contains a complete metadata file.

## Figures and Tables

```bash
python scripts/plot_results.py \
  --results data/results/istras_all_results.csv \
  --output figures/
```

Produces:

- `fig1_architecture.pdf` — System architecture (hand-drawn in draw.io, not auto-generated)
- `fig2_e1_comparison.pdf` — Bar chart: distance + latency across 3 methods at 3 drones
- `fig3_e2_scaling.pdf` — Line plot: distance vs swarm size, adaptive vs lawnmower
- `fig4_e3_latency.pdf` — Per-event latency across 6 staggered events
- `table1_e1_summary.tex` — Full E1 metrics table
- `table2_e2_e3_summary.tex` — E2 + E3 key metrics

## Troubleshooting

**"Kite" detections appearing.** You are running the archived YOLOv8 script instead of the HSV detector. Check which script `run_adaptive.sh` actually launches. Detection publisher should be `detection_publisher.py` (HSV), not `scripts/_archive/yolo_detector.py`.

**HSV detector misses targets.** Save a single camera frame to disk and read the pixel HSV values at the target center. Adjust `COLOR_CLASSES` in `detection_publisher.py`: widen hue window by ±15, saturation/value windows by ±50.

**Gazebo slow with 5 drones.** Lower camera resolution in `config/camera_bridge.yaml` to 320×240, reduce frame rate to 10 Hz. Process every second frame by adding `if self.frame_count % 2: return` at the top of the image callback.

**Dynamic spawning fires all targets at once.** Your `spawn_targets.sh` is in legacy mode; add `--schedule` flag. If the schedule is passed but times are ignored, check that the shell's `sleep` resolution is in seconds (it is on Ubuntu — not a source of issues in practice).

**Drones don't split when event fires.** Check `ros2 topic echo /swarm/mode` — if it stays in EXPLORE forever, the coordinator isn't receiving detections. Test with `ros2 topic echo /drone1/detection` to confirm detections flow. If they do, check the confidence threshold in `config/adaptive_params.yaml` isn't set too high.

**Experiments don't reset cleanly between trials.** `run_all_experiments.sh` should run `pkill -f px4 && pkill -f gz && pkill -f "ros2 run"` between trials. On slower machines, increase the settling sleep from 5 s to 15 s.

**Voronoi cells become degenerate (drones clustered).** The adaptive density's Gaussian bumps may be overlapping. Reduce `detection_gaussian_sigma` in `config/adaptive_params.yaml` from 5.0 to 3.0, or check that `detection_decay_s` is set correctly (if detections never decay, all drones pile onto the first event).

**E2 adaptive doesn't scale better than lawnmower.** Verify that `n_drones` is correctly reaching the coordinator (`ros2 param get /voronoi_coordinator n_drones`). If it's still 3, the launch file isn't propagating the parameter.

**Plagiarism check flags simulation-description paragraphs.** These overlap with `swarm_drones_report.md` and your MDPI draft. Rewrite them in fresh wording for the ISTRAS version; the underlying facts are the same but the sentences need to be original.

## Key Design Decisions

**Decentralized over centralized.** Centralized coordination is a single point of failure and scales poorly. Voronoi partitioning only needs neighbor positions.

**Voronoi over grid or formation control.** Voronoi provably minimizes expected distance to nearest drone for a given density function (Cortés et al. 2004). Confidence-weighted extension lets detections drive swarm reallocation without explicit voting.

**HSV detection over fine-tuned YOLO.** Pragmatic: works instantly with colored Gazebo models. The coordinator consumes `Detection` messages and doesn't care how they're produced; a CNN is a 1-file swap for real-world deployment.

**Distance as energy proxy over actual battery modeling.** Distance is cheap to compute from position logs, is anchored to a published hover-power rate, and correlates tightly with forward-flight energy at sub-5 m/s speeds. Full battery modeling adds complexity without narrative payoff at short-paper scale.

**Dynamic target spawning.** Events in real transport corridors are unscheduled (accidents, incidents, debris); evaluating only with all-targets-at-t=0 would hide the response-time advantage of adaptive methods.

## Citation

```
[Placeholder for ISTRAS'26 publication; to be updated after acceptance.]
```

## License

[Choose: MIT or Apache 2.0 for research code.]

## Acknowledgements

Built on PX4-Autopilot, ROS 2 Humble, Gazebo, MAVSDK, OpenCV, and Ultralytics YOLOv8 (archived as reference in `scripts/_archive/`). See individual package licenses.
