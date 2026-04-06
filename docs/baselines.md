# Baseline & Ablation Controllers — Area Monitoring Comparison Methods

Five comparison controllers are implemented: three standard baselines and two
ablation baselines. All share the same ROS 2 interface — they publish
`swarm_msgs/TargetWaypoint` on `/droneN/target_waypoint`, making them drop-in
replacements for `voronoi_coordinator.py` in the experiment pipeline.

**Standard baselines** (E1) test whether adaptive coverage outperforms
conventional strategies. **Ablation baselines** (E5) isolate the contribution
of each novel element by removing one at a time.

---

## 1. Static Grid (`baselines/static_grid.py`)

**Strategy:** Divide the monitoring area into N equal rectangular cells and
assign each drone to hover at its cell center. No adaptation to detections.

**Behavior:**
- On startup, computes a grid layout (rows x cols) that best tiles N drones
  across the square monitoring area.
- Each drone is assigned one cell and given a fixed GPS waypoint at its center.
- Waypoints are re-published every 5 seconds (drones hold position).
- Completely static — ignores all detections.

**Why this baseline:** Represents the simplest possible coverage strategy.
If the adaptive method cannot beat hovering at fixed positions, the added
complexity of Voronoi partitioning is not justified.

**Usage:**
```bash
python3 baselines/static_grid.py --drones 1 2 3 --area-size 40 --altitude 30
```

**Parameters:**
| Parameter | Default | Description |
|-----------|---------|-------------|
| `--drones` | 1 2 3 | Drone IDs to control |
| `--area-size` | 40 | Monitoring area side length (meters) |
| `--altitude` | 30 | Hover altitude (meters) |

**Expected results:** High coverage of assigned cells, zero adaptability,
no response to detection events.

---

## 2. Lawnmower (`baselines/lawnmower.py`)

**Strategy:** Divide the monitoring area into N vertical strips. Each drone
flies a back-and-forth sweep pattern within its assigned strip, looping
forever.

**Behavior:**
- Area is split into N equal-width vertical strips.
- Each drone's strip is further divided into sweep lines spaced 8 meters
  apart (configurable via `spacing`).
- The drone flies to the bottom of the first sweep line, then to the top,
  then moves to the next line and reverses direction (back-and-forth).
- On reaching the last sweep line, the pattern wraps and restarts.
- Arrival detection: when the drone is within 5 meters of the current
  waypoint, it advances to the next.
- Subscribes to `/droneN/position` for arrival checking.

**Why this baseline:** Lawnmower patterns are the standard systematic
search strategy in search-and-rescue and agricultural surveying. They
guarantee complete area coverage over time, making them a strong baseline
for coverage percentage.

**Usage:**
```bash
python3 baselines/lawnmower.py --drones 1 2 3 --area-size 40 --altitude 30
```

**Parameters:**
| Parameter | Default | Description |
|-----------|---------|-------------|
| `--drones` | 1 2 3 | Drone IDs to control |
| `--area-size` | 40 | Monitoring area side length (meters) |
| `--altitude` | 30 | Flight altitude (meters) |
| `--speed` | 5 | Drone speed in m/s (informational) |

**Expected results:** High eventual coverage (sweeps the entire area),
but slow detection latency since drones must traverse their full strip
before revisiting any point.

---

## 3. Random Waypoints (`baselines/random_waypoints.py`)

**Strategy:** Each drone independently flies to a random point within
the monitoring area, then picks a new random point on arrival.

**Behavior:**
- On startup, each drone is assigned a random initial waypoint within
  the area bounds (seeded with `--seed` for reproducibility).
- Every 2 seconds, the node checks if each drone has arrived within
  5 meters of its current target.
- On arrival, a new random point is generated and published.
- No coordination between drones — they may overlap or leave gaps.

**Why this baseline:** Represents the "uncoordinated" approach. Each
drone acts independently with no awareness of others. This tests whether
coordination provides a measurable benefit over random exploration.

**Usage:**
```bash
python3 baselines/random_waypoints.py --drones 1 2 3 --area-size 40 --seed 42
```

**Parameters:**
| Parameter | Default | Description |
|-----------|---------|-------------|
| `--drones` | 1 2 3 | Drone IDs to control |
| `--area-size` | 40 | Monitoring area side length (meters) |
| `--altitude` | 30 | Flight altitude (meters) |
| `--seed` | 42 | Random seed for reproducibility |

**Expected results:** Moderate coverage with high variance. Poor
redundancy control (drones may accidentally cluster or leave large gaps).
Reasonable detection latency on average due to movement, but inconsistent.

---

---

## 4. Binary Voronoi — Ablation (`baselines/binary_voronoi.py`)

**Strategy:** Same Voronoi coverage algorithm as the full adaptive system, but
the density function uses **binary detection weights** — any detection gets
weight 1.0 regardless of confidence score or class priority.

**What it removes:** Novel Element 2 (confidence-weighted density function).

**Behavior:**
- Subscribes to `/droneN/detection` and `/droneN/position`.
- Maintains a density map with Gaussian bumps at detection locations.
- All detection bumps have the **same weight** (1.0), regardless of YOLO
  confidence or whether the detected object is a person, vehicle, or fire.
- Temporal decay still applies (bumps fade over time).
- All drones participate in Voronoi coverage — **no split-and-reform**.
- This matches the approach used in existing literature (e.g., CVT flood
  monitoring paper, which uses binary flood/not-flood density).

**Why this ablation:** Quantifies the contribution of confidence x priority
weighting. If the full system outperforms binary_voronoi, it demonstrates
that proportional density weighting produces better drone allocation than
binary detection triggers.

**Usage:**
```bash
python3 baselines/binary_voronoi.py --drones 1 2 3 --area-size 200 --altitude 40
```

**Parameters:**
| Parameter | Default | Description |
|-----------|---------|-------------|
| `--drones` | 1 2 3 | Drone IDs to control |
| `--area-size` | 200 | Monitoring area side length (meters) |
| `--altitude` | 40 | Flight altitude (meters) |
| `--update-rate` | 0.5 | Voronoi update frequency (Hz) |
| `--detection-weight` | 10.0 | Peak weight of detection Gaussian |
| `--detection-sigma` | 15.0 | Gaussian spread (meters) |
| `--decay-half-life` | 30.0 | Detection decay half-life (seconds) |

**Expected results:** Similar coverage to the full system in uniform-class
scenarios, but suboptimal allocation when detections have varying importance
(e.g., allocating the same resources to a low-confidence car detection as
to a high-confidence person detection).

---

## 5. All-Converge — Ablation (`baselines/all_converge.py`)

**Strategy:** Same confidence-weighted Voronoi density function as the full
system, but when a detection occurs, **ALL drones converge** toward it via
the density map. There is no dual-mode split — no drones are reserved for
maintaining baseline coverage during an event.

**What it removes:** Novel Element 3 (dual-mode exploration/exploitation
with split-and-reform).

**Behavior:**
- Uses confidence x class_priority weighting (same as full system).
- All drones always participate in weighted Voronoi coverage.
- When a high-priority detection occurs, the density map attracts ALL
  nearby drones toward it — no drones are explicitly split into an
  exploit set vs. an explore set.
- Does not publish `SwarmMode` messages (always in explore mode).
- Does not use `response_selector.py` (no class-specific formations).

**Why this ablation:** Quantifies the contribution of split-and-reform.
If the full system maintains higher coverage-during-event than all_converge,
it demonstrates that reserving drones for exploration during exploitation
events is valuable.

**Usage:**
```bash
python3 baselines/all_converge.py --drones 1 2 3 --area-size 200 --altitude 40
```

**Parameters:**
| Parameter | Default | Description |
|-----------|---------|-------------|
| `--drones` | 1 2 3 | Drone IDs to control |
| `--area-size` | 200 | Monitoring area side length (meters) |
| `--altitude` | 40 | Flight altitude (meters) |
| `--update-rate` | 0.5 | Voronoi update frequency (Hz) |
| `--detection-weight` | 10.0 | Peak weight of detection Gaussian |
| `--detection-sigma` | 15.0 | Gaussian spread (meters) |
| `--decay-half-life` | 30.0 | Detection decay half-life (seconds) |
| `--config` | auto | Path to adaptive_params.yaml (for class priorities) |

**Expected results:** Similar detection response to the full system, but
**lower coverage-during-event** because all drones are pulled toward the
detection, leaving the rest of the area unmonitored.

---

## Comparison Summary

| Property | Static Grid | Lawnmower | Random | Binary Voronoi | All-Converge | Adaptive (ours) |
|----------|-------------|-----------|--------|----------------|--------------|-----------------|
| **Coordination** | None | Implicit (strips) | None | Voronoi | Voronoi | Voronoi |
| **Adapts to detections** | No | No | No | Yes (binary) | Yes (weighted) | Yes (weighted) |
| **Confidence weighting** | N/A | N/A | N/A | No | Yes | Yes |
| **Class-specific response** | No | No | No | No | No | Yes |
| **Split-and-reform** | No | No | No | No | No | Yes |
| **Movement** | Stationary | Sweep | Random walk | Centroids | Centroids | Centroids + formations |
| **Energy** | Minimal | High | Medium | Medium | Medium | Medium |
| **Experiment** | E1 | E1 | E1 | E5 (ablation) | E5 (ablation) | E1, E2-E5 |

---

## Running Baselines in Experiments

All controllers are integrated into the experiment automation via `run_experiment.sh`:

```bash
# Standard baselines (E1)
./scripts/run_experiment.sh --method static --drones 3 --targets 5 --duration 180 --trial 1
./scripts/run_experiment.sh --method lawnmower --drones 3 --targets 5 --duration 180 --trial 1
./scripts/run_experiment.sh --method random --drones 3 --targets 5 --duration 180 --trial 1

# Ablation baselines (E5)
./scripts/run_experiment.sh --method binary_voronoi --drones 3 --targets 5 --duration 180 --trial 1
./scripts/run_experiment.sh --method all_converge --drones 3 --targets 5 --duration 180 --trial 1

# Run all E1 baselines comparison (10 trials x 4 methods)
./scripts/run_all_experiments.sh --exp e1 --trials 10

# Run E5 ablation study (10 trials x 2 ablation methods)
./scripts/run_all_experiments.sh --exp e5 --trials 10
```

Each trial outputs:
- `data/logs/<method>_d3_t5_trial01/positions.csv`
- `data/logs/<method>_d3_t5_trial01/detections.csv`
- `data/logs/<method>_d3_t5_trial01/waypoints.csv`
- `data/logs/<method>_d3_t5_trial01/swarm_mode.csv` (adaptive/all_converge only)
- `data/logs/<method>_d3_t5_trial01/metrics_summary.json`

---

## Shared Interface

All controllers (adaptive + baselines + ablations) communicate via the same
ROS 2 topics:

```
Controller (any)  ──publish──→  /droneN/target_waypoint  ──subscribe──→  waypoint_executor.py
                  ──subscribe──→  /droneN/position        ──publish──→   waypoint_executor.py
                  ──subscribe──→  /droneN/detection        ──publish──→   detection_publisher.py
```

The full adaptive coordinator additionally publishes:
```
voronoi_coordinator.py  ──publish──→  /swarm/mode  ──subscribe──→  data_logger.py
```

This design allows swapping controllers without changing any other node in the
pipeline. The `run_experiment.sh` script handles this via the `--method` flag.

Valid methods: `adaptive`, `static`, `lawnmower`, `random`, `binary_voronoi`, `all_converge`

---

## Ablation Study Design (E5)

The ablation study compares three conditions to isolate each novel element:

| Condition | Conf. Weighting | Split-and-Reform | Class Formations | Tests |
|-----------|-----------------|------------------|------------------|-------|
| **Full adaptive** | Yes | Yes | Yes | All novel elements |
| **Binary Voronoi** | **No** | No | No | Removes Novel Element 2 |
| **All-Converge** | Yes | **No** | No | Removes Novel Element 3 |

**Key metrics for ablation:**
- **Binary Voronoi vs Full:** Compare coverage %, redundancy ratio. Expect full
  system to allocate drones more efficiently (fewer drones on low-confidence
  detections).
- **All-Converge vs Full:** Compare coverage-during-event %. Expect full system
  to maintain higher baseline coverage while exploit events are active.
