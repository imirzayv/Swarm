# Baseline Controllers — Area Monitoring Comparison Methods

Three baseline controllers are implemented for comparison against the adaptive
Voronoi-based coordinator. All baselines share the same ROS 2 interface: they
publish `swarm_msgs/TargetWaypoint` on `/droneN/target_waypoint`, making them
drop-in replacements for `voronoi_coordinator.py` in the experiment pipeline.

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

## Comparison Summary

| Property | Static Grid | Lawnmower | Random | Adaptive (ours) |
|----------|-------------|-----------|--------|-----------------|
| **Coordination** | None | Implicit (strip assignment) | None | Voronoi partitioning |
| **Adapts to detections** | No | No | No | Yes |
| **Movement** | Stationary | Systematic sweep | Random walk | Density-weighted centroids |
| **Coverage pattern** | Fixed cells | Full sweep over time | Stochastic | Dynamic partitioning |
| **Energy** | Minimal (hover) | High (continuous flight) | Medium | Medium (repositioning) |
| **Implementation** | ~90 lines | ~140 lines | ~100 lines | ~200 lines + utils |

---

## Running Baselines in Experiments

All baselines are integrated into the experiment automation via `run_experiment.sh`:

```bash
# Run a single trial with a specific baseline
./scripts/run_experiment.sh --method static --drones 3 --targets 5 --duration 180 --trial 1
./scripts/run_experiment.sh --method lawnmower --drones 3 --targets 5 --duration 180 --trial 1
./scripts/run_experiment.sh --method random --drones 3 --targets 5 --duration 180 --trial 1

# Run all E1 baselines comparison (10 trials x 4 methods)
./scripts/run_all_experiments.sh --exp e1 --trials 10
```

Each trial outputs:
- `data/logs/<method>_d3_t5_trial01/positions.csv`
- `data/logs/<method>_d3_t5_trial01/detections.csv`
- `data/logs/<method>_d3_t5_trial01/waypoints.csv`
- `data/logs/<method>_d3_t5_trial01/metrics_summary.json`

---

## Shared Interface

All controllers (adaptive + baselines) communicate via the same ROS 2 topics:

```
Controller (any)  ──publish──→  /droneN/target_waypoint  ──subscribe──→  waypoint_executor.py
                  ──subscribe──→  /droneN/position        ──publish──→   waypoint_executor.py
```

This design allows swapping controllers without changing any other node in the
pipeline. The `run_experiment.sh` script handles this via the `--method` flag.
