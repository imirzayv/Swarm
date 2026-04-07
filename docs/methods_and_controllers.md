# Methods & Controllers Reference

This document explains the relationship between experiment methods, their
controller scripts, and the algorithms they implement. It also covers candidate
methods from the literature, feasibility analysis, and visualization options.

---

## Architecture Overview

Every experiment follows the same pipeline:

```
┌─────────────┐     ┌────────────┐     ┌──────────────┐     ┌──────────────┐
│  PX4 SITL   │────▶│  Waypoint   │◀────│  Controller  │◀────│  Detection   │
│  (per drone)│     │  Executor   │     │  (method)    │     │  Publisher   │
└─────────────┘     └────────────┘     └──────────────┘     └──────────────┘
                     flies to the       decides WHERE        YOLOv8 on
                     target waypoint    each drone goes      camera feed
```

The **controller** is the only component that changes between methods.
Everything else (PX4, waypoint executor, detection publisher, data logger)
stays the same. The controller subscribes to drone positions and (optionally)
detections, then publishes `TargetWaypoint` messages telling each drone where
to fly next.

This means any new method only requires a single Python script (~60-150 lines)
that:
1. Subscribes to `/droneN/position` (and optionally `/droneN/detection`)
2. Computes target positions using its algorithm
3. Publishes `TargetWaypoint` to `/droneN/target_waypoint`

---

## Implemented Methods (8)

### Summary Table

| # | Method | Controller Script | Algorithm Type | Reacts to Detections? | Region Assignment |
|---|--------|------------------|----------------|----------------------|-------------------|
| 1 | `adaptive` | `scripts/voronoi_coordinator.py` | Voronoi (weighted) | Yes (confidence x priority) | Dynamic Voronoi cells |
| 2 | `static_grid` | `scripts/baselines/static_grid.py` | Grid partition | No | Fixed grid cells |
| 3 | `lawnmower` | `scripts/baselines/lawnmower.py` | Strip sweep | No | Fixed vertical strips |
| 4 | `random` | `scripts/baselines/random_waypoints.py` | Random walk | No | None (random) |
| 5 | `binary_voronoi` | `scripts/baselines/binary_voronoi.py` | Voronoi (binary) | Yes (weight=1.0 always) | Dynamic Voronoi cells |
| 6 | `all_converge` | `scripts/baselines/all_converge.py` | Voronoi (weighted) | Yes (confidence x priority) | Dynamic Voronoi cells (all drones) |
| 7 | `pso` | `scripts/baselines/pso_coverage.py` | Particle Swarm Opt. | Yes (fitness bonus) | Implicit (swarm dynamics) |
| 8 | `apf` | `scripts/baselines/apf_coverage.py` | Artificial Potential Field | Yes (attraction force) | Implicit (force zones) |

### Detailed Descriptions

**1. Adaptive (Full System)** — `voronoi_coordinator.py`

Algorithm: Confidence-weighted Voronoi + dual-mode explore/exploit.
Every 2s, computes a bounded Voronoi diagram dividing the area into one cell
per drone. A density map (uniform baseline + Gaussian bumps weighted by
`YOLO_confidence x class_priority`) shifts each cell's centroid toward
high-value detections. On a high-confidence detection (>0.4), K nearest drones
enter a class-specific formation (cluster/chain/perimeter) while the rest
continue Voronoi coverage. Reforms after timeout (30s) or confidence drop.
All three novel elements are active.

**2. Static Grid** — `static_grid.py`

Algorithm: Fixed grid partitioning. Area is divided into `ceil(sqrt(N))`
columns and `ceil(N/cols)` rows. Each drone hovers at the center of its grid
cell permanently. No reaction to detections.
Purpose: Simplest possible coverage baseline.

**3. Lawnmower** — `lawnmower.py`

Algorithm: Back-and-forth strip sweeping. Area is divided into N equal-width
vertical strips. Each drone sweeps its strip in a boustrophedon pattern
(bottom-to-top, shift 8m, top-to-bottom, repeat). Guaranteed full coverage
of each strip. No reaction to detections.
Purpose: Systematic full-coverage baseline (common in agriculture/SAR).

**4. Random Waypoints** — `random_waypoints.py`

Algorithm: Independent random walk. Each drone picks a random (x,y) within the
area, flies there, picks another. No coordination. No reaction to detections.
Purpose: Lower bound — any coordination strategy should beat this.

**5. Binary Voronoi (Ablation)** — `binary_voronoi.py`

Algorithm: Same Voronoi as adaptive but density uses `weight=1.0` for every
detection regardless of confidence or class. No dual-mode or formations.
Purpose: Isolates Novel Element 2 (confidence weighting). Matches the approach
used in existing literature (e.g., CVT flood monitoring).

**6. All-Converge (Ablation)** — `all_converge.py`

Algorithm: Same confidence-weighted density as adaptive but ALL drones
participate in Voronoi (no exploit set split). No class-specific formations.
Purpose: Isolates Novel Element 3 (dual-mode split-and-reform).

**7. PSO Coverage** — `pso_coverage.py`

Algorithm: Particle Swarm Optimization. Each drone is a particle with position
and velocity. At each step, fitness is evaluated based on: (a) proximity to
uncovered grid cells (exploration pull), (b) bonus from recent detection
locations weighted by confidence (exploitation pull). Velocities are updated
using the standard PSO equations: `v = w*v + c1*r1*(pbest - x) + c2*r2*(gbest - x)`.
Detection-reactive: detection locations boost the fitness function, attracting
particles toward high-value areas. Parameters: inertia w=0.7, cognitive c1=1.5,
social c2=1.5, max velocity 15 m/step.
Purpose: Most-cited bio-inspired baseline in MDPI Drones. Provides a
metaheuristic comparison against geometric (Voronoi) approaches.

**8. APF Coverage** — `apf_coverage.py`

Algorithm: Artificial Potential Field. Each drone experiences virtual forces:
repulsive from other drones (inverse-square, to spread out), attractive from
uncovered grid cells (to explore), attractive from detection locations
(confidence-scaled, to exploit), and boundary repulsion (to stay in area).
The net force vector determines the drone's next waypoint displacement, clamped
to a maximum step size. Detection-reactive: detection locations become
attractive force sources with strength proportional to confidence × decay.
Parameters: k_rep=500, k_att=0.5, k_det=20, k_bnd=50, max_step=15m.
Purpose: Classic decentralized coverage method using a fundamentally different
paradigm (force-field vs. geometric partitioning). Frequently appears as a
baseline in multi-robot coverage papers.

---

## Candidate Methods from Literature (6)

The following methods were identified through a survey of recent UAV swarm
coverage papers (MDPI Drones, IEEE, ICRA/IROS). Each is evaluated for
feasibility within our existing codebase.

### Feasibility Summary

| # | Method | Effort | Detection-Reactive? | Implementation Feasible? | Recommended? |
|---|--------|--------|---------------------|--------------------------|--------------|
| 9 | Frontier-Based Exploration | ~2-3h | Partially | Yes | **Yes** |
| 10 | Hungarian Assignment | ~1h | Yes (dynamic targets) | Yes | **Yes** |
| 11 | Ergodic Coverage | ~3-4h | Yes (density-driven) | Yes (complex math) | Yes (stretch) |
| 12 | DARP (Divide Areas) | ~2-3h | No | Yes | Optional |
| 13 | Reynolds Flocking | ~1h | Possible | Yes | Optional |
| 14 | ACO (Ant Colony) | ~2h | Possible | Yes | Optional |

### 9. Frontier-Based Exploration

**Core idea:** Maintain a 2D coverage grid. "Frontiers" are boundaries between
covered and uncovered cells. Each drone is assigned to the nearest (or
highest-utility) frontier point. As drones visit locations, cells are marked
covered and new frontiers emerge.

**Key references:**
- Yamauchi, "A Frontier-Based Approach for Autonomous Exploration" (CIRA, 1997)
- "Frontier-led swarming: Robust multi-robot coverage of unknown environments"
  (Swarm and Evolutionary Computation, 2022)

**Detection-reactive?** Not inherently — it is a coverage-completion method.
Detections could boost the "value" of nearby uncovered cells but the core
algorithm is about systematic coverage.

**Implementation in our codebase:**
- Maintain NxN boolean grid (e.g., 100x100 for 200m area at 2m resolution)
- Each update: mark cells within FOV radius of each drone as covered
- Find frontier cells (uncovered cells adjacent to covered ones)
- Assign each drone to nearest frontier (greedy or Hungarian)
- Convert grid coords to GPS, publish TargetWaypoint
- ~80-120 lines, could use `scipy.ndimage` for efficient frontier detection

**Why include it:** Frontier exploration is the de facto standard for
multi-robot exploration. Fundamentally different strategy (systematic coverage
completion) vs. density-driven approaches. Shows whether "complete the map"
beats "go where detections are."

---

### 10. Hungarian Assignment (Optimal Centralized)

**Core idea:** Discretize the area into a set of target waypoints (grid points
or detection locations). Compute a cost matrix (distance from each drone to
each target). Solve the optimal assignment using the Hungarian algorithm to
minimize total travel distance. Targets cycle as drones visit them.

**Key references:**
- Kuhn, "The Hungarian Method for the Assignment Problem" (1955)
- "Comparative Analysis of Centralized and Distributed Multi-UAV Task
  Allocation Algorithms" (MDPI Drones, 2025)

**Detection-reactive?** Yes. Detection locations become high-priority targets
in the assignment matrix with reduced cost.

**Implementation in our codebase:**
- Generate target waypoints: grid of coverage points + detection locations
- Compute NxM distance matrix (drones x targets)
- Call `scipy.optimize.linear_sum_assignment` (already have scipy)
- Publish assigned waypoints, regenerate targets on arrival
- ~30-50 lines, minimal new code

**Why include it:** Represents the theoretical optimum for assignment at each
step. If adaptive Voronoi performs close to Hungarian quality while being
reactive and decentralized, that is a compelling result.

---

### 11. Ergodic Coverage Control

**Core idea:** Plan trajectories such that the time-averaged drone distribution
matches a spatial information density. Uses Fourier decomposition to compute an
ergodic metric and gradient-based control to minimize it.

**Key references:**
- Mathew & Mezic, "Metrics for ergodicity and design of ergodic dynamics for
  multi-agent systems" (Physica D, 2011)
- "Decentralized Ergodic Control" (IEEE ICRA, 2018)

**Detection-reactive?** Yes, inherently. The target density is updated with
detections (same concept as our density map). Drones spend more time near
density peaks.

**Implementation in our codebase:**
- Compute Fourier coefficients of target density and time-averaged trajectory
- Control law: gradient descent on ergodic metric
- ~100-150 lines, requires understanding of Fourier basis computation
- Tuning: number of Fourier modes, control gains

**Why include it:** Most theoretically principled comparison — same
density-driven paradigm as adaptive Voronoi, but uses trajectory optimization
instead of geometric partitioning. Strongest "apples-to-apples" comparison.
However, the math is more involved.

---

### 12. DARP (Divide Areas based on Robot Positions)

**Core idea:** Divide the area into connected, approximately equal sub-regions
(one per drone) based on current positions. Each drone then performs a
lawnmower sweep within its assigned sub-region.

**Key references:**
- Kapoutsis et al., "DARP: Divide Areas Algorithm for Optimal Multi-Robot
  Coverage Path Planning" (J. Intelligent & Robotic Systems, 2017)

**Detection-reactive?** No. Coverage-completion method with predetermined
sweep patterns.

**Implementation in our codebase:**
- Discretize area into grid, iteratively assign cells to nearest drone
- Ensure connected regions of equal size
- Generate lawnmower path within each region
- ~100-150 lines, open-source Python implementation exists

**Why include it:** Better version of `static_grid` — produces optimal static
partitions. Only worth adding if you want to upgrade the static baseline.

---

### 13. Reynolds Flocking + Dispersion

**Core idea:** Three rules: separation (avoid crowding), alignment (match
neighbors' heading), cohesion (move toward group center). Add a fourth rule:
dispersion (spread to uncovered areas).

**Key references:**
- Reynolds, "Flocks, Herds and Schools" (SIGGRAPH, 1987)
- "Optimized flocking of autonomous drones in confined environments"
  (Science Robotics, 2018)

**Detection-reactive?** With a "detection attraction" rule added, yes.

**Implementation in our codebase:**
- For each drone: compute separation, alignment, cohesion forces from neighbor
  positions (already subscribed), add dispersion force
- Sum forces, normalize, scale, publish as waypoint
- ~40-50 lines, no new dependencies

**Why include it:** Well-known bio-inspired baseline. However, it is a weak
baseline unlikely to outperform Voronoi. The `random` method already fills the
"simple decentralized" comparison slot.

---

### 14. ACO-Based Coverage (Ant Colony Optimization)

**Core idea:** Area is discretized into a graph. Drones act as "ants" depositing
pheromone on visited cells. Pheromone evaporates over time. Drones choose next
cells probabilistically, preferring low-pheromone (less-visited) areas.

**Key references:**
- "A Novel Ant Colony-Inspired Coverage Path Planning for Internet of Drones"
  (Computer Networks, 2023)
- "Multi-UAV Cooperative 3D Coverage Path Planning Based on Asynchronous Ant
  Colony Optimization" (IEEE ICUS, 2021)

**Detection-reactive?** Can boost pheromone at detection locations.

**Implementation in our codebase:**
- Maintain pheromone grid (NxN float array)
- Each step: evaporate (multiply by decay), compute transition probabilities,
  sample next cell, deposit pheromone
- ~80-100 lines, NumPy only

**Why include it:** Another bio-inspired metaheuristic. If PSO is already
included, ACO adds redundancy — pick one or the other.

---

## Recommendation: Which Methods to Implement

### Tier 1 — Implemented

| Method | Status | Script |
|--------|--------|--------|
| **PSO** | **DONE** | `scripts/baselines/pso_coverage.py` |
| **APF / Virtual Force** | **DONE** | `scripts/baselines/apf_coverage.py` |

These two, combined with the existing 6, give **8 total methods** covering
four algorithmic paradigms:
- Geometric partitioning (adaptive, binary_voronoi, all_converge)
- Bio-inspired optimization (PSO)
- Force-field / potential (APF)
- Predetermined patterns (static_grid, lawnmower, random)

### Tier 1.5 — Next Priority

| Method | Rationale | Effort |
|--------|-----------|--------|
| **Hungarian Assignment** | Theoretical optimal baseline. scipy does the heavy lifting. | ~1h |

### Tier 2 — Recommended if Time Permits

| Method | Rationale | Effort |
|--------|-----------|--------|
| **Frontier-Based** | Standard exploration baseline, fundamentally different approach. | ~2-3h |
| **Ergodic Coverage** | Strongest theoretical comparison, same density paradigm. | ~3-4h |

### Tier 3 — Optional (diminishing returns)

| Method | Rationale | Effort |
|--------|-----------|--------|
| **DARP** | Only if you want to replace static_grid with a better static baseline. | ~2-3h |
| **Flocking** | Weak baseline, `random` already fills this slot. | ~1h |
| **ACO** | Redundant with PSO. Pick one bio-inspired metaheuristic. | ~2h |

---

## Updated Experiment Design (with new methods)

| Experiment | Methods | Purpose |
|------------|---------|---------|
| **E1** (Coverage) | adaptive, static_grid, lawnmower, random, pso, apf | Coverage efficiency across paradigms |
| **E2** (Detection Response) | adaptive, static_grid, lawnmower, random, pso, apf | Detection-reactive methods vs. static baselines |
| **E3** (Energy) | adaptive, static_grid, lawnmower, random, pso, apf | Energy cost per coverage unit |
| **E4** (Scalability) | adaptive, static_grid, lawnmower, random, pso, apf | 3 vs. 5 drones scaling |
| **E5** (Ablation) | adaptive, binary_voronoi, all_converge | Novel element contribution |

With 6 methods in E1-E4 at 10 trials each, that is 60 trials per experiment
(240 total for E1-E4 + 30 for E5 = 270 trials).

---

## Visualization: Which Plot for Which Method?

| Plot Type | Flag | Applicable Methods | What It Shows |
|-----------|------|--------------------|---------------|
| Flight paths | *(default)* | All | Drone trajectories with start/end markers |
| Per-drone path | *(default)* | All | Individual drone trajectory |
| Voronoi partitions | `--voronoi` | adaptive, binary_voronoi, all_converge | Colored cells showing each drone's assigned region |
| Lawnmower strips | `--strips` | lawnmower | Colored vertical strips showing each drone's sweep area |
| APF force field | `--force-field` | apf | Repulsion zones + force vector arrows on grid |
| PSO vectors | `--pso-vectors` | pso | Particle trajectories with velocity arrows + best dwell points |

### Usage Examples

```bash
# Adaptive — Voronoi regions at end of trial
python3 scripts/plot_paths.py \
    --data-dir data/logs/adaptive_d3_t5_trial01 \
    --area-size 200 --voronoi

# Adaptive — Voronoi snapshot at t=60s
python3 scripts/plot_paths.py \
    --data-dir data/logs/adaptive_d3_t5_trial01 \
    --area-size 200 --voronoi --snapshot-time 60

# Lawnmower — strip assignments
python3 scripts/plot_paths.py \
    --data-dir data/logs/lawnmower_d3_t5_trial01 \
    --area-size 200 --strips

# PSO — particle trajectories with velocity vectors
python3 scripts/plot_paths.py \
    --data-dir data/logs/pso_d3_t5_trial01 \
    --area-size 200 --pso-vectors

# APF — force field visualization with repulsion zones
python3 scripts/plot_paths.py \
    --data-dir data/logs/apf_d3_t5_trial01 \
    --area-size 200 --force-field

# APF — force field snapshot at t=60s
python3 scripts/plot_paths.py \
    --data-dir data/logs/apf_d3_t5_trial01 \
    --area-size 200 --force-field --snapshot-time 60
```

---

## Algorithm Taxonomy

```
                         ┌────────────────────────────┐
                         │   Coverage Controllers     │
                         └──────────┬─────────────────┘
                                    │
          ┌─────────────────────────┼─────────────────────────┐
          │                         │                         │
   Detection-Reactive          Static/Predetermined      Ablation
          │                         │                         │
    ┌─────┴──────┐           ┌──────┴──────┐           ┌──────┴──────┐
    │            │           │             │           │             │
 Geometric   Force/Meta     Grid        Sweep      Binary Vor.  All-Converge
    │            │           │             │
 adaptive    apf          static_grid   lawnmower
             pso                        random
```

### Relationship Between Methods

```
┌─ Voronoi-Based (geometric partitioning) ────────────────────────────┐
│                                                                     │
│  binary_voronoi ──▶ all_converge ──▶ adaptive (full system)         │
│  (binary density)   (conf-weighted)  (conf-weighted + dual-mode     │
│  (no formations)    (no split)        + class formations)           │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘

┌─ Non-Voronoi (implemented) ──────────────────────────────────────────┐
│                                                                     │
│  PSO                          APF / Virtual Force                   │
│  (particles with velocity,    (repulsion/attraction forces          │
│   fitness-based movement,      between drones and targets,          │
│   gbest/pbest tracking)        boundary repulsion)                  │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘

┌─ Non-Voronoi (candidates) ──────────────────────────────────────────┐
│                                                                     │
│  Hungarian Assignment         Frontier-Based Exploration            │
│  (optimal centralized         (coverage grid + boundary             │
│   assignment via cost          expansion)                           │
│   matrix + scipy solver)                                            │
│                                                                     │
│  Ergodic Coverage             DARP                                  │
│  (Fourier-based trajectory    (optimal static partition +           │
│   matching to density)         local sweep)                         │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘

┌─ Static Baselines (no coordination) ────────────────────────────────┐
│                                                                     │
│  static_grid         lawnmower              random                  │
│  (hover at center)   (strip sweep)          (random walk)           │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

---

## Implementation Status

### Completed
- [x] PSO — `scripts/baselines/pso_coverage.py` (~200 lines)
- [x] PSO case added to `scripts/run_experiment.sh`
- [x] PSO visualization: `--pso-vectors` flag in `plot_paths.py`
- [x] APF — `scripts/baselines/apf_coverage.py` (~210 lines)
- [x] APF case added to `scripts/run_experiment.sh`
- [x] APF visualization: `--force-field` flag in `plot_paths.py`

### Remaining for New Methods
- [ ] Add PSO + APF to `scripts/run_all_experiments.sh` experiment batches
- [ ] Dry run PSO with 3 drones (60s)
- [ ] Dry run APF with 3 drones (60s)
- [ ] Tune PSO parameters (w, c1, c2) if needed
- [ ] Tune APF parameters (k_rep, k_att, k_det) if needed

### Next Candidate (if time permits)
- [ ] Hungarian Assignment — `scipy.optimize.linear_sum_assignment` (~30-50 lines)
