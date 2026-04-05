# Adaptive Area Monitoring — 5-Week Research Plan

> **Topic:** Vision-Guided Adaptive Area Monitoring with Drone Swarms
> **Duration:** 5 weeks (2-3 hours/day)
> **Target Venue:** MDPI Drones
> **Start Date:** 2026-04-04
> **End Date:** 2026-05-05

---

## Project Summary

A swarm of drones cooperatively monitors a large area, dynamically reallocating coverage based on real-time YOLOv8 detections from onboard cameras. When one drone detects an object of interest, nearby drones converge to form a tighter surveillance cluster while the rest maintain baseline coverage. The coordination uses a decentralized **weighted Voronoi** partitioning algorithm.

---

## What's Already Done (Days 1-3)

| Capability | Script | Status |
|-----------|--------|--------|
| 3 PX4 SITL drones in Gazebo Harmonic | `scripts/launch_multi_drone.sh` | ✅ |
| Downward cameras bridged to ROS 2 | `ros2_ws/src/swarm_bringup/` | ✅ |
| MAVSDK waypoint control | `scripts/multi_drone_waypoints.py` | ✅ |
| Target object spawning | `scripts/spawn_targets.sh` | ✅ |
| YOLOv8 inference on camera feeds | `scripts/yolo_detector.py` | ✅ |
| Full pipeline orchestration | `scripts/run_day3.sh` | ✅ |

---

## System Architecture (Target)

```
┌────────────────────────────────────────┐
│        voronoi_coordinator.py          │
│                                        │
│  Subscribes:                           │
│    /droneN/detection    (Detection)    │
│    /droneN/position     (Position)     │
│                                        │
│  Publishes:                            │
│    /droneN/target_wp    (Waypoint)     │
│                                        │
│  Algorithm:                            │
│    Weighted Voronoi + Lloyd's relax    │
│    Density map: uniform + Gaussian     │
│    bumps at detection locations         │
└──────────┬──────────────┬──────────────┘
           │              │
           ▼              ▼
┌──────────────────┐  ┌───────────────────┐
│ detection_pub.py │  │ wp_executor.py    │
│                  │  │                   │
│ Sub: image_raw   │  │ Sub: target_wp    │
│ Pub: /detection  │  │ Pub: /position    │
│                  │  │                   │
│ YOLOv8 inference │  │ MAVSDK goto_loc() │
│ + world position │  │ + telemetry pub   │
│   estimation     │  │                   │
└──────────────────┘  └───────────────────┘

  data_logger.py — subscribes to all topics → CSV files
  metrics_compute.py — offline analysis from CSV → results
```

**Custom ROS 2 messages** (`ros2_ws/src/swarm_msgs/msg/`):
- `Detection.msg` — drone_id, class_name, confidence, bbox, world_position, stamp
- `TargetWaypoint.msg` — drone_id, latitude, longitude, altitude, priority

---

## Files to Create

| File | Purpose |
|------|---------|
| `ros2_ws/src/swarm_msgs/` | Custom message package |
| `scripts/detection_publisher.py` | YOLOv8 + ROS 2 Detection publisher |
| `scripts/waypoint_executor.py` | MAVSDK subscriber-based control |
| `scripts/voronoi_coordinator.py` | Voronoi coverage + adaptive reallocation |
| `scripts/voronoi_utils.py` | Lloyd's algorithm, GPS conversion, geometry |
| `scripts/data_logger.py` | Position/detection CSV logging |
| `scripts/metrics_compute.py` | Offline metrics computation |
| `scripts/baselines/static_grid.py` | Fixed grid baseline |
| `scripts/baselines/lawnmower.py` | Sweep pattern baseline |
| `scripts/baselines/random_waypoints.py` | Random patrol baseline |
| `scripts/run_experiment.sh` | Parameterized experiment runner |
| `scripts/run_all_experiments.sh` | Batch experiment runner |
| `scripts/plot_results.py` | Matplotlib figure generation |
| `config/adaptive_params.yaml` | Algorithm parameters |
| `paper/main.tex` | Research paper (MDPI template) |

**Files to modify:**
- `scripts/spawn_targets.sh` — add `--count N`, `--random`, `--seed S`
- `ros2_ws/src/swarm_bringup/config/camera_bridge.yaml` — extend for 5 drones

---

## WEEK 1: Core System Implementation

**Goal:** Working adaptive coverage system with baselines and logging.

### Day 4 (Fri, Apr 4) — ROS 2 Messages + Detection Publisher
- [ ] Create `swarm_msgs` package with `Detection.msg` and `TargetWaypoint.msg`
- [ ] `colcon build --packages-select swarm_msgs`
- [ ] Create `detection_publisher.py` (refactor from `yolo_detector.py`): add Detection publisher on `/droneN/detection`, estimate world position from bbox center + altitude + camera FOV
- [ ] Test: `ros2 topic echo /drone1/detection` shows valid messages

### Day 5 (Sat, Apr 5) — Waypoint Executor Node
- [ ] Create `waypoint_executor.py`: subscribes to `/droneN/target_waypoint`, calls MAVSDK `goto_location()`, publishes position at 2 Hz
- [ ] Use same port pattern as existing code (UDP 14540+i, gRPC 50040+i)
- [ ] Threading: ROS 2 spin in thread, asyncio MAVSDK in main
- [ ] Test: manual `ros2 topic pub` a waypoint → drone moves

### Day 6 (Sun, Apr 6) — Voronoi Coverage Algorithm
- [ ] Implement `voronoi_utils.py`:
  - `compute_voronoi(points, bounds)` — scipy.spatial.Voronoi clipped to monitoring area
  - `lloyd_step(points, bounds)` — one Lloyd's relaxation iteration
  - `weighted_voronoi_centroid(cell, density_fn)` — centroid weighted by density map
  - `xy_to_gps()` / `gps_to_xy()` — flat-earth from PX4 home (47.397742, 8.545594)
- [ ] Implement `voronoi_coordinator.py`:
  - Subscribes to all `/droneN/position` and `/droneN/detection`
  - Density map: uniform baseline + Gaussian bumps at detections (sigma=5m, decay=30s)
  - Every 2s: compute weighted Voronoi → publish `TargetWaypoint` per drone
  - Params: `area_size=40m`, `detection_weight`, `detection_decay_s`, `update_rate`

### Day 7 (Mon, Apr 7) — End-to-End Integration
- [ ] Write `scripts/run_adaptive.sh` (full pipeline launcher)
- [ ] Debug: drones spread via Voronoi at startup, converge on detection, relax back
- [ ] Tune: detection_weight, decay (30s), max cluster size (2 drones)
- [ ] Record demo video (Gazebo screen capture)

### Day 8 (Tue, Apr 8) — Data Logging
- [ ] Create `data_logger.py`: subscribes to positions + detections → CSV files in `data/logs/<experiment_id>/`
- [ ] Create `metrics_compute.py`: reads CSVs, computes:
  - Coverage %: grid discretization + FOV projection
  - Detection latency: target spawn → first detection
  - Redundancy ratio: avg drone FOV overlaps per cell
  - Energy proxy: total distance traveled
- [ ] Test: run 2-min session → verify CSVs and metrics output

### Day 9 (Wed, Apr 9) — Baselines
- [ ] `baselines/static_grid.py` — divide area into N cells, hover at centers
- [ ] `baselines/lawnmower.py` — assign strips, sweep back-and-forth, loop
- [ ] `baselines/random_waypoints.py` — fly to random point, pick new one on arrival
- [ ] All use same TargetWaypoint interface as coordinator

### Day 10 (Thu, Apr 10) — Experiment Automation
- [ ] Create `run_experiment.sh`: `--method --drones --targets --duration --trial --output-dir`
- [ ] Create `run_all_experiments.sh`: loops over experiment configs
- [ ] Create config YAMLs in `config/experiment_configs/` (E1-E4)
- [ ] Fix accumulated bugs from the week

### Week 1 Deliverables
- [ ] Adaptive coverage system running end-to-end
- [ ] 3 baselines implemented and tested
- [ ] Data logging + metrics pipeline working
- [ ] Experiment automation scripts ready
- [ ] Demo video recorded

---

## WEEK 2: Robustness + Dry Runs + Literature Review

**Goal:** System validated at scale, all experiments dry-tested, literature review complete.

### Day 11 (Fri, Apr 11) — Scalability: 5 Drones + Variable Targets
- [ ] Enhance `spawn_targets.sh`: `--count N`, `--random`, `--seed S` for reproducible random placement
- [ ] Extend `camera_bridge.yaml` for 5 drones (drone4, drone5)
- [ ] Test `launch_multi_drone.sh 5` — verify all nodes work with 5 drones

### Day 12 (Sat, Apr 12) — Edge Cases + Tuning
- [ ] Handle: no detections → uniform Voronoi; same target seen by all → cap cluster at 2; drone disconnect → skip in computation
- [ ] Profile CPU/GPU with full pipeline running
- [ ] Document final parameters in `config/adaptive_params.yaml`

### Day 13 (Sun, Apr 13) — Dry Run: E1 (Baselines)
- [ ] Run 2 trials each: adaptive, static, lawnmower, random (3 drones, 5 targets, 120s)
- [ ] Verify data logging and metrics for all 4 methods
- [ ] Debug process cleanup between trials

### Day 14 (Mon, Apr 14) — Dry Run: E2 + E3
- [ ] E2: adaptive, 5 drones, 5 targets, 2 trials (check Gazebo perf)
- [ ] E3: adaptive, 3 drones, 10 targets, 2 trials

### Day 15 (Tue, Apr 15) — Response Time Instrumentation (E4)
- [ ] Add timestamps in coordinator: detection received → waypoints published → drones arrive
- [ ] Dry run E4: 2 trials, verify reconfiguration time logging

### Day 16 (Wed, Apr 16) — Literature Review Part 1
- [ ] Read 8-10 papers: Voronoi coverage (Cortes et al., Schwager), UAV monitoring, adaptive coverage
- [ ] Annotate: method, metrics, agent count, sim/real, limitations
- [ ] Start `paper/references.bib`

### Day 17 (Thu, Apr 17) — Literature Review Part 2
- [ ] Read 8-10 more: YOLOv8 on UAVs, PX4 SITL papers, coverage planning surveys
- [ ] Review dry run results, create bug fix list

### Week 2 Deliverables
- [ ] System handles 5 drones and 10 targets
- [ ] All 4 experiments dry-run validated
- [ ] 20 papers annotated, BibTeX started
- [ ] Final parameter set documented

---

## WEEK 3: Full Experiments

**Goal:** All experiments complete with statistical significance.

### Day 18 (Fri, Apr 18) — Bug Fixes + E1 Adaptive 1-5
- [ ] Fix all issues from dry runs
- [ ] Run adaptive: 5 trials x 180s each (3 drones, 5 targets)

### Day 19 (Sat, Apr 19) — E1 Adaptive 6-10 + Static 1-10
- [ ] Finish adaptive trials (6-10)
- [ ] Run static_grid: 10 trials

### Day 20 (Sun, Apr 20) — E1 Lawnmower + Random + E2
- [ ] Lawnmower: 10 trials
- [ ] Random: 10 trials
- [ ] E2 (5 drones): 10 trials

### Day 21 (Mon, Apr 21) — E3
- [ ] 3 targets: 10 trials
- [ ] 10 targets: 10 trials (5-target data reused from E1)

### Day 22 (Tue, Apr 22) — E4 + Reruns
- [ ] E4 response time: 10 trials
- [ ] Rerun any crashed/incomplete trials from E1-E3

### Day 23 (Wed, Apr 23) — Results Analysis
- [ ] Aggregate into `data/results/all_results.csv`
- [ ] Compute mean, std, 95% CI per condition
- [ ] Create `plot_results.py`

### Day 24 (Thu, Apr 24) — Figures + Statistics
- [ ] Generate all paper figures (300 DPI, PDF): bar charts (E1), line plots (E2-E3), box plot (E4)
- [ ] Paired t-tests / Wilcoxon signed-rank between adaptive and each baseline
- [ ] Create system architecture diagram + Voronoi before/after visualization

### Week 3 Deliverables
- [ ] ~80 trials complete (E1: 40, E2: 10, E3: 20, E4: 10)
- [ ] Master results CSV
- [ ] All figures generated (publication quality)
- [ ] Statistical analysis with p-values

---

## WEEK 4: Paper Writing — Technical Core

**Goal:** Sections 2-7 drafted.

### Day 25 (Fri, Apr 25) — Paper Setup + System Architecture
- [ ] Download MDPI Drones LaTeX template, set up `paper/` directory
- [ ] Write Section 4: System Architecture (~1.5 pages) — simulation stack, ROS 2 node graph, communication

### Day 26 (Sat, Apr 26) — Methodology Part 1
- [ ] Write Section 5.1-5.4 (~2 pages): Voronoi partitioning, Lloyd's algorithm, YOLOv8 integration, camera→world mapping, adaptive density function, detection-triggered reallocation
- [ ] Include equations: Voronoi cell definition, Lloyd's centroid, weighted centroid, Gaussian density

### Day 27 (Sun, Apr 27) — Methodology Part 2 + Experimental Setup
- [ ] Write Section 5.5-5.6: baseline methods, formal metric definitions
- [ ] Write Section 6 (~1 page): simulation details, drone/camera specs, parameters table

### Day 28 (Mon, Apr 28) — Results Part 1
- [ ] Write 7.1: E1 baselines comparison (coverage, latency, energy tables + charts)
- [ ] Write 7.2: E2 scalability analysis

### Day 29 (Tue, Apr 29) — Results Part 2 + Discussion
- [ ] Write 7.3: E3 target density, 7.4: E4 response time
- [ ] Discussion: limitations, why adaptive works, computational overhead

### Day 30 (Wed, Apr 30) — Related Work
- [ ] Write Section 3 (~2 pages): multi-robot coverage, UAV monitoring, vision-guided ops, gap
- [ ] Cite 25-35 papers

### Day 31 (Thu, May 1) — Introduction
- [ ] Write Section 2 (~1.5 pages): problem, motivation, contributions:
  1. Vision-guided adaptive Voronoi coverage algorithm
  2. Realistic multi-drone simulation integration (PX4 + Gazebo + ROS 2)
  3. Comprehensive evaluation against 3 baselines across 4 dimensions

### Week 4 Deliverables
- [ ] Sections 2-7 drafted (~10-12 pages)
- [ ] All figures and tables inserted
- [ ] References cited in text

---

## WEEK 5: Completion + Submission

**Goal:** Paper submitted.

### Day 32 (Fri, May 2) — Conclusion + Abstract + References
- [ ] Section 8: Conclusion (~0.5 pages) — contributions, key numbers, future work (real-world, moving targets, heterogeneous swarms)
- [ ] Abstract (200-250 words)
- [ ] Complete `references.bib` (30-40 entries), verify no missing refs

### Day 33 (Sat, May 3) — Full Review + Figure Polish
- [ ] Read entire paper end-to-end for logical flow
- [ ] Verify all figures/tables referenced in text
- [ ] Polish figure labels, legends, captions, font consistency

### Day 34 (Sun, May 4) — Formatting + Proofread
- [ ] MDPI template compliance (author info, keywords, page count)
- [ ] Grammar/spelling check, verify numbers match figures
- [ ] Prepare supplementary: demo video, GitHub README

### Day 35 (Mon, May 5) — Final Review + Submit
- [ ] One last full read
- [ ] Submit to MDPI Drones + cover letter

### Week 5 Deliverables
- [ ] Complete paper (all 8 sections + references)
- [ ] Submitted to MDPI Drones
- [ ] Supplementary material (video + code repo) prepared

---

## Experiments

| ID | Question | IV | Conditions | Drones | Targets | Trials |
|----|----------|----|-----------|--------|---------|--------|
| E1 | Does adaptive beat baselines? | Method | adaptive, static, lawnmower, random | 3 | 5 | 10×4 |
| E2 | How does it scale? | Drone count | 3, 5 | 3,5 | 5 | 10×2 |
| E3 | Effect of target density? | Target count | 3, 5, 10 | 3 | var | 10×3 |
| E4 | How fast does it reconfigure? | — | adaptive only | 3 | 5 | 10 |

**Metrics:**
- Coverage % — fraction of area within at least one drone's camera FOV
- Detection latency — time from target placement to first detection
- Redundancy ratio — average drone FOV overlaps per grid cell
- Energy — total distance traveled (sum of position deltas)
- Reconfiguration time (E4) — detection event → drones reach new positions

---

## Risk Mitigations

| Risk | Mitigation |
|------|-----------|
| Gazebo unstable at 5 drones | Reduce camera res (320x240), process every 10th frame |
| YOLO can't detect simple objects | Lower confidence, use textured objects, fall back to color detection |
| Adaptive doesn't beat baselines | Frame as evaluation study, focus on response time advantage |
| Writing takes longer than planned | Draft methodology during Week 3 experiment downtime |
| Not enough trials for significance | Increase to 15-20 for key comparisons, reduce duration |

---

## Time Budget

| Week | Focus | Est. Hours |
|------|-------|-----------|
| 1 | Core implementation | ~19h |
| 2 | Robustness + dry runs + lit review | ~18h |
| 3 | Full experiments + analysis | ~20h |
| 4 | Paper writing (technical) | ~18h |
| 5 | Paper completion + submission | ~10h |
| **Total** | | **~85h** |

---

*Plan created: 2026-04-04*
