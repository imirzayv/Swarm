# Adaptive Area Monitoring — 5-Week Research Plan

> **Topic:** Semantic Event-Driven Adaptive Area Monitoring with Drone Swarms
> **Duration:** 5 weeks (2-3 hours/day)
> **Target Venue:** MDPI Drones
> **Start Date:** 2026-04-04
> **End Date:** 2026-05-05

---

## Literature Gap Analysis

### Closest Existing Works

| Paper | Year | What They Do | Key Limitation |
|-------|------|-------------|----------------|
| **CVT Flood Monitoring** (Gaussian Mixture Density + Voronoi, ROS/Gazebo) | 2025 | UAVs detect flooded regions via OpenCV color thresholding → update CVT density → reposition swarm | Binary detection (flood/not flood); simple thresholding, no learned detector; single event type |
| **Enhanced Voronoi + YOLOv8 Tracking** | 2024 | Voronoi partitioning with predictive edge-crossing + YOLOv8 for energy-efficient target tracking | Tracks a *known, single* moving target — not multi-class discovery of unknown events across an area |
| **Bio-inspired SAR Swarm** (PX4 + Gazebo, Scientific Reports) | 2025 | PSO/GWO/ACO-driven grid search with shared thermal confidence map for survivor detection | Grid-based (no Voronoi); binary thermal signal; no semantic classification of what is detected |
| **Anomaly Detection in Dense Vegetation** (Nature Comms Eng) | 2025 | Centralized swarm adapts synthetic-aperture sampling pattern to track anomalies under foliage | Synthetic aperture imaging, not standard CV; centralized architecture; anomaly-only (no class info) |
| **IRADA Persistent Monitoring** (Scientific Reports) | 2025 | Distributed task allocation with energy-aware reward modulation and communication-driven recovery | No vision component at all — models abstract "uncertainty accumulation," not real detections |
| **Multi-UAV CPP with In-Flight Re-Planning** (J. Field Robotics) | 2024 | Coverage path re-planning on UAV failure or new region addition | No object detection in the loop; replanning is operator-triggered, not detection-triggered |

### The Gap

No existing paper closes the **full loop** from:

**Real-time deep learning detection → semantic event classification → class-specific density weighting → Voronoi coverage replanning → heterogeneous swarm reconfiguration**

Specifically, three things are missing across the literature:

1. **Multi-class detection driving different swarm behaviors.** Every existing system treats detection as binary (found / not found). No one maps detection *class and confidence* to *differentiated* swarm responses.

2. **Confidence-weighted density functions for Voronoi replanning.** The CVT flood paper updates density with binary presence. Nobody weights the Voronoi density function by YOLO confidence scores combined with event priority, producing a proportional swarm response.

3. **Dual-mode exploration/exploitation with split-and-reform.** No paper implements a swarm that dynamically splits — some drones converge for close inspection (exploitation) while others maintain area coverage (exploration) — and reforms after the event is resolved.

### Our Novel Contribution (1 paragraph)

We propose a **semantic event-driven adaptive coverage** framework in which a swarm of UAVs running real-time YOLOv8 detection uses the *class, confidence, and priority* of detected objects to modulate a weighted Voronoi density function, producing class-specific swarm reconfigurations. Unlike prior work that treats detections as binary triggers or tracks a single known target, our system assigns heterogeneous responses — a detected person triggers a tight two-drone inspection cluster, a vehicle triggers a tracking chain, and a fire triggers a wide perimeter formation — while remaining drones maintain baseline exploration coverage. We validate the framework in a PX4 + Gazebo + ROS 2 simulation with up to 5 drones and compare against three baselines across coverage efficiency, detection latency, energy, and reconfiguration time.

---

## Project Summary

A swarm of drones cooperatively monitors a large area, dynamically reallocating coverage based on real-time YOLOv8 detections from onboard cameras. When one drone detects an object of interest, the **class and confidence** of that detection determines how the swarm responds: nearby drones converge with a class-specific formation (tight cluster for persons, tracking chain for vehicles, wide perimeter for fires) while the rest maintain baseline coverage via a **dual-mode exploration/exploitation** strategy. The coordination uses a decentralized **confidence-weighted Voronoi** partitioning algorithm where detection confidence scores and event priority jointly modulate the density function.

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
┌─────────────────────────────────────────────────────────┐
│              voronoi_coordinator.py                      │
│                                                         │
│  Subscribes:                                            │
│    /droneN/detection    (Detection)                     │
│    /droneN/position     (Position)                      │
│                                                         │
│  Publishes:                                             │
│    /droneN/target_wp    (Waypoint)                      │
│    /swarm/mode          (SwarmMode: explore/exploit)    │
│                                                         │
│  Algorithm:                                             │
│    1. Confidence-weighted Voronoi + Lloyd's relax       │
│    2. Density map: uniform base + Gaussian bumps        │
│       weighted by (confidence × class_priority)         │
│    3. Class-specific response selector:                 │
│       person → tight 2-drone cluster                    │
│       vehicle → tracking chain formation                │
│       fire → wide 3-drone perimeter                     │
│    4. Dual-mode: split drones into exploit set +        │
│       explore set; explore set maintains baseline CVT   │
└──────────┬─────────────────────┬────────────────────────┘
           │                     │
           ▼                     ▼
┌──────────────────┐  ┌───────────────────┐
│ detection_pub.py │  │ wp_executor.py    │
│                  │  │                   │
│ Sub: image_raw   │  │ Sub: target_wp    │
│ Pub: /detection  │  │ Pub: /position    │
│                  │  │                   │
│ YOLOv8 inference │  │ MAVSDK goto_loc() │
│ + class label    │  │ + telemetry pub   │
│ + confidence     │  │                   │
│ + world position │  │                   │
│   estimation     │  │                   │
└──────────────────┘  └───────────────────┘

  data_logger.py — subscribes to all topics → CSV files
  metrics_compute.py — offline analysis from CSV → results
```

**Custom ROS 2 messages** (`ros2_ws/src/swarm_msgs/msg/`):
- `Detection.msg` — drone_id, class_name, confidence, bbox, world_position, stamp
- `TargetWaypoint.msg` — drone_id, latitude, longitude, altitude, priority
- `SwarmMode.msg` — mode (EXPLORE/EXPLOIT), exploit_drone_ids[], event_class, event_position

---

## Novel Elements — Implementation Checklist

These are the features that differentiate our paper from existing work. Each must be implemented and evaluated.

### Novel Element 1: Multi-Class Detection → Heterogeneous Swarm Response
- [ ] Define class-priority mapping in `config/adaptive_params.yaml`: `{person: 0.9, vehicle: 0.6, fire: 1.0}`
- [ ] Define class-specific formations in `config/adaptive_params.yaml`:
  - `person` → tight cluster (2 drones, 3m radius)
  - `vehicle` → tracking chain (2 drones, 8m separation along heading)
  - `fire` → wide perimeter (3 drones, 10m radius triangle)
- [ ] Implement `response_selector.py`: takes (class, confidence, drone_positions) → returns formation waypoints for assigned drones
- [ ] Spawn 3 object types in Gazebo (person model, vehicle model, fire texture patch)
- [ ] Evaluate: per-class detection rate and appropriate formation triggering

### Novel Element 2: Confidence-Weighted Voronoi Density Function
- [ ] Modify density function: `density(x) = base + Σ (conf_i × priority_i × Gaussian(x, det_pos_i, sigma))`
- [ ] Implement confidence decay: weight decreases with time since last detection at that location
- [ ] Compare in ablation: binary density (existing approach) vs. confidence-weighted density (ours)
- [ ] Metric: show that confidence weighting produces measurably better drone allocation (fewer drones on low-confidence detections)

### Novel Element 3: Dual-Mode Exploration/Exploitation with Split-and-Reform
- [ ] Implement mode logic in coordinator: when detection event fires, assign K nearest drones to exploit set, remaining drones to explore set
- [ ] Explore set: continue weighted Voronoi over full area (excluding exploit zone)
- [ ] Exploit set: fly class-specific formation around detection
- [ ] Reform trigger: exploit set returns to explore mode after `exploit_timeout_s` (configurable, default 30s) or when detection confidence drops below threshold
- [ ] Compare in ablation: all-converge (existing approach) vs. split-and-reform (ours)
- [ ] Metric: show that split-and-reform maintains higher baseline coverage during events

---

## Files to Create

| File | Purpose |
|------|---------|
| `ros2_ws/src/swarm_msgs/` | Custom message package (Detection, TargetWaypoint, SwarmMode) |
| `scripts/detection_publisher.py` | YOLOv8 + ROS 2 Detection publisher (with class + confidence) |
| `scripts/waypoint_executor.py` | MAVSDK subscriber-based control |
| `scripts/voronoi_coordinator.py` | Confidence-weighted Voronoi + dual-mode logic |
| `scripts/voronoi_utils.py` | Lloyd's algorithm, GPS conversion, geometry |
| `scripts/response_selector.py` | Class-specific formation generator |
| `scripts/data_logger.py` | Position/detection/mode CSV logging |
| `scripts/metrics_compute.py` | Offline metrics computation |
| `scripts/baselines/static_grid.py` | Fixed grid baseline |
| `scripts/baselines/lawnmower.py` | Sweep pattern baseline |
| `scripts/baselines/random_waypoints.py` | Random patrol baseline |
| `scripts/baselines/binary_voronoi.py` | **Ablation baseline**: Voronoi with binary (unweighted) density |
| `scripts/baselines/all_converge.py` | **Ablation baseline**: adaptive Voronoi but all drones converge (no split) |
| `scripts/run_experiment.sh` | Parameterized experiment runner |
| `scripts/run_all_experiments.sh` | Batch experiment runner |
| `scripts/plot_results.py` | Matplotlib figure generation |
| `config/adaptive_params.yaml` | Algorithm parameters + class-priority map + formation configs |
| `paper/main.tex` | Research paper (MDPI template) |

**Files to modify:**
- `scripts/spawn_targets.sh` — add `--count N`, `--random`, `--seed S`, `--classes person,vehicle,fire`
- `ros2_ws/src/swarm_bringup/config/camera_bridge.yaml` — extend for 5 drones

---

## WEEK 1: Core System + Novel Elements Implementation

**Goal:** Working adaptive coverage system with all 3 novel elements, baselines, and logging.

### Day 4 (Fri, Apr 4) — ROS 2 Messages + Detection Publisher
- [ ] Create `swarm_msgs` package with `Detection.msg`, `TargetWaypoint.msg`, `SwarmMode.msg`
- [ ] `colcon build --packages-select swarm_msgs`
- [ ] Create `detection_publisher.py` (refactor from `yolo_detector.py`): add Detection publisher on `/droneN/detection`, include **class_name + confidence**, estimate world position from bbox center + altitude + camera FOV
- [ ] Test: `ros2 topic echo /drone1/detection` shows valid messages with class and confidence

### Day 5 (Sat, Apr 5) — Waypoint Executor + Multi-Class Target Spawning
- [ ] Create `waypoint_executor.py`: subscribes to `/droneN/target_waypoint`, calls MAVSDK `goto_location()`, publishes position at 2 Hz
- [ ] Use same port pattern as existing code (UDP 14540+i, gRPC 50040+i)
- [ ] Threading: ROS 2 spin in thread, asyncio MAVSDK in main
- [ ] Update `spawn_targets.sh`: add `--classes person,vehicle,fire` to spawn different Gazebo models
- [ ] Test: manual `ros2 topic pub` a waypoint → drone moves; 3 object types visible in Gazebo

### Day 6 (Sun, Apr 6) — Confidence-Weighted Voronoi Algorithm [Novel Element 2]
- [ ] Implement `voronoi_utils.py`:
  - `compute_voronoi(points, bounds)` — scipy.spatial.Voronoi clipped to monitoring area
  - `lloyd_step(points, bounds)` — one Lloyd's relaxation iteration
  - `weighted_voronoi_centroid(cell, density_fn)` — centroid weighted by **confidence × class_priority** density map
  - `xy_to_gps()` / `gps_to_xy()` — flat-earth from PX4 home (47.397742, 8.545594)
- [ ] Implement `voronoi_coordinator.py`:
  - Subscribes to all `/droneN/position` and `/droneN/detection`
  - Density map: `base + Σ (conf_i × priority_i × Gaussian(x, pos_i, sigma))` with temporal decay
  - Every 2s: compute weighted Voronoi → publish `TargetWaypoint` per drone
  - Params from `config/adaptive_params.yaml`: `area_size=40m`, `class_priorities`, `detection_decay_s=30`, `update_rate=2`

### Day 7 (Mon, Apr 7) — Class-Specific Response + Dual-Mode [Novel Elements 1 & 3]
- [ ] Implement `response_selector.py`: given (class, confidence, drone_positions) → returns formation waypoints
  - person → tight cluster (2 nearest drones, 3m radius)
  - vehicle → tracking chain (2 nearest drones, 8m separation)
  - fire → wide perimeter (3 nearest drones, 10m triangle)
- [ ] Add dual-mode logic to `voronoi_coordinator.py`:
  - On high-confidence detection: assign K nearest drones to **exploit** (fly formation via response_selector), remaining to **explore** (continue Voronoi over full area minus exploit zone)
  - Publish `SwarmMode` message
  - Reform after `exploit_timeout_s` or confidence drop
- [ ] End-to-end test: spawn person → 2 drones cluster, others maintain coverage; timeout → reform

### Day 8 (Tue, Apr 8) — Integration + Data Logging
- [ ] Write `scripts/run_adaptive.sh` (full pipeline launcher)
- [ ] Debug: drones spread via Voronoi at startup, converge with class-specific formation on detection, split-and-reform works, relax back
- [ ] Create `data_logger.py`: subscribes to positions + detections + swarm_mode → CSV files in `data/logs/<experiment_id>/`
- [ ] Create `metrics_compute.py`: reads CSVs, computes:
  - Coverage %: grid discretization + FOV projection
  - Detection latency: target spawn → first detection
  - Redundancy ratio: avg drone FOV overlaps per cell
  - Energy proxy: total distance traveled
  - **Coverage-during-event**: coverage % maintained while exploit mode is active (novel metric)
- [ ] Test: run 2-min session → verify CSVs and metrics output
- [ ] Record demo video (Gazebo screen capture)

### Day 9 (Wed, Apr 9) — Baselines (Standard + Ablation)
- [ ] `baselines/static_grid.py` — divide area into N cells, hover at centers
- [ ] `baselines/lawnmower.py` — assign strips, sweep back-and-forth, loop
- [ ] `baselines/random_waypoints.py` — fly to random point, pick new one on arrival
- [ ] `baselines/binary_voronoi.py` — **ablation**: same Voronoi but density uses binary detection (weight=1 if detected, regardless of confidence/class) — matches existing literature approach
- [ ] `baselines/all_converge.py` — **ablation**: confidence-weighted Voronoi but ALL drones converge on detection (no split-and-reform) — tests dual-mode contribution
- [ ] All use same TargetWaypoint interface as coordinator

### Day 10 (Thu, Apr 10) — Experiment Automation
- [ ] Create `run_experiment.sh`: `--method --drones --targets --duration --trial --output-dir`
- [ ] Create `run_all_experiments.sh`: loops over experiment configs
- [ ] Create config YAMLs in `config/experiment_configs/` (E1-E5)
- [ ] Fix accumulated bugs from the week

### Week 1 Deliverables
- [ ] Adaptive coverage system with all 3 novel elements running end-to-end
- [ ] 3 standard baselines + 2 ablation baselines implemented and tested
- [ ] Data logging + metrics pipeline working
- [ ] Experiment automation scripts ready
- [ ] Demo video recorded

---

## WEEK 2: Robustness + Dry Runs + Literature Review

**Goal:** System validated at scale, all experiments dry-tested, literature review complete.

### Day 11 (Fri, Apr 11) — Scalability: 5 Drones + Variable Targets
- [ ] Enhance `spawn_targets.sh`: `--count N`, `--random`, `--seed S`, `--classes` for reproducible mixed-class placement
- [ ] Extend `camera_bridge.yaml` for 5 drones (drone4, drone5)
- [ ] Test `launch_multi_drone.sh 5` — verify all nodes work with 5 drones
- [ ] Verify class-specific formations scale: fire perimeter needs 3 drones; with 5 drones, verify 2 remain exploring

### Day 12 (Sat, Apr 12) — Edge Cases + Tuning
- [ ] Handle: no detections → uniform Voronoi; same target seen by all → cap exploit set; drone disconnect → skip in computation; simultaneous multi-class events → prioritize by class_priority
- [ ] Profile CPU/GPU with full pipeline running
- [ ] Document final parameters in `config/adaptive_params.yaml`

### Day 13 (Sun, Apr 13) — Dry Run: E1 (Baselines) + E5 (Ablations)
- [ ] Run 2 trials each: adaptive, static, lawnmower, random, binary_voronoi, all_converge (3 drones, 5 targets, 120s)
- [ ] Verify data logging and metrics for all 6 methods
- [ ] Debug process cleanup between trials

### Day 14 (Mon, Apr 14) — Dry Run: E2 + E3
- [ ] E2: adaptive, 5 drones, 5 mixed-class targets, 2 trials (check Gazebo perf)
- [ ] E3: adaptive, 3 drones, 10 mixed-class targets, 2 trials

### Day 15 (Tue, Apr 15) — Response Time Instrumentation (E4)
- [ ] Add timestamps in coordinator: detection received → formation selected → waypoints published → drones arrive at formation positions
- [ ] Dry run E4: 2 trials, verify reconfiguration time logging per class

### Day 16 (Wed, Apr 16) — Literature Review Part 1
- [ ] Read 8-10 papers: Voronoi coverage (Cortes et al., Schwager), CVT flood monitoring, Enhanced Voronoi + YOLO, adaptive coverage
- [ ] Annotate: method, metrics, agent count, sim/real, limitations, **whether detection feeds back into coverage**
- [ ] Start `paper/references.bib`

### Day 17 (Thu, Apr 17) — Literature Review Part 2
- [ ] Read 8-10 more: bio-inspired SAR swarm, IRADA persistent monitoring, multi-UAV task reallocation survey, YOLOv8 on UAVs, PX4 SITL papers
- [ ] Review dry run results, create bug fix list
- [ ] Draft gap analysis table for Related Work section

### Week 2 Deliverables
- [ ] System handles 5 drones and 10 mixed-class targets
- [ ] All 5 experiments (+ ablations) dry-run validated
- [ ] 20 papers annotated, BibTeX started
- [ ] Final parameter set documented

---

## WEEK 3: Full Experiments

**Goal:** All experiments complete with statistical significance.

### Day 18 (Fri, Apr 18) — Bug Fixes + E1 Adaptive 1-5
- [ ] Fix all issues from dry runs
- [ ] Run adaptive: 5 trials x 180s each (3 drones, 5 mixed-class targets)

### Day 19 (Sat, Apr 19) — E1 Adaptive 6-10 + Static + Lawnmower
- [ ] Finish adaptive trials (6-10)
- [ ] Run static_grid: 10 trials
- [ ] Run lawnmower: 10 trials

### Day 20 (Sun, Apr 20) — E1 Random + E5 Ablations
- [ ] Random: 10 trials
- [ ] binary_voronoi (ablation): 10 trials
- [ ] all_converge (ablation): 10 trials

### Day 21 (Mon, Apr 21) — E2 (Scalability) + E3 (Target Density)
- [ ] E2 (5 drones): 10 trials
- [ ] E3 (3 targets): 10 trials; E3 (10 targets): 10 trials (5-target data reused from E1)

### Day 22 (Tue, Apr 22) — E4 (Response Time) + Reruns
- [ ] E4 response time per class: 10 trials
- [ ] Rerun any crashed/incomplete trials from E1-E5

### Day 23 (Wed, Apr 23) — Results Analysis
- [ ] Aggregate into `data/results/all_results.csv`
- [ ] Compute mean, std, 95% CI per condition
- [ ] Create `plot_results.py`
- [ ] **Key ablation comparison**: full system vs. binary_voronoi vs. all_converge → quantify contribution of each novel element

### Day 24 (Thu, Apr 24) — Figures + Statistics
- [ ] Generate all paper figures (300 DPI, PDF):
  - Bar charts: E1 (6 methods comparison including ablations)
  - Line plots: E2 scalability, E3 target density
  - Box plot: E4 reconfiguration time per class
  - **Voronoi visualization**: before/after detection showing confidence-weighted density heatmap
  - **Coverage-during-event plot**: coverage % maintained during exploit mode (split-and-reform vs. all-converge)
- [ ] Paired t-tests / Wilcoxon signed-rank between adaptive and each baseline/ablation
- [ ] Create system architecture diagram

### Week 3 Deliverables
- [ ] ~100 trials complete (E1: 60 with ablations, E2: 10, E3: 20, E4: 10)
- [ ] Master results CSV
- [ ] All figures generated (publication quality)
- [ ] Statistical analysis with p-values
- [ ] Ablation results quantifying each novel element's contribution

---

## WEEK 4: Paper Writing — Technical Core

**Goal:** Sections 2-7 drafted.

### Day 25 (Fri, Apr 25) — Paper Setup + System Architecture
- [ ] Download MDPI Drones LaTeX template, set up `paper/` directory
- [ ] Write Section 4: System Architecture (~1.5 pages) — simulation stack, ROS 2 node graph, communication, SwarmMode message

### Day 26 (Sat, Apr 26) — Methodology Part 1
- [ ] Write Section 5.1-5.3 (~2 pages): Voronoi partitioning, Lloyd's algorithm, **confidence-weighted density function** (with equations), detection-triggered reallocation
- [ ] Include equations: Voronoi cell definition, Lloyd's centroid, **confidence × priority weighted centroid**, Gaussian density with temporal decay
- [ ] Clearly state how our density function differs from binary approaches (cite CVT flood paper, Enhanced Voronoi paper)

### Day 27 (Sun, Apr 27) — Methodology Part 2 + Experimental Setup
- [ ] Write Section 5.4-5.6: **class-specific response selector** (formation definitions), **dual-mode exploration/exploitation** (split-and-reform algorithm), baseline and ablation method descriptions
- [ ] Write Section 6 (~1 page): simulation details, drone/camera specs, class-priority table, formation parameters table

### Day 28 (Mon, Apr 28) — Results Part 1
- [ ] Write 7.1: E1 baselines comparison (coverage, latency, energy tables + charts)
- [ ] Write 7.2: E2 scalability analysis
- [ ] Write 7.3: **E5 ablation study** — quantify individual contribution of confidence weighting and split-and-reform

### Day 29 (Tue, Apr 29) — Results Part 2 + Discussion
- [ ] Write 7.4: E3 target density, 7.5: E4 response time per class
- [ ] Discussion: why confidence weighting outperforms binary, why split-and-reform preserves coverage, computational overhead, limitations

### Day 30 (Wed, Apr 30) — Related Work
- [ ] Write Section 3 (~2 pages): multi-robot coverage, UAV monitoring, vision-guided ops
- [ ] Include **gap analysis table** (the one from this document) comparing our work to the 6 closest papers
- [ ] Cite 25-35 papers

### Day 31 (Thu, May 1) — Introduction
- [ ] Write Section 2 (~1.5 pages): problem, motivation, contributions:
  1. **Semantic event-driven adaptive Voronoi coverage** with confidence-weighted density function
  2. **Class-specific heterogeneous swarm response** with dual-mode exploration/exploitation
  3. Realistic multi-drone simulation integration (PX4 + Gazebo + ROS 2)
  4. Comprehensive evaluation with **ablation study** against 3 baselines + 2 ablations across 5 dimensions

### Week 4 Deliverables
- [ ] Sections 2-7 drafted (~10-12 pages)
- [ ] All figures and tables inserted (including gap analysis table and ablation results)
- [ ] References cited in text

---

## WEEK 5: Completion + Submission

**Goal:** Paper submitted.

### Day 32 (Fri, May 2) — Conclusion + Abstract + References
- [ ] Section 8: Conclusion (~0.5 pages) — contributions, key numbers, future work (real-world validation, moving targets, heterogeneous sensor payloads, larger swarms 10-50 drones)
- [ ] Abstract (200-250 words) — must mention: semantic event-driven, confidence-weighted Voronoi, class-specific response, dual-mode, ablation study
- [ ] Complete `references.bib` (30-40 entries), verify no missing refs

### Day 33 (Sat, May 3) — Full Review + Figure Polish
- [ ] Read entire paper end-to-end for logical flow
- [ ] Verify all figures/tables referenced in text
- [ ] Polish figure labels, legends, captions, font consistency
- [ ] Ensure gap analysis table clearly positions our work against literature

### Day 34 (Sun, May 4) — Formatting + Proofread
- [ ] MDPI template compliance (author info, keywords, page count)
- [ ] Grammar/spelling check, verify numbers match figures
- [ ] Prepare supplementary: demo video (showing all 3 class-specific formations), GitHub README

### Day 35 (Mon, May 5) — Final Review + Submit
- [ ] One last full read
- [ ] Submit to MDPI Drones + cover letter emphasizing novelty (semantic event-driven response + ablation)

### Week 5 Deliverables
- [ ] Complete paper (all 8 sections + references)
- [ ] Submitted to MDPI Drones
- [ ] Supplementary material (video + code repo) prepared

---

## Experiments

| ID | Question | IV | Conditions | Drones | Targets | Trials |
|----|----------|----|-----------|--------|---------|--------|
| E1 | Does adaptive beat baselines? | Method | adaptive, static, lawnmower, random | 3 | 5 (mixed class) | 10×4 |
| E2 | How does it scale? | Drone count | 3, 5 | 5 (mixed class) | 10×2 |
| E3 | Effect of target density? | Target count | 3, 5, 10 (mixed class) | 3 | 10×3 |
| E4 | How fast does it reconfigure per class? | Event class | person, vehicle, fire | 3 | 5 | 10 |
| **E5** | **What does each novel element contribute?** | **Method** | **full, binary_voronoi (no conf. weighting), all_converge (no split)** | **3** | **5 (mixed class)** | **10×3** |

**Metrics:**
- Coverage % — fraction of area within at least one drone's camera FOV
- Detection latency — time from target placement to first detection
- Redundancy ratio — average drone FOV overlaps per grid cell
- Energy — total distance traveled (sum of position deltas)
- Reconfiguration time (E4) — detection event → drones reach formation positions, **per class**
- **Coverage-during-event** (E5) — coverage % maintained while exploit mode is active

---

## Risk Mitigations

| Risk | Mitigation |
|------|-----------|
| Gazebo unstable at 5 drones | Reduce camera res (320x240), process every 10th frame |
| YOLO can't detect simple objects | Lower confidence, use textured objects, fall back to color detection |
| Adaptive doesn't beat baselines | Frame as evaluation study, focus on response time + ablation contributions |
| Multi-class formations too complex | Start with 2 classes (person + vehicle); add fire only if stable |
| Ablation differences not significant | Increase trials to 15-20 for ablation comparisons |
| Writing takes longer than planned | Draft methodology during Week 3 experiment downtime |

---

## Time Budget

| Week | Focus | Est. Hours |
|------|-------|-----------|
| 1 | Core implementation + novel elements | ~19h |
| 2 | Robustness + dry runs + lit review | ~18h |
| 3 | Full experiments (including ablations) + analysis | ~20h |
| 4 | Paper writing (technical) | ~18h |
| 5 | Paper completion + submission | ~10h |
| **Total** | | **~85h** |

---

*Plan created: 2026-04-04*
*Updated: 2026-04-06 — Added gap analysis, novel elements, ablation baselines (E5), class-specific response, dual-mode exploration/exploitation*
