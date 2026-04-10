# Week 2 Task Report — Detailed Breakdown

**Project:** Vision-Guided Adaptive Area Monitoring with Drone Swarms
**Week:** 2 (Days 11–17), April 6–17, 2026
**Status:** In progress — Days 11–13 completed

---

## Overview

Week 2 focuses on **robustness, scalability, and validation**. The goal is to
ensure the system handles 5 drones + 10 targets, dry-run all 5 experiment
configurations (E1–E5 including ablations), and complete a 20-paper literature
review. All 3 novel elements were implemented on Day 12.

---

## Day 11 (Apr 6) — Simulation Realism & Scalability ✅ COMPLETED

**Planned tasks (from `adaptive_area_monitoring.md`):**
- Enhance `spawn_targets.sh`: `--count N`, `--random`, `--seed S`
- Extend `camera_bridge.yaml` for 5 drones
- Test 5-drone launch

**Actual work completed:**

### 1. Custom Gazebo World — Shadow Removal
- **Problem:** Drone shadows on ground plane caused YOLO false positives
  (classified as "kite", "airplane").
- **Solution:** Created `worlds/default.sdf` — copy of PX4's default world
  with `<shadows>false</shadows>` and `<cast_shadows>false</cast_shadows>`.
- **Challenge:** PX4's `gz_env.sh` overrides `PX4_GZ_WORLDS` at startup,
  so external env vars have no effect.
- **Final approach:** Symlink `worlds/default.sdf` into PX4's worlds directory
  as `swarm_default.sdf`, set `PX4_GZ_WORLD=swarm_default`.
- **Files changed:** `worlds/default.sdf` (new), `scripts/launch_multi_drone.sh`
- **Doc:** `docs/gazebo_disable_shadows.md`

### 2. Realistic Target Objects (Gazebo Fuel Vehicles)
- **Problem:** Geometric shapes (boxes, cylinders) are not in the COCO dataset.
  YOLOv8 cannot reliably detect them.
- **Solution:** Rewrote `scripts/spawn_targets.sh` to use 3D vehicle models
  from Gazebo Fuel: SUV, Hatchback, Hatchback Red, Hatchback Blue, Pickup.
- **COCO classes:** `car`, `truck` — reliably detected by YOLOv8.
- **Models downloaded from:** `https://fuel.gazebosim.org/1.0/OpenRobotics/models/`
- **Dropped models:** Prius (Ogre1 `<script>` materials → black), Bus (broken
  `model://suv/...` wheel texture references).
- **Texture fix:** Patched `.mtl` files in `~/.gz/fuel/` cache for hatchback
  variants — bare filenames and `model://` URIs replaced with correct relative
  paths (`../materials/textures/filename.png`). Copied missing `wheels3.png`
  from base hatchback to red/blue variants.
- **Files changed:** `scripts/spawn_targets.sh` (rewritten)
- **Doc:** `docs/gazebo_fuel_targets.md`

### 3. Monitoring Area Scaled to 200×200m
- **Previous:** 40×40m area — too small for meaningful swarm coverage experiments.
- **New:** 200×200m area. Targets spread 40–60m apart, 60m+ from origin.
- **Target positions:**
  | Model | X | Y |
  |-------|---|---|
  | SUV | 70 | -60 |
  | Hatchback Red | -80 | 70 |
  | Hatchback Blue | 50 | 80 |
  | Pickup | -60 | -50 |
  | Hatchback | 85 | -30 |
- **Files changed:** `scripts/run_experiment.sh` (`AREA_SIZE=200`),
  `scripts/spawn_targets.sh` (new positions)

### 4. Dynamic Camera Bridge for N Drones
- **Problem:** `camera_bridge.yaml` was hardcoded for 3 drones. Adding drone 4
  or 5 required manual YAML editing.
- **Solution:** `run_experiment.sh` and `run_adaptive.sh` now generate the
  camera bridge YAML at runtime based on `NUM_DRONES`:
  ```bash
  for i in $(seq 1 $NUM_DRONES); do
      cat >> "$BRIDGE_YAML" <<YAML
  - ros_topic_name: /drone${i}/camera/image_raw
    gz_topic_name: /world/${GZ_WORLD}/model/x500_mono_cam_down_${i}/...
  YAML
  done
  ```
- Uses `ros2 run ros_gz_bridge parameter_bridge` instead of launch file.
- **Files changed:** `scripts/run_experiment.sh`, `scripts/run_adaptive.sh`

### 5. Detection Coordinate Bug Fix
- **Problem:** `data_logger.py` logged `msg.world_position` (drone-relative
  offsets) as global coordinates. Detections appeared scattered randomly on plots.
- **Fix:** Added `self._last_pos` dict to track each drone's last known position.
  Detection callback now computes `global_x = drone_x + offset_x`.
- **Files changed:** `scripts/data_logger.py`

### 6. Flight Path Visualization Tool
- **Created:** `scripts/plot_paths.py` — generates combined + per-drone path plots.
- **Features:**
  - Colored flight path lines per drone
  - Start markers (▲) and end markers (■)
  - Ground-truth target positions (red ✕ markers)
  - Detection locations (gold ★ markers)
  - Auto-scaled bounds (handles drones starting outside monitoring area)
  - Legend placed outside plot to avoid overlap
  - Supports `--targets-file` for custom target CSV
- **Integrated into:** `run_experiment.sh` (auto-generates after metrics computation)
- **Files created:** `scripts/plot_paths.py`

### 7. Attempted Realistic Environment — Reverted
- Downloaded House 1/2/3 and Oak/Pine Tree from Gazebo Fuel.
- All house models rendered solid black (Ogre1 `<script>` materials incompatible
  with Gazebo Harmonic's Ogre2 renderer).
- Attempted PBR injection into DAE files — unsuccessful.
- **Decision:** Reverted to plain ground plane. Focus on vehicle targets only.

### Day 11 Deviations from Plan
| Planned | Actual | Reason |
|---------|--------|--------|
| `--count N`, `--random`, `--seed S` for spawn_targets.sh | `NUM_TARGETS` arg only, no random/seed yet | Focused on replacing geometric shapes with Fuel models first; completed on Day 12 |
| Extend camera_bridge.yaml for 5 drones | Dynamic generation at runtime | Better solution — works for any N |
| Test 5-drone launch | Not tested yet | Shadow/target issues took priority |

---

## Day 12 (Apr 7) — Novel Elements + Ablation Baselines ✅ COMPLETED

**Planned tasks (from `adaptive_area_monitoring.md` Days 6–9, compressed):**
- Implement all 3 novel elements
- Create ablation baselines
- Multi-class target spawning

**Actual work completed:**

### 1. SwarmMode.msg ROS 2 Message
- New message type for explore/exploit mode transitions
- Fields: mode (EXPLORE/EXPLOIT), exploit_drone_ids[], event_class, event_position, event_confidence
- Added to `CMakeLists.txt` for build
- **Files:** `ros2_ws/src/swarm_msgs/msg/SwarmMode.msg`, `CMakeLists.txt`

### 2. Configuration File (`config/adaptive_params.yaml`)
- Class priorities: person=0.9, vehicle/car/truck=0.6, fire=1.0
- Class-specific formations:
  - person → cluster (2 drones, 3m radius)
  - vehicle → chain (2 drones, 8m separation)
  - fire → perimeter (3 drones, 10m radius)
- Exploit parameters: confidence_threshold=0.4, timeout=30s, confidence_drop=0.2
- **Files:** `config/adaptive_params.yaml` (new)

### 3. Novel Element 2: Confidence-Weighted Density (voronoi_utils.py)
- `DetectionEvent` expanded with `confidence`, `class_priority`, `class_name`
- `DensityMap` supports `confidence_weighted` flag:
  - True: `weight = confidence × class_priority` (novel)
  - False: `weight = 1.0` (binary — for ablation baseline)
- **Files:** `scripts/voronoi_utils.py`

### 4. Novel Element 1: Class-Specific Response Selector
- `ResponseSelector` class loads formation configs from YAML
- `select_exploit_drones()`: picks K nearest drones, respects `min_explore`
- `compute_formation()`: cluster/chain/perimeter layouts around detection
- **Files:** `scripts/response_selector.py` (new)

### 5. Novel Element 3: Dual-Mode Coordinator (voronoi_coordinator.py rewrite)
- `ExploitEvent` class tracks active exploitation (class, position, timeout)
- On high-confidence detection → enters exploit mode:
  - K nearest drones fly class-specific formation
  - Remaining drones continue Voronoi coverage
- Reform after timeout or confidence drop
- Publishes `SwarmMode` on `/swarm/mode`
- Auto-discovers `config/adaptive_params.yaml`
- **Files:** `scripts/voronoi_coordinator.py` (rewritten)

### 6. Ablation Baseline: Binary Voronoi
- Same Voronoi algorithm, but `confidence_weighted=False`
- All detections weighted equally (1.0) regardless of confidence/class
- No split-and-reform (all drones in Voronoi)
- Isolates contribution of Novel Element 2
- **Files:** `scripts/baselines/binary_voronoi.py` (new)

### 7. Ablation Baseline: All-Converge
- Uses confidence × priority weighting (same as full system)
- But ALL drones converge on detections (no split-and-reform)
- No class-specific formations
- Isolates contribution of Novel Element 3
- **Files:** `scripts/baselines/all_converge.py` (new)

### 8. Multi-Class Target Spawning (spawn_targets.sh)
- New flags: `--classes person,vehicle,fire`, `--count N`, `--random`, `--seed S`
- Person models: Rescue Randy, Casual female (Gazebo Fuel)
- Fire model: inline SDF orange box (3×3m ground patch)
- Legacy positional args still supported for backwards compatibility
- **Files:** `scripts/spawn_targets.sh` (rewritten)

### 9. Data Logger + Metrics Updates
- `data_logger.py`: subscribes to `/swarm/mode`, logs to `swarm_mode.csv`
- `metrics_compute.py`: new `compute_coverage_during_event()` — computes
  coverage % during exploit vs explore periods
- **Files:** `scripts/data_logger.py`, `scripts/metrics_compute.py`

### 10. Experiment Automation Updates
- `run_experiment.sh`: added `binary_voronoi`, `all_converge` methods; `--classes` flag
- `run_all_experiments.sh`: added E5 ablation study batch
- `run_adaptive.sh`: updated defaults (area=200m, altitude=40m, sigma=15m)
- **Files:** `scripts/run_experiment.sh`, `scripts/run_all_experiments.sh`, `scripts/run_adaptive.sh`

### 11. Documentation
- Created `docs/custom_messages.md`: complete reference for all 3 ROS 2 messages
- Updated `docs/baselines.md`: added binary_voronoi, all_converge, ablation study design
- Updated `docs/progress_report.md`: Day 7 entry, architecture diagram, file lists
- **Files:** `docs/custom_messages.md` (new), `docs/baselines.md`, `docs/progress_report.md`

---

## Day 13 (Apr 8) — Edge Cases + PSO/APF + Response Time ✅ COMPLETED

**Planned tasks:**
- [x] Handle edge cases:
  - No detections → uniform Voronoi (already implemented in coordinator)
  - Same target seen by all drones → cap exploit set via `min_explore_drones`
  - Drone disconnection → skip in Voronoi computation
  - Simultaneous multi-class events → prioritize by class_priority
- [x] Implement PSO and APF baseline controllers
- [x] Add `--no-detections` flag to PSO/APF
- [x] Add response time instrumentation to voronoi_coordinator.py
- [x] Add response time metrics to metrics_compute.py
- [x] Add PSO/APF to run_all_experiments.sh (E1: 6 methods)
- [x] Create `paper/references.bib` (25 key references)
- [ ] Profile CPU/GPU utilization during full pipeline (deferred to dry runs)

**Actual work completed:**

### 1. Edge Case Verification
All 4 edge cases were already handled in the coordinator from Day 12:
- No detections → `polygon_centroid()` used instead of `weighted_centroid()` (line 323)
- Exploit cap → `min_explore_drones` parameter enforced in `response_selector.py`
- Disconnection → `_update_voronoi()` returns early if not all drones have positions
- Multi-class priority → `class_priority` weighting applied in density map

### 2. PSO Baseline Controller (`scripts/baselines/pso_coverage.py`)
- Particle Swarm Optimization: each drone is a particle with position + velocity
- Standard PSO equations: `v = w*v + c1*r1*(pbest-x) + c2*r2*(gbest-x)`
- Fitness: uncovered area attraction + inter-drone repulsion penalty + optional detection bonus
- Parameters: w=0.7, c1=1.5, c2=1.5, v_max=15m/step
- Bug fix: random initial velocities (zero init caused drones to stay put)

### 3. APF Baseline Controller (`scripts/baselines/apf_coverage.py`)
- Artificial Potential Field: 4 virtual forces per drone
- Forces: drone-drone repulsion, uncovered-area attraction, detection attraction, boundary repulsion
- Parameters: k_rep=500, k_att=0.5, k_det=20, k_bnd=50, max_step=15m
- Bug fix: full-grid sparse sampling + reduced distance falloff (1/dist^1.5)

### 4. Detection Toggle (`--no-detections` flag)
- Both PSO and APF support `--no-detections` CLI flag
- When set, controllers don't subscribe to Detection topics at all
- `run_experiment.sh` passes the flag through for pso/apf methods

### 5. Response Time Instrumentation (E4)
- `voronoi_coordinator.py`: tracks `t_detect`, `t_publish`, `t_arrive` per exploit event
- Arrival detection: checks if exploit drones are within 5m of formation waypoints
- On exploit exit: logs reconfiguration event to `data/logs/reconfig_events.csv`
- Fields: timestamp, event_class, event_x/y, confidence, t_detect, t_publish, t_arrive, reconfig_time_s, publish_delay_s, exploit_drones, reason
- `metrics_compute.py`: new `compute_response_time()` function reads reconfig CSV
- Summary includes: mean/min/max reconfig time, mean publish delay, event count

### 6. Experiment Automation Updates
- `run_all_experiments.sh`: E1 now runs 6 methods (added pso, apf)
- `run_experiment.sh`: already had pso/apf cases from earlier session

### 7. Literature References (`paper/references.bib`)
- 25 BibTeX entries covering:
  - Surveys: Chung 2018, Sai 2023
  - Voronoi coverage: Cortes 2004, Schwager 2009, Lloyd 1982, Du 1999
  - Object detection: YOLO, YOLOv8, VisDrone
  - PSO/APF: Kennedy 1995, Khatib 1986
  - MARL: MAPPO (Yu 2022), QMIX (Rashid 2020)
  - Simulation: PX4, ROS 2, Gazebo, MAVSDK
  - Coverage planning: Galceran 2013, Cabreira 2019
  - Formation control: Oh 2015
  - Adaptive monitoring: Nigam 2012, Robin 2016

**Key parameters to tune (during dry runs):**
| Parameter | Current | Range to Test |
|-----------|---------|---------------|
| `AREA_SIZE` | 200m | 100–200m |
| `ALTITUDE` | 40m | 20–40m |
| `CONFIDENCE` | 0.15 | 0.10–0.30 |
| `detection_weight` | 10.0 | 5.0–20.0 |
| `detection_sigma` | 15.0m | 5.0–20.0m |
| `decay_half_life` | 30s | 15–60s |
| `exploit_confidence_threshold` | 0.4 | 0.3–0.5 |
| `exploit_timeout_s` | 30s | 20–45s |

---

## Day 14 (Apr 9) — Dry Run: E1 (Baselines) + E5 (Ablations) ⬜ PENDING

**Planned tasks:**
- [ ] Run 2 trials each: adaptive, static, lawnmower, random, pso, apf, binary_voronoi, all_converge
  - 3 drones, 5 mixed-class targets, 120s duration
- [ ] Verify data logging produces valid CSVs for all 8 methods
- [ ] Verify `swarm_mode.csv` is generated for adaptive method
- [ ] Verify metrics computation works on all trial data (including coverage-during-event)
- [ ] Verify path plot generation for all trials
- [ ] Debug process cleanup between consecutive trials

**Expected output per trial:**
```
data/logs/<method>_d3_t5_trial0X/
├── positions.csv
├── detections.csv
├── waypoints.csv
├── swarm_mode.csv     (adaptive only)
├── metrics_summary.json
├── coverage_over_time.csv
├── paths_combined.png
├── path_drone1.png
├── path_drone2.png
└── path_drone3.png
```

---

## Day 15 (Apr 10) — Dry Run: E2 + E3 ⬜ PENDING

**Planned tasks:**
- [ ] E2 — Scalability: adaptive, **5 drones**, 5 targets, 2 trials
  - First real test with 5 drones
  - Monitor Gazebo performance (FPS, memory)
  - Verify camera bridge works for drones 4 and 5
- [ ] E3 — Target density: adaptive, 3 drones, **10 targets**, 2 trials
  - Requires `spawn_targets.sh` to support >5 targets (positions 6–10 already
    defined in the script)
  - Test how coordinator handles high-density detection events

**Risk:** Gazebo may struggle with 5 drones + cameras. Mitigation: reduce
camera resolution to 320×240, process every 10th frame in detection_publisher.

---

## Day 15 (cont.) — Response Time Dry Run (E4) ⬜ PENDING

**Note:** Response time instrumentation was completed on Day 13. Only dry run remains.

**Planned tasks:**
- [x] Add timestamps to coordinator for reconfiguration timing (DONE Day 13)
- [x] Log reconfiguration events to `reconfig_events.csv` (DONE Day 13)
- [ ] Dry run E4: 2 trials, verify reconfiguration time is captured

---

## Day 16 (Apr 11) — Literature Review Part 1 ⬜ PENDING

**Planned tasks:**
- [ ] Read 8–10 papers on:
  - Voronoi-based multi-robot coverage (Cortes et al. 2004, Schwager et al. 2009)
  - Vision-based UAV monitoring systems
  - Adaptive area coverage algorithms
- [ ] For each paper, annotate:
  - Method used
  - Metrics reported
  - Number of agents
  - Simulation vs. real hardware
  - Key limitations
- [x] Start `paper/references.bib` (DONE Day 13 — 25 entries)

**Target papers (Tier 1 — surveys):**
1. Cortes et al., "Coverage control for mobile sensing networks" (IEEE Trans. Robotics, 2004)
2. Schwager et al., "Decentralized, adaptive coverage control" (IJRR, 2009)
3. Chung et al., "A Survey on Aerial Swarm Robotics" (IEEE Trans. Robotics, 2018)
4. Sai et al., "Comprehensive Survey on AI for UAVs" (IEEE, 2023)
5. MDPI Drones survey on UxV Swarms (2025)

---

## Day 17 (Apr 12) — Literature Review Part 2 + Week Review ⬜ PENDING

**Planned tasks:**
- [ ] Read 8–10 more papers on:
  - YOLOv8 on UAV platforms (VisDrone dataset papers)
  - PX4 SITL simulation architecture
  - Coverage path planning surveys
- [ ] Review all dry run results from Days 13–15
- [ ] Create bug fix checklist for Week 3
- [ ] Update `adaptive_area_monitoring.md` with any plan changes

---

## Week 2 Deliverables Summary

| # | Deliverable | Status | Day |
|---|-------------|--------|-----|
| 1 | Custom Gazebo world (no shadows) | ✅ Done | 11 |
| 2 | Realistic Fuel vehicle targets | ✅ Done | 11 |
| 3 | 200×200m monitoring area | ✅ Done | 11 |
| 4 | Dynamic camera bridge (N drones) | ✅ Done | 11 |
| 5 | Detection coordinate bug fix | ✅ Done | 11 |
| 6 | Flight path visualization tool | ✅ Done | 11 |
| 7 | `--count`, `--random`, `--seed`, `--classes` for spawn_targets.sh | ✅ Done | 12 |
| 8 | `SwarmMode.msg` ROS 2 message | ✅ Done | 12 |
| 9 | `config/adaptive_params.yaml` | ✅ Done | 12 |
| 10 | Novel Element 1: class-specific response selector | ✅ Done | 12 |
| 11 | Novel Element 2: confidence-weighted density function | ✅ Done | 12 |
| 12 | Novel Element 3: dual-mode explore/exploit coordinator | ✅ Done | 12 |
| 13 | Ablation baseline: binary_voronoi | ✅ Done | 12 |
| 14 | Ablation baseline: all_converge | ✅ Done | 12 |
| 15 | Coverage-during-event metric | ✅ Done | 12 |
| 16 | SwarmMode logging in data_logger | ✅ Done | 12 |
| 17 | E5 ablation experiment automation | ✅ Done | 12 |
| 18 | Custom messages documentation | ✅ Done | 12 |
| 19 | Edge case handling (verified all 4 cases) | ✅ Done | 13 |
| 20 | PSO baseline controller | ✅ Done | 13 |
| 21 | APF baseline controller | ✅ Done | 13 |
| 22 | `--no-detections` flag for PSO/APF | ✅ Done | 13 |
| 23 | Response time instrumentation (t_detect/t_publish/t_arrive) | ✅ Done | 13 |
| 24 | Response time metrics in metrics_compute.py | ✅ Done | 13 |
| 25 | PSO/APF added to run_all_experiments.sh (E1: 6 methods) | ✅ Done | 13 |
| 26 | `paper/references.bib` (25 entries) | ✅ Done | 13 |
| 27 | 5-drone launch test | ⬜ Pending | 14 |
| 28 | E1 + E5 dry run (8 methods × 2 trials) | ⬜ Pending | 14 |
| 29 | E2 dry run (5 drones × 2 trials) | ⬜ Pending | 15 |
| 30 | E3 dry run (10 targets × 2 trials) | ⬜ Pending | 15 |
| 31 | E4 response time dry run | ⬜ Pending | 15 |
| 32 | Literature review (20 papers annotated) | ⬜ Pending | 16–17 |
| 33 | Bug fix checklist for Week 3 | ⬜ Pending | 17 |

---

## Risk Assessment

| Risk | Likelihood | Impact | Mitigation |
|------|-----------|--------|------------|
| Gazebo crashes with 5 drones + cameras | Medium | High | Reduce camera res to 320×240, process every 10th frame |
| YOLO misses Fuel vehicles at altitude | Low | Medium | Lower confidence (0.10), reduce altitude (20m) |
| Trials produce 0 detections | Medium | High | Verify targets visible from flight altitude before batch runs |
| Literature review takes >2 days | Medium | Low | Prioritize 10 most relevant papers, skim the rest |
| Process cleanup fails between trials | Medium | Medium | Add `kill_all.sh` helper, verify clean state before each trial |
