# ISTRAS'26 — Experiment Runbook (v3)

> **⚠ HISTORICAL — pre-energy-scaling pivot (2026-04-25).** The active
> paper no longer follows the E1/E2/E3 design described here. The
> current reference is **`istras_energy_scaling_guide.md`**; primary
> series is `Sw1`–`Sw5` (lawnmower N ∈ {1..5}, 27 trials at
> 600 m × 40 m × 8 targets); writeup is
> `istras_results_discussion_conclusion.md`. Keep this doc only if
> the v2 E1/E2/E3 framing returns.

> Companion to `istras_development_guide_v2.md`. Lists every experiment trial
> required for the submission, with the exact CLI command to run it.
>
> **Scope (v2 plan):** 3 experiments × multiple methods × multiple trials =
> **31 trials total** (~90 min wall-clock simulation).
> **Output root:** `data/logs/istras/<E1|E2|E3>/`
> **Source-of-truth configs:** `config/experiment_configs/{E1,E2,E3}.yaml`,
> `scripts/run_all_experiments.sh`.

---

## What Changed Since v2 of This File

- **Dynamic target spawning.** Targets no longer all appear at t=0. They are
  scheduled with `--schedule t1,t2,...` (one entry per target) so trials have
  a quiet steady-state phase before events begin firing. This is what makes
  *event response latency* a meaningful metric.
- **Three experiments instead of one.** E1 (head-to-head at 3 drones) is the
  former entire experiment set. E2 (scalability to 5 drones) and E3 (dense
  staggered events) extend it to answer SQ2 and SQ3 from the v2 guide.
- **One-shot batch runner.** `run_all_experiments.sh --experiments E1 E2 E3`
  runs the full battery; `--resume` skips trials that already produced
  `metrics_summary.json`, so re-runs after a Gazebo crash are cheap.
- **Phased distance metric.** `metrics_compute.py` now reports
  `steady_state_distance_m` (t < 20 s, no events) and `event_phase_distance_m`
  (t ≥ 20 s) so reviewers can separate baseline coverage motion from
  event-driven motion.
- **Per-event latency.** Each scheduled target carries its `spawn_t` in
  `targets.csv`; latency is now `first_detection_s − spawn_t`, not
  `first_detection_s − trial_start_s`. The mean is reported as
  `mean_event_latency_s`.

---

## Prerequisites (once per shell session)

```bash
conda activate swarm
source /opt/ros/humble/setup.bash
source ~/Desktop/Swarm/ros2_ws/install/setup.bash
cd ~/Desktop/Swarm
```

Shared parameters across all v3 trials:

| Parameter | Value | Flag |
|---|---|---|
| Monitoring area | 40 × 40 m | `--area-size 40` |
| Flight altitude | 10 m | `--altitude 10` |
| Trial duration | 150 s | `--duration 150` |
| Output root | `data/logs/istras/<E#>/` | `--output-base data/logs/istras/<E#>` |
| Steady-state cutoff | 20 s (E1, E2) or 15 s (E3) | `metrics_compute.py --steady-state-cutoff` |

`--targets` is no longer set explicitly — the scheduler's length determines
target count.

---

## E1 — Head-to-Head at 3 Drones (SQ1)

Three coordination strategies compete on identical worlds, schedules, and
class sequences. Trials vary only by random seed.

| | |
|---|---|
| Methods | `adaptive`, `lawnmower`, `static` |
| Drones | 3 |
| Schedule | `20,50,80` (s) |
| Classes | `person,vehicle,fire` |
| Trials per method | 5 |
| Total trials | **15** |
| Wall-clock estimate | ~40 min |

### Per-trial command pattern

```bash
bash scripts/run_experiment.sh \
  --method <adaptive|lawnmower|static> \
  --drones 3 \
  --schedule 20,50,80 \
  --classes person,vehicle,fire \
  --duration 150 \
  --area-size 40 --altitude 10 \
  --trial <N> --seed <42+N> \
  --output-base data/logs/istras/E1
```

### Inline loop (15 trials)

```bash
for method in adaptive lawnmower static; do
  for trial in 1 2 3 4 5; do
    bash scripts/run_experiment.sh \
      --method $method \
      --drones 3 \
      --schedule 20,50,80 \
      --classes person,vehicle,fire \
      --duration 150 \
      --area-size 40 --altitude 10 \
      --trial $trial --seed $((42 + trial)) \
      --output-base data/logs/istras/E1
  done
done
```

---

## E2 — Scalability to 5 Drones (SQ2)

Re-run only the two contenders that change interestingly with swarm size.
Static-grid scaling is uninteresting (more idle hovering) and is cut.

| | |
|---|---|
| Methods | `adaptive`, `lawnmower` |
| Drones | 5 |
| Schedule | `20,50,80` (s) |
| Classes | `person,vehicle,fire` |
| Trials per method | 5 |
| Total trials | **10** |
| Wall-clock estimate | ~30 min |

```bash
for method in adaptive lawnmower; do
  for trial in 1 2 3 4 5; do
    bash scripts/run_experiment.sh \
      --method $method \
      --drones 5 \
      --schedule 20,50,80 \
      --classes person,vehicle,fire \
      --duration 150 \
      --area-size 40 --altitude 10 \
      --trial $trial --seed $((42 + trial)) \
      --output-base data/logs/istras/E2
  done
done
```

The paper's Figure 3 (scaling curve) is built by joining E1's adaptive +
lawnmower at 3 drones with E2's adaptive + lawnmower at 5 drones — same
schedule, same classes, same trial count, so the only changing variable is
swarm size.

---

## E3 — Dense Dynamic Event Response (SQ3)

Six events at non-uniform spacing within the same 150 s mission. Class
sequence cycles person → vehicle → fire so each class fires twice and
triggers its formation twice.

| | |
|---|---|
| Methods | `adaptive`, `lawnmower` |
| Drones | 3 |
| Schedule | `15,30,45,60,90,120` (s) |
| Classes | `person,vehicle,fire,person,vehicle,fire` |
| Trials per method | 3 |
| Total trials | **6** |
| Wall-clock estimate | ~18 min |

```bash
for method in adaptive lawnmower; do
  for trial in 1 2 3; do
    bash scripts/run_experiment.sh \
      --method $method \
      --drones 3 \
      --schedule 15,30,45,60,90,120 \
      --classes person,vehicle,fire,person,vehicle,fire \
      --duration 150 \
      --area-size 40 --altitude 10 \
      --trial $trial --seed $((42 + trial)) \
      --output-base data/logs/istras/E3
  done
done
```

E3 produces 18 latency data points per method (6 events × 3 trials), enough
to compute a per-event latency distribution for Figure 4.

---

## Run Everything (one-shot)

The batch runner reads the configs above out of `scripts/run_all_experiments.sh`
(mirrored in `config/experiment_configs/E*.yaml` for human inspection).

```bash
# Full battery (31 trials, ~90 min)
bash scripts/run_all_experiments.sh --experiments E1 E2 E3

# Subset
bash scripts/run_all_experiments.sh --experiments E1
bash scripts/run_all_experiments.sh --experiments E1 E2

# Override default trial count (e.g. quick smoke test)
bash scripts/run_all_experiments.sh --experiments E1 --trials 2

# Resume after a Gazebo crash — skip any trial whose metrics_summary.json
# already exists. Fully idempotent.
bash scripts/run_all_experiments.sh --experiments E1 E2 E3 --resume

# Print what would run without launching anything
bash scripts/run_all_experiments.sh --experiments E1 E2 E3 --dry-run
```

The runner picks seeds deterministically as `seed = 42 + trial_index`, so
seed 1 of E1 adaptive matches seed 1 of E1 lawnmower in target placement.

---

## Trial Output Layout

Each trial writes to
`data/logs/istras/<E#>/<method>_d<N>_t<T>_trial<NN>/`:

| File | Content |
|---|---|
| `positions.csv` | `timestamp, elapsed_s, drone_id, latitude, longitude, altitude, x_local, y_local` |
| `detections.csv` | `timestamp, elapsed_s, drone_id, class_name, confidence, bbox_*, world_x, world_y` |
| `waypoints.csv` | `timestamp, elapsed_s, drone_id, latitude, longitude, altitude, priority, x_local, y_local` |
| `swarm_mode.csv` | `timestamp, elapsed_s, mode, exploit_drone_ids, event_class, event_x, event_y, event_confidence` |
| `targets.csv` | `name, x, y, class, spawn_t` — manifest with scheduled spawn time per target |
| `metrics_summary.json` | All metrics computed by `metrics_compute.py` |
| `coverage_over_time.csv` | Sampled `(elapsed_s, coverage_percent)` for plots |
| `paths_combined.png` | 2D path plot |

`spawn_t` defaults to 0 in static (non-scheduled) mode; in v3 experiments
it carries the actual scheduled spawn time, which `metrics_compute.py`
uses to compute per-event latency relative to spawn rather than relative
to trial start.

---

## Metrics Reference

`metrics_compute.py --experiment-id <id> --steady-state-cutoff 20` writes
`metrics_summary.json` containing (selected fields):

| Field | Meaning |
|---|---|
| `total_distance_m` | Sum of inter-sample distances over all drones, full trial |
| `steady_state_distance_m` | Same, restricted to elapsed_s < cutoff (no events yet) |
| `event_phase_distance_m` | Same, restricted to elapsed_s ≥ cutoff (event response regime) |
| `total_energy_wh` / `co2_azerbaijan_g` / `co2_eu_g` | Hover-power × duration × n_drones, with regional grid factors |
| `targets_total` / `targets_found` | Detection coverage in absolute counts |
| `time_to_first_target_s` | Earliest first-detection across all targets |
| `time_to_all_targets_s` | Latest first-detection (only if every target was found) |
| `per_event_latency_s` | Per-target list of `first_detection_s − spawn_t` |
| `mean_event_latency_s` | Mean of the above |
| `schedule_aware` | True if any target had a non-zero `spawn_t` |
| `coverage_percent` / `redundancy_ratio` | Time-averaged FOV coverage and overlap |

For the paper's Figure 2 (E1 head-to-head) report `event_phase_distance_m`
and `mean_event_latency_s` per method. For Figure 3 (E2 scaling) plot
`event_phase_distance_m` vs `n_drones` for each method. For Figure 4 (E3
per-event response) flatten `per_event_latency_s` across trials and bin
by event index.

---

## Post-Experiment Analysis

```bash
# Recompute metrics for every trial after a code change
for d in data/logs/istras/E*/*; do
  python3 scripts/metrics_compute.py \
    --experiment-id "$(basename "$d")" \
    --log-base "$(dirname "$d")" \
    --area-size 40 --altitude 10 --steady-state-cutoff 20
done

# Aggregate every trial's metrics_summary.json into one CSV row per trial
python3 scripts/aggregate_results.py \
  --logs-dir data/logs/istras \
  --output data/results/istras_all_results.csv

# Render comparison plots
python3 scripts/plot_paths.py \
  --log-base data/logs/istras/E1 \
  --area-size 40 --altitude 10
```

---

## Definition of Done

- [ ] `data/logs/istras/E1/` contains 15 trial folders, each with `metrics_summary.json`
- [ ] `data/logs/istras/E2/` contains 10 trial folders, each with `metrics_summary.json`
- [ ] `data/logs/istras/E3/` contains 6 trial folders, each with `metrics_summary.json`
- [ ] Every `targets.csv` has the `spawn_t` column populated for E1/E2/E3 trials
- [ ] `data/results/istras_all_results.csv` has 31 rows
- [ ] Figures 2–4 generated from the aggregate CSV

If a trial fails (Gazebo crash, MAVSDK timeout), the simplest recovery is:

```bash
bash scripts/run_all_experiments.sh --experiments E1 E2 E3 --resume
```

`--resume` skips any trial whose `metrics_summary.json` already exists.

---

## Out of Scope (Saved for MDPI)

The v2 guide explicitly cuts these from ISTRAS'26 and reserves them for
the longer MDPI submission:

- Ablations: `binary_voronoi`, `all_converge`, `pso`, `apf`, `random`
- Larger-than-5-drone scaling
- Target-density sweep (3 vs 5 vs 10 targets)
- Heterogeneous sensors / payloads
- MARL-based coordination
- HSV vs CNN detector comparison
- Real-world deployment

If E3 slips on Day 4, drop it; E1 + E2 alone is still a coherent paper
("head-to-head at 3 drones, plus scalability to 5"). The triage hierarchy
is documented in `istras_development_guide_v2.md` §9.
