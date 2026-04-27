# ISTRAS'26 — Dynamic Experiments Runbook (D-series)

> **⚠ HISTORICAL — pre-energy-scaling pivot (2026-04-25).** The active
> paper drops dynamic-event experiments entirely. The current reference
> is **`istras_energy_scaling_guide.md`**. Keep this doc only for the
> event-response framing if it returns in a follow-up paper.

> **Scope:** D1, D2, D3 — 31 trials. Targets are scheduled to spawn during
> the mission (not all at t=0). Measures event response, reconfiguration
> latency, and concurrent-event handling. Supersedes `istras_experiments_v3.md`.
>
> **Legacy mapping:** D1 replaces the old E1, D2 replaces E2, D3 replaces E3.
> The experiment runner accepts the old names as aliases so existing scripts
> that reference E1/E2/E3 keep working.

---

## 1. What Dynamic Experiments Isolate

Every D-series trial has a quiet steady-state phase (no events yet) followed
by a burst of scheduled target appearances. This lets us measure two things
that the static series can't:

1. **Event response latency** — time from an event's spawn until any drone
   first detects it. Reported as `per_event_latency_s`, `mean_event_latency_s`,
   `median_event_latency_s`, and `max_event_latency_s`.

2. **Reconfiguration time** — time from the detection until all exploit
   drones arrive at their class-specific formation positions. Reported as
   `mean_reconfig_time_s`, `mean_publish_delay_s`.

Adaptive is expected to win both comfortably against lawnmower (which
cannot change its sweep pattern mid-mission) and static-grid (which cannot
change anything). The D-series is where the *core* paper claim lives.

---

## 2. Prerequisites (once per shell session)

```bash
conda activate swarm
source /opt/ros/humble/setup.bash
source ~/Desktop/Swarm/ros2_ws/install/setup.bash
cd ~/Desktop/Swarm
```

Shared parameters across all D-series trials:

| Parameter | Value | Flag |
|---|---|---|
| Monitoring area | 400 × 400 m | `--area-size 400` |
| Flight altitude | 60 m | `--altitude 60` |
| Trial duration | 200 s | `--duration 200` |
| Output root | `data/logs/istras/<D#>/` | `--output-base ...` |
| Steady-state cutoff | 25 s (D1/D2) or 20 s (D3) | `metrics_compute.py --steady-state-cutoff` |

The steady-state cutoff partitions `total_distance_m` into
`steady_state_distance_m` (before first event) and
`event_phase_distance_m` (from first event onward). The paper's
distance comparisons use `event_phase_distance_m` so that matched
baseline coverage (which all methods do during 0–25 s) doesn't dilute
the event-driven behaviour.

---

## 3. D1 — Dynamic Head-to-Head at 3 Drones (RQ-D1)

Three coordination strategies compete on identical schedules and classes.
This is the canonical experiment — even if D2/D3 are cut for time, D1
alone constitutes a publishable result.

| | |
|---|---|
| Methods | `adaptive`, `lawnmower`, `static` |
| Drones | 3 |
| Schedule | `25,85,145` (s) — 3 events, ~60 s apart |
| Classes | `person,vehicle,fire` |
| Trials per method | 5 |
| Total trials | **15** |
| Wall-clock estimate | ~55 min |

### Per-trial command

```bash
bash scripts/run_experiment.sh \
  --method <adaptive|lawnmower|static> \
  --drones 3 \
  --schedule 25,85,145 \
  --classes person,vehicle,fire \
  --duration 200 \
  --area-size 400 --altitude 60 \
  --trial <N> --seed <42+N> \
  --output-base data/logs/istras/D1
```

### Inline loop

```bash
for method in adaptive lawnmower static; do
  for trial in 1 2 3 4 5; do
    bash scripts/run_experiment.sh \
      --method $method \
      --drones 3 \
      --schedule 25,85,145 \
      --classes person,vehicle,fire \
      --duration 200 \
      --area-size 400 --altitude 60 \
      --trial $trial --seed $((42 + trial)) \
      --output-base data/logs/istras/D1
  done
done
```

---

## 4. D2 — Dynamic Scaling to 5 Drones (RQ-D2)

Re-run only the two methods that change interestingly with swarm size.
Static-grid is cut.

| | |
|---|---|
| Methods | `adaptive`, `lawnmower` |
| Drones | 5 |
| Schedule | `25,85,145` (same as D1) |
| Classes | `person,vehicle,fire` |
| Trials per method | 5 |
| Total trials | **10** |
| Wall-clock estimate | ~40 min |

```bash
for method in adaptive lawnmower; do
  for trial in 1 2 3 4 5; do
    bash scripts/run_experiment.sh \
      --method $method \
      --drones 5 \
      --schedule 25,85,145 \
      --classes person,vehicle,fire \
      --duration 200 \
      --area-size 400 --altitude 60 \
      --trial $trial --seed $((42 + trial)) \
      --output-base data/logs/istras/D2
  done
done
```

**For 5 drones:** edit `config/adaptive_params.yaml` and set
`min_explore_drones: 2` before running D2. This keeps at least two
drones on Voronoi coverage while up to three serve a fire perimeter.
Restore to 1 before returning to D1/D3 at 3 drones.

The paper's Figure 3 (scaling curve) is built by joining D1's adaptive
and lawnmower at 3 drones with D2's at 5 drones — same schedule, same
classes, same trial count, so swarm size is the only changing variable.

---

## 5. D3 — Dense Dynamic Event Response (RQ-D3)

Six events at non-uniform spacing in the same 200 s mission. Class
sequence cycles person→vehicle→fire so each class fires twice and each
formation is exercised twice. Mean inter-event gap is ~31 s — just at
the `exploit_timeout_s = 30 s` threshold — so events will frequently
overlap, exercising the new `max_concurrent_exploits = 2` logic.

| | |
|---|---|
| Methods | `adaptive`, `lawnmower` |
| Drones | 3 |
| Schedule | `20,45,75,105,140,175` (s) |
| Classes | `person,vehicle,fire,person,vehicle,fire` |
| Trials per method | 3 |
| Total trials | **6** |
| Wall-clock estimate | ~22 min |

```bash
for method in adaptive lawnmower; do
  for trial in 1 2 3; do
    bash scripts/run_experiment.sh \
      --method $method \
      --drones 3 \
      --schedule 20,45,75,105,140,175 \
      --classes person,vehicle,fire,person,vehicle,fire \
      --duration 200 \
      --area-size 400 --altitude 60 \
      --trial $trial --seed $((42 + trial)) \
      --output-base data/logs/istras/D3
  done
done
```

D3 produces 18 latency data points per method (6 events × 3 trials),
enough to compute the per-event latency distribution for Figure 4.

### D3 diagnostics to watch

Under this schedule, events 2–3 and 5–6 will fall within 30 s of their
predecessors. Two things the analysis should check:

- `targets_found == 6` for every D3 trial. If adaptive drops below 6
  while lawnmower covers 6/6 the `max_concurrent_exploits` cap is too
  aggressive (or `min_explore_drones` is stealing too many).
- `max_event_latency_s` across trials. The old single-slot design
  would produce gigantic max values when a second event spawned during
  an exploit; with concurrent slots the distribution should tighten.

---

## 6. Batch (one-shot)

```bash
# All dynamic trials
bash scripts/run_all_experiments.sh --experiments D1 D2 D3

# Legacy aliases also work:
bash scripts/run_all_experiments.sh --experiments E1 E2 E3

# Subset + resume
bash scripts/run_all_experiments.sh --experiments D1
bash scripts/run_all_experiments.sh --experiments D1 D2 D3 --resume

# Dry-run (prints the plan without launching)
bash scripts/run_all_experiments.sh --experiments D1 D2 D3 --dry-run
```

The runner picks seeds deterministically as `seed = 42 + trial_index`
so trial 1 of D1/adaptive matches trial 1 of D1/lawnmower in spawn
locations — every method sees the same target positions.

---

## 7. Expected Metrics Per Trial

| Metric | Why it matters for the dynamic series |
|---|---|
| `per_event_latency_s` | Per-target list of `first_detection_s − spawn_t` |
| `mean_event_latency_s` | Headline response metric — primary D-series result |
| `median_event_latency_s` | Robust alternative when one event misfires |
| `max_event_latency_s` | Worst-case responsiveness — drops events are flagged here |
| `event_phase_distance_m` | Distance travelled during the event regime (cutoff onward) |
| `mean_reconfig_time_s` | Time from detection → all exploit drones arrived at formation |
| `coverage_during_exploit` / `coverage_during_explore` | Does splitting degrade baseline coverage? |
| `num_exploit_events` | Count of exploit activations; under D3, should be close to 6 for adaptive |
| `targets_found` / `targets_total` | Completeness — don't celebrate adaptive's latency if it dropped events |

---

## 8. Post-Processing

```bash
# Recompute metrics for every trial after a code change
for d in data/logs/istras/D*/*; do
  # D1 and D2 use steady-state cutoff 25; D3 uses 20.
  cutoff=25
  case "$d" in *data/logs/istras/D3/*) cutoff=20 ;; esac
  python3 scripts/metrics_compute.py \
    --experiment-id "$(basename "$d")" \
    --log-base "$(dirname "$d")" \
    --area-size 400 --altitude 60 --steady-state-cutoff "$cutoff"
done

# Aggregate every trial's metrics_summary.json into one CSV
python3 scripts/aggregate_results.py \
  --logs-dir data/logs/istras \
  --output data/results/istras_dynamic_results.csv

# Render comparison plots
python3 scripts/plot_paths.py \
  --log-base data/logs/istras/D1 \
  --area-size 400 --altitude 60

# Close-vs-far separation analysis across ALL trials
python3 scripts/analysis/target_separation_analysis.py \
  --logs-dir data/logs/istras \
  --output-dir data/results/separation_analysis \
  --plots
```

---

## 9. Trial Output Layout

Each trial writes to
`data/logs/istras/<D#>/<method>_d<N>_t<T>_trial<NN>/`:

| File | Content |
|---|---|
| `positions.csv` | `timestamp, elapsed_s, drone_id, latitude, longitude, altitude, x_local, y_local` |
| `detections.csv` | `timestamp, elapsed_s, drone_id, class_name, confidence, bbox_*, world_x, world_y` |
| `waypoints.csv` | `timestamp, elapsed_s, drone_id, latitude, longitude, altitude, priority, x_local, y_local` |
| `swarm_mode.csv` | `timestamp, elapsed_s, mode, exploit_drone_ids, event_class, event_x, event_y, event_confidence` |
| `targets.csv` | `name, x, y, class, spawn_t` — manifest with scheduled spawn time per target |
| `reconfig_events.csv` | One row per reconfiguration (detection → arrival); aggregated across concurrent slots |
| `metrics_summary.json` | All metrics computed by `metrics_compute.py` |
| `coverage_over_time.csv` | Sampled `(elapsed_s, coverage_percent)` for plots |
| `paths_combined.png` | 2D path plot |

---

## 10. Definition of Done

- [ ] `data/logs/istras/D1/` contains 15 trial folders, each with `metrics_summary.json`
- [ ] `data/logs/istras/D2/` contains 10 trial folders, each with `metrics_summary.json`
- [ ] `data/logs/istras/D3/` contains 6 trial folders, each with `metrics_summary.json`
- [ ] Every `targets.csv` has non-zero `spawn_t` values matching the experiment's schedule
- [ ] Every D3 `metrics_summary.json` shows `num_exploit_events ≥ 5` for adaptive
- [ ] `data/results/istras_dynamic_results.csv` has ≥31 rows
- [ ] Figures 3 and 4 generated from the aggregate CSV

If a trial fails (Gazebo crash, MAVSDK timeout), the simplest recovery is:

```bash
bash scripts/run_all_experiments.sh --experiments D1 D2 D3 --resume
```

`--resume` skips any trial whose `metrics_summary.json` already exists.

---

## 11. Out of Scope (Saved for MDPI)

Reserved for the longer MDPI follow-up:

- Ablations: `binary_voronoi`, `all_converge`, `pso`, `apf`, `random`
- Larger-than-5-drone scaling
- Target-density sweep (3 vs 5 vs 10 targets)
- Heterogeneous sensors / payloads
- MARL-based coordination
- HSV vs CNN detector comparison
- Real-world deployment

If D3 slips on Day 4, drop it; D1 + D2 + S1 + S2 is still a coherent
paper ("static and dynamic head-to-head, plus scaling to 5"). The
triage hierarchy is documented in `istras_final_guide.md` §7.
