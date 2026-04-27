# ISTRAS'26 — Static Experiments Runbook (S-series)

> **⚠ HISTORICAL — pre-energy-scaling pivot (2026-04-25).** The active
> paper no longer runs the S1/S2 cells described here. The current
> reference is **`istras_energy_scaling_guide.md`**, whose primary
> series is `Sw1/Sw3/Sw5` lawnmower at 600 m × 40 m × 8 targets with
> run-until-completion semantics. Geometry, methods, and metrics here
> do not match the active scaling cells — keep this doc only for
> recovering the original adaptive-vs-baseline framing if it returns.

> **Scope:** S1, S2 — 25 trials. Targets spawn at t=0 (no schedule). Measures
> pure coverage efficiency, isolated from event response. Companion to
> `istras_experiments_dynamic.md` and the ground-truth `istras_final_guide.md`.

---

## 1. What Static Experiments Isolate

In the static series, every target is already in the world before the first
Voronoi update fires. The swarm starts from a cold state (nothing detected
yet) and has to *search* the whole area to find targets. This isolates the
cooperative-search efficiency of a method from its event-response reflexes.

**This matters because** if adaptive wins only on scheduled events (the
dynamic series) a reviewer can fairly ask "so you just have faster
triggers — can your method actually find things better?" The static series
is the rebuttal to that question.

**What we expect to see:**
- On uniformly-scattered targets, adaptive ≈ lawnmower on coverage %
  (lawnmower is provably complete for its own footprint pattern).
- When targets are clustered (low min_pairwise_target_distance_m),
  adaptive should find targets *faster* — its density bumps pull drones
  toward the cluster after the first detection, speeding subsequent ones.
- Static-grid should be uniformly worst — no reaction at all.

---

## 2. Prerequisites (once per shell session)

```bash
conda activate swarm
source /opt/ros/humble/setup.bash
source ~/Desktop/Swarm/ros2_ws/install/setup.bash
cd ~/Desktop/Swarm
```

Shared parameters across all S-series trials:

| Parameter | Value | Flag |
|---|---|---|
| Monitoring area | 400 × 400 m | `--area-size 400` |
| Flight altitude | 60 m | `--altitude 60` |
| Trial duration | 200 s | `--duration 200` |
| Targets per trial | 8 | `--targets 8` |
| Output root | `data/logs/istras/<S#>/` | `--output-base ...` |
| Seed per trial | 42 + trial_index | `--seed $((42 + trial))` |

Target positions are drawn uniformly inside 80% of the area (to keep them
visible from any overhead drone). The seed drives the draw — trial N of
adaptive and trial N of lawnmower see the **same** target layout.

---

## 3. S1 — Static Head-to-Head at 3 Drones (RQ-S1)

Three coordination strategies compete on identical static worlds.

| | |
|---|---|
| Methods | `adaptive`, `lawnmower`, `static` |
| Drones | 3 |
| Targets | 8 (classes cycle person → vehicle → fire) |
| Schedule | — (all at t=0) |
| Trials per method | 5 |
| Total trials | **15** |
| Wall-clock estimate | ~50 min |

### Per-trial command

```bash
bash scripts/run_experiment.sh \
  --method <adaptive|lawnmower|static> \
  --drones 3 \
  --targets 8 \
  --classes person,vehicle,fire \
  --duration 200 \
  --area-size 400 --altitude 60 \
  --trial <N> --seed <42+N> \
  --output-base data/logs/istras/S1
```

### Inline loop (15 trials)

```bash
for method in lawnmower; do
  for trial in 1 2 3 4 5; do
    bash scripts/run_experiment.sh \
      --method $method \
      --drones 3 \
      --targets 8 \
      --classes person,vehicle,fire \
      --duration 400 \
      --area-size 600 --altitude 40 \
      --trial $trial --seed $((42 + trial)) \
      --output-base data/logs/istras/S
  done
done
```

---

## 4. S2 — Static Scaling to 5 Drones (RQ-S2)

Only the two contenders that *change* with swarm size (adaptive, lawnmower).
Static-grid is cut here for the same reason it was cut from the original
E2/D2 — more idle hovering with more drones is uninteresting.

| | |
|---|---|
| Methods | `adaptive`, `lawnmower` |
| Drones | 5 |
| Targets | 8 |
| Schedule | — |
| Trials per method | 5 |
| Total trials | **10** |
| Wall-clock estimate | ~35 min |

```bash
for method in adaptive lawnmower; do
  for trial in 1 2 3 4 5; do
    bash scripts/run_experiment.sh \
      --method $method \
      --drones 5 \
      --targets 8 \
      --classes person,vehicle,fire \
      --duration 200 \
      --area-size 400 --altitude 60 \
      --trial $trial --seed $((42 + trial)) \
      --output-base data/logs/istras/S2
  done
done
```

**Note on 5-drone adaptive:** set `min_explore_drones: 2` in
`config/adaptive_params.yaml` before running S2 so at least two drones
stay in Voronoi coverage even when fire (3-drone perimeter) fires.
After S2 completes, restore it to 1 for the dynamic series at 3 drones.

---

## 5. Batch (one-shot)

```bash
bash scripts/run_all_experiments.sh --experiments S1 S2
bash scripts/run_all_experiments.sh --experiments S1 S2 --resume    # fill in gaps
bash scripts/run_all_experiments.sh --experiments S1 S2 --dry-run   # preview only
```

---

## 6. Expected Metrics Per Trial

Each trial writes `data/logs/istras/S#/<method>_d<N>_t<T>_trial<NN>/metrics_summary.json`
containing (among many others):

| Metric | Why it matters for the static series |
|---|---|
| `coverage_percent` | Headline coverage efficiency — primary S-series result |
| `total_distance_m` | Search effort; lower = more efficient search |
| `total_energy_wh` | Now Abeywickrama 3-term sum (hover + fwd + comm) |
| `forward_energy_wh` | Distance-driven component; adaptive should be lower |
| `time_to_first_target_s` | How quickly any target is found at all |
| `time_to_all_targets_s` | Only defined if every target was found — the *real* mission-completion metric |
| `targets_found` / `targets_total` | Completeness — if lawnmower finds 8/8 and adaptive 7/8, adaptive's low distance doesn't help |
| `min_pairwise_target_distance_m` | Binning variable for the σ/close-vs-far analysis (§4 of `istras_final_guide.md`) |

---

## 7. Post-Processing

```bash
# 1. Re-derive metrics (e.g. after pulling a code change that tweaks the
#    energy formula) without re-running simulation:
for d in data/logs/istras/S*/*; do
  python3 scripts/metrics_compute.py \
    --experiment-id "$(basename "$d")" \
    --log-base "$(dirname "$d")" \
    --area-size 400 --altitude 60 --steady-state-cutoff 0
done

# 2. Flatten every trial to one CSV row for downstream tables:
python3 scripts/aggregate_results.py \
  --logs-dir data/logs/istras \
  --output data/results/istras_static_results.csv

# 3. Close-vs-far separation analysis (this is the σ figure):
python3 scripts/analysis/target_separation_analysis.py \
  --logs-dir data/logs/istras \
  --output-dir data/results/separation_analysis \
  --plots

# 4. Comparison plots for paper Figure 2:
python3 scripts/plot_paths.py \
  --log-base data/logs/istras/S1 \
  --area-size 400 --altitude 60
```

Note the `--steady-state-cutoff 0` in step 1 — S-series trials have no
event phase, so the entire trial is "steady-state" for the purpose of
distance accounting. This keeps the new `steady_state_distance_m` column
from being misleading for static rows.

---

## 8. Close-vs-Far Sub-Analysis (σ regime)

Because trial seeds 43..47 draw different random target layouts, some
trials naturally land with closely-packed targets (min pairwise
distance < 30 m ≈ 2σ) while others don't. We don't control this
directly — the post-hoc analysis slices by it:

```bash
python3 scripts/analysis/target_separation_analysis.py \
  --logs-dir data/logs/istras \
  --output-dir data/results/separation_analysis \
  --bin-edges 0 30 60 120 9999 \
  --plots
```

The output `adaptive_vs_lawnmower_by_separation.csv` has one row per
(experiment, bin) with the adaptive−lawnmower delta on every metric.
Reporting in the paper: cite the 0-30 m bin as "close regime" and the
120+ m bin as "far regime".

**If you want controlled close/far seeds**, re-run S1 with specific
seeds whose draws you've inspected — the `targets.csv` for any earlier
trial reveals the layout, and re-running with that seed reproduces it.

---

## 9. Definition of Done

- [ ] `data/logs/istras/S1/` contains 15 trial folders, each with `metrics_summary.json`
- [ ] `data/logs/istras/S2/` contains 10 trial folders, each with `metrics_summary.json`
- [ ] Every `targets.csv` has 8 rows with `spawn_t = 0`
- [ ] Every `metrics_summary.json` carries `min_pairwise_target_distance_m`
      (if missing, the trial predates the separation patch — re-run `metrics_compute.py`)
- [ ] `data/results/separation_analysis/adaptive_vs_lawnmower_by_separation.csv` exists
- [ ] Figure 2 generated
