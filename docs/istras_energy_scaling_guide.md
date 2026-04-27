# ISTRAS'26 — Energy Scaling Research Guide

> Refocused scope: **how does mission energy scale with swarm size, and
> where is the Pareto frontier between energy and mission time?**
>
> Concrete step-by-step. The original framing (adaptive-vs-baseline,
> σ analysis, dynamic events) lives in `istras_final_guide.md` and is
> historical for this paper.

---

## 0. Headline (for context)

The paper's central claim is the **swarm-size Pareto curve**: each added
drone trades energy for mission time at a measurable, decreasing rate.
The Abeywickrama three-term decomposition (already logged) gives the
mechanism. The σ analyses and dynamic-target experiments from the prior
framing are dropped.

**Headline numbers from the lawnmower scaling cells (Sw1–Sw5 complete, 27 trials):**

| N | Total energy | Mission time | Wh per target | Speedup S(N) | Parallel efficiency |
|---|---|---|---|---|---|
| 1 | 50 Wh | 965 s | 6.3 | 1.00 | 1.00 |
| 2 | 59 Wh | 490 s | 7.4 | 1.97 | 0.985 |
| 3 | 68 Wh | 347 s | 8.5 | 2.78 | 0.927 |
| 4 | 65 Wh | 237 s | 8.2 | 4.08 | **1.019** (super-linear) |
| 5 | 94 Wh | 289 s | 11.8 | 3.34 | 0.667 |

Per-target energy degrades almost 2× as N grows from 1→5; parallel
efficiency collapses from 0.93 (near-ideal) at N=3 to 0.67 at N=5.
The Pareto frontier is **{N=1, N=2, N=4}** — N=3 and N=5 are strictly
dominated by N=4. The mechanism is FOV-strip rounding: cells where
total sweep lines = 8 (the geometric minimum) sit on the frontier;
N=3 (9 lines) and N=5 (10 lines) sit off it. **This is the paper.**
See `istras_pareto_analysis.md` for the full dominance walk and
`istras_results_discussion_conclusion.md` for the writeup.

---

## 1. Re-runs — short answer: NO for the existing cells

All four derived metrics in the analysis pipeline are **post-hoc
computations on existing primitive fields** in `metrics_summary.json`:

| Derived metric | Formula | Inputs (already logged) |
|---|---|---|
| Per-target energy | `total_energy_wh / targets_found` | both fields exist |
| Per-area energy | `total_energy_wh / coverage_percent` | both fields exist |
| Energy–time Pareto | scatter `total_energy_wh` vs `time_to_all_targets_s` | both fields exist |
| Marginal Nth-drone cost | `(E[N+1] − E[N]) / ΔN` aggregated across cells | all primitives |

**No changes to `metrics_compute.py`. No re-running existing trials.**
The new analysis script `scripts/analysis/energy_scaling_analysis.py`
does all of this in pure post-hoc analysis.

You only need new experiments to **add cells** to the curve (N=2, N=4,
optional static-grid ablation).

---

## 2. Data state (what you actually have)

Audit of `data/logs/istras/` (current as of 2026-04-27):

| Dir | Cell | Geometry | Status |
|---|---|---|---|
| `Sw1/` | lawnmower N=1, 5 trials | 600 m / 40 m / 8 targets | ✅ all 8/8 found |
| `Sw2/` | lawnmower N=2, 5 trials | 600 m / 40 m / 8 targets | ✅ all 8/8 found |
| `Sw3/` | lawnmower N=3, 5 trials | 600 m / 40 m / 8 targets | ✅ all 8/8 found |
| `Sw4/` | lawnmower N=4, 5 trials | 600 m / 40 m / 8 targets | ✅ all 8/8 found |
| `Sw5/` | lawnmower N=5, 7 trials (5 + 2 spare) | 600 m / 40 m / 8 targets | ✅ all 8/8 found |
| `S1/` | adaptive/static/lawnmower N=3 | **400 m / 60 m** (legacy) | ❌ ignore — wrong geometry, mostly broken |

**Use `Sw1`–`Sw5`.** Pass them explicitly via `--include-dirs Sw1 Sw2 Sw3 Sw4 Sw5`
(the script's default still lists `Sw1 Sw3 Sw5` from the pivot cut, so
override it for the full curve).

When you run new ablation cells (static-grid, adaptive), mirror the
geometry exactly: **600 m × 600 m area, 40 m altitude, 8 targets,
run-until-completion.** Place the runs in `data/logs/istras/Sg3/`
(static-grid) or `Sa3/` (adaptive).

---

## 3. Concrete step-by-step

### Step 1 — Run the analysis on existing data

```bash
conda activate swarm
python3 scripts/analysis/energy_scaling_analysis.py \
    --include-dirs Sw1 Sw2 Sw3 Sw4 Sw5 --plots
```

This walks all five cells, filters to `lawnmower` (default `--methods`),
and writes:

- `data/results/energy_scaling/trial_index.csv` — one row per trial,
  including the derived per-target / per-coverage metrics.
- `data/results/energy_scaling/scaling_summary.csv` — one row per
  (method, N) cell with mean / std / 95% CI.
- `data/results/energy_scaling/marginal_analysis.csv` — speedup,
  parallel efficiency, ΔE/Δdrone, ΔT/Δdrone for each consecutive pair.
- `data/results/energy_scaling/figures/F1..F5.png` — paper figures.

### Step 2 — Inspect the results

Open the figures and the three CSVs. Cross-check that the headline
numbers match the table in §0. If anything looks off, the most likely
cause is a stray broken trial in one of the included dirs — check
`trial_index.csv` for trials with `targets_found < targets_total`.

### Step 3 — Run N=2 and N=4 lawnmower cells ✅ DONE (2026-04-26)

**Status:** completed. Both `Sw2/` and `Sw4/` exist with 5 trials each,
all 8/8 targets found. The five-point curve is the dataset that drives
the current paper figures and the writeup in
`istras_results_discussion_conclusion.md`.

Why this was worth the ~½ day: the 3-point regression (N=1, 3, 5)
hid two of the most important findings — N=4 super-linear and the
{1, 2, 4} Pareto frontier. The runbook below is preserved for
reproducibility.

Required setup (must match existing cells exactly):

| Parameter | Value |
|---|---|
| Area | 600 m × 600 m |
| Altitude | 40 m |
| Targets | 8 (same spawn distribution) |
| Method | `lawnmower` |
| Trials | 5 per cell |
| Stop condition | run-until-all-targets-found |
| Output dir | `data/logs/istras/Sw2/`, `data/logs/istras/Sw4/` |

Single-trial command:

```bash
bash scripts/run_experiment.sh \
    --method lawnmower \
    --drones 2 \
    --targets 8 \
    --area-size 600 \
    --altitude 40 \
    --duration 1500 \
    --trial 1 \
    --output-base data/logs/istras/Sw2
```

Loop trials 1..5; repeat with `--drones 4 --output-base data/logs/istras/Sw4`.

After both cells exist:

```bash
python3 scripts/analysis/energy_scaling_analysis.py \
    --include-dirs Sw1 Sw2 Sw3 Sw4 Sw5 --plots
```

### Step 4 — Static-grid ablation at N=3 (recommended for v1)

**Decision (2026-04-25):** include lawnmower + static-grid in the
paper, drop adaptive. Static-grid is a literal "no movement after
deploy" lower-bound and gives the paper a clean two-policy comparison
without the engineering overhead of validating adaptive.

5 trials at the canonical geometry, output to `data/logs/istras/Sg3/`:

```bash
bash scripts/run_experiment.sh \
    --method static \
    --drones 3 \
    --targets 8 \
    --area-size 600 \
    --altitude 40 \
    --duration 1500 \
    --trial 1 \
    --output-base data/logs/istras/Sg3
```

Note: static-grid does not signal completion — it has no notion of
"done". Use a fixed `--duration` (1500 s — long enough that energy is
"hover energy across the whole window"), and report static-grid's
`targets_found` instead of `time_to_all_targets_s`. Static-grid is
operationally a bound, not a competitor on the Pareto curve.

After the cell:

```bash
python3 scripts/analysis/energy_scaling_analysis.py \
    --include-dirs Sw1 Sw2 Sw3 Sg3 Sw4 Sw5 \
    --methods lawnmower static --plots
```

### Step 5 — (Optional) Adaptive ablation at N=3

Adaptive is now run-until-completion-capable thanks to the
`/adaptive/complete` trigger added to the coordinator (2026-04-25, see
§5 below). Adding it back is no longer the major engineering lift it
was — but it's still extra moving parts to defend against reviewers
(parameter tuning, registry behaviour, exploit-vs-explore separation).

If you do want it, run 5 trials matching the canonical geometry:

```bash
bash scripts/run_experiment.sh \
    --method adaptive \
    --drones 3 \
    --targets 8 \
    --area-size 600 \
    --altitude 40 \
    --duration 400 \
    --trial 1 \
    --output-base data/logs/istras/Sa3
```

The `--duration 400` value scales the safety cap inside
`run_experiment.sh` to `max(1500, 4×400) = 1600 s`. Adaptive will stop
earlier on `/adaptive/complete` once all 8 targets are confirmed.

After the cell:

```bash
python3 scripts/analysis/energy_scaling_analysis.py \
    --include-dirs Sw1 Sw2 Sw3 Sa3 Sg3 Sw4 Sw5 \
    --methods lawnmower adaptive static --plots
```

### Step 6 — Write the paper

Section order (write Results first while data is fresh):

1. **Results** (~2.5 pages) — F1 (Pareto) is the headline; F2
   (decomposition) supports the mechanism; F3 (per-target energy)
   makes the duration-normalised case; F4 (CO₂) is the sustainability
   hook; F5 (speedup/efficiency) carries the diminishing-returns story.
2. **Methods** (~2 pages) — Abeywickrama three-term, Gazebo SITL stack,
   lawnmower coverage, run-until-completion semantics.
3. **Experimental Setup** (~1 page) — geometry, targets, trial count,
   stop condition.
4. **Related Work** (~1.5 pages) — UAV energy models (Abeywickrama and
   followups), coverage-swarm scalability surveys, parallel-computing
   speedup/efficiency analyses (Amdahl analogue).
5. **Introduction** (~1.5 pages) — write last; framing should mirror
   what F1 actually showed.
6. **Discussion** (~1 page) — diminishing returns, deployment
   recommendations, limitations.
7. **Conclusion + Future Work** (~0.5 pages).

---

## 4. What the analysis script computes (reference)

`scripts/analysis/energy_scaling_analysis.py` is the single post-hoc
analysis tool. It is intentionally read-only: it never writes back into
the experiment logs or `metrics_compute.py`.

Inputs per trial (read from `metrics_summary.json` — already logged):

```
n_drones, duration_s, area_size_m, altitude_m,
total_distance_m, coverage_percent, redundancy_ratio,
hover_energy_wh, forward_energy_wh, comm_energy_wh,
total_energy_wh, energy_per_drone_wh,
co2_azerbaijan_g, co2_eu_g,
targets_found, targets_total,
time_to_first_target_s, time_to_all_targets_s,
total_detections, per_drone_distance_m
```

Derived per trial:

```
wh_per_target_found    = total_energy_wh / targets_found
wh_per_pct_coverage    = total_energy_wh / coverage_percent
wh_per_minute_mission  = total_energy_wh / (time_to_all_targets_s/60)
hover_share_pct        = 100 * hover_energy_wh / total_energy_wh   (likewise forward, comm)
per_drone_dist_cv      = stdev(per_drone_distance) / mean(per_drone_distance)
```

Aggregated per (method, N):

```
mean, std, 95% CI on every primitive and every derived metric
(t-distribution critical value for small n; falls back to 1.96 for n>30).
```

Marginal across cells (per method, by sorted N):

```
speedup_vs_baseline      = T(N=1) / T(N)
parallel_efficiency      = speedup / N
delta_energy_per_drone   = (E(N) − E(prev)) / (N − prev)
delta_time_per_drone     = (T(prev) − T(N))  / (N − prev)
```

Figures (`--plots`):

```
F1_pareto.png             energy vs mission time, points coloured by N, frontier through cell means
F2_decomposition.png      stacked hover/forward/comm bars vs N
F3_per_target_energy.png  Wh per target found vs N with 95% CI bars
F4_co2_emissions.png      Azerbaijan + EU grid CO2 vs N with 95% CI bars
F5_speedup_efficiency.png speedup (vs ideal) and parallel efficiency, dual y-axis
```

CLI summary:

```
--logs-dir       data/logs/istras                  log root
--include-dirs   Sw1 Sw3 Sw5                       script default — pre-pivot cut
                                                   (override with: Sw1 Sw2 Sw3 Sw4 Sw5)
--methods        lawnmower                         methods to include
--baseline-n     1                                 speedup baseline
--primary-method lawnmower                         drives headline figures
--plots                                            generate F1–F5
--output-dir     data/results/energy_scaling       where CSVs+figures land
```

---

## 5. Adaptive coordinator runtime changes (2026-04-25)

The adaptive coordinator gained three features that make it
cell-comparable to lawnmower's run-until-completion semantics. These
are off the critical path for the v1 paper (which uses lawnmower +
static-grid only) but documented here because the code is in `main`.

### 5.1 Cross-drone target dedup — `TargetRegistry`

Each detection now passes through a swarm-wide registry that tracks
unique physical targets. Detections of the same class within
`target_merge_radius_m = 8 m` of an existing entry update it (running
mean position, bumped detection count) instead of creating a new one.
A candidate is promoted to "confirmed" only after `min_confirmations
= 3` reinforcements — single-frame YOLO false positives never count.

This is what makes a target seen by drone B *the same target* as the
one drone A saw earlier. The legacy `exploit_merge_radius_m` only
deduplicated within the lifecycle of one exploit event; the registry
is persistent.

Coordinator log line on each detection:
```
[Detection] person (0.62, priority=0.9) from drone 1 at global (X,Y)m
  — registry: 2 confirmed / 5 candidates (merged|new)
[CONFIRMED] person #3 at (X,Y)m after 3 detections
```

### 5.2 Explore-mode repulsion from confirmed targets

The density map's positive Gaussian bumps still attract drones —
correct for **exploit** (go inspect) but wrong for **explore** (we want
drones to drift toward unknown regions). For explore-mode drones only,
the coordinator now evaluates a *repulsed* density:

```
explore_density(x, y) = density(x, y)
                      − w · detection_weight · Σ Gaussian(x, y; tgt, σ_rep)
```

with `confirmed_repulsion_weight = 1.0` (so the repulsion fully
cancels the detection bump at a confirmed target's centre) and
`confirmed_repulsion_sigma_m = 25.0` (slightly wider than
`detection_sigma = 15` so suppression extends past the bump it
cancels). Clamped to a small positive minimum so weighted-centroid
integration never sees zero/negative density.

Exploit drones still see the original (positive) density and the
registry's stabilised position — their formations are unchanged.

### 5.3 Completion trigger — `/adaptive/complete`

The coordinator publishes a latched `Bool(True)` on
`/adaptive/complete` and shuts down once either:

1. `confirmed_count ≥ expected_targets` (oracle-N stopping —
   `run_experiment.sh` passes the trial's `--targets` value via
   `--expected-targets`), or
2. `quiet_seconds ≥ completion_quiet_period_s` (heuristic stopping
   when expected_targets is unknown — disabled by default).

The QoS profile mirrors `/lawnmower/complete` (RELIABLE +
TRANSIENT_LOCAL + KEEP_LAST/depth=1). The data logger subscribes to
both topics with the same handler, so adaptive trials now stop when
the search is done — not when a wall-clock timer expires. Mission
energy becomes directly comparable to lawnmower's.

### 5.4 New CLI args (`scripts/voronoi_coordinator.py`)

```
--target-merge-radius        cross-drone dedup radius        (default 8.0 m)
--min-confirmations          K detections to confirm         (default 3)
--confirmed-repulsion-sigma  explore-mode repulsion radius   (default 25.0 m)
--confirmed-repulsion-weight repulsion strength multiplier   (default 1.0)
--expected-targets           oracle-N completion trigger     (default unset)
--completion-quiet-period    quiet-period completion (s)     (default 0 — off)
```

Defaults are also documented in `config/adaptive_params.yaml`.

---

## 6. Honesty clauses (paper should flag up front)

1. **Variable mission duration.** Trials run until all targets are
   found. The hover term scales with N×T, so larger swarms partially
   recoup their added drones via shorter missions. This is *the*
   mechanism the paper studies, not a confound.

2. **Energy is modelled, not measured.** Abeywickrama's empirical
   curve applied post-hoc to logged trajectories. Standard for
   simulation-only papers.

3. **Five swarm sizes is sparse.** N∈{1,2,3,4,5} gives a 5-point
   curve; N>5 is left to future work. The FOV-rounding mechanism is
   sensitive to the exact `600/N` ratio, so the specific Pareto-optimal
   set {1, 2, 4} is geometry-specific even if the *shape* of the
   non-monotonicity is general. State this.

4. **Two coverage policies (lawnmower + static-grid).** Lawnmower is
   the active scaling series; static-grid is a stationary lower bound.
   Adaptive is mentioned as future work (the runtime is implemented but
   not benchmarked in this paper).

5. **Sim-only.** Calibrated against DJI Matrice 100 numbers, no real
   flights.

---

## 7. Files

```
docs/
  istras_energy_scaling_guide.md   ← you are here
  istras_final_guide.md            historical, for the σ / dynamic story
  istras_experiments_static.md     legacy runbook (S1/S2 framing)
  istras_experiments_dynamic.md    legacy runbook (D1/D2/D3 framing)
scripts/
  analysis/
    energy_scaling_analysis.py     this paper's only analysis tool
    target_separation_analysis.py  legacy, σ analysis (out of scope)
  voronoi_coordinator.py           +TargetRegistry, +/adaptive/complete (§5)
  data_logger.py                   subscribes to /adaptive/complete now
  run_experiment.sh                adaptive branch passes --expected-targets
  metrics_compute.py               unchanged — no new fields needed
  baselines/
    lawnmower.py                   primary scaling-series controller
    static_grid.py                 ablation lower-bound
config/
  adaptive_params.yaml             +registry, +repulsion, +completion knobs
data/
  logs/istras/{Sw1,Sw2,Sw3,Sw4,Sw5}/...  primary scaling cells (all 5 done, 27 trials)
  logs/istras/Sg3/                       static-grid ablation (Step 4, optional)
  logs/istras/Sa3/                       adaptive ablation (Step 5, optional)
  logs/istras/S1/                        legacy 400m/60m cells — ignore
  results/energy_scaling/          analysis outputs (CSVs + figures)
```

---

*Last updated: 2026-04-25 (adaptive completion trigger + TargetRegistry)*
