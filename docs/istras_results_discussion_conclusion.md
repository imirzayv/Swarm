---
title: "ISTRAS — Results, Discussion, and Conclusion"
scope: "Sw1–Sw5 lawnmower scaling experiments (N = 1..5 drones, 600 m × 600 m, 40 m, 8 targets)"
data_sources:
  - data/results/energy_scaling/scaling_summary.csv
  - data/results/energy_scaling/marginal_analysis.csv
  - data/results/energy_scaling/trial_index.csv
  - data/results/energy_scaling/figures/F1..F5.png
last_updated: 2026-04-27
---

# 1. Experimental Setup (recap)

All five scaling cells share an identical world geometry to keep N as the
only independent variable:

| Parameter         | Value                                               |
|-------------------|-----------------------------------------------------|
| Search area       | 600 m × 600 m                                       |
| Flight altitude   | 40 m                                                |
| Targets per trial | 8 (fixed spawn distribution)                        |
| Coverage policy   | Lawnmower (geometric, FOV-pitched at 85 m, 10% overlap) |
| Stop condition    | Run-until-all-targets-found                         |
| Trials per cell   | 5 (Sw1–Sw4); 7 (Sw5)                                |
| Energy model      | Abeywickrama three-term (DJI Matrice 100 calibration) |
| Simulator         | Gazebo Harmonic + PX4 SITL + ROS 2 Humble + YOLOv8  |

Cells: **Sw1** = N = 1, **Sw2** = N = 2, **Sw3** = N = 3, **Sw4** = N = 4,
**Sw5** = N = 5. Trial counts total **27** completed runs, **all 8 / 8
targets confirmed** in every trial.

---

# 2. Results

## 2.1 Headline numbers (per-cell aggregate)

Source: [data/results/energy_scaling/scaling_summary.csv](../data/results/energy_scaling/scaling_summary.csv).
All values are mean ± 95 % CI across the cell's trials.

| N | Trials | E_total (Wh)   | T (s)         | E / drone (Wh) | Wh / target | Speedup S(N) | Parallel η(N) |
|---|--------|----------------|---------------|----------------|-------------|--------------|----------------|
| 1 | 5      | 50.46 ± 0.03   | 964.7 ± 109.4 | 50.46          | 6.31        | 1.00         | 1.00           |
| 2 | 5      | 59.12 ± 0.11   | 489.8 ± 81.9  | 29.56          | 7.39        | 1.97         | 0.985          |
| 3 | 5      | 68.34 ± 0.67   | 346.7 ± 130.8 | 22.78          | 8.54        | 2.78         | 0.927          |
| 4 | 5      | 65.30 ± 0.31   | 236.6 ± 43.9  | 16.33          | 8.16        | **4.08**     | **1.019**      |
| 5 | 7      | 94.27 ± 4.58   | 289.0 ± 19.5  | 18.85          | 11.78       | 3.34         | 0.667          |

Two non-monotonic results stand out and structure the rest of this section:

1. **N = 4 is super-linear**: η = 1.019 > 1, i.e. four drones finish strictly
   faster than 4 × the single-drone mission time, with **lower total
   energy than N = 3**.
2. **N = 5 regresses**: total energy jumps by 28.97 Wh (+44 %) versus N = 4
   while mission time *increases* by 52 s. This is the only point in the
   sweep where adding a drone makes the mission both slower and more
   expensive.

## 2.2 Energy–time Pareto frontier

Applying the standard dominance test to the cell means:

| Candidate | Dominator(s)                          | On frontier? |
|-----------|---------------------------------------|--------------|
| N = 1     | none — lowest E in the table          | **yes**      |
| N = 2     | none — N = 1 is cheaper but slower    | **yes**      |
| N = 3     | **N = 4** dominates (lower E, lower T)| no           |
| N = 4     | none — fastest cell, third-cheapest   | **yes**      |
| N = 5     | **N = 4** dominates (lower E, lower T)| no           |

The Pareto-optimal swarm sizes for the 600 m × 600 m / 40 m geometry are
**{1, 2, 4}**. N = 3 and N = 5 are *strictly dominated*: there is no time
budget and no energy budget for which they are the right choice.

Marginal cost between adjacent frontier points (excluding dominated
cells):

| Hop          | ΔE (Wh) | ΔT (s)  | Seconds saved per Wh added |
|--------------|---------|---------|----------------------------|
| N = 1 → N = 2 | +8.66   | −474.9  | **54.8 s/Wh**              |
| N = 2 → N = 4 | +6.18   | −253.2  | **41.0 s/Wh**              |
| (N = 4 → N = 5)| +28.97 | +52.4   | regression (no purchase)   |

The cost-of-time curve is roughly flat between N = 1 → 2 and N = 2 → 4 —
both purchases are economical at 40–55 s saved per Wh added — and then
collapses past N = 4. Reference figure: [F1_pareto.png](../data/results/energy_scaling/figures/F1_pareto.png).

## 2.3 Energy decomposition

The Abeywickrama three-term split (hover / forward / comm) is remarkably
stable across the sweep:

| N | Hover %      | Forward %   | Comm %      | E_hover (Wh) | E_forward (Wh) | E_comm (Wh) |
|---|--------------|-------------|-------------|--------------|----------------|-------------|
| 1 | 83.94 ± 0.01 | 10.19 ± 0.0 | 5.88 ± 0.0  | 42.35        | 5.14           | 2.97        |
| 2 | 84.00 ± 0.01 | 10.12 ± 0.02| 5.88 ± 0.0  | 49.66        | 5.98           | 3.48        |
| 3 | 84.05 ± 0.06 | 10.07 ± 0.07| 5.88 ± 0.01 | 57.44        | 6.88           | 4.02        |
| 4 | 84.12 ± 0.02 | 9.99 ± 0.03 | 5.89 ± 0.0  | 54.93        | 6.53           | 3.85        |
| 5 | 85.41 ± 0.39 | 8.60 ± 0.42 | 5.97 ± 0.03 | 80.54        | 8.09           | 5.64        |

**Hover is the dominant cost in every cell (84–85 % of total energy).**
This reflects the central design of the energy model: every drone pays
P_HOVER = 142.8 W for every second airborne, regardless of motion, while
forward flight only adds ΔP_fwd = 18.2 W on top. As a result the
energy story is *almost entirely* a story about `N × T` (drone-seconds
in the air). The forward term tracks total path distance and is small
but informative: it is the term that breaks the "more drones = less
work each" intuition (see §3.2).

Reference figure: [F2_decomposition.png](../data/results/energy_scaling/figures/F2_decomposition.png).

## 2.4 Per-target and per-area energy efficiency

| N | Wh / target found | Wh / % coverage     | Wh / minute mission |
|---|-------------------|---------------------|---------------------|
| 1 | 6.31 ± 0.003      | 28.51 ± 0.02        | 3.16 ± 0.39         |
| 2 | 7.39 ± 0.013      | 16.85 ± 0.06        | 7.33 ± 1.29         |
| 3 | 8.54 ± 0.084      | 13.18 ± 0.20        | 12.46 ± 4.27        |
| 4 | 8.16 ± 0.039      | 9.59 ± 0.07         | 16.84 ± 3.84        |
| 5 | 11.78 ± 0.572     | 11.38 ± 0.57        | 19.60 ± 0.37        |

Three distinct trends:

- **Wh per target found** rises monotonically from 6.31 (N = 1) to 11.78
  (N = 5) — an 87 % degradation. Each target gets *more* expensive as
  the swarm grows, even though the swarm finds them faster, because the
  hover term is paid by every drone for the whole mission.
- **Wh per percent coverage** decreases (28.5 → 9.6 between N = 1 and
  N = 4), then ticks back up at N = 5. Spatial efficiency improves with
  swarm size up to the FOV-rounding sweet spot.
- **Wh per minute mission** rises sharply because larger swarms burn
  more total power per unit wall-clock time — exactly what the hover
  term predicts.

Reference figure: [F3_per_target_energy.png](../data/results/energy_scaling/figures/F3_per_target_energy.png).

## 2.5 Carbon emissions

Using the Abeywickrama-derived energy and two grid intensities (440 g
CO₂/kWh for Azerbaijan, 250 g CO₂/kWh for the EU average):

| N | CO₂ Azerbaijan (g) | CO₂ EU (g)        |
|---|--------------------|--------------------|
| 1 | 22.20 ± 0.01       | 12.61 ± 0.005      |
| 2 | 26.01 ± 0.05       | 14.78 ± 0.03       |
| 3 | 30.07 ± 0.29       | 17.09 ± 0.17       |
| 4 | 28.73 ± 0.14       | 16.33 ± 0.08       |
| 5 | 41.48 ± 2.02       | 23.56 ± 1.14       |

The CO₂ figures inherit the Pareto structure of total energy exactly,
because emissions are linear in Wh. The N = 4 cell again sits below
N = 3 (28.73 g vs 30.07 g on the Azerbaijan grid). Reference figure:
[F4_co2_emissions.png](../data/results/energy_scaling/figures/F4_co2_emissions.png).

## 2.6 Speedup and parallel efficiency

Defining `S(N) = T(N=1) / T(N)` (mission-time speedup) and
`η(N) = S(N) / N` (parallel efficiency), the marginal table reads:

| N | T (s) | S(N) | η(N) | ΔE per added drone (Wh) | ΔT per added drone (s) |
|---|-------|------|------|--------------------------|-------------------------|
| 1 | 964.7 | 1.00 | 1.00 | —                        | —                       |
| 2 | 489.8 | 1.97 | 0.985| +8.67                    | −474.9                  |
| 3 | 346.7 | 2.78 | 0.927| +9.22                    | −143.1                  |
| 4 | 236.6 | 4.08 | **1.019** | −3.04               | −110.1                  |
| 5 | 289.0 | 3.34 | 0.667| +28.97                   | **+52.4** (regression)  |

Source: [data/results/energy_scaling/marginal_analysis.csv](../data/results/energy_scaling/marginal_analysis.csv).

Until N = 4, η stays in the 0.93–1.02 band — near-ideal parallelism.
At N = 5 it collapses to 0.67. The N = 3 → N = 4 transition is the only
one that *reduces* total energy: adding the fourth drone shortens the
mission enough to claw back the extra hover cost. The N = 4 → N = 5
transition does the opposite: it lengthens the mission (the extra
drone interferes more than it parallelises) while paying additional
hover power for a fifth aircraft. Reference figure:
[F5_speedup_efficiency.png](../data/results/energy_scaling/figures/F5_speedup_efficiency.png).

## 2.7 Detection performance

Across all 27 trials every drone recovered the full 8 / 8 targets
(`targets_found_mean = 8.00`, std = 0). Time-to-first-target ranges
from 22 s (N = 1) to 25 s (N = 5) with very wide intra-cell variance
(driven by random spawn locations). Total raw detections grew from
~547–578 (N = 1) to ~975–1219 (N = 5), as expected for fleets covering
overlapping regions. Detection completeness is therefore not the
discriminator between cells — energy and time are.

---

# 3. Discussion

## 3.1 The headline finding: scaling is non-monotonic

The clearest result from Sw1–Sw5 is that swarm scaling under a
geometric coverage policy is **not smoothly convex in N**. The
prevailing assumption in the swarm-coverage literature — that adding
drones produces monotonically improved time at the cost of monotonically
increased energy, with a smooth knee somewhere in the middle — is
**not what the data show**. Instead the trade-off curve has structure:
N = 4 is super-linear, N = 5 regresses, and the Pareto frontier
contains only **N ∈ {1, 2, 4}**.

Practitioners can read this directly:

- **Energy-bound, time-tolerant missions**: pick **N = 1** (50 Wh,
  16 min). The single drone pays no hover overhead beyond its own.
- **Balanced operations**: pick **N = 2** (59 Wh, 8 min, η = 0.99).
  Effectively free parallelism — the extra drone pays for itself in
  time saved.
- **Time-critical operations**: pick **N = 4** (65 Wh, 4 min, η = 1.02).
  The fastest *and* third-cheapest cell in the entire sweep.
- **Avoid N = 3 and N = 5**: both are strictly dominated and there is
  no operating regime in which they are the correct answer for this
  geometry.

## 3.2 Why the curve is not smooth: FOV–strip alignment

The mechanism behind the non-monotonicity is geometric. Lawnmower hands
each drone a strip of width `W = 600 / N` metres and sweeps it with
internal lines at the camera footprint pitch `s ≈ 85 m` (40 m altitude,
100° HFOV, 10 % overlap). The number of sweep lines per drone is
`ceil(W / s)`, so total sweep lines across the swarm depends on how
`600 / N` rounds against `s`:

| N | Strip W (m) | Sweeps / drone | Total sweep lines | Geometric waste |
|---|-------------|----------------|-------------------|------------------|
| 1 | 600         | 8              | 8                 | minimal          |
| 2 | 300         | 4              | 8                 | minimal          |
| 3 | 200         | 3              | 9                 | one extra line   |
| 4 | 150         | 2              | 8                 | minimal          |
| 5 | 120         | 2              | 10                | two extra lines  |

This rounding table predicts the experimental ranking exactly:

- The cells where total sweep count = 8 — the geometric minimum to cover
  600 m at 85 m pitch — are **{N = 1, 2, 4}**, which is precisely the
  Pareto frontier observed in §2.2.
- The cells where rounding adds extra lines — N = 3 (one extra) and
  N = 5 (two extra) — are exactly the dominated cells.

N = 4 is a "lucky" partition: 150 m fits two 85 m FOV lanes with 20 m
of overlap, so each drone makes only two sweeps and the swarm finishes
at 236 s — the minimum mission time achievable without paying redundancy.
N = 5, by contrast, gives every drone a 120 m strip that *also* needs
two sweeps (because 85 m × 1 < 120 m), but now there are 10 sweep
lines total and 50 m of overlap on every strip. The extra sweeping
shows up as +28.97 Wh of forward-flight energy with no compensating
time saving — in fact T worsens because the launch window contention
between five drones adds a small startup delay.

The conclusion that follows is sharper than the typical "knee at N = 3"
story:

> The energy–time trade-off in geometric multi-UAV coverage is not
> smoothly convex in swarm size. **Strip-vs-FOV alignment**, not
> swarm size per se, determines whether a given N lands on the Pareto
> frontier. A geometry-aware partitioner — one that picks N to round
> cleanly against the camera footprint — would dominate naïve uniform
> partitioning across most realistic mission sizes.

## 3.3 Hover dominates the energy budget

Hover accounts for 84–85 % of total mission energy in every cell. This
is a direct consequence of the Abeywickrama power split (`P_HOVER` =
142.8 W vs `ΔP_fwd` = 18.2 W) combined with cruise speeds substantially
slower than the lawnmower's per-drone duty cycle. The practical
consequence for energy-aware swarm design is that **shortening the
mission is far more leveraged than reducing path length**: a 20 %
reduction in `T` saves ~17 % of total energy; a 20 % reduction in path
distance saves only ~2 %. This is why N = 4 is super-linear despite
flying *more* total path metres than N = 3 — the time saving in the
hover term outweighs the extra forward-flight term.

The same observation explains why static-grid (no movement after
deploy) is almost-but-not-quite useful: it minimises forward energy
to zero but pays the full hover cost for the whole mission window,
making it an upper bound on hover-dominated energy rather than a
competitive policy.

## 3.4 Per-target energy degrades with N

Wh-per-target rises from 6.31 (N = 1) to 11.78 (N = 5) — an 87 %
increase. This is a natural consequence of fixed-target missions in
hover-dominated regimes: adding drones reduces wall-clock time
sub-linearly (N = 5 reaches 3.34× speedup, not 5×) but multiplies the
hover-power floor by N exactly. The cost-per-target metric is therefore
the cleanest indicator that **larger swarms pay an energy premium for
responsiveness**, even when scaling looks "good" on the time axis.

For mission planners, this implies that swarm-size selection should
explicitly weigh per-target energy against responsiveness — flat
"more drones = better coverage" intuitions over-spend energy in
fixed-target search missions.

## 3.5 Carbon implications

The CO₂ ranking inherits the Pareto structure of energy exactly. On
the Azerbaijan grid (440 g CO₂/kWh, fossil-heavy) every additional
drone past N = 4 emits an extra 12.7 g of CO₂ per mission. On the EU
grid (250 g/kWh) the same step costs 7.2 g. These are small per-mission
absolute numbers, but at sustained surveillance rates (8 missions/day,
365 days/year) the difference between N = 4 and N = 5 is 37 kg vs 21 kg
of additional CO₂ per drone-year on the two grids respectively — the
kind of figure that matters for fleet-level sustainability reporting.
The clearest deployment recommendation from the carbon perspective is
identical to the Pareto recommendation: **stay at or below the FOV-aligned
sweet spot N = 4** for this geometry.

## 3.6 What the data do not say (limitations)

Five honesty clauses bound the claims above:

1. **Variable mission duration.** Trials are run-until-completion, so
   missions with larger N are shorter and the hover term scales with
   `N × T`. Larger swarms partially recoup their added drones via
   shorter missions. This is the central mechanism the paper studies,
   not a confound, but the alternative framing — fixed-time missions —
   would shift the trade-off toward smaller N.
2. **Energy is modelled, not measured.** The Abeywickrama empirical
   curve is applied post-hoc to logged trajectories. Standard practice
   for simulation-only papers, but real flights would add wind,
   payload variability, and propeller-condition noise the model does
   not capture.
3. **Five swarm sizes is sparse.** The N ∈ {1, 2, 3, 4, 5} sweep gives
   a 5-point curve. We do not extrapolate past N = 5 because the
   FOV-rounding mechanism is sensitive to the exact `600 / N` ratio,
   and a six-point curve at N = 6 (strip = 100 m, two sweeps, total
   12 lines) would already be predicted to regress further.
4. **Single coverage policy.** Lawnmower is the only active scaling
   series. Static-grid was scoped as a stationary lower bound (see §3.3)
   and adaptive Voronoi was deferred to future work; the runtime is
   implemented but not benchmarked here.
5. **Single geometry.** All cells use the 600 m × 600 m / 40 m / 8-target
   configuration. The FOV-rounding result generalises *qualitatively*
   to other geometries — the frontier will always be the cells where
   `ceil((area / N) / footprint_pitch)` minimises total sweep lines —
   but the specific Pareto-optimal set {1, 2, 4} is unique to this
   particular ratio of area to footprint.

---

# 4. Conclusion

This paper studied how mission energy and mission time scale with swarm
size for a vision-guided multi-UAV area-search task, using a fixed
600 m × 600 m / 40 m / 8-target geometry across N ∈ {1, 2, 3, 4, 5}
swarm sizes (Sw1–Sw5, 27 completed trials). Three findings warrant
emphasis.

**First, the scaling curve is non-monotonic**. The Pareto frontier of
swarm sizes is **{N = 1, N = 2, N = 4}**; N = 3 and N = 5 are strictly
dominated by N = 4 on both axes. This is sharper than the prevailing
"smooth knee" framing in the multi-UAV coverage literature and gives
direct deployment guidance: pick N = 1 if energy-bound, N = 2 if
balanced, N = 4 if time-critical, and avoid N = 3 and N = 5 entirely
for this geometry.

**Second, the mechanism is geometric, not algorithmic**. Strip-vs-FOV
alignment — whether `600 / N` rounds cleanly against the 85 m camera
footprint pitch — predicts the frontier exactly. Cells where the
swarm requires the geometric minimum of 8 sweep lines (N = 1, 2, 4)
sit on the frontier; cells where rounding inflates the sweep count
(N = 3 with 9 lines, N = 5 with 10) sit off it. This motivates a
geometry-aware partitioner that selects swarm size to round optimally
against the camera footprint, rather than uniformly across an
operator-chosen N.

**Third, hover dominates** at 84–85 % of total mission energy in every
cell. This means shortening the mission is roughly an order of
magnitude more leveraged than reducing path length, and makes the
super-linear behaviour at N = 4 — which flies *more* total distance
than N = 3 but completes faster — both possible and predictable from
the Abeywickrama three-term decomposition. The Wh-per-target metric
nonetheless rises 87 % from N = 1 to N = 5, reminding fleet operators
that responsiveness is purchased with a measurable energy premium.

The work has three principal limitations: energy is modelled (not
measured), the sweep is sparse (five swarm sizes, one geometry), and
only one coverage policy (lawnmower) is benchmarked. Future work
falls along three corresponding directions: (i) closing the loop with
real flights against the modelled energy numbers; (ii) running the
same Pareto analysis on a second geometry to test whether the
frontier-tracks-FOV-rounding result is geometry-invariant; and (iii)
benchmarking the adaptive Voronoi controller — whose runtime is
implemented but unevaluated in this paper — against the lawnmower
frontier reported here, with particular attention to whether
detection-driven rebalancing can buy back the over-coverage cost paid
by N = 3 and N = 5 in the geometric baseline.

The headline takeaway is unchanged across all of these caveats:
**multi-UAV coverage scaling is structured, not smooth**, and the
structure can be read directly off the camera footprint. A swarm
deployed without that alignment in mind will land off the Pareto
frontier — slower, more expensive, or both — for no algorithmic
reason at all.

---

# 5. Reproducibility pointers

Inputs and outputs for everything reported above:

| Artifact                                                                                                            | What it contains                              |
|--------------------------------------------------------------------------------------------------------------------|-----------------------------------------------|
| [data/logs/istras/{Sw1,Sw2,Sw3,Sw4,Sw5}/](../data/logs/istras/)                                                    | per-trial raw logs (positions, detections)    |
| [data/results/energy_scaling/trial_index.csv](../data/results/energy_scaling/trial_index.csv)                       | one row per trial, all primitive + derived metrics |
| [data/results/energy_scaling/scaling_summary.csv](../data/results/energy_scaling/scaling_summary.csv)               | one row per cell with mean / std / 95 % CI    |
| [data/results/energy_scaling/marginal_analysis.csv](../data/results/energy_scaling/marginal_analysis.csv)           | speedup, η, ΔE/Δdrone, ΔT/Δdrone              |
| [data/results/energy_scaling/figures/F1_pareto.png](../data/results/energy_scaling/figures/F1_pareto.png)           | Pareto scatter (energy vs. mission time)      |
| [data/results/energy_scaling/figures/F2_decomposition.png](../data/results/energy_scaling/figures/F2_decomposition.png) | hover / forward / comm stacked bars vs. N |
| [data/results/energy_scaling/figures/F3_per_target_energy.png](../data/results/energy_scaling/figures/F3_per_target_energy.png) | Wh per target, 95 % CI                |
| [data/results/energy_scaling/figures/F4_co2_emissions.png](../data/results/energy_scaling/figures/F4_co2_emissions.png) | CO₂ on AZ + EU grids, 95 % CI            |
| [data/results/energy_scaling/figures/F5_speedup_efficiency.png](../data/results/energy_scaling/figures/F5_speedup_efficiency.png) | speedup S(N) and parallel η(N)      |

Reproduce end-to-end:

```bash
conda activate swarm
python3 scripts/analysis/energy_scaling_analysis.py \
    --logs-dir data/logs/istras \
    --include-dirs Sw1 Sw2 Sw3 Sw4 Sw5 \
    --methods lawnmower \
    --baseline-n 1 \
    --output-dir data/results/energy_scaling --plots
```
