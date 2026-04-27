# ISTRAS Pareto analysis — energy vs. mission time across swarm size

This report explains what the Pareto plot in [`figures/F1_pareto.png`](../data/results/energy_scaling/figures/F1_pareto.png)
is showing, how to read it, and what the actual frontier looks like for the
current data at [`data/results/energy_scaling/scaling_summary.csv`](../data/results/energy_scaling/scaling_summary.csv).

---

## 1. What "Pareto" means here

We are optimising two objectives simultaneously:

- minimise **mission energy** `E_total` (Wh) — a stand-in for environmental
  cost, battery wear, and fleet operating expense.
- minimise **mission time** `T` (s) — measured as time-to-all-targets, i.e.
  responsiveness of the swarm.

Each operating point is one swarm size `N`. A point `A = (E_A, T_A)` is said to
**dominate** another point `B = (E_B, T_B)` if `E_A ≤ E_B` and `T_A ≤ T_B`,
with at least one strict inequality. The **Pareto frontier** is the set of
non-dominated points — the operating sizes that are optimal in the sense that
no other size gives you both lower energy *and* lower time at the same time.

The substantive question is not "what is the cheapest N?" or "what is the
fastest N?" — it is "which N's are *worth considering at all*, and what do
you pay in energy when you trade time for it?"

---

## 2. The current data

From `scaling_summary.csv` (5 cells × 5–7 trials):

| N | E (Wh)       | T (s)        | Speedup S(N) | Parallel eff. η(N) | E / target (Wh) |
|---|--------------|--------------|--------------|---------------------|-----------------|
| 1 | 50.46 ± 0.03 | 964.7 ± 109  | 1.00         | 1.00                | 6.31            |
| 2 | 59.12 ± 0.11 | 489.8 ± 82   | 1.97         | 0.985               | 7.39            |
| 3 | 68.34 ± 0.67 | 346.7 ± 131  | 2.78         | 0.927               | 8.54            |
| 4 | 65.30 ± 0.31 | 236.6 ± 44   | 4.08         | **1.019**           | 8.16            |
| 5 | 94.27 ± 4.58 | 289.0 ± 19.5 | 3.34         | 0.667               | 11.78           |

Two surprises in this table that the published paper PDF does not report:

1. **N=4 is super-linear** (η = 1.019 vs ideal 1.000). This means N=4 finishes
   the mission faster than 4× a single-drone mission. Causes are discussed in
   [§4](#4-why-n4-is-super-linear-and-why-n5-regresses) below — short version:
   the 150 m strip happens to align almost exactly with the FOV pitch.
2. **N=5 is slower *and* more expensive than N=4.** N=5 spends 28.97 Wh more
   energy than N=4 and is 52 s *slower*. That is an unambiguous regression.

---

## 3. The actual Pareto frontier

Walking the dominance test point-by-point against the 5-cell data:

| Candidate | Dominator(s)                               | On frontier? |
|-----------|--------------------------------------------|--------------|
| N=1       | none — lowest E in the table               | **yes**      |
| N=2       | none — N=1 is cheaper but slower           | **yes**      |
| N=3       | **N=4** dominates (lower E *and* lower T)  | no           |
| N=4       | none                                       | **yes**      |
| N=5       | **N=4** dominates (lower E *and* lower T)  | no           |

So the Pareto-optimal swarm sizes for this geometry are **{N=1, N=2, N=4}**.
N=3 and N=5 are strictly dominated by N=4 — there is no time budget and no
energy budget for which they are the right answer.

This is a sharper, more useful conclusion than the paper currently states. The
paper reports a smooth curve with a knee at N=3 and recommends N=2–3 as the
operating range. The data, when N=4 is included, says:

- For low-energy / time-tolerant missions: pick **N=1** (50 Wh, 16 min).
- For balanced operations: pick **N=2** (59 Wh, 8 min, parallel efficiency
  0.99 — essentially free parallelism).
- For time-critical operations: pick **N=4** (65 Wh, 4 min, the actual fastest
  point in the table).
- **Never pick N=3 or N=5** — both are strictly dominated.

---

## 4. Why N=4 is super-linear and why N=5 regresses

This is the same FOV-rounding story from
[`istras_energy_explainer.md`](istras_energy_explainer.md), now applied to the
Pareto axis directly.

The lawnmower controller hands each drone a region of width `600 / N` metres
and sweeps it with internal lines at the camera footprint pitch `s ≈ 85 m`
(40 m altitude, 100° HFOV, 10% overlap). The number of sweep lines per drone
is `ceil(region_width / s)`:

| N | region (m) | sweeps/drone | total sweep lines | over-coverage waste |
|---|------------|--------------|-------------------|----------------------|
| 1 | 600        | 8            | 8                 | minimal              |
| 2 | 300        | 4            | 8                 | minimal              |
| 3 | 200        | 3            | 9                 | one extra line       |
| 4 | 150        | 2            | 8                 | minimal              |
| 5 | 120        | 2            | 10                | two extra lines      |

The right column predicts the experimental ranking exactly: cells where total
sweep lines = 8 (N=1, 2, 4) sit on the frontier; cells where the rounding
adds extra lines (N=3, 5) sit off it.

N=4 in particular is a "lucky" partition: 150 m fits two 85 m FOV lanes with
just 20 m of overlap, so coverage is achieved with the minimum 8 lines spread
across 4 drones — i.e. each drone only does 2 sweeps, finishing the work in
the shortest possible mission time (236 s) without paying redundancy. Hence
super-linear speedup.

N=5 is the worst case in the range: each drone has a 120 m strip that *also*
needs 2 sweeps (because 85 m × 1 < 120 m), but now there are 10 sweep lines
total and 50 m of overlap on every drone's strip. The extra sweeping shows up
as `+28.97 Wh` of forward-flight energy with no compensating time saving (in
fact T worsens because all 5 drones contend for the same launch window).

The conclusion you can defend in the paper:

> The energy–time trade-off is not smoothly convex in N. Strip-vs-FOV
> alignment determines whether a given swarm size lands on the frontier.
> For the 600 m × 600 m corridor at 40 m altitude, the frontier is
> N ∈ {1, 2, 4}; N=3 and N=5 are strictly dominated.

This is a more interesting result than the paper's current "knee at N=3"
framing — it directly motivates the geometry-aware partitioning that you list
under future work.

---

## 5. Marginal cost along the frontier

A practitioner with a time budget reads the frontier left-to-right and asks
"what does the next time-saving cost me in energy?" Computed only between
adjacent frontier points (dominated points excluded):

| Hop          | ΔE (Wh) | ΔT (s)  | s saved per Wh added |
|--------------|---------|---------|----------------------|
| N=1 → N=2    | +8.66   | −474.9  | **54.8**             |
| N=2 → N=4    | +6.18   | −253.2  | **41.0**             |
| (N=4 → N=5   | +28.97  | +52.4   | **regression**)      |

The marginal cost of buying more responsiveness is roughly flat between N=1→2
and N=2→4 — both purchases are economical at ~40–55 s saved per Wh added.
There is **no economical hop past N=4** for this geometry; the only way to go
faster than 236 s is to change the geometry or the policy, not the swarm size.

This is also why we recommend reporting the frontier explicitly in the paper
rather than the smoothed curve from the current Fig. 1.

---

## 6. How to read `F1_pareto.png`

The figure plots one filled circle per individual trial (colour-coded by N)
plus a dashed line through the cell means. Things to verify when reading it:

- The dashed line is the **convex hull through cell means**, not the true
  Pareto frontier. To get the true frontier visually, ignore any cell whose
  marker sits up-and-right of any other cell's marker — those are dominated.
- The N=4 cluster should sit clearly to the left and below the N=3 and N=5
  clusters in the regenerated figure. If it does not, the trial dataset has
  drifted and the analysis should be re-run.
- Trial-level scatter is informative: at N=5 the trials are tightly clustered
  in time (small std) but at N=2 and N=3 they spread widely. That is because
  T is bounded below by the slowest drone in the cell, not the fastest, and
  small swarms have higher per-drone T variance.

---

## 7. Reproducing the Pareto numbers

```bash
conda activate swarm

python3 scripts/analysis/energy_scaling_analysis.py \
    --logs-dir data/logs/istras \
    --include-dirs Sw1 Sw2 Sw3 Sw4 Sw5 \
    --output-dir data/results/energy_scaling --plots

# Inspect the frontier candidates and marginals
column -s, -t data/results/energy_scaling/scaling_summary.csv | less -S
column -s, -t data/results/energy_scaling/marginal_analysis.csv | less -S

# Visually:
xdg-open data/results/energy_scaling/figures/F1_pareto.png
```
