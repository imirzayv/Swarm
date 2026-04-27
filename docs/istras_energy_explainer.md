# ISTRAS energy model — what it computes and why forward-flight energy grows with N

This is a self-contained explainer for the energy numbers reported in the ISTRAS
paper. It targets two questions that come up every time the paper is read:

1. How is total mission energy computed from a trial log?
2. Why does forward-flight energy grow with the swarm size N, when each
   individual drone covers fewer sweep lines?

All implementation references are to [`scripts/metrics_compute.py`](../scripts/metrics_compute.py).

---

## 1. Three-term energy model

The model is the Abeywickrama et al. (2018) decomposition for a DJI Matrice
100-class quadrotor. Power constants come from their light-payload calibration
([metrics_compute.py:49-53](../scripts/metrics_compute.py#L49-L53)):

| Symbol      | Value     | Meaning                                       |
|-------------|-----------|-----------------------------------------------|
| `P_HOVER`   | 142.8 W   | power to hold attitude at zero ground speed   |
| `P_FWD@5`   | 161.0 W   | power for level flight at 5 m/s ground speed  |
| `ΔP_fwd`    | 18.2 W    | extra power above hover when translating      |
| `V_CRUISE`  | 5.0 m/s   | cruise speed used for time-of-flight scaling  |
| `P_COMM`    | 10.0 W    | always-on telemetry / detection messaging     |

Total mission energy decomposes additively into three terms, each in watt-hours:

```
E_hover   = P_HOVER × T × N        / 3600
E_forward = ΔP_fwd  × (D / V_CRUISE) / 3600
E_comm    = P_COMM  × T × N        / 3600
E_total   = E_hover + E_forward + E_comm
```

Where:

- `T` = mission duration in seconds — first launch to last-target detection.
- `N` = swarm size.
- `D` = **sum of horizontal distances** flown across all drones (computed by
  integrating consecutive-sample displacements from `positions.csv`).
- `D / V_CRUISE` = total seconds spent in forward flight, summed across drones.

The forward term uses the **delta** above hover (18.2 W, not 161 W). This avoids
double-counting: the hover term already pays the 142.8 W floor for every second
every drone is airborne; the forward term only adds the extra propulsion cost
of translation.

CO₂ then follows linearly:

```
CO2_AZ = E_total[kWh] × 440 g/kWh    # Azerbaijan grid
CO2_EU = E_total[kWh] × 250 g/kWh    # EU average grid
```

Implementation lives at
[`compute_energy_and_emissions`](../scripts/metrics_compute.py#L228) in
`metrics_compute.py`. It is called once per trial and writes the breakdown into
`metrics_summary.json`.

---

## 2. Why E_forward grows with N

The naive intuition — *"one drone covers all the lawnmower lines, five drones
each cover one fifth, so total ground distance D should stay constant"* — is
the right picture for an **idealised** lawnmower with a single, infinitely-thin
sweep pitch and zero ferry overhead. In our scenario two geometric facts break
that ideal, and forward-flight energy is the term that pays for both.

### 2.1 Ferry-to-strip overhead is per-drone

All drones spawn at a common launch pad in one corner of the 600 m × 600 m
area. With N=1 the single drone enters its strip almost immediately. With N=5,
four of the five drones must fly **across the area** to reach their assigned
strip before any useful sweeping starts.

That ferry distance contributes nothing to coverage — but it is added to `D`
and therefore to `E_forward`.

```
N=1:  ferry ≈ 0 m       → 1× negligible overhead
N=5:  ferry ≈ 0…480 m   → 5× ferry legs of varying length, summed in D
```

This is a fixed-cost-per-drone effect: every drone you add pays its own ferry
toll regardless of how much sweeping it does afterwards.

### 2.2 Strip-vs-FOV rounding stacks across drones

The lawnmower controller assigns one **region** of width `area / N` to each
drone, then sweeps that region with internal lines whose pitch is set by the
camera footprint (≈ 95 m diameter at 40 m altitude with 10% overlap → effective
pitch `s ≈ 85 m`).

The number of sweep lines a drone needs **inside its own region** is
`ceil(region_width / s)`. The ceiling is the problem.

| N | region width (m) | sweep lines per drone | total sweep lines |
|---|------------------|------------------------|-------------------|
| 1 | 600              | ceil(600 / 85) = 8     | 1 × 8 = **8**     |
| 2 | 300              | ceil(300 / 85) = 4     | 2 × 4 = **8**     |
| 3 | 200              | ceil(200 / 85) = 3     | 3 × 3 = **9**     |
| 4 | 150              | ceil(150 / 85) = 2     | 4 × 2 = **8**     |
| 5 | 120              | ceil(120 / 85) = 2     | 5 × 2 = **10**    |

Each sweep line is ≈ 600 m long, so total sweep distance scales directly with
the right column. With N=1 you get the most efficient packing (8 lines, each
covering an 85 m wide strip of the 600 m region). With N=5 you pay for **10**
sweep lines because each per-drone region rounds up — the 120 m region needs
two 85 m sweep passes, leaving 50 m of overlap that is pure redundancy.

The rounding loss is not monotonic in N: N=4 happens to land at 8 lines (same
as N=1), which is exactly the dip you see in the measured data
(`E_total = 65 Wh at N=4` vs `68 Wh at N=3`). The paper's smoothed table hides
this dip; the actual `scaling_summary.csv` shows it clearly.

### 2.3 Strip-end turnarounds are per-drone

At every end of every sweep line the drone decelerates, reverses, and
re-accelerates — a short curved path at off-cruise speed. The number of
turnarounds equals the number of sweep lines, so it follows the rightmost
column of the table above. More drones with narrower regions ⇒ more
turnarounds ⇒ more time spent off-cruise ⇒ longer integrated path length in
`D`.

### 2.4 Why hover energy stays roughly flat — the contrast

The same accounting that makes `E_forward` grow makes `E_hover` *not* grow much:

```
E_hover ∝ N × T
```

`T` falls roughly as `1/N` because the work is parallelised, so `N × T` moves
slowly. Empirically (full Sw1–Sw5 dataset, mean per cell):

| N | T (s) | N × T (drone-seconds) | E_hover (Wh) |
|---|-------|------------------------|--------------|
| 1 | 965   | 965                    | 42           |
| 2 | 490   | 980                    | 50           |
| 3 | 347   | 1041                   | 57           |
| 4 | 237   | 947                    | 55           |
| 5 | 289   | 1445                   | 81           |

So `N × T` drifts in a relatively narrow 950–1450 drone-second band while
`D` more than triples. Hover therefore grows roughly linearly with `N × T`
(42 → 81 Wh), but its *share* of the budget stays near 84–85 % because
the forward-flight term is small in absolute terms. The N=4 row also
shows the FOV-rounding sweet spot directly: `N × T` is *smaller* there
than at N=2 or N=3, which is why N=4 lands on the Pareto frontier
(see `istras_pareto_analysis.md`).

### 2.5 Putting it all together

The forward-flight term is the one that captures the geometric overheads of
parallelising the coverage problem:

- ferry-to-strip: fixed cost per added drone
- per-region sweep-line rounding: super-linear in N when N is "between" pitch
  multiples, near-flat when N happens to align
- per-region strip-end turnarounds: linear in number of sweep lines

Hover is bounded by the run-until-completion stop rule. Communication is
linear in `N × T` and small. Everything else lands in `E_forward`, and that is
why the energy–time trade-off curve has its knee where it does.

---

## 3. Reproducing the numbers

```bash
conda activate swarm

# 1. Per-trial energy is already in each metrics_summary.json.
#    To re-derive from raw positions/detections only:
python3 scripts/metrics_compute.py \
    --data-dir data/logs/istras/Sw3/lawnmower_d3_t8_trial01 \
    --area-size 600 --altitude 40

# 2. Aggregate across all five cells:
python3 scripts/analysis/energy_scaling_analysis.py \
    --logs-dir data/logs/istras \
    --include-dirs Sw1 Sw2 Sw3 Sw4 Sw5 \
    --output-dir data/results/energy_scaling --plots

# 3. Inspect headline numbers:
column -s, -t data/results/energy_scaling/scaling_summary.csv | less -S
```

Outputs:

- `data/results/energy_scaling/trial_index.csv` — one row per trial.
- `data/results/energy_scaling/scaling_summary.csv` — one row per (method, N).
- `data/results/energy_scaling/marginal_analysis.csv` — between-cell deltas
  (ΔE, ΔT, speedup, parallel efficiency).
- `data/results/energy_scaling/figures/F{1..5}_*.png` — paper figures.

---

## References

Abeywickrama, H. V., Jayawickrama, B. A., He, Y., & Dutkiewicz, E. (2018).
Comprehensive energy consumption model for unmanned aerial vehicles, based on
empirical studies of battery performance. *IEEE Access*, 6, 58383–58394.
