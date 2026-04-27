# ISTRAS'26 — Final Research Guide

> **⚠ HISTORICAL — superseded 2026-04-25 by the energy-scaling pivot.**
> The active paper reference is **`istras_energy_scaling_guide.md`**
> (lawnmower N-scaling Pareto, no σ analysis, no dynamic events).
> Read that first. This doc remains useful for the energy-model
> derivation (§6) and the σ / close-vs-far framing if a follow-up
> paper revives them. The Abeywickrama three-term implementation in
> `metrics_compute.py` is unchanged from this guide.

> Ground-truth reference for the paper. Explains every experiment's goal,
> the close-vs-far target regime, the static-vs-dynamic split, the energy
> model, and the analyses that turn trial logs into paper figures.
>
> **Supersedes:** `istras_development_guide_v2.md`, `istras_experiments_v3.md`,
> and the legacy E1/E2/E3 configs (now retained only as aliases for D1/D2/D3).
>
> **Companion runbooks:** `istras_experiments_static.md` (static-target runs)
> and `istras_experiments_dynamic.md` (dynamic-target runs).

---

## 1. Research Questions

The paper defends a single central claim:

> **Claim.** Detection-aware weighted Voronoi coverage outperforms sweep/grid
> baselines on event-response latency at equal swarm size, and its advantage
> carries — though shrinks — in the static-world regime where all targets
> are visible from t=0.

The claim decomposes into five questions, matched one-to-one with the five
experiments run by `scripts/run_all_experiments.sh`:

| RQ | Variable | Experiment | Contrast |
|----|----------|------------|----------|
| RQ-S1 | static world, fixed swarm | S1 | adaptive vs lawnmower vs static-grid at 3 drones |
| RQ-S2 | static world, swarm size | S2 | adaptive vs lawnmower at 5 drones |
| RQ-D1 | dynamic world, fixed swarm | D1 | adaptive vs lawnmower vs static-grid at 3 drones |
| RQ-D2 | dynamic world, swarm size | D2 | adaptive vs lawnmower at 5 drones |
| RQ-D3 | dynamic world, event density | D3 | adaptive vs lawnmower under overlapping events |

The static-vs-dynamic split is deliberate: keeping both lets the paper
disentangle two orthogonal wins of adaptive coverage — *efficient search*
(static world) and *fast event response* (dynamic world).

---

## 2. Is Splitting Static and Dynamic Worth It?

**Yes — for three reasons:**

1. **It separates the mechanism from the metric.** In a mixed regime
   (targets scheduled but present at t=0 anyway) adaptive's win can be
   attributed to *either* better baseline search *or* better event
   response. Splitting removes the confound: S1/S2 measure search, D1–D3
   measure response. Each figure then has a single interpretation.

2. **Reviewers ask for it.** "What if all targets are known at t=0?"
   is the first question any sweep/grid advocate raises — our prior
   version couldn't answer it. Running S1/S2 removes that rebuttal.

3. **Two plots are cheaper than one disputed plot.** Figures 2 (static
   head-to-head) and Figures 3–4 (dynamic head-to-head + scaling) fit
   naturally into separate Results subsections, and the paper reads as
   a stronger claim.

**Cost:** S1/S2 add 25 trials (~90 min wall-clock) on top of the 31 dynamic
trials. Total battery ≈ 56 trials, 3.5–4 h of simulation. Acceptable.

---

## 3. Geometry and Target Layout

Every experiment in the battery uses the same physical stage:

| Parameter | Value | Why |
|---|---|---|
| Monitoring area | 400 m × 400 m | 16 ha — realistic single-mission search footprint; 10× the toy 40 m area used in the pilot runs |
| Altitude | 60 m | HSV/YOLO-legible footprint at this altitude is ~142 m diameter (radius ≈ 71 m) |
| Trial duration | 200 s | Large enough for 3-event dynamic schedule + exploit_timeout |
| Cruise speed (energy model calibration point) | 5 m/s | Matches Abeywickrama 2018 Table III |
| Target side length | 3 m × 3 m × 0.1 m flat slab | Nadir-view reliable; colour = class |

### 3.1 Target count — static runs

`n_targets_static = 8` in a 400 m × 400 m square.

Derivation: at altitude 60 m the camera footprint radius is ≈ 71 m, so
each drone covers ~4% of the area instantaneously. 8 targets spread over
160 000 m² gives one target per 20 000 m² — a mean spacing of ~140 m, or
~2× the footprint radius. That's sparse enough that random coverage
can't hit all targets by chance but dense enough that mission completion
is possible within 200 s.

### 3.2 Schedule — dynamic runs

Two schedules, reused across D1/D2 and D3:

| Experiment | Schedule (s) | Mean inter-event gap | Purpose |
|---|---|---|---|
| D1, D2 | `25, 85, 145` | 60 s | Comfortably above `exploit_timeout_s = 30`; each event fully retires before the next |
| D3 | `20, 45, 75, 105, 140, 175` | 31 s | Matches `exploit_timeout_s` → events overlap → exercises `max_concurrent_exploits = 2` |

---

## 4. The σ / Target-Separation Analysis (close vs far targets)

### 4.1 Why separation matters

Adaptive's density function writes a Gaussian bump of width σ on each
detection. With `detection_sigma = 15 m` in a 400 m area, a single bump
covers roughly 2σ ≈ 30 m diameter of significant mass. So:

- **Close regime (min pairwise distance < 2σ ≈ 30 m):** Bumps overlap.
  Weighted-centroid Lloyd's iteration pulls multiple drones toward the
  same high-density region. Adaptive's *coverage* behaviour diverges from
  lawnmower's — it really does concentrate drones.
- **Far regime (min pairwise distance > 2σ):** Bumps are disjoint. Each
  acts independently. A drone near one bump sees the same centroid
  regardless of whether the other bump exists. Adaptive's coverage
  behaviour collapses onto the lawnmower/Voronoi baseline.

**In other words: σ is the scale at which "concentration" stops being a
thing.** The paper's coverage-benefit claims must be qualified by whether
the targets actually sit in the close regime.

### 4.2 Where adaptive still wins in the far regime

Even when bumps are disjoint, adaptive wins on **event-response
latency**. The exploit branch fires on any high-confidence detection
regardless of whether it overlaps with another. The class-specific
formation (cluster / chain / perimeter) is a local, per-event behaviour,
not a density-field phenomenon. So the paper's response-latency claim
holds in both regimes — only the coverage/distance claim is σ-gated.

### 4.3 How we measure it

Every `metrics_summary.json` now carries three fields populated by
`compute_target_separation()` in `scripts/metrics_compute.py`:

| Field | Meaning |
|---|---|
| `min_pairwise_target_distance_m` | Closest pair in the trial — the binning variable |
| `mean_pairwise_target_distance_m` | Average pair — sanity check |
| `max_pairwise_target_distance_m` | Spread of the layout |

Post-hoc, `scripts/analysis/target_separation_analysis.py` buckets every
trial into separation bins and computes the adaptive−lawnmower delta on
each metric per bin:

```bash
python3 scripts/analysis/target_separation_analysis.py \
    --logs-dir data/logs/istras \
    --output-dir data/results/separation_analysis \
    --bin-edges 0 30 60 120 9999 \
    --plots
```

Default bin edges `0, 30, 60, 120, ∞` correspond to
`< 2σ`, `2σ-4σ`, `4σ-8σ`, `> 8σ`. The `adaptive_vs_lawnmower_by_separation.csv`
output is the one the paper's discussion section cites: if the delta on
`event_phase_distance_m` shrinks toward zero as separation grows (and the
delta on `mean_event_latency_s` holds), the paper's narrative is correct
and σ-gated.

### 4.4 Recommended reporting in the paper

- **Figure 2 caption:** Report event_phase_distance delta across
  separation bins. Expect the close-bin delta to be ≥ 20% and the far-bin
  delta to be near zero.
- **Figure 4 caption:** Report per-event latency delta across bins.
  Expect it to be roughly constant (σ doesn't gate response).
- **Discussion:** State σ = 15 m explicitly, note the 2σ cross-over
  scale, and say that a deployment would tune σ to the expected event
  density. This turns a latent limitation into an explicit design knob.

---

## 5. Exploration-vs-Exploitation Rebalance

### 5.1 What changed

The old coordinator held one `active_exploit` at a time. Consequence:
while exploit mode was running for target A, any new detection on target
B within that 30 s window was *ignored* for the purposes of triggering
a formation response — only the density map absorbed it. Under D3
(6 events in 200 s, mean gap ≈ 30 s) this meant one in three events
never got its formation.

### 5.2 What it does now

The coordinator now maintains `active_exploits: list[ExploitEvent]` with
two new parameters (`config/adaptive_params.yaml`):

- `max_concurrent_exploits: 2` — cap on simultaneous formations
- `exploit_merge_radius_m: 20` — detections within this of an existing
  event reinforce it; outside this, they may open a second slot

Admission policy for a new detection (in `_detection_callback`):

1. Find the nearest active exploit. If within `exploit_merge_radius_m`,
   bump its `last_confidence` and return. *(Keeps duplicates out.)*
2. Otherwise, if `confidence ≥ exploit_confidence_threshold` and the
   event count is below `max_concurrent_exploits`, try to admit.
3. Admission succeeds only if there are enough free (non-busy) drones
   left to form the new formation while keeping `min_explore_drones`
   globally exploring.

`min_explore_drones` is now a **global** floor, not per-event. For
5-drone runs it should be 2; for 3-drone runs it stays at 1 so one
class (fire) can still get its 3-drone perimeter.

### 5.3 New per-event metrics

`metrics_compute.py` now reports `median_event_latency_s` and
`max_event_latency_s` alongside the mean. Under D3 the max is the
meaningful diagnostic: if the system dropped an event entirely, its
latency is undefined and the max picks up only the events that *were*
served. Compare `targets_found / targets_total` across methods to see
whether adaptive actually serves the extra events.

### 5.4 Does the swarm "explore" new targets, or just chase the first?

Both, now. Every 2 s the coordinator publishes Voronoi waypoints to
**every explore-set drone** using the density map, which still contains
every detection as a decaying bump. So exploring drones bias toward
undervisited corners of the field even while two other drones are
locked on exploit formations elsewhere. The old single-slot design was
what blocked exploration of a *new* high-confidence event while one was
already running; that specific failure mode is now fixed.

---

## 6. Energy Model (Abeywickrama 2018)

### 6.1 What changed

The old model was hover-only:

```
E = P_hover × duration × n_drones
```

Distance did not enter, so the paper's core claim ("adaptive saves energy
by moving less") was vacuously true only because duration was fixed.
Reviewers would (correctly) flag this.

### 6.2 What it is now

`compute_energy_and_emissions` in `scripts/metrics_compute.py` implements
the three-term decomposition from **Abeywickrama, Jayawickrama, He &
Dutkiewicz, 2018, *IEEE Access* 6:58383–58394** (DJI Matrice 100,
light-payload regime):

```
E_total = P_hover × T × n_drones                       hover term
        + ΔP_fwd  × (D / v_cruise)                    forward-flight term
        + P_comm  × T × n_drones                      telemetry term
```

Constants used:

| Symbol | Value | Source |
|---|---|---|
| `P_HOVER_W` | 142.8 W | Abeywickrama Table II |
| `P_FWD_5MS_W` | 161.0 W | Abeywickrama Table III, 5 m/s cruise |
| `DELTA_FWD_W` | 18.2 W | = P_fwd − P_hover |
| `V_CRUISE_MS` | 5.0 m/s | Matches calibration point |
| `P_COMM_W` | 10.0 W | Abeywickrama §IV-C, always-on 2.4 GHz link |

`D` is `total_distance_m` — the sum of inter-sample segments across all
drones, already computed by `compute_energy()` (the distance metric, not
the energy one, despite the name). The new hook passes it to
`compute_energy_and_emissions()` automatically.

### 6.3 Breakdown now exposed

Every `metrics_summary.json` carries:
- `hover_energy_wh`, `forward_energy_wh`, `comm_energy_wh` — per-term
- `total_energy_wh`, `energy_per_drone_wh` — sums
- `total_energy_kwh`, `co2_azerbaijan_g`, `co2_eu_g` — downstream

The forward-flight term will be the dominant source of adaptive's energy
advantage in the paper's Results section. Before this fix it was zero.

### 6.4 Sanity check (back-of-envelope)

For a 3-drone, 200 s trial where each drone flies 500 m (typical):
- hover: 142.8 × 200 × 3 / 3600 = 23.8 Wh
- forward: 18.2 × (1500 / 5) / 3600 = 1.5 Wh
- comm: 10 × 200 × 3 / 3600 = 1.7 Wh
- total: 27.0 Wh (~1.5 L diesel-equivalent on an Azerbaijani grid)

Adaptive runs that cover less distance will shave a few percent off the
total via the forward-flight term. Not huge, but large enough to show up
as a statistically significant bar in Figure 2.

---

## 7. Summary of Experiments

(Full commands live in the two runbooks.)

```
S1 — Static head-to-head     : 3 drones, 8 targets at t=0        , 3 methods × 5 trials = 15
S2 — Static scaling          : 5 drones, 8 targets at t=0        , 2 methods × 5 trials = 10
D1 — Dynamic head-to-head    : 3 drones, schedule [25,85,145]    , 3 methods × 5 trials = 15
D2 — Dynamic scaling         : 5 drones, schedule [25,85,145]    , 2 methods × 5 trials = 10
D3 — Dense dynamic           : 3 drones, schedule [20,45,75,     , 2 methods × 3 trials =  6
                                           105,140,175]
                                                                  ────────────────────────
                                                                                       56
```

One-shot command:

```bash
bash scripts/run_all_experiments.sh                     # full battery
bash scripts/run_all_experiments.sh --experiments S1 S2 # static-only
bash scripts/run_all_experiments.sh --experiments D1 D2 D3   # dynamic-only
bash scripts/run_all_experiments.sh --resume            # re-run only missing trials
```

Legacy `E1/E2/E3` names are preserved as aliases (E1→D1, E2→D2, E3→D3)
so scripts that reference them still work.

---

## 8. Paper Figures → Experiment Mapping

| Figure | Source | What it shows |
|---|---|---|
| **F1. System block diagram** | none | Conceptual |
| **F2. Static head-to-head (3 drones)** | S1 | Coverage %, total_distance_m, total_energy_wh for 3 methods |
| **F3. Dynamic scaling** | D1 @ 3 drones + D2 @ 5 drones | event_phase_distance + mean_event_latency vs swarm size, per method |
| **F4. Per-event latency distribution** | D3 | Box plot of per-event latency by event index (1..6), adaptive vs lawnmower |
| **F5. Separation sensitivity** | All experiments combined via `target_separation_analysis.py` | Δ(adaptive − lawnmower) on event_phase_distance vs min target separation |

F5 is the one that operationalises Section 4.4's close/far story.

---

## 9. Limitations and Honesty Clauses

The paper should flag three items up front:

1. **σ is tuned, not learned.** `detection_sigma = 15 m` was chosen to
   match the HSV detector's localisation precision; it is not part of
   the method's contribution. A separation sweep (F5) shows where the
   choice gates behaviour. Future work: adapt σ per detection class.

2. **Energy is modelled, not measured.** The SITL drones are PX4
   multirotors whose "flight" is kinematic. We apply the Abeywickrama
   empirical curve post-hoc to the logged trajectories. This is
   standard in simulation-only papers but deserves explicit mention.

3. **`max_concurrent_exploits = 2` is conservative.** A deployment with
   10+ drones could run more concurrent formations. We capped at 2
   because 3- and 5-drone swarms can't form more than two `fire`
   perimeters (3 drones each) anyway. This is a design choice, not a
   limitation of the algorithm.

---

## 10. File Map

```
scripts/
  metrics_compute.py                  energy, latency stats, separation
  voronoi_coordinator.py              concurrent exploit slots
  run_all_experiments.sh              S1,S2,D1,D2,D3 battery driver
  run_experiment.sh                   single-trial orchestrator
  analysis/
    target_separation_analysis.py     σ / close-vs-far post-hoc
config/
  adaptive_params.yaml                all runtime knobs
  experiment_configs/
    S1.yaml  S2.yaml  D1.yaml  D2.yaml  D3.yaml
docs/
  istras_final_guide.md               ← you are here
  istras_experiments_static.md        S1, S2 runbook
  istras_experiments_dynamic.md       D1, D2, D3 runbook
```

---

*Last updated: 2026-04-24*
