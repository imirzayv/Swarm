# ISTRAS'26 — 10-Day Experiment Runbook

> **⚠ HISTORICAL — pre-energy-scaling pivot (2026-04-25).** The active
> paper no longer runs the 9-trial / 3-method cell described here. The
> current reference is **`istras_energy_scaling_guide.md`**; the
> completed dataset is `Sw1`–`Sw5` (lawnmower N ∈ {1..5}, 27 trials)
> and the writeup is `istras_results_discussion_conclusion.md`.

> Companion to `10_day_development_guide.md`. Lists every experiment trial
> required for the submission, with the exact CLI command to run it.
>
> **Scope:** 3 methods × 3 trials = **9 trials total** (~18 min wall-clock sim).
> **Output root:** `data/logs/istras/`

---

## Prerequisites (once per shell session)

```bash
conda activate swarm
source /opt/ros/humble/setup.bash
source ~/Desktop/Swarm/ros2_ws/install/setup.bash
cd ~/Desktop/Swarm
```

Shared parameters for all trials:

| Parameter | Value | Flag |
|---|---|---|
| Drones | 3 | `--drones 3` |
| Targets per trial | 3 (person + vehicle + fire) | `--targets 3 --classes person,vehicle,fire` |
| Trial duration | 120 s | `--duration 120` |
| Monitoring area | 40 × 40 m | `--area-size 40` |
| Flight altitude | 10 m | `--altitude 10` |
| Output directory | `data/logs/istras/` | `--output-base data/logs/istras/` |

---

## E1 — Adaptive Voronoi (proposed method)

Decentralized weighted Voronoi coverage with detection-driven density updates.

```bash
# Trial 1
bash scripts/run_experiment.sh \
  --method adaptive \
  --drones 3 --targets 3 --classes person,vehicle,fire \
  --duration 120 --area-size 40 --altitude 10 \
  --trial 1 --seed 1 \
  --output-base data/logs/istras/

# Trial 2
bash scripts/run_experiment.sh \
  --method adaptive \
  --drones 3 --targets 3 --classes person,vehicle,fire \
  --duration 120 --area-size 40 --altitude 10 \
  --trial 2 --seed 2 \
  --output-base data/logs/istras/

# Trial 3
bash scripts/run_experiment.sh \
  --method adaptive \
  --drones 3 --targets 3 --classes person,vehicle,fire \
  --duration 120 --area-size 40 --altitude 10 \
  --trial 3 --seed 3 \
  --output-base data/logs/istras/
```

---

## E2 — Lawnmower Baseline

Deterministic back-and-forth sweep covering the monitoring area.

```bash
# Trial 1
bash scripts/run_experiment.sh \
  --method lawnmower \
  --drones 3 --targets 3 --classes person,vehicle,fire \
  --duration 120 --area-size 40 --altitude 10 \
  --trial 1 --seed 1 \
  --output-base data/logs/istras/

# Trial 2
bash scripts/run_experiment.sh \
  --method lawnmower \
  --drones 3 --targets 3 --classes person,vehicle,fire \
  --duration 240 --area-size 4000 --altitude 20 \
  --trial 2 --seed 2 \
  --output-base data/logs/istras/

# Trial 3
bash scripts/run_experiment.sh \
  --method lawnmower \
  --drones 3 --targets 3 --classes person,vehicle,fire \
  --duration 120 --area-size 40 --altitude 10 \
  --trial 3 --seed 3 \
  --output-base data/logs/istras/
```z

---

## E3 — Static Grid Baseline

Drones hold fixed cell-center positions for the full trial.

```bash
# Trial 1
bash scripts/run_experiment.sh \
  --method static \
  --drones 3 --targets 3 --classes person,vehicle,fire \
  --duration 120 --area-size 40 --altitude 10 \
  --trial 1 --seed 1 \
  --output-base data/logs/istras/

# Trial 2
bash scripts/run_experiment.sh \
  --method static \
  --drones 3 --targets 3 --classes person,vehicle,fire \
  --duration 120 --area-size 40 --altitude 10 \
  --trial 2 --seed 2 \
  --output-base data/logs/istras/

# Trial 3
bash scripts/run_experiment.sh \
  --method static \
  --drones 3 --targets 3 --classes person,vehicle,fire \
  --duration 120 --area-size 40 --altitude 10 \
  --trial 3 --seed 3 \
  --output-base data/logs/istras/
```

---

## Run Everything (one-shot loop)

Copy-paste this block to execute all 9 trials sequentially:

```bash
for method in adaptive lawnmower static; do
  for trial in 1 2 3; do
    bash scripts/run_experiment.sh \
      --method $method \
      --drones 3 --targets 3 --classes person,vehicle,fire \
      --duration 120 --area-size 40 --altitude 10 \
      --trial $trial --seed $trial \
      --output-base data/logs/istras/
    sleep 5
  done
done
```

---

## Post-Experiment Analysis (Day 4)

After all trials complete, aggregate metrics and render figures:

```bash
# Aggregate metrics across all 9 trials into one table
python3 scripts/metrics_compute.py \
  --log-base data/logs/istras/ \
  --aggregate \
  --output data/logs/istras/summary.csv

# Generate comparison plots (distance + latency bar charts)
python3 scripts/plot_paths.py \
  --log-base data/logs/istras/ \
  --compare adaptive lawnmower static \
  --output figures/
```

---

## Trial Output Layout

Each trial writes to `data/logs/istras/<method>_d3_t3_trial0N/` and contains:

| File | Content |
|---|---|
| `drone1_position.csv`, `drone2_position.csv`, `drone3_position.csv` | Per-drone `timestamp, x, y, z` |
| `drone{N}_detection.csv` | `timestamp, class_name, confidence, target_id` |
| `swarm_mode.csv` | `timestamp, mode, exploit_drone_ids` |
| `trial_meta.json` | method, seed, target positions, drone start positions |
| `metrics_summary.json` | Distance, energy, CO₂, coverage, latency |
| `paths_combined.png` | 2D path plot for the trial |

---

## Definition of Done (Day 3)

- [ ] `data/logs/istras/` contains **9 trial folders**
- [ ] Each folder has position CSVs for all 3 drones + detection log + metadata
- [ ] `metrics_summary.json` exists in every folder

If any trial fails (Gazebo crash, MAVSDK timeout), re-run only that single
trial using its exact command above — seeds are per-trial so results stay
reproducible.

---

## Out of Scope (Save for MDPI)

These experiments are explicitly **skipped** for ISTRAS'26 per the 10-day plan:

- Ablations: `binary_voronoi`, `all_converge`, `pso`, `apf`, `random`
- 5-drone scalability sweep
- Multi-class formation analysis beyond the 3-class mix
- Sensitivity analysis on HSV thresholds or Voronoi parameters
