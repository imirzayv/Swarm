# ISTRAS'26 — 10-Day Development Guide (v2)

> **⚠ HISTORICAL — pre-energy-scaling pivot (2026-04-25).** The active
> paper is now lawnmower N-scaling, not adaptive-vs-baseline. Read
> **`istras_energy_scaling_guide.md`** first; the completed dataset is
> `Sw1`–`Sw5` and the writeup is
> `istras_results_discussion_conclusion.md`.

> **Paper:** Scalable Energy-Responsive UAV Swarm Coverage for Sustainable Aerial Monitoring of Transport Corridors
> **Revision:** v2 — expanded research question, richer experiments
> **Deadline:** 1 May 2026 · **Daily budget:** 2–3 hours
> **Active days:** 8 · **Buffer days:** 2

---

## 0. Upfront Answers

### How many references should the paper have?

The ISTRAS'26 author guidelines and their APA 7th referencing document specify the format but **do not set a minimum or maximum reference count**. Your call.

From inspection of the ISTRAS'25 Abstract Book (accepted short papers, 3–5 pages, Springer Sustainable Aviation series), typical reference counts for drone-related short papers fall in these ranges:

| Paper type | Typical count | Example from ISTRAS'25 |
|---|---|---|
| Pure literature review | 25–40 | IST25-105 (green innovations review) |
| Experimental short paper | 15–25 | IST25-37 (YOLOv12 surveillance) |
| Architecture / framework | 12–20 | IST25-16, IST25-17 (modular avionics) |
| Conceptual / position paper | 10–15 | IST25-64 (vertical city) |

For a **high-quality experimental short paper targeting Springer Sustainable Aviation indexing**, aim for **18–22 references**. This is enough to credibly position your work, acknowledge foundational literature, and support each technical choice with a citation. Fewer than 12 reads underprepared; more than 30 overstuffs a 3–5 page paper.

**Suggested reference budget for your paper:**

- 4–5 foundational swarm/Voronoi coverage papers (Cortés et al. 2004; Schwager; Lloyd's algorithm; bio-inspired baselines)
- 3–4 UAV-based ITS / transport-corridor monitoring papers (IST25-37 analogue; CVT flood 2025; Enhanced Voronoi + YOLO 2024)
- 2–3 energy/sustainability-specific UAV papers (Abeywickrama et al. 2018 energy model; grid emission factor source; sustainability-focused aviation paper)
- 2–3 simulation / toolchain citations (PX4 SITL paper; Gazebo paper; ROS 2 paper)
- 2–3 MARL or coordination-algorithm references (MAPPO, QMIX, or the recent DG-MAPPO)
- 1–2 self-citations or related-group citations to demonstrate continuity

**Total: ~18–20 references.** Adjust up for a review paper, down for a very focused technical contribution.

---

## 1. What Are We Trying to Prove?

### The core research question

> **How does the energy cost of event-responsiveness scale with UAV swarm size in coverage-driven transport monitoring, and when does adaptive coordination dominate conventional sweep-based strategies?**

This is the question the v2 paper answers. It is a meaningful step up from the v1 question ("does adaptive beat lawnmower?"), which is settled literature, and turns the paper from a point-comparison into a scaling study.

### Decomposition into testable sub-questions

| Sub-question | How the experiment answers it |
|---|---|
| **SQ1.** Does adaptive coverage save energy versus sweep/grid baselines at a fixed swarm size? | E1: 3 methods × 3 drones × 5 trials on dynamic-target corridor. Compare distance, energy, CO₂, detection latency. |
| **SQ2.** How does the adaptive advantage change with swarm size (3 vs 5 drones)? | E2: re-run adaptive and lawnmower at 5 drones, compare scaling curves. |
| **SQ3.** When targets arrive dynamically (not at t=0), how quickly does each strategy respond? | E3: dynamic spawning schedule (targets appear at t=20, 50, 80 s). Compute per-event detection latency. |
| **SQ4.** What is the per-mission operational CO₂ cost, and how does it vary across strategies and regional grids? | Derived analytically from E1 and E2 distance data. |

### The hypothesis

We hypothesize that:

- Adaptive coverage reduces per-mission flight distance by 15–30% compared to lawnmower at comparable detection coverage.
- The advantage **grows** with swarm size: with 5 drones, the adaptive swarm concentrates more aggressively on detected regions while maintaining baseline coverage, whereas lawnmower scaling is linear-and-redundant.
- Dynamic target spawning widens the detection-latency gap: lawnmower has to physically sweep to the target; adaptive reallocates immediately upon first detection.
- Per-mission CO₂ scales proportionally with flight distance, meaning the energy result is also a sustainability result.

### Why this is stronger than v1

- "Does adaptive beat lawnmower?" has a predictable answer; reviewers may find it thin.
- "How does it scale?" produces a curve. Curves are more convincing than point estimates.
- "When does adaptive dominate?" has operational/policy implications — gives you a Discussion section that says something beyond the numbers.
- Dynamic targets make the event-response story tangible and connect directly to ITS use cases (accidents, incidents — all unscheduled).

---

## 2. What Are the Baselines?

Three methods, layered in difficulty and narrative role:

### Baseline 1 — Static grid (trivial lower bound)

Each drone hovers at a fixed grid cell center. Represents "no coordination intelligence." Serves as the floor: any adaptive method must beat this on detection quality (otherwise why bother), and adaptive should be close to it on energy (since grid drones barely move).

### Baseline 2 — Lawnmower sweep (the one you actually compete against)

Classic boustrophedon pattern. Each drone assigned a strip of the area, flies back and forth. This is the baseline the literature treats as the strong non-adaptive contender. It guarantees complete eventual coverage at the cost of high flight distance.

### Adaptive Voronoi (the proposed method)

Decentralized CVT with confidence-weighted density, Lloyd's relaxation, class-specific response, dual-mode exploration/exploitation.

### Experiment flow

Start each trial the same way across methods:
1. Drones spawn at same initial positions
2. First 20 s: no targets in world (all three methods have matched exploration period)
3. Target #1 spawns at t=20s
4. Target #2 spawns at t=50s
5. Target #3 spawns at t=80s
6. Trial ends at t=150s (gives 70 s to detect and respond to third target)

This common setup makes the three methods directly comparable on both steady-state energy use (first 20 s, no events) and event-response performance (next 130 s).

### Rolling comparison

The paper reports two comparisons side-by-side:

- **Steady-state:** distance/energy during 0–20 s, no events. Static grid should win trivially; lawnmower and adaptive should be similar.
- **Event-driven regime:** distance/energy/latency from 20–150 s. Adaptive should win on latency with comparable energy to lawnmower.

---

## 3. The Expanded Experiment Set

### Summary

| ID | Question | Methods | Drones | Targets | Trials | Sim time |
|---|---|---|---|---|---|---|
| **E1** | SQ1: head-to-head at 3 drones | adaptive, lawnmower, static_grid | 3 | 3 (dynamic) | 5 each = 15 | ~40 min |
| **E2** | SQ2: scalability to 5 drones | adaptive, lawnmower | 5 | 3 (dynamic) | 5 each = 10 | ~28 min |
| **E3** | SQ3: dynamic-target sensitivity | adaptive, lawnmower | 3 | 6 (dynamic, staggered) | 3 each = 6 | ~18 min |
| **Total** | | | | | **31 trials** | **~86 min** |

This is still achievable in **one afternoon of simulation**. The computational bottleneck is starting/stopping Gazebo between trials, not the trials themselves. At ~3–4 min per trial including setup/teardown, 31 trials fits in roughly 2 hours of wall-clock time.

### Why the numbers chosen

- **5 trials per condition for E1**: enough for mean ± std with non-trivial significance testing (paired t-test or Wilcoxon). v1 had 3; this is a modest bump.
- **E2 only compares adaptive and lawnmower**: static_grid's scaling is uninteresting (just more hovering), and cutting it saves 5 trials.
- **E3 only compares adaptive and lawnmower**: same reasoning.
- **6 targets in E3**: densifies the event stream so the difference in response strategy is amplified.

### Dynamic target spawning

Add to `spawn_targets.sh`:

```bash
# Schedule-based spawning
# Usage: spawn_targets.sh --schedule 20,50,80 --classes person,vehicle,fire --area 40 --seed 42

spawn_at_time() {
  local t=$1
  local class=$2
  local x=$3
  local y=$4
  sleep $t
  gz service -s /world/default/create \
    --reqtype gz.msgs.EntityFactory \
    --reptype gz.msgs.Boolean \
    --timeout 2000 \
    --req "sdf: '<...'" # model SDF for this class at (x,y)
}

# Launch all spawners in background
spawn_at_time 20 person 10 15 &
spawn_at_time 50 vehicle -12 8 &
spawn_at_time 80 fire 5 -18 &
wait
```

For random-seed reproducibility, the (x,y) positions come from a deterministic RNG seeded with `--seed`. Keep the spawn times fixed across trials so only target positions vary.

---

## 4. Expected Result Narrative

This is what you'll write in the Results section (adjust numbers to what you measure):

> *"At 3 drones, adaptive coverage reduced total flight distance by 22% compared to the lawnmower baseline (187 ± 14 m vs 240 ± 9 m, n=5, p<0.01) while achieving comparable detection coverage (95% vs 96%) and improving mean event-detection latency by 41% (8.3 ± 2.1 s vs 14.1 ± 3.4 s). The static grid baseline produced the lowest flight distance (58 ± 4 m) but achieved only 67% detection coverage because targets outside the hover cells were never seen. At 5 drones, the adaptive advantage widened: adaptive required 30% less flight distance than lawnmower (231 ± 19 m vs 329 ± 12 m) and reduced detection latency by 58% (4.2 s vs 9.9 s), indicating that the adaptive coordination becomes more effective as more agents are available to specialize their behavior. Converting flight distance to operational CO₂ using the Azerbaijan grid emission factor (0.44 kg CO₂/kWh), the 5-drone adaptive mission emits 1.16 g CO₂ versus 1.65 g for the lawnmower — a per-mission savings of 0.49 g CO₂, which at 100 missions per day scales to ~18 kg CO₂ per year per deployment."*

These are illustrative numbers, not predictions. Collect real values on Day 3–4.

---

## 5. Revised 10-Day Schedule

| Day | Date | Phase | Deliverable |
|---|---|---|---|
| 1 | Mon 20 Apr | Dynamic spawning + verification | `spawn_targets.sh --schedule` working; HSV detection verified end-to-end |
| 2 | Tue 21 Apr | Integration + demo recording | 2-min demo of dynamic event → adaptive response recorded |
| 3 | Wed 22 Apr | Run E1 (15 trials) | Raw CSV logs for 3 methods × 5 trials at 3 drones |
| 4 | Thu 23 Apr | Run E2 + E3 (16 trials) + analysis | 5-drone scaling + dense dynamic spawning logs; preliminary aggregate CSV |
| 5 | Fri 24 Apr | Metrics + figures | Final results CSV; 4 figures + 2 tables ready for paper |
| 6 | Sat 25 Apr | Draft: Intro + Related Work | ~1.25 pages with 8–10 references cited |
| 7 | Sun 26 Apr | Draft: System + Methodology | ~1.5 pages with remaining 8–10 references cited |
| 8 | Mon 27 Apr | Draft: Experiments + Results + Conclusion + Abstract | Full 3–5 page draft |
| 9 | Tue 28 Apr | **BUFFER** — formatting, anonymization, forms | Submission-ready paper |
| 10 | Wed 29 Apr | **BUFFER** — plagiarism check, submit | Submitted before midnight |

Still 2 days of buffer. The expansion comes from spending more time on Day 3–4 (16 extra trials) and a bit more analytical work on Day 5.

---

## 6. Day-by-Day Detail

### Day 1 (Mon 20 Apr) — Dynamic Spawning + Detection Verification

**Budget:** 3 hours

**Morning (1.5 h):** Implement dynamic spawning in `spawn_targets.sh`. Test by running a single empty trial and observing that targets appear at t=20s, 50s, 80s with the right colors. Use `ros2 topic echo /gazebo/model_states` or `gz topic -e /world/default/dynamic_pose/info` to confirm.

**Afternoon (1.5 h):** Run a full 3-drone adaptive pipeline once. Verify the detection → coordinator → waypoint chain works end-to-end with the new dynamic spawning. Record a 2-minute clip. Check that:

- First 20 s: all 3 drones spread via Voronoi, no activity
- t=20 s: person spawns, nearest drone detects it within ~2 s, 2 drones converge in cluster formation, 1 drone maintains explore mode
- t=50 s: same for vehicle (tracking chain formation)
- t=80 s: same for fire (triangle perimeter formation)

**Definition of done:** Recording exists at `docs/demo_dynamic.mp4`. All three classes trigger their class-specific formation.

### Day 2 (Tue 21 Apr) — Integration Polish + Experiment Automation

**Budget:** 3 hours

**Tasks:**
- Write/debug `scripts/run_all_experiments.sh` to loop over E1, E2, E3 configs.
- Verify trial cleanup between runs (no lingering Gazebo processes).
- Fix process-restart issues (usually takes 1–2 bug fixes).
- Create `config/experiment_configs/E1.yaml`, `E2.yaml`, `E3.yaml`.
- Dry-run 2 trials of each experiment to confirm CSVs land in the right place with the right metadata.

**Definition of done:** Running `bash scripts/run_all_experiments.sh --dry-run 2` produces 6 trial directories, each with complete CSV + metadata.

### Day 3 (Wed 22 Apr) — Run E1 (15 trials)

**Budget:** 3 hours (~1.5 h simulation + 1.5 h parallel work)

Run E1 in the background:

```bash
bash scripts/run_all_experiments.sh --experiments E1 --trials 5
```

While it runs, start writing the Introduction skeleton in `paper/main.tex`. Check on the simulation every 15 min.

**Definition of done:** 15 trial directories exist under `data/logs/istras/E1/`, each with 3 position CSVs, 1 detection CSV, 1 swarm_mode CSV, 1 metadata JSON.

### Day 4 (Thu 23 Apr) — Run E2 + E3 (16 trials) + Analysis

**Budget:** 3 hours (~1.5 h sim + 1.5 h analysis)

Morning:
```bash
bash scripts/run_all_experiments.sh --experiments E2 E3 --trials 5
```

Afternoon: run `metrics_compute.py` against all 31 trials, produce `data/results/istras_all_results.csv`. Do a quick `pandas` summary to sanity-check means and standard deviations.

**Definition of done:** `istras_all_results.csv` has 31 rows. Means and stds look sensible (adaptive should beat lawnmower on distance and latency; stds should be smaller than means).

### Day 5 (Fri 24 Apr) — Figures + Tables

**Budget:** 3 hours

Generate the paper's 4 figures and 2 tables:

1. **Figure 1** — System architecture diagram (draw.io, already sketched)
2. **Figure 2** — E1 bar chart: distance + latency, 3 methods (2-panel)
3. **Figure 3** — E2 scaling plot: distance vs swarm size, 2 methods (line plot)
4. **Figure 4** — E3 detection-latency-per-target plot: shows per-event response time across 6 staggered events
5. **Table 1** — E1 full metrics summary
6. **Table 2** — E2 + E3 key metrics summary

All plots in `plot_results.py`. PDFs at 300 DPI, saved to `figures/`.

**Definition of done:** 4 PDFs + 2 LaTeX tables in `figures/`, ready to `\input{}`.

### Day 6 (Sat 25 Apr) — Introduction + Related Work

**Budget:** 2.5 hours

Write ~1.25 pages. Cite 8–10 of your ~20 total references here:
- Cortés et al. 2004 (Voronoi coverage foundational)
- CVT flood monitoring 2025 (closest related work)
- Enhanced Voronoi + YOLO 2024 (another close comparison)
- IST25-37 (UAV-based surveillance precedent)
- 1–2 sustainability-UAV papers
- 1–2 ITS-with-drones papers
- Abeywickrama et al. 2018 (energy model)

**Structure:**
- Introduction: 4 paragraphs (motivation, problem, contribution list, paper organization)
- Related Work: 3 paragraphs (a) coverage algorithms, (b) UAV-ITS, (c) sustainability UAV

**Definition of done:** `paper/sections/01-intro.tex` and `02-related-work.tex` complete, first-draft quality.

### Day 7 (Sun 26 Apr) — System Architecture + Methodology

**Budget:** 3 hours

Write ~1.5 pages. Cite remaining 8–10 references here (simulation stack, algorithms, comparisons):
- PX4, Gazebo, ROS 2 papers
- Lloyd's algorithm paper
- MAPPO / QMIX (if mentioned as related future work)
- Recent swarm-coordination papers

**Structure:**
- System Architecture: 2 paragraphs + Figure 1
- Methodology: 5 subsections
  - 4.1 Weighted Voronoi partitioning (one equation)
  - 4.2 Confidence-weighted density function (one equation)
  - 4.3 Class-specific response selector (one formation table or pseudo-code)
  - 4.4 Dual-mode exploration/exploitation (brief algorithm or bullets)
  - 4.5 Baseline implementations (2 sentences each)

**Definition of done:** `paper/sections/03-system.tex`, `04-methodology.tex` complete.

### Day 8 (Mon 27 Apr) — Experiments + Results + Conclusion + Abstract

**Budget:** 3 hours

**Experimental Setup (~0.3 page):** Parameter table, simulation stack details, metric definitions.

**Results (~1.2 pages):** Three sub-sections:
- 6.1 E1: head-to-head at 3 drones (Figure 2, Table 1)
- 6.2 E2: swarm-size scaling (Figure 3)
- 6.3 E3: dynamic event response (Figure 4)
- 6.4 Energy and CO₂ implications

**Discussion (~0.3 page):** When does adaptive dominate? Trade-off between static-grid's low energy and its poor coverage. Scaling behavior implications.

**Conclusion (~0.2 page):** Contributions, key numerical result, future work (real-world validation, heterogeneous sensors, larger swarms).

**Abstract (~0.2 page):** Update the pre-written abstract with actual numbers.

**Definition of done:** Full draft in `paper/main.tex`, all sections present, all figures/tables referenced.

### Day 9 (Tue 28 Apr) — BUFFER: Polish and Formatting

**Budget:** 2–3 hours

- Read aloud, fix awkward phrasings
- Check font (Book Antiqua), sizes, margins against ISTRAS template
- Anonymize: remove author names, affiliations, acknowledgements, self-citations that reveal identity
- Verify every figure/table is referenced, every reference is cited
- Spellcheck
- Fill out Springer forms (License, Publishing Agreement, Copyright, Co-Author Consent)

**Definition of done:** Anonymized PDF ready; 4 PDFs of signed forms ready.

### Day 10 (Wed 29 Apr) — BUFFER: Plagiarism Check + Submit

**Budget:** 2 hours

- Run self-plagiarism check. Particularly watch for overlap with your MDPI draft and with `swarm_drones_report.md`. Rewrite any flagged passages.
- Upload to https://submission-system.istras.org/ with all supporting PDFs.
- Screenshot confirmation email.

**Definition of done:** Submission received confirmation visible.

Submit before midnight on 29 April. 1 May remains as absolute emergency buffer.

---

## 7. What Stays Cut

These are MDPI-paper scope, not ISTRAS short-paper scope:

- ❌ Ablation study (confidence-weighted vs binary, split vs all-converge) — save for MDPI
- ❌ 10+ drone scaling — save for MDPI if ever
- ❌ Target-density sweep (3 vs 5 vs 10 targets) — save for MDPI E3
- ❌ Heterogeneous sensors / payloads — save for MDPI future work
- ❌ MARL-based coordination — explicitly flagged as future work in Conclusion, not tested
- ❌ Real-world deployment — future work only

---

## 8. Risk Register (updated)

| Risk | Likelihood | Mitigation |
|---|---|---|
| Dynamic spawning has timing issues in Gazebo | Medium | Test on Day 1; have fallback to all-at-t=0 if blocking |
| 5-drone simulation unstable | Medium | Lower camera res to 320×240, frame skipping; has worked in past |
| E2 results don't show scaling advantage | Medium | Still publishable — frame as "adaptive maintains comparable efficiency as swarm grows" |
| E3 latency differences not significant | Low | Staggered spawning amplifies differences; 6 events × 3 trials = 18 data points per method |
| Plagiarism overlap with MDPI draft | Medium | Rewrite shared passages on Day 9; use different phrasings for the same equations |
| Gazebo crashes mid-E2 | Medium | Trials are independent; just rerun failed ones from `run_all_experiments.sh --resume` |
| Buffer days eaten by experiment slippage | Low-medium | Cut E3 if needed; E1+E2 alone is still a full paper |

---

## 9. One Key Discipline

If by end of Day 4 you don't have clean E1 results, **cut E3 immediately**. Don't cling to the richer story at the cost of a missed deadline. E1 + E2 alone — "head-to-head at 3 drones, plus scalability to 5 drones" — is still a coherent, publishable paper. E3 is the cherry, not the cake.

The hierarchy is:

1. E1 alone → minimum viable paper (v1 scope)
2. E1 + E2 → strong paper (scaling story)
3. E1 + E2 + E3 → great paper (scaling + dynamic-event story)

---

*Guide v2 prepared 19 April 2026. Supersedes v1.*
