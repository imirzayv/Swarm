#!/usr/bin/env python3
"""
Energy-scaling analysis: how does mission energy and mission time scale
with swarm size N for a fixed coverage policy (lawnmower)?

Reads `metrics_summary.json` from one trial directory per cell. Cells are
selected by which subdirectory of `--logs-dir` they live in (default:
Sw1, Sw3, Sw5 — the geometry-consistent 600 m × 40 m series at N=1, 3, 5).

Produces three CSVs and (optionally) five paper-ready figures:

  1. trial_index.csv            — one row per trial, all primitives
                                   plus derived per-target / per-coverage
                                   energy metrics.
  2. scaling_summary.csv        — one row per (method, N) cell with mean,
                                   std, and 95% CI on every metric.
  3. marginal_analysis.csv      — between-cell deltas: ΔE, ΔT, parallel
                                   efficiency for each consecutive-N pair.

Plots (--plots):
  F1_pareto.png             — total_energy_wh vs time_to_all_targets_s
  F2_decomposition.png      — hover/forward/comm stacked bars vs N
  F3_per_target_energy.png  — Wh per target found vs N
  F4_co2_emissions.png      — Azerbaijan + EU grid CO2 vs N
  F5_speedup_efficiency.png — speedup and parallel efficiency vs N

No new fields are computed in metrics_compute.py — every value here is a
post-hoc derivation from existing per-trial primitives.

Usage
-----
    conda activate swarm
    python3 scripts/analysis/energy_scaling_analysis.py \\
        --logs-dir data/logs/istras \\
        --output-dir data/results/energy_scaling \\
        --plots

    # Add new cells (e.g. N=2, N=4) by including their parent dirs:
    python3 scripts/analysis/energy_scaling_analysis.py \\
        --include-dirs Sw1 Sw2 S Sw4 Sw5 --plots
"""

import argparse
import csv
import json
import math
import os
import re
import sys
from statistics import mean, stdev


PRIMITIVE_FIELDS = [
    "n_drones",
    "duration_s",
    "area_size_m",
    "altitude_m",
    "total_distance_m",
    "coverage_percent",
    "redundancy_ratio",
    "hover_energy_wh",
    "forward_energy_wh",
    "comm_energy_wh",
    "total_energy_wh",
    "energy_per_drone_wh",
    "co2_azerbaijan_g",
    "co2_eu_g",
    "targets_found",
    "targets_total",
    "time_to_first_target_s",
    "time_to_all_targets_s",
    "first_detection_s",
    "total_detections",
]

DEFAULT_INCLUDE_DIRS = ["Sw1", "Sw3", "Sw5"]

# 95% CI half-width factor (Student-t, two-sided) for small n.
# n=5 → df=4 → t_{.975,4} = 2.776; n=3 → 4.303; n=10 → 2.262.
_T_CRIT = {
    2: 12.706, 3: 4.303, 4: 3.182, 5: 2.776, 6: 2.571, 7: 2.447,
    8: 2.365, 9: 2.306, 10: 2.262, 12: 2.201, 15: 2.145, 20: 2.093,
    30: 2.045,
}

_TRIAL_RE = re.compile(
    r"^(?P<method>[a-z_]+)_d(?P<drones>\d+)_t(?P<targets>\d+)_trial(?P<trial>\d+)$"
)


def _safe_float(x):
    try:
        v = float(x)
        if math.isnan(v):
            return None
        return v
    except (TypeError, ValueError):
        return None


def _t_crit(n: int) -> float:
    """Two-sided 95% Student-t critical value with df = n-1. Linearly
    interpolated from a small table; falls back to 1.96 for large n."""
    df = max(n - 1, 1)
    if df in _T_CRIT:
        return _T_CRIT[df]
    keys = sorted(_T_CRIT)
    if df < keys[0]:
        return _T_CRIT[keys[0]]
    if df > keys[-1]:
        return 1.96
    for i in range(len(keys) - 1):
        if keys[i] <= df <= keys[i + 1]:
            lo, hi = keys[i], keys[i + 1]
            frac = (df - lo) / (hi - lo)
            return _T_CRIT[lo] + frac * (_T_CRIT[hi] - _T_CRIT[lo])
    return 1.96


def collect_trials(logs_dir: str, include_dirs: list[str],
                   methods: list[str]) -> list[dict]:
    """Walk selected subdirs of `logs_dir` and return one dict per trial."""
    rows = []
    for sub in include_dirs:
        exp_dir = os.path.join(logs_dir, sub)
        if not os.path.isdir(exp_dir):
            print(f"warning: skip missing dir {exp_dir}", file=sys.stderr)
            continue
        for trial_name in sorted(os.listdir(exp_dir)):
            trial_dir = os.path.join(exp_dir, trial_name)
            if not os.path.isdir(trial_dir):
                continue
            m = _TRIAL_RE.match(trial_name)
            if not m:
                continue
            if methods and m.group("method") not in methods:
                continue
            summary_path = os.path.join(trial_dir, "metrics_summary.json")
            if not os.path.exists(summary_path):
                continue
            with open(summary_path) as f:
                summary = json.load(f)

            row = {
                "experiment": sub,
                "method": m.group("method"),
                "trial": int(m.group("trial")),
                "trial_dir": trial_name,
            }
            for field in PRIMITIVE_FIELDS:
                row[field] = _safe_float(summary.get(field))

            # per-drone distance CV (workload-balance metric)
            pdd = summary.get("per_drone_distance_m") or {}
            vals = [v for v in pdd.values() if isinstance(v, (int, float))]
            if len(vals) >= 2:
                m_ = mean(vals)
                row["per_drone_dist_cv"] = round(stdev(vals) / m_, 4) if m_ > 0 else None
            else:
                row["per_drone_dist_cv"] = None

            # ── Derived efficiency metrics (the core of this analysis) ──
            E = row["total_energy_wh"]
            tf = row["targets_found"]
            cov = row["coverage_percent"]
            t_all = row["time_to_all_targets_s"]
            row["wh_per_target_found"] = round(E / tf, 4) if E and tf and tf > 0 else None
            row["wh_per_pct_coverage"] = round(E / cov, 4) if E and cov and cov > 0 else None
            row["wh_per_minute_mission"] = (
                round(E / (t_all / 60.0), 4)
                if E is not None and t_all and t_all > 0 else None
            )
            # share of each energy term
            if E and E > 0:
                row["hover_share_pct"] = round(100 * (row["hover_energy_wh"] or 0) / E, 2)
                row["forward_share_pct"] = round(100 * (row["forward_energy_wh"] or 0) / E, 2)
                row["comm_share_pct"] = round(100 * (row["comm_energy_wh"] or 0) / E, 2)
            else:
                row["hover_share_pct"] = row["forward_share_pct"] = row["comm_share_pct"] = None

            rows.append(row)
    return rows


# Fields whose mean/CI we want in scaling_summary.csv (and also in the
# marginal-analysis deltas where it makes sense).
SUMMARY_FIELDS = [
    "duration_s",
    "total_distance_m",
    "coverage_percent",
    "hover_energy_wh", "forward_energy_wh", "comm_energy_wh",
    "total_energy_wh", "energy_per_drone_wh",
    "co2_azerbaijan_g", "co2_eu_g",
    "time_to_first_target_s", "time_to_all_targets_s",
    "targets_found",
    "wh_per_target_found", "wh_per_pct_coverage", "wh_per_minute_mission",
    "hover_share_pct", "forward_share_pct", "comm_share_pct",
    "per_drone_dist_cv",
]


def summarize_by_cell(rows: list[dict]) -> list[dict]:
    """Group by (method, n_drones), compute mean / std / 95% CI half-width."""
    cells: dict[tuple[str, int], list[dict]] = {}
    for r in rows:
        n = r.get("n_drones")
        if n is None:
            continue
        cells.setdefault((r["method"], int(n)), []).append(r)

    out = []
    for (method, n), trials in sorted(cells.items(), key=lambda kv: (kv[0][0], kv[0][1])):
        entry = {
            "method": method,
            "n_drones": n,
            "n_trials": len(trials),
        }
        n_trials = len(trials)
        tcrit = _t_crit(n_trials) if n_trials >= 2 else None
        for field in SUMMARY_FIELDS:
            vals = [t[field] for t in trials if t.get(field) is not None]
            if not vals:
                entry[f"{field}_mean"] = None
                entry[f"{field}_std"] = None
                entry[f"{field}_ci95"] = None
                continue
            m_ = mean(vals)
            entry[f"{field}_mean"] = round(m_, 4)
            if len(vals) >= 2 and tcrit is not None:
                s = stdev(vals)
                ci = tcrit * s / math.sqrt(len(vals))
                entry[f"{field}_std"] = round(s, 4)
                entry[f"{field}_ci95"] = round(ci, 4)
            else:
                entry[f"{field}_std"] = 0.0
                entry[f"{field}_ci95"] = 0.0
        out.append(entry)
    return out


def marginal_between_cells(summary: list[dict], baseline_n: int = 1) -> list[dict]:
    """For each method, compute deltas between consecutive N cells.

    speedup(N)             = T(baseline) / T(N)
    parallel_efficiency(N) = speedup(N) / N
    energy_per_drone_added = (E(N) - E(prev)) / (N - prev)
    time_saved_per_drone   = (T(prev) - T(N)) / (N - prev)
    """
    rows = []
    by_method: dict[str, list[dict]] = {}
    for s in summary:
        by_method.setdefault(s["method"], []).append(s)
    for method, cells in by_method.items():
        cells = sorted(cells, key=lambda c: c["n_drones"])
        # Find baseline cell for this method (same method at baseline_n).
        baseline = next((c for c in cells if c["n_drones"] == baseline_n), None)
        T_base = baseline.get("time_to_all_targets_s_mean") if baseline else None
        for i, c in enumerate(cells):
            n = c["n_drones"]
            T_n = c.get("time_to_all_targets_s_mean")
            E_n = c.get("total_energy_wh_mean")
            entry = {
                "method": method,
                "n_drones": n,
                "n_trials": c["n_trials"],
                "total_energy_wh_mean": E_n,
                "time_to_all_targets_s_mean": T_n,
            }
            # speedup vs baseline
            if T_base and T_n and T_n > 0:
                sp = T_base / T_n
                entry["speedup_vs_baseline"] = round(sp, 4)
                entry["parallel_efficiency"] = round(sp / n, 4)
            else:
                entry["speedup_vs_baseline"] = None
                entry["parallel_efficiency"] = None
            # marginal vs previous cell of this method
            if i > 0:
                prev = cells[i - 1]
                dn = n - prev["n_drones"]
                if dn > 0:
                    pE = prev.get("total_energy_wh_mean")
                    pT = prev.get("time_to_all_targets_s_mean")
                    entry["delta_energy_per_drone_wh"] = (
                        round((E_n - pE) / dn, 4) if E_n is not None and pE is not None else None
                    )
                    entry["delta_time_per_drone_s"] = (
                        round((pT - T_n) / dn, 4) if T_n is not None and pT is not None else None
                    )
            else:
                entry["delta_energy_per_drone_wh"] = None
                entry["delta_time_per_drone_s"] = None
            rows.append(entry)
    return rows


# ── CSV writers ─────────────────────────────────────────────────────────────

def write_trial_index(rows: list[dict], path: str):
    if not rows:
        return
    fieldnames = [
        "experiment", "method", "trial", "trial_dir",
        *PRIMITIVE_FIELDS,
        "wh_per_target_found", "wh_per_pct_coverage", "wh_per_minute_mission",
        "hover_share_pct", "forward_share_pct", "comm_share_pct",
        "per_drone_dist_cv",
    ]
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fieldnames)
        w.writeheader()
        for r in rows:
            w.writerow({k: r.get(k) for k in fieldnames})


def write_summary(summary: list[dict], path: str):
    if not summary:
        return
    fieldnames = ["method", "n_drones", "n_trials"]
    for f_ in SUMMARY_FIELDS:
        fieldnames += [f"{f_}_mean", f"{f_}_std", f"{f_}_ci95"]
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fieldnames)
        w.writeheader()
        for r in summary:
            w.writerow({k: r.get(k) for k in fieldnames})


def write_marginal(rows: list[dict], path: str):
    if not rows:
        return
    fieldnames = [
        "method", "n_drones", "n_trials",
        "total_energy_wh_mean", "time_to_all_targets_s_mean",
        "speedup_vs_baseline", "parallel_efficiency",
        "delta_energy_per_drone_wh", "delta_time_per_drone_s",
    ]
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fieldnames)
        w.writeheader()
        for r in rows:
            w.writerow({k: r.get(k) for k in fieldnames})


# ── Plots ───────────────────────────────────────────────────────────────────

def make_plots(trials: list[dict], summary: list[dict], marginal: list[dict],
               out_dir: str, primary_method: str = "lawnmower"):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib not available; skipping plots", file=sys.stderr)
        return

    os.makedirs(out_dir, exist_ok=True)

    # filter to primary method for the headline figures
    p_trials = [t for t in trials if t["method"] == primary_method]
    p_summary = sorted(
        [s for s in summary if s["method"] == primary_method],
        key=lambda s: s["n_drones"],
    )
    Ns = [s["n_drones"] for s in p_summary]

    # ── F1: Pareto scatter (energy vs time-to-all-targets) ────────────────
    fig, ax = plt.subplots(figsize=(7, 5))
    cmap = plt.get_cmap("viridis")
    if Ns:
        norm = plt.Normalize(vmin=min(Ns), vmax=max(Ns))
        for n in Ns:
            xs = [t["time_to_all_targets_s"] for t in p_trials
                  if t["n_drones"] == n and t["time_to_all_targets_s"] is not None]
            ys = [t["total_energy_wh"] for t in p_trials
                  if t["n_drones"] == n and t["time_to_all_targets_s"] is not None]
            ax.scatter(xs, ys, s=70, alpha=0.85, color=cmap(norm(n)),
                       edgecolor="black", linewidth=0.5, label=f"N={n}")
        # frontier through cell means
        xs_m = [s["time_to_all_targets_s_mean"] for s in p_summary
                if s.get("time_to_all_targets_s_mean") is not None]
        ys_m = [s["total_energy_wh_mean"] for s in p_summary
                if s.get("time_to_all_targets_s_mean") is not None]
        if len(xs_m) >= 2:
            paired = sorted(zip(xs_m, ys_m), key=lambda p: p[0])
            ax.plot([p[0] for p in paired], [p[1] for p in paired],
                    "k--", linewidth=1, alpha=0.6, label="frontier (cell means)")
    ax.set_xlabel("Mission time to all targets found (s)")
    ax.set_ylabel("Total mission energy (Wh)")
    ax.set_title(f"Energy–Time Pareto ({primary_method})")
    ax.legend()
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "F1_pareto.png"), dpi=200)
    plt.close(fig)

    # ── F2: stacked decomposition vs N ────────────────────────────────────
    fig, ax = plt.subplots(figsize=(7, 5))
    if p_summary:
        hover = [s.get("hover_energy_wh_mean") or 0 for s in p_summary]
        forward = [s.get("forward_energy_wh_mean") or 0 for s in p_summary]
        comm = [s.get("comm_energy_wh_mean") or 0 for s in p_summary]
        x = list(range(len(Ns)))
        ax.bar(x, hover, label="hover", color="#4c72b0")
        ax.bar(x, forward, bottom=hover, label="forward flight", color="#dd8452")
        ax.bar(x, comm, bottom=[h + f for h, f in zip(hover, forward)],
               label="comm", color="#55a467")
        ax.set_xticks(x)
        ax.set_xticklabels([f"N={n}" for n in Ns])
        ax.set_ylabel("Energy (Wh)")
        ax.set_title(f"Energy decomposition vs swarm size ({primary_method})")
        ax.legend()
        ax.grid(True, alpha=0.3, axis="y")
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "F2_decomposition.png"), dpi=200)
    plt.close(fig)

    # ── F3: per-target energy (the fair, duration-normalised metric) ──────
    fig, ax = plt.subplots(figsize=(7, 5))
    if p_summary:
        means = [s.get("wh_per_target_found_mean") for s in p_summary]
        cis = [s.get("wh_per_target_found_ci95") or 0 for s in p_summary]
        ax.errorbar(Ns, means, yerr=cis, fmt="o-", capsize=4, linewidth=1.5,
                    color="#4c72b0", ecolor="black", markersize=8)
        ax.set_xticks(Ns)
        ax.set_xlabel("Swarm size N")
        ax.set_ylabel("Energy per target found (Wh / target)")
        ax.set_title(f"Per-target energy efficiency vs swarm size ({primary_method})")
        ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "F3_per_target_energy.png"), dpi=200)
    plt.close(fig)

    # ── F4: CO2 on the EU-average grid ────────────────────────────────────
    fig, ax = plt.subplots(figsize=(7, 5))
    if p_summary:
        eu = [s.get("co2_eu_g_mean") or 0 for s in p_summary]
        eu_ci = [s.get("co2_eu_g_ci95") or 0 for s in p_summary]
        x = list(range(len(Ns)))
        ax.bar(x, eu, 0.6, yerr=eu_ci, capsize=3,
               label="EU avg grid (250 g/kWh)", color="#8172b3")
        ax.set_xticks(x)
        ax.set_xticklabels([f"N={n}" for n in Ns])
        ax.set_ylabel("CO₂ per mission (g)")
        ax.set_title(f"Mission CO₂ vs swarm size ({primary_method})")
        ax.legend()
        ax.grid(True, alpha=0.3, axis="y")
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "F4_co2_emissions.png"), dpi=200)
    plt.close(fig)

    # ── F5: speedup and parallel efficiency ───────────────────────────────
    fig, ax1 = plt.subplots(figsize=(7, 5))
    p_marg = sorted(
        [m for m in marginal if m["method"] == primary_method],
        key=lambda m: m["n_drones"],
    )
    if p_marg:
        Ns_m = [m["n_drones"] for m in p_marg]
        sp = [m.get("speedup_vs_baseline") for m in p_marg]
        eff = [m.get("parallel_efficiency") for m in p_marg]
        ax1.plot(Ns_m, sp, "o-", color="#4c72b0", label="speedup", markersize=8)
        ax1.plot(Ns_m, Ns_m, "k:", alpha=0.5, label="ideal linear speedup")
        ax1.set_xlabel("Swarm size N")
        ax1.set_ylabel("Speedup vs N=1", color="#4c72b0")
        ax1.tick_params(axis="y", labelcolor="#4c72b0")
        ax1.set_xticks(Ns_m)
        ax1.grid(True, alpha=0.3)
        ax2 = ax1.twinx()
        ax2.plot(Ns_m, eff, "s--", color="#dd8452",
                 label="parallel efficiency", markersize=8)
        ax2.set_ylabel("Parallel efficiency (speedup / N)", color="#dd8452")
        ax2.tick_params(axis="y", labelcolor="#dd8452")
        ax2.set_ylim(0, 1.1)
        # combine legends
        lines1, labels1 = ax1.get_legend_handles_labels()
        lines2, labels2 = ax2.get_legend_handles_labels()
        ax1.legend(lines1 + lines2, labels1 + labels2, loc="upper right")
        ax1.set_title(f"Swarm-size speedup and efficiency ({primary_method})")
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "F5_speedup_efficiency.png"), dpi=200)
    plt.close(fig)


# ── CLI ─────────────────────────────────────────────────────────────────────

def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--logs-dir", default="data/logs/istras",
                   help="Root of the experiment log tree.")
    p.add_argument("--output-dir", default="data/results/energy_scaling",
                   help="Where CSVs and figures are written.")
    p.add_argument("--include-dirs", nargs="+", default=DEFAULT_INCLUDE_DIRS,
                   help="Subdirectories of --logs-dir to include "
                        "(default: %(default)s — geometry-consistent cells).")
    p.add_argument("--methods", nargs="+", default=["lawnmower"],
                   help="Methods to include (default: lawnmower only).")
    p.add_argument("--baseline-n", type=int, default=1,
                   help="N to use as the baseline for speedup (default: 1).")
    p.add_argument("--primary-method", default="lawnmower",
                   help="Method whose cells drive the headline figures.")
    p.add_argument("--plots", action="store_true",
                   help="Generate F1–F5 PNG figures.")
    args = p.parse_args()

    rows = collect_trials(args.logs_dir, args.include_dirs, args.methods)
    if not rows:
        print(f"No trials found under {args.logs_dir} in {args.include_dirs} "
              f"matching methods {args.methods}.", file=sys.stderr)
        return 1
    summary = summarize_by_cell(rows)
    marginal = marginal_between_cells(summary, baseline_n=args.baseline_n)

    write_trial_index(rows, os.path.join(args.output_dir, "trial_index.csv"))
    write_summary(summary, os.path.join(args.output_dir, "scaling_summary.csv"))
    write_marginal(marginal, os.path.join(args.output_dir, "marginal_analysis.csv"))

    print(f"Loaded {len(rows)} trials across {len(summary)} cells.")
    for s in summary:
        print(f"  {s['method']:>10s}  N={s['n_drones']}  trials={s['n_trials']}  "
              f"E={s.get('total_energy_wh_mean')}±{s.get('total_energy_wh_ci95')} Wh  "
              f"T={s.get('time_to_all_targets_s_mean')}±"
              f"{s.get('time_to_all_targets_s_ci95')} s  "
              f"Wh/target={s.get('wh_per_target_found_mean')}")
    print(f"Wrote: {args.output_dir}/{{trial_index,scaling_summary,marginal_analysis}}.csv")

    if args.plots:
        make_plots(rows, summary, marginal,
                   os.path.join(args.output_dir, "figures"),
                   primary_method=args.primary_method)
        print(f"Wrote 5 figures to {args.output_dir}/figures/")
    return 0


if __name__ == "__main__":
    sys.exit(main())
