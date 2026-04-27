#!/usr/bin/env python3
"""
Target-separation analysis: does adaptive coverage's advantage depend on
how close the targets are to each other?

Motivation
----------
Adaptive coverage concentrates drones where the density function has mass.
A single Gaussian bump of sigma σ has most of its mass inside ~2σ. So when
two targets are more than ~2σ apart, adaptive sees two independent bumps
and behaves like the lawnmower baseline on *coverage* — no meaningful
reallocation happens. Adaptive is expected to win only on event-response
latency in the far-separation regime, not on travelled distance.

This script groups every trial in a log tree by the minimum pairwise
distance between its targets (read from `metrics_summary.json`, or
computed on the fly from `targets.csv` for older runs), and regresses the
adaptive−lawnmower difference on key metrics against that distance.

Outputs
-------
1. `<outdir>/trial_separation_index.csv`  — one row per trial with
   experiment/method/trial/seed, n_targets, min/mean pairwise distance,
   and the metrics we care about (event_phase_distance_m, coverage,
   mean/median/max per-event latency, targets_found).

2. `<outdir>/per_method_by_separation_bin.csv` — grouped by (experiment,
   method, separation_bin) with mean ± stdev of each metric.

3. `<outdir>/adaptive_vs_lawnmower_by_separation.csv` — for each
   experiment and separation bin, the paired delta between adaptive and
   lawnmower on each metric.

4. Plots (if --plots): scatter of metric vs separation, coloured by
   method. One PNG per metric.

Usage
-----
    python3 scripts/analysis/target_separation_analysis.py \\
        --logs-dir data/logs/istras \\
        --output-dir data/results/separation_analysis

    # With plots:
    python3 scripts/analysis/target_separation_analysis.py \\
        --logs-dir data/logs/istras \\
        --output-dir data/results/separation_analysis \\
        --plots

    # Custom bin edges (metres):
    python3 scripts/analysis/target_separation_analysis.py \\
        --logs-dir data/logs/istras \\
        --output-dir data/results/separation_analysis \\
        --bin-edges 0 30 60 120 9999
"""

import argparse
import csv
import json
import math
import os
import re
import sys
from statistics import mean, stdev


# Metrics we care about for the separation analysis.
METRIC_FIELDS = [
    "event_phase_distance_m",
    "total_distance_m",
    "coverage_percent",
    "mean_event_latency_s",
    "median_event_latency_s",
    "max_event_latency_s",
    "targets_found",
    "targets_total",
    "total_energy_wh",
    "co2_azerbaijan_g",
]

# Regex to pull (method, n_drones, n_targets, trial) from folder names like
# "adaptive_d3_t8_trial01".
_TRIAL_RE = re.compile(r"^(?P<method>[a-z_]+)_d(?P<drones>\d+)_t(?P<targets>\d+)_trial(?P<trial>\d+)$")


def _safe_float(x):
    try:
        return float(x)
    except (TypeError, ValueError):
        return None


def load_targets_csv(path: str) -> list[tuple[float, float]]:
    """Read target x,y coordinates from a trial's targets.csv."""
    if not os.path.exists(path):
        return []
    pts = []
    with open(path, "r") as f:
        for row in csv.DictReader(f):
            try:
                pts.append((float(row["x"]), float(row["y"])))
            except (KeyError, ValueError):
                continue
    return pts


def min_pairwise_distance(points: list[tuple[float, float]]) -> float | None:
    if len(points) < 2:
        return None
    best = float("inf")
    for i in range(len(points)):
        for j in range(i + 1, len(points)):
            d = math.hypot(points[i][0] - points[j][0],
                           points[i][1] - points[j][1])
            if d < best:
                best = d
    return best


def collect_trials(logs_dir: str) -> list[dict]:
    """Walk a log tree and return one row per trial with the metrics we care about."""
    rows = []
    for exp_entry in sorted(os.listdir(logs_dir)):
        exp_dir = os.path.join(logs_dir, exp_entry)
        if not os.path.isdir(exp_dir):
            continue

        for trial_name in sorted(os.listdir(exp_dir)):
            trial_dir = os.path.join(exp_dir, trial_name)
            if not os.path.isdir(trial_dir):
                continue

            summary_path = os.path.join(trial_dir, "metrics_summary.json")
            if not os.path.exists(summary_path):
                continue

            m = _TRIAL_RE.match(trial_name)
            if not m:
                continue

            with open(summary_path, "r") as f:
                summary = json.load(f)

            # Prefer the pre-computed separation if it's in the summary;
            # fall back to computing from targets.csv (handles older runs
            # that were metric-computed before the separation patch landed).
            min_sep = summary.get("min_pairwise_target_distance_m")
            mean_sep = summary.get("mean_pairwise_target_distance_m")
            if min_sep is None:
                pts = load_targets_csv(os.path.join(trial_dir, "targets.csv"))
                min_sep = min_pairwise_distance(pts)
                if pts and len(pts) >= 2:
                    dists = []
                    for i in range(len(pts)):
                        for j in range(i + 1, len(pts)):
                            dists.append(math.hypot(pts[i][0] - pts[j][0],
                                                    pts[i][1] - pts[j][1]))
                    mean_sep = sum(dists) / len(dists)

            row = {
                "experiment": exp_entry,
                "method": m.group("method"),
                "drones": int(m.group("drones")),
                "targets_in_name": int(m.group("targets")),
                "trial": int(m.group("trial")),
                "trial_dir": trial_name,
                "min_pairwise_target_distance_m": min_sep,
                "mean_pairwise_target_distance_m": mean_sep,
            }
            for field in METRIC_FIELDS:
                row[field] = _safe_float(summary.get(field))
            rows.append(row)

    return rows


def bin_label(value: float | None, edges: list[float]) -> str:
    """Bucket a value into bin labels derived from `edges`."""
    if value is None:
        return "unknown"
    for i in range(len(edges) - 1):
        if edges[i] <= value < edges[i + 1]:
            lo = int(edges[i])
            hi = edges[i + 1]
            hi_label = "∞" if hi >= 9999 else f"{int(hi)}"
            return f"{lo}-{hi_label}m"
    return f"≥{int(edges[-1])}m"


def write_index_csv(rows: list[dict], path: str):
    """Flat per-trial CSV — one row per trial, one column per metric."""
    if not rows:
        return
    fieldnames = [
        "experiment", "method", "drones", "trial", "trial_dir",
        "targets_in_name",
        "min_pairwise_target_distance_m", "mean_pairwise_target_distance_m",
        *METRIC_FIELDS,
    ]
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fieldnames)
        w.writeheader()
        for r in rows:
            w.writerow({k: r.get(k) for k in fieldnames})


def summarize_by_bin(rows: list[dict], edges: list[float]) -> list[dict]:
    """Mean ± stdev per (experiment, method, separation_bin) cell."""
    bucket: dict[tuple[str, str, str], list[dict]] = {}
    for r in rows:
        key = (r["experiment"], r["method"],
               bin_label(r["min_pairwise_target_distance_m"], edges))
        bucket.setdefault(key, []).append(r)

    out = []
    for (exp, method, bin_l), trials in sorted(bucket.items()):
        entry = {
            "experiment": exp,
            "method": method,
            "separation_bin": bin_l,
            "n_trials": len(trials),
        }
        for field in METRIC_FIELDS:
            vals = [r[field] for r in trials if r[field] is not None]
            if vals:
                entry[f"{field}_mean"] = round(mean(vals), 4)
                entry[f"{field}_std"] = round(stdev(vals), 4) if len(vals) > 1 else 0.0
            else:
                entry[f"{field}_mean"] = None
                entry[f"{field}_std"] = None
        out.append(entry)
    return out


def write_bin_summary_csv(summary: list[dict], path: str):
    if not summary:
        return
    fieldnames = ["experiment", "method", "separation_bin", "n_trials"]
    for field in METRIC_FIELDS:
        fieldnames += [f"{field}_mean", f"{field}_std"]
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fieldnames)
        w.writeheader()
        for row in summary:
            w.writerow({k: row.get(k) for k in fieldnames})


def paired_delta(summary: list[dict], path: str):
    """For each (experiment, bin) where both adaptive and lawnmower exist,
    compute adaptive_mean − lawnmower_mean. Positive = adaptive pays more
    (bad for distance/energy/latency; good for coverage/targets_found)."""
    by_key: dict[tuple[str, str], dict[str, dict]] = {}
    for row in summary:
        key = (row["experiment"], row["separation_bin"])
        by_key.setdefault(key, {})[row["method"]] = row

    out = []
    for (exp, bin_l), methods in sorted(by_key.items()):
        if "adaptive" not in methods or "lawnmower" not in methods:
            continue
        a = methods["adaptive"]
        l = methods["lawnmower"]
        entry = {
            "experiment": exp,
            "separation_bin": bin_l,
            "n_trials_adaptive": a["n_trials"],
            "n_trials_lawnmower": l["n_trials"],
        }
        for field in METRIC_FIELDS:
            am = a.get(f"{field}_mean")
            lm = l.get(f"{field}_mean")
            if am is not None and lm is not None:
                entry[f"delta_{field}"] = round(am - lm, 4)
            else:
                entry[f"delta_{field}"] = None
        out.append(entry)

    if not out:
        return
    fieldnames = ["experiment", "separation_bin", "n_trials_adaptive",
                  "n_trials_lawnmower"]
    for field in METRIC_FIELDS:
        fieldnames.append(f"delta_{field}")
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fieldnames)
        w.writeheader()
        w.writerows(out)


def draw_plots(rows: list[dict], outdir: str):
    """Scatter plots of metric vs min-pairwise-separation, coloured by method.

    Optional — requires matplotlib. If matplotlib is missing we just skip
    plotting and still emit the CSVs.
    """
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("  [plots skipped] matplotlib not available")
        return

    plots_dir = os.path.join(outdir, "plots")
    os.makedirs(plots_dir, exist_ok=True)

    methods = sorted({r["method"] for r in rows})
    color_cycle = {m: c for m, c in zip(methods, ["#1f77b4", "#ff7f0e",
                                                  "#2ca02c", "#d62728",
                                                  "#9467bd"])}

    for field in METRIC_FIELDS:
        fig, ax = plt.subplots(figsize=(7, 5))
        drew_anything = False
        for m in methods:
            xs = [r["min_pairwise_target_distance_m"] for r in rows
                  if r["method"] == m
                  and r[field] is not None
                  and r["min_pairwise_target_distance_m"] is not None]
            ys = [r[field] for r in rows
                  if r["method"] == m
                  and r[field] is not None
                  and r["min_pairwise_target_distance_m"] is not None]
            if xs:
                ax.scatter(xs, ys, label=m, alpha=0.6, color=color_cycle.get(m))
                drew_anything = True
        if not drew_anything:
            plt.close(fig)
            continue
        ax.set_xlabel("min pairwise target distance (m)")
        ax.set_ylabel(field)
        ax.set_title(f"{field} vs target separation")
        ax.grid(alpha=0.3)
        ax.legend()
        fig.tight_layout()
        fig.savefig(os.path.join(plots_dir, f"{field}.png"), dpi=150)
        plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--logs-dir", required=True,
                        help="Root of per-experiment trial dirs (e.g. data/logs/istras)")
    parser.add_argument("--output-dir", required=True,
                        help="Where to write index/summary CSVs and plots")
    parser.add_argument("--bin-edges", type=float, nargs="+",
                        default=[0, 30, 60, 120, 9999],
                        help="Separation bin edges in metres (default: 0 30 60 120 9999)")
    parser.add_argument("--plots", action="store_true",
                        help="Emit per-metric scatter plots (requires matplotlib)")
    args = parser.parse_args()

    if not os.path.isdir(args.logs_dir):
        print(f"ERROR: logs dir not found: {args.logs_dir}", file=sys.stderr)
        sys.exit(1)

    print(f"Scanning {args.logs_dir} ...")
    rows = collect_trials(args.logs_dir)
    print(f"  loaded {len(rows)} trials")

    if not rows:
        print("No trials found. Are metrics_summary.json files present?")
        sys.exit(1)

    index_path = os.path.join(args.output_dir, "trial_separation_index.csv")
    bin_path = os.path.join(args.output_dir, "per_method_by_separation_bin.csv")
    delta_path = os.path.join(args.output_dir, "adaptive_vs_lawnmower_by_separation.csv")

    write_index_csv(rows, index_path)
    summary = summarize_by_bin(rows, args.bin_edges)
    write_bin_summary_csv(summary, bin_path)
    paired_delta(summary, delta_path)
    if args.plots:
        draw_plots(rows, args.output_dir)

    print(f"  wrote {index_path}")
    print(f"  wrote {bin_path}")
    print(f"  wrote {delta_path}")
    if args.plots:
        print(f"  wrote plots/ under {args.output_dir}")


if __name__ == "__main__":
    main()
