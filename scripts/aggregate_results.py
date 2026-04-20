#!/usr/bin/env python3
"""
Day-4 aggregation: roll up per-trial metrics_summary.json files into the
table + bar charts the ISTRAS paper needs.

Walks a logs root (default `data/logs/istras/`), groups trials by method
(the directory-name prefix up to `_d<N>`), and produces:

  figures/fig_distance.pdf   — flight distance per method (mean ± std)
  figures/fig_energy.pdf     — mission energy per method (mean ± std)
  figures/fig_co2.pdf        — CO2 emissions (Azerbaijan grid) per method
  figures/fig_ttf_target.pdf — time to first target found (mean ± std)
  figures/fig_tta_target.pdf — time to all targets found (mean ± std)
  figures/table1_summary.tex — LaTeX snippet for paper Table 1
  figures/aggregate_summary.csv — machine-readable mean/std per metric

Usage:
  python3 aggregate_results.py
  python3 aggregate_results.py --log-base data/logs/istras --out figures
"""

import argparse
import csv
import json
import os
import re
import statistics
from collections import defaultdict

import matplotlib.pyplot as plt
import numpy as np


# Method-order for figures/tables. Anything not listed is appended alphabetically.
METHOD_ORDER = ["static", "lawnmower", "adaptive"]
METHOD_LABELS = {
    "static": "Static grid",
    "lawnmower": "Lawnmower",
    "adaptive": "Adaptive (ours)",
    "random": "Random",
    "binary_voronoi": "Binary Voronoi",
    "all_converge": "All-converge",
    "pso": "PSO",
    "apf": "APF",
}
METHOD_COLORS = {
    "static": "#999999",
    "lawnmower": "#377eb8",
    "adaptive": "#e41a1c",
    "random": "#984ea3",
    "binary_voronoi": "#ff7f00",
    "all_converge": "#4daf4a",
    "pso": "#a65628",
    "apf": "#f781bf",
}

# Metrics pulled from each trial. Missing/None → NaN so numpy can ignore them.
METRICS = [
    ("total_distance_m", "Flight distance (m)"),
    ("total_energy_wh", "Mission energy (Wh)"),
    ("energy_per_drone_wh", "Energy per drone (Wh)"),
    ("co2_azerbaijan_g", "CO$_2$ — Azerbaijan grid (g)"),
    ("co2_eu_g", "CO$_2$ — EU avg grid (g)"),
    ("coverage_percent", "Coverage (%)"),
    ("time_to_first_target_s", "Time to first target (s)"),
    ("time_to_all_targets_s", "Time to all targets (s)"),
    ("targets_found", "Targets found"),
    ("duration_s", "Trial duration (s)"),
]


def infer_method(experiment_id: str) -> str:
    """Extract method name from an experiment id like 'adaptive_d3_t3_trial01'."""
    m = re.match(r"([a-zA-Z_]+?)_d\d+_t\d+_trial\d+", experiment_id)
    if m:
        return m.group(1).rstrip("_")
    # Fallback: take everything before the first digit.
    return re.split(r"_?\d", experiment_id, maxsplit=1)[0] or experiment_id


def ordered_methods(methods: list[str]) -> list[str]:
    known = [m for m in METHOD_ORDER if m in methods]
    extra = sorted(m for m in methods if m not in METHOD_ORDER)
    return known + extra


def to_float(value) -> float:
    if value is None:
        return float("nan")
    try:
        return float(value)
    except (TypeError, ValueError):
        return float("nan")


def collect_summaries(log_base: str) -> dict[str, list[dict]]:
    """Walk `log_base/*/metrics_summary.json` and group by method."""
    grouped: dict[str, list[dict]] = defaultdict(list)
    if not os.path.isdir(log_base):
        raise SystemExit(f"ERROR: log base not found: {log_base}")

    for name in sorted(os.listdir(log_base)):
        trial_dir = os.path.join(log_base, name)
        summary_path = os.path.join(trial_dir, "metrics_summary.json")
        if not os.path.isfile(summary_path):
            continue
        with open(summary_path) as f:
            summary = json.load(f)
        method = infer_method(summary.get("experiment_id", name))
        summary["_method"] = method
        summary["_trial_dir"] = trial_dir
        grouped[method].append(summary)
    return grouped


def aggregate_metric(trials: list[dict], key: str) -> tuple[float, float, int]:
    """mean, std, n_valid for a metric across trials (skips NaNs)."""
    values = [to_float(t.get(key)) for t in trials]
    valid = [v for v in values if not np.isnan(v)]
    if not valid:
        return (float("nan"), float("nan"), 0)
    mean = statistics.mean(valid)
    std = statistics.stdev(valid) if len(valid) > 1 else 0.0
    return (mean, std, len(valid))


def bar_chart(methods: list[str], stats: dict[str, tuple[float, float, int]],
              title: str, ylabel: str, out_path: str,
              baseline_method: str | None = None,
              annotate_delta: bool = True) -> None:
    """Render a mean±std bar chart with optional delta-vs-baseline annotations."""
    fig, ax = plt.subplots(figsize=(5.2, 3.6))
    xs = np.arange(len(methods))
    means = np.array([stats[m][0] for m in methods])
    stds = np.array([stats[m][1] for m in methods])
    colors = [METHOD_COLORS.get(m, "#666666") for m in methods]
    labels = [METHOD_LABELS.get(m, m) for m in methods]

    ax.bar(xs, means, yerr=stds, capsize=4, color=colors,
           edgecolor="black", linewidth=0.6)
    ax.set_xticks(xs)
    ax.set_xticklabels(labels, rotation=0)
    ax.set_ylabel(ylabel)
    ax.set_title(title)
    ax.grid(axis="y", alpha=0.3, linestyle="--")

    baseline_mean = None
    if baseline_method and baseline_method in stats:
        baseline_mean = stats[baseline_method][0]

    for i, (m, mean, std) in enumerate(zip(methods, means, stds)):
        if np.isnan(mean):
            continue
        label = f"{mean:.1f}"
        if annotate_delta and baseline_mean and not np.isnan(baseline_mean) \
                and baseline_mean != 0 and m != baseline_method:
            delta = 100.0 * (mean - baseline_mean) / baseline_mean
            label += f"\n({delta:+.0f}% vs {baseline_method})"
        y = mean + std + 0.02 * max(means[~np.isnan(means)], default=1)
        ax.text(i, y, label, ha="center", va="bottom", fontsize=8)

    fig.tight_layout()
    fig.savefig(out_path, bbox_inches="tight")
    fig.savefig(out_path.replace(".pdf", ".png"), dpi=200, bbox_inches="tight")
    plt.close(fig)
    print(f"  wrote {out_path}")


def render_latex_table(methods: list[str], agg: dict[str, dict],
                       out_path: str) -> None:
    """Write the all-metrics summary as a standalone LaTeX table snippet."""
    rows = [
        ("Distance (m)", "total_distance_m", "{:.1f}"),
        ("Energy (Wh)", "total_energy_wh", "{:.2f}"),
        ("CO$_2$ AZ (g)", "co2_azerbaijan_g", "{:.2f}"),
        ("CO$_2$ EU (g)", "co2_eu_g", "{:.2f}"),
        ("Coverage (\\%)", "coverage_percent", "{:.1f}"),
        ("T$_{1st}$ target (s)", "time_to_first_target_s", "{:.2f}"),
        ("T$_{all}$ targets (s)", "time_to_all_targets_s", "{:.2f}"),
    ]

    def cell(method: str, key: str, fmt: str) -> str:
        mean, std, n = agg[method][key]
        if n == 0 or np.isnan(mean):
            return "--"
        if n == 1:
            return fmt.format(mean)
        return f"{fmt.format(mean)} $\\pm$ {fmt.format(std)}"

    lines = []
    lines.append("% Auto-generated by scripts/aggregate_results.py — do not edit by hand.")
    lines.append("\\begin{table}[t]")
    lines.append("\\centering")
    lines.append("\\caption{Per-method mean $\\pm$ std across trials. Energy uses "
                 "$P_\\text{hover}=150$\\,W per drone; CO$_2$ factors: Azerbaijan "
                 "440\\,g/kWh, EU avg 250\\,g/kWh.}")
    lines.append("\\label{tab:summary}")
    lines.append("\\small")
    col_spec = "l" + "c" * len(methods)
    lines.append(f"\\begin{{tabular}}{{{col_spec}}}")
    lines.append("\\hline")
    header = ["Metric"] + [METHOD_LABELS.get(m, m) for m in methods]
    lines.append(" & ".join(header) + " \\\\")
    lines.append("\\hline")
    for label, key, fmt in rows:
        cells = [label] + [cell(m, key, fmt) for m in methods]
        lines.append(" & ".join(cells) + " \\\\")
    lines.append("\\hline")
    lines.append("\\end{tabular}")
    lines.append("\\end{table}")

    with open(out_path, "w") as f:
        f.write("\n".join(lines) + "\n")
    print(f"  wrote {out_path}")


def render_csv(methods: list[str], agg: dict[str, dict],
               out_path: str) -> None:
    with open(out_path, "w", newline="") as f:
        writer = csv.writer(f)
        header = ["method", "n_trials"]
        for key, _ in METRICS:
            header += [f"{key}_mean", f"{key}_std"]
        writer.writerow(header)
        for m in methods:
            n = max(agg[m][key][2] for key, _ in METRICS)
            row = [m, n]
            for key, _ in METRICS:
                mean, std, _ = agg[m][key]
                row += [
                    "" if np.isnan(mean) else f"{mean:.4f}",
                    "" if np.isnan(std) else f"{std:.4f}",
                ]
            writer.writerow(row)
    print(f"  wrote {out_path}")


def main():
    parser = argparse.ArgumentParser(
        description="Aggregate per-trial metrics into paper figures and tables"
    )
    parser.add_argument(
        "--log-base", default="data/logs/istras",
        help="Directory containing trial folders (default: data/logs/istras)",
    )
    parser.add_argument(
        "--out", default="figures",
        help="Output directory for figures/tables (default: figures)",
    )
    parser.add_argument(
        "--baseline", default="lawnmower",
        help="Method used for %% delta annotations (default: lawnmower)",
    )
    args = parser.parse_args()

    log_base = os.path.abspath(args.log_base)
    out_dir = os.path.abspath(args.out)
    os.makedirs(out_dir, exist_ok=True)

    grouped = collect_summaries(log_base)
    if not grouped:
        raise SystemExit(f"No metrics_summary.json files under {log_base}")

    methods = ordered_methods(list(grouped.keys()))
    print(f"Methods: {methods}")
    for m in methods:
        print(f"  {m}: {len(grouped[m])} trial(s)")

    # Aggregate every metric per method.
    agg: dict[str, dict] = {m: {} for m in methods}
    for m in methods:
        for key, _ in METRICS:
            agg[m][key] = aggregate_metric(grouped[m], key)

    # Figures.
    fig_specs = [
        ("total_distance_m", "Flight distance",
         "Total flight distance (m)", "fig_distance.pdf"),
        ("total_energy_wh", "Mission energy",
         "Total mission energy (Wh)", "fig_energy.pdf"),
        ("co2_azerbaijan_g", "CO$_2$ emissions (Azerbaijan grid)",
         "CO$_2$ (g)", "fig_co2.pdf"),
        ("time_to_first_target_s", "Time to first target",
         "Seconds from trial start", "fig_ttf_target.pdf"),
        ("time_to_all_targets_s", "Time to all targets",
         "Seconds from trial start", "fig_tta_target.pdf"),
    ]
    for key, title, ylabel, fname in fig_specs:
        stats = {m: agg[m][key] for m in methods}
        bar_chart(methods, stats, title, ylabel,
                  os.path.join(out_dir, fname),
                  baseline_method=args.baseline)

    render_latex_table(methods, agg, os.path.join(out_dir, "table1_summary.tex"))
    render_csv(methods, agg, os.path.join(out_dir, "aggregate_summary.csv"))

    # Console snapshot.
    print("\n=== Aggregate summary ===")
    for m in methods:
        n_trials = len(grouped[m])
        d = agg[m]["total_distance_m"]
        e = agg[m]["total_energy_wh"]
        ttf = agg[m]["time_to_first_target_s"]
        tta = agg[m]["time_to_all_targets_s"]
        print(f"  {m:14s} n={n_trials}  "
              f"dist={d[0]:7.1f}±{d[1]:6.1f}m  "
              f"energy={e[0]:6.2f}±{e[1]:5.2f}Wh  "
              f"T_first={ttf[0]:6.2f}s  T_all={tta[0]:6.2f}s")


if __name__ == "__main__":
    main()
