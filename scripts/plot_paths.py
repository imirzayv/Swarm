#!/usr/bin/env python3
"""
Plot drone flight paths with camera FOV coverage from experiment CSV data.

Generates:
  - Combined image with all drones' paths on one plot
  - Separate images for each drone's path

Usage:
  python3 plot_paths.py --experiment-id adaptive_d3_t5_trial01
  python3 plot_paths.py --experiment-id adaptive_d3_t5_trial01 --area-size 40 --altitude 30
  python3 plot_paths.py --data-dir data/logs/adaptive_d3_t5_trial01
"""

import argparse
import os
import csv
import math
from collections import defaultdict

import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.collections import PatchCollection
import numpy as np

# Camera parameters (must match detection_publisher.py / metrics_compute.py)
CAMERA_HFOV_RAD = 1.74  # ~100 degrees

# Distinct colors per drone
DRONE_COLORS = {
    1: "#e41a1c",  # red
    2: "#377eb8",  # blue
    3: "#4daf4a",  # green
    4: "#ff7f00",  # orange
    5: "#984ea3",  # purple
    6: "#a65628",  # brown
    7: "#f781bf",  # pink
    8: "#999999",  # grey
}


def fov_radius(altitude: float) -> float:
    """Ground FOV radius at given altitude."""
    return altitude * math.tan(CAMERA_HFOV_RAD / 2.0)


def load_positions(csv_path: str) -> dict[int, list[tuple[float, float]]]:
    """Load positions.csv and return {drone_id: [(x, y), ...]}."""
    paths = defaultdict(list)
    with open(csv_path) as f:
        reader = csv.DictReader(f)
        for row in reader:
            did = int(row["drone_id"])
            x = float(row["x_local"])
            y = float(row["y_local"])
            paths[did].append((x, y))
    return dict(paths)


def load_detections(csv_path: str) -> list[tuple[float, float, str, int]]:
    """Load detections.csv and return [(x, y, class_name, drone_id), ...]."""
    dets = []
    if not os.path.exists(csv_path):
        return dets
    with open(csv_path) as f:
        reader = csv.DictReader(f)
        for row in reader:
            x = float(row["world_x"])
            y = float(row["world_y"])
            cls = row["class_name"]
            did = int(row["drone_id"])
            dets.append((x, y, cls, did))
    return dets


def load_targets(targets_file: str) -> list[tuple[float, float, str]]:
    """
    Load ground-truth target positions from a CSV file.
    CSV format: name,x,y  (no header)

    If not provided, falls back to the hardcoded default positions
    matching spawn_targets.sh.
    """
    targets = []
    if targets_file and os.path.exists(targets_file):
        with open(targets_file) as f:
            reader = csv.reader(f)
            for row in reader:
                if len(row) >= 3 and not row[0].startswith("#"):
                    name = row[0].strip()
                    x = float(row[1])
                    y = float(row[2])
                    targets.append((x, y, name))
        return targets
    return get_default_targets()


def get_default_targets() -> list[tuple[float, float, str]]:
    """Default target positions matching spawn_targets.sh."""
    return [
        ( 70.0, -60.0, "suv"),
        (-80.0,  70.0, "hatchback_red"),
        ( 50.0,  80.0, "hatchback_blue"),
        (-60.0, -50.0, "pickup"),
        ( 85.0, -30.0, "prius"),
    ]


def plot_drone_paths(
    paths: dict[int, list[tuple[float, float]]],
    detections: list[tuple[float, float, str, int]],
    targets: list[tuple[float, float, str]],
    area_size: float,
    altitude: float,
    output_path: str,
    drone_ids: list[int] | None = None,
    title: str = "",
    dpi: int = 300,
):
    """
    Plot drone paths on the monitoring area.

    Args:
        paths: {drone_id: [(x, y), ...]}
        detections: [(x, y, class, drone_id), ...]
        targets: [(x, y, name), ...] ground-truth target positions
        area_size: monitoring area side length (meters)
        altitude: flight altitude for FOV calculation
        output_path: where to save the figure
        drone_ids: which drones to include (None = all)
        title: plot title
        dpi: output resolution
    """
    half = area_size / 2.0

    # Compute bounds that include all positions + monitoring area
    all_x, all_y = [], []
    for did in (drone_ids or paths.keys()):
        if did in paths:
            pts = np.array(paths[did])
            all_x.extend(pts[:, 0])
            all_y.extend(pts[:, 1])
    if all_x:
        data_xmin, data_xmax = min(all_x), max(all_x)
        data_ymin, data_ymax = min(all_y), max(all_y)
    else:
        data_xmin, data_xmax = -half, half
        data_ymin, data_ymax = -half, half

    xmin = min(-half, data_xmin) - 3
    xmax = max(half, data_xmax) + 3
    ymin = min(-half, data_ymin) - 3
    ymax = max(half, data_ymax) + 3

    fig, ax = plt.subplots(figsize=(10, 10))
    ax.set_xlim(xmin, xmax)
    ax.set_ylim(ymin, ymax)
    ax.set_aspect("equal")
    ax.set_xlabel("X (m)", fontsize=12)
    ax.set_ylabel("Y (m)", fontsize=12)
    ax.set_title(title or "Drone Flight Paths", fontsize=14)
    ax.grid(True, alpha=0.3)
    ax.set_facecolor("#f5f5f5")

    # Draw monitoring area boundary
    border = plt.Rectangle(
        (-half, -half), area_size, area_size,
        linewidth=2, edgecolor="black", facecolor="white", linestyle="--",
        zorder=0
    )
    ax.add_patch(border)

    if drone_ids is None:
        drone_ids = sorted(paths.keys())

    legend_handles = []

    for did in drone_ids:
        if did not in paths or len(paths[did]) < 2:
            continue

        pts = np.array(paths[did])
        color = DRONE_COLORS.get(did, "#333333")

        # Flight path line
        ax.plot(
            pts[:, 0], pts[:, 1],
            color=color, linewidth=2.0, alpha=0.85, zorder=3
        )

        # Start marker (triangle pointing up)
        ax.plot(
            pts[0, 0], pts[0, 1],
            marker="^", color=color, markersize=14,
            markeredgecolor="black", markeredgewidth=1.2, zorder=5
        )
        # End marker (square)
        ax.plot(
            pts[-1, 0], pts[-1, 1],
            marker="s", color=color, markersize=11,
            markeredgecolor="black", markeredgewidth=1.2, zorder=5
        )

        legend_handles.append(
            plt.Line2D([0], [0], color=color, linewidth=2.0,
                       label=f"Drone {did}")
        )

    # Plot ground-truth target positions
    if targets:
        tgt_x = [t[0] for t in targets]
        tgt_y = [t[1] for t in targets]
        ax.scatter(
            tgt_x, tgt_y,
            marker="X", c="red", edgecolors="black",
            s=200, zorder=7, linewidths=1.0
        )
        for tx, ty, tname in targets:
            ax.annotate(
                tname, (tx, ty),
                textcoords="offset points", xytext=(0, -14),
                fontsize=7, ha="center", color="#333333",
            )
        legend_handles.append(plt.Line2D(
            [0], [0], marker="X", color="red", linestyle="None",
            markeredgecolor="black", markersize=10, label="Targets (truth)"
        ))

    # Plot detections as star markers
    if detections:
        det_x = [d[0] for d in detections if drone_ids is None or d[3] in drone_ids]
        det_y = [d[1] for d in detections if drone_ids is None or d[3] in drone_ids]
        if det_x:
            ax.scatter(
                det_x, det_y,
                marker="*", c="gold", edgecolors="black",
                s=150, zorder=6, linewidths=0.8
            )
            legend_handles.append(plt.Line2D(
                [0], [0], marker="*", color="gold", linestyle="None",
                markeredgecolor="black", markersize=12, label="Detections"
            ))

    # Legend entries for start/end markers
    legend_handles.append(plt.Line2D(
        [0], [0], marker="^", color="gray", linestyle="None",
        markeredgecolor="black", markersize=10, label="Start"
    ))
    legend_handles.append(plt.Line2D(
        [0], [0], marker="s", color="gray", linestyle="None",
        markeredgecolor="black", markersize=8, label="End"
    ))

    ax.legend(handles=legend_handles, loc="upper left",
              bbox_to_anchor=(1.02, 1.0), fontsize=10, framealpha=0.9,
              borderaxespad=0)
    fig.savefig(output_path, dpi=dpi, bbox_inches="tight")
    plt.close(fig)
    print(f"  Saved: {output_path}")


def main():
    parser = argparse.ArgumentParser(description="Plot drone flight paths from experiment data")
    parser.add_argument("--experiment-id", type=str, help="Experiment ID (e.g., adaptive_d3_t5_trial01)")
    parser.add_argument("--data-dir", type=str, help="Direct path to experiment log directory")
    parser.add_argument("--log-base", type=str, default=None, help="Base log directory")
    parser.add_argument("--area-size", type=float, default=40.0, help="Monitoring area side length (m)")
    parser.add_argument("--altitude", type=float, default=30.0, help="Flight altitude for FOV calc (m)")
    parser.add_argument("--targets-file", type=str, default=None,
                        help="CSV with target positions: name,x,y (one per line)")
    parser.add_argument("--dpi", type=int, default=300, help="Output image resolution")
    args = parser.parse_args()

    # Resolve data directory
    project_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    if args.data_dir:
        data_dir = args.data_dir
    elif args.experiment_id:
        base = args.log_base or os.path.join(project_dir, "data", "logs")
        data_dir = os.path.join(base, args.experiment_id)
    else:
        parser.error("Provide --experiment-id or --data-dir")

    positions_csv = os.path.join(data_dir, "positions.csv")
    detections_csv = os.path.join(data_dir, "detections.csv")

    if not os.path.exists(positions_csv):
        print(f"ERROR: {positions_csv} not found")
        return

    # Load data
    paths = load_positions(positions_csv)
    detections = load_detections(detections_csv)
    targets = load_targets(args.targets_file)
    drone_ids = sorted(paths.keys())

    if not paths:
        print("No position data found")
        return

    exp_name = os.path.basename(data_dir)
    fov_r = fov_radius(args.altitude)
    print(f"Plotting paths for: {exp_name}")
    print(f"  Drones: {drone_ids}")
    print(f"  Area: {args.area_size}m x {args.area_size}m")
    print(f"  FOV radius: {fov_r:.1f}m (at {args.altitude}m altitude)")
    print(f"  Targets: {len(targets)}")
    print(f"  Detections: {len(detections)}")

    # Combined plot
    combined_path = os.path.join(data_dir, "paths_combined.png")
    plot_drone_paths(
        paths, detections, targets, args.area_size, args.altitude,
        combined_path, drone_ids=drone_ids,
        title=f"All Drone Paths — {exp_name}",
        dpi=args.dpi,
    )

    # Per-drone plots
    for did in drone_ids:
        drone_path = os.path.join(data_dir, f"path_drone{did}.png")
        drone_dets = [d for d in detections if d[3] == did]
        plot_drone_paths(
            paths, drone_dets, targets, args.area_size, args.altitude,
            drone_path, drone_ids=[did],
            title=f"Drone {did} Path — {exp_name}",
            dpi=args.dpi,
        )

    print(f"Done — {1 + len(drone_ids)} images saved to {data_dir}/")


if __name__ == "__main__":
    main()
