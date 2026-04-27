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
from matplotlib.patches import Polygon as MplPolygon
import numpy as np

# Import Voronoi utilities (for partition visualization)
import sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from voronoi_utils import compute_bounded_voronoi

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


def load_targets(targets_file: str) -> list[tuple[float, float, str, str]]:
    """
    Load ground-truth target positions from a CSV file.

    Accepted formats (comma separated):
      name,x,y
      name,x,y,class
    A leading `name,x,y,class` header row is skipped automatically.

    Returns a list of (x, y, name, class) tuples. If `targets_file` is None or
    missing, returns an empty list (no targets drawn, rather than a fake
    fallback that would mis-locate markers for a differently sized area).
    """
    targets: list[tuple[float, float, str, str]] = []
    if not targets_file or not os.path.exists(targets_file):
        return targets
    with open(targets_file) as f:
        reader = csv.reader(f)
        for row in reader:
            if not row or row[0].startswith("#"):
                continue
            if row[0].strip().lower() == "name":
                continue
            if len(row) < 3:
                continue
            name = row[0].strip()
            try:
                x = float(row[1])
                y = float(row[2])
            except ValueError:
                continue
            cls = row[3].strip() if len(row) >= 4 else ""
            targets.append((x, y, name, cls))
    return targets


# Per-class target marker colors (matches HSV classes in detection_publisher.py)
TARGET_CLASS_COLORS = {
    "person":  "#e41a1c",  # red
    "vehicle": "#377eb8",  # blue
    "fire":    "#ff7f00",  # orange
}
DEFAULT_TARGET_COLOR = "#d62728"


def draw_targets(ax, targets, legend_handles):
    """
    Draw ground-truth target markers on `ax`, colored per-class when class
    metadata is available. Appends matching legend entries to `legend_handles`.
    """
    if not targets:
        return

    by_class: dict[str, list[tuple[float, float, str]]] = {}
    for tx, ty, tname, tcls in targets:
        key = tcls if tcls in TARGET_CLASS_COLORS else ""
        by_class.setdefault(key, []).append((tx, ty, tname))

    for cls, entries in by_class.items():
        color = TARGET_CLASS_COLORS.get(cls, DEFAULT_TARGET_COLOR)
        xs = [e[0] for e in entries]
        ys = [e[1] for e in entries]
        ax.scatter(
            xs, ys,
            marker="X", c=color, edgecolors="black",
            s=200, zorder=7, linewidths=1.0,
        )
        for tx, ty, tname in entries:
            ax.annotate(
                tname, (tx, ty),
                textcoords="offset points", xytext=(0, -28),
                fontsize=14, ha="center", color="#333333",
            )
        label = f"Target ({cls})" if cls else "Target (truth)"
        legend_handles.append(plt.Line2D(
            [0], [0], marker="X", color=color, linestyle="None",
            markeredgecolor="black", markersize=10, label=label,
        ))


def draw_detections(ax, detections, legend_handles, drone_ids=None):
    """
    Draw detection markers as stars colored by class (matching the ground-truth
    target color for that class). Groups automatically by class so the legend
    grows or shrinks with the actual set of classes present.
    """
    if not detections:
        return

    by_class: dict[str, list[tuple[float, float]]] = {}
    for x, y, cls, did in detections:
        if drone_ids is not None and did not in drone_ids:
            continue
        by_class.setdefault(cls, []).append((x, y))

    for cls, pts in by_class.items():
        if not pts:
            continue
        color = TARGET_CLASS_COLORS.get(cls, DEFAULT_TARGET_COLOR)
        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]
        ax.scatter(
            xs, ys,
            marker="*", c=color, edgecolors="black",
            s=150, zorder=6, linewidths=0.8, alpha=0.85,
        )
        label = f"Detection ({cls})" if cls else "Detection"
        legend_handles.append(plt.Line2D(
            [0], [0], marker="*", color=color, linestyle="None",
            markeredgecolor="black", markersize=12, label=label,
        ))


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
    ax.set_xlabel("X (m)", fontsize=24)
    ax.set_ylabel("Y (m)", fontsize=24)
    ax.set_title(title or "Drone Flight Paths", fontsize=28)
    ax.tick_params(axis="both", labelsize=20)
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
    draw_targets(ax, targets, legend_handles)

    # Plot detections as per-class colored stars
    draw_detections(ax, detections, legend_handles, drone_ids=drone_ids)

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
              bbox_to_anchor=(1.02, 1.0), fontsize=20, framealpha=0.9,
              borderaxespad=0)
    fig.savefig(output_path, dpi=dpi, bbox_inches="tight")
    plt.close(fig)
    print(f"  Saved: {output_path}")


def load_timed_positions(csv_path: str) -> dict[int, list[tuple[float, float, float]]]:
    """Load positions.csv and return {drone_id: [(timestamp, x, y), ...]}."""
    paths = defaultdict(list)
    with open(csv_path) as f:
        reader = csv.DictReader(f)
        for row in reader:
            did = int(row["drone_id"])
            t = float(row["elapsed_s"])
            x = float(row["x_local"])
            y = float(row["y_local"])
            paths[did].append((t, x, y))
    return dict(paths)


def get_positions_at_time(timed_paths: dict[int, list[tuple[float, float, float]]],
                          t: float) -> dict[int, tuple[float, float]]:
    """Get each drone's position at time t (nearest timestamp <= t)."""
    positions = {}
    for did, entries in timed_paths.items():
        best = None
        for ts, x, y in entries:
            if ts <= t:
                best = (x, y)
            else:
                break
        if best is None and entries:
            best = (entries[0][1], entries[0][2])
        if best:
            positions[did] = best
    return positions


def plot_voronoi_partitions(
    paths: dict[int, list[tuple[float, float]]],
    timed_paths: dict[int, list[tuple[float, float, float]]],
    targets: list[tuple[float, float, str]],
    area_size: float,
    output_path: str,
    snapshot_time: float | None = None,
    title: str = "",
    dpi: int = 300,
):
    """
    Plot Voronoi partitions showing which area is assigned to which drone.

    If snapshot_time is None, uses the final recorded positions.
    """
    half = area_size / 2.0
    bounds = (-half, -half, half, half)

    # Get drone positions at the requested time
    if timed_paths:
        if snapshot_time is None:
            # Use the max timestamp across all drones
            max_t = max(entries[-1][0] for entries in timed_paths.values() if entries)
            snapshot_time = max_t
        pos_map = get_positions_at_time(timed_paths, snapshot_time)
    else:
        # Fallback: use last position from paths
        pos_map = {did: pts[-1] for did, pts in paths.items() if pts}

    drone_ids = sorted(pos_map.keys())
    if len(drone_ids) < 2:
        print(f"  Skipping Voronoi plot — need at least 2 drones, got {len(drone_ids)}")
        return

    points = np.array([pos_map[did] for did in drone_ids])
    cells = compute_bounded_voronoi(points, bounds)

    fig, ax = plt.subplots(figsize=(10, 10))
    ax.set_xlim(-half - 3, half + 3)
    ax.set_ylim(-half - 3, half + 3)
    ax.set_aspect("equal")
    ax.set_xlabel("X (m)", fontsize=24)
    ax.set_ylabel("Y (m)", fontsize=24)
    time_label = f" (t={snapshot_time:.0f}s)" if snapshot_time is not None else ""
    ax.set_title(title or f"Voronoi Partitions{time_label}", fontsize=28)
    ax.tick_params(axis="both", labelsize=20)
    ax.grid(True, alpha=0.3)
    ax.set_facecolor("#f5f5f5")

    # Draw monitoring area boundary
    border = plt.Rectangle(
        (-half, -half), area_size, area_size,
        linewidth=2, edgecolor="black", facecolor="white", linestyle="--",
        zorder=0
    )
    ax.add_patch(border)

    legend_handles = []

    # Draw each Voronoi cell as a filled polygon
    for i, did in enumerate(drone_ids):
        cell = cells[i]
        color = DRONE_COLORS.get(did, "#333333")
        if len(cell) >= 3:
            polygon = MplPolygon(
                cell, closed=True,
                facecolor=color, alpha=0.25,
                edgecolor=color, linewidth=2.0,
                zorder=1
            )
            ax.add_patch(polygon)

        # Drone position marker
        px, py = pos_map[did]
        ax.plot(
            px, py,
            marker="o", color=color, markersize=14,
            markeredgecolor="black", markeredgewidth=1.5, zorder=5
        )
        ax.annotate(
            f"D{did}", (px, py),
            textcoords="offset points", xytext=(0, 12),
            fontsize=20, fontweight="bold", ha="center", color=color,
            zorder=6
        )

        legend_handles.append(
            mpatches.Patch(facecolor=color, alpha=0.4, edgecolor=color,
                           linewidth=2.0, label=f"Drone {did} region")
        )

    # Plot targets
    draw_targets(ax, targets, legend_handles)

    ax.legend(handles=legend_handles, loc="upper left",
              bbox_to_anchor=(1.02, 1.0), fontsize=20, framealpha=0.9,
              borderaxespad=0)
    fig.savefig(output_path, dpi=dpi, bbox_inches="tight")
    plt.close(fig)
    print(f"  Saved: {output_path}")


def plot_lawnmower_strips(
    paths: dict[int, list[tuple[float, float]]],
    targets: list[tuple[float, float, str]],
    area_size: float,
    output_path: str,
    drone_ids: list[int] | None = None,
    title: str = "",
    dpi: int = 300,
):
    """
    Plot lawnmower strip assignments showing each drone's vertical strip.

    Divides the monitoring area into N equal-width vertical strips (matching
    the lawnmower baseline controller logic) and colors each strip.
    """
    half = area_size / 2.0

    if drone_ids is None:
        drone_ids = sorted(paths.keys())
    n = len(drone_ids)
    if n == 0:
        print("  Skipping strip plot — no drones")
        return

    strip_width = area_size / n

    fig, ax = plt.subplots(figsize=(10, 10))
    ax.set_xlim(-half - 3, half + 3)
    ax.set_ylim(-half - 3, half + 3)
    ax.set_aspect("equal")
    ax.set_xlabel("X (m)", fontsize=24)
    ax.set_ylabel("Y (m)", fontsize=24)
    ax.set_title(title or "Lawnmower Strip Assignments", fontsize=28)
    ax.tick_params(axis="both", labelsize=20)
    ax.grid(True, alpha=0.3)
    ax.set_facecolor("#f5f5f5")

    # Draw monitoring area boundary
    border = plt.Rectangle(
        (-half, -half), area_size, area_size,
        linewidth=2, edgecolor="black", facecolor="white", linestyle="--",
        zorder=0
    )
    ax.add_patch(border)

    legend_handles = []

    # Draw each drone's strip
    for i, did in enumerate(drone_ids):
        color = DRONE_COLORS.get(did, "#333333")
        strip_x_min = -half + i * strip_width
        strip_rect = plt.Rectangle(
            (strip_x_min, -half), strip_width, area_size,
            facecolor=color, alpha=0.2,
            edgecolor=color, linewidth=2.0, linestyle="-",
            zorder=1
        )
        ax.add_patch(strip_rect)

        # Label the strip center
        strip_cx = strip_x_min + strip_width / 2
        ax.annotate(
            f"D{did}", (strip_cx, 0),
            fontsize=24, fontweight="bold", ha="center", va="center",
            color=color, zorder=6,
        )

        # Draw the flight path if available
        if did in paths and len(paths[did]) >= 2:
            pts = np.array(paths[did])
            ax.plot(
                pts[:, 0], pts[:, 1],
                color=color, linewidth=1.5, alpha=0.7, zorder=3
            )

        legend_handles.append(
            mpatches.Patch(facecolor=color, alpha=0.3, edgecolor=color,
                           linewidth=2.0,
                           label=f"Drone {did} strip [{strip_x_min:.0f}, {strip_x_min + strip_width:.0f}]m")
        )

    # Plot targets
    draw_targets(ax, targets, legend_handles)

    ax.legend(handles=legend_handles, loc="upper left",
              bbox_to_anchor=(1.02, 1.0), fontsize=20, framealpha=0.9,
              borderaxespad=0)
    fig.savefig(output_path, dpi=dpi, bbox_inches="tight")
    plt.close(fig)
    print(f"  Saved: {output_path}")


def plot_force_field(
    paths: dict[int, list[tuple[float, float]]],
    timed_paths: dict[int, list[tuple[float, float, float]]],
    targets: list[tuple[float, float, str]],
    area_size: float,
    output_path: str,
    snapshot_time: float | None = None,
    title: str = "",
    dpi: int = 300,
):
    """
    Plot APF force field showing repulsion zones around drones and attraction
    toward uncovered areas. Visualizes the virtual force landscape at a snapshot.
    """
    half = area_size / 2.0

    # Get drone positions at snapshot time
    if timed_paths:
        if snapshot_time is None:
            max_t = max(entries[-1][0] for entries in timed_paths.values() if entries)
            snapshot_time = max_t
        pos_map = get_positions_at_time(timed_paths, snapshot_time)
    else:
        pos_map = {did: pts[-1] for did, pts in paths.items() if pts}

    drone_ids = sorted(pos_map.keys())
    if len(drone_ids) < 1:
        print("  Skipping force field plot — no drones")
        return

    fig, ax = plt.subplots(figsize=(10, 10))
    ax.set_xlim(-half - 3, half + 3)
    ax.set_ylim(-half - 3, half + 3)
    ax.set_aspect("equal")
    ax.set_xlabel("X (m)", fontsize=24)
    ax.set_ylabel("Y (m)", fontsize=24)
    time_label = f" (t={snapshot_time:.0f}s)" if snapshot_time is not None else ""
    ax.set_title(title or f"APF Force Field{time_label}", fontsize=28)
    ax.tick_params(axis="both", labelsize=20)
    ax.grid(True, alpha=0.3)
    ax.set_facecolor("#f5f5f5")

    # Draw monitoring area boundary
    border = plt.Rectangle(
        (-half, -half), area_size, area_size,
        linewidth=2, edgecolor="black", facecolor="white", linestyle="--",
        zorder=0
    )
    ax.add_patch(border)

    # Compute repulsion field on a grid
    grid_n = 20
    gx = np.linspace(-half, half, grid_n)
    gy = np.linspace(-half, half, grid_n)
    GX, GY = np.meshgrid(gx, gy)
    FX = np.zeros_like(GX)
    FY = np.zeros_like(GY)

    k_rep = 500.0
    for i in range(grid_n):
        for j in range(grid_n):
            px, py = GX[i, j], GY[i, j]
            fx, fy = 0.0, 0.0
            for did in drone_ids:
                ox, oy = pos_map[did]
                dx, dy = px - ox, py - oy
                dist = max(math.sqrt(dx * dx + dy * dy), 1.0)
                fx += k_rep * dx / (dist ** 3)
                fy += k_rep * dy / (dist ** 3)
            FX[i, j] = fx
            FY[i, j] = fy

    # Normalize for display
    mag = np.sqrt(FX ** 2 + FY ** 2)
    mag_max = np.percentile(mag[mag > 0], 95) if np.any(mag > 0) else 1.0
    FX_norm = FX / (mag_max + 1e-8)
    FY_norm = FY / (mag_max + 1e-8)

    ax.quiver(GX, GY, FX_norm, FY_norm, mag, cmap="YlOrRd", alpha=0.6,
              scale=25, zorder=2)

    # Draw repulsion circles around each drone
    legend_handles = []
    for did in drone_ids:
        color = DRONE_COLORS.get(did, "#333333")
        ox, oy = pos_map[did]

        # Repulsion radius visualization (where force is strong)
        rep_circle = plt.Circle((ox, oy), 20.0, facecolor=color, alpha=0.1,
                                edgecolor=color, linewidth=1.5, linestyle=":",
                                zorder=1)
        ax.add_patch(rep_circle)

        # Drone position
        ax.plot(ox, oy, marker="o", color=color, markersize=14,
                markeredgecolor="black", markeredgewidth=1.5, zorder=5)
        ax.annotate(f"D{did}", (ox, oy),
                    textcoords="offset points", xytext=(0, 12),
                    fontsize=20, fontweight="bold", ha="center", color=color,
                    zorder=6)

        legend_handles.append(
            mpatches.Patch(facecolor=color, alpha=0.2, edgecolor=color,
                           linewidth=2.0, label=f"Drone {did} repulsion zone")
        )

    # Draw flight paths
    for did in drone_ids:
        if did in paths and len(paths[did]) >= 2:
            color = DRONE_COLORS.get(did, "#333333")
            pts = np.array(paths[did])
            ax.plot(pts[:, 0], pts[:, 1], color=color, linewidth=1.5,
                    alpha=0.5, zorder=3)

    # Plot targets
    draw_targets(ax, targets, legend_handles)

    ax.legend(handles=legend_handles, loc="upper left",
              bbox_to_anchor=(1.02, 1.0), fontsize=20, framealpha=0.9,
              borderaxespad=0)
    fig.savefig(output_path, dpi=dpi, bbox_inches="tight")
    plt.close(fig)
    print(f"  Saved: {output_path}")


def plot_pso_vectors(
    paths: dict[int, list[tuple[float, float]]],
    timed_paths: dict[int, list[tuple[float, float, float]]],
    targets: list[tuple[float, float, str]],
    area_size: float,
    output_path: str,
    title: str = "",
    dpi: int = 300,
):
    """
    Plot PSO particle trajectories with velocity vectors showing the direction
    of movement at each sampled position. Shows pbest markers and overall
    swarm movement pattern.
    """
    half = area_size / 2.0

    drone_ids = sorted(paths.keys())
    if not drone_ids:
        print("  Skipping PSO vectors plot — no drones")
        return

    fig, ax = plt.subplots(figsize=(10, 10))
    ax.set_xlim(-half - 3, half + 3)
    ax.set_ylim(-half - 3, half + 3)
    ax.set_aspect("equal")
    ax.set_xlabel("X (m)", fontsize=24)
    ax.set_ylabel("Y (m)", fontsize=24)
    ax.set_title(title or "PSO Particle Trajectories with Velocity Vectors", fontsize=28)
    ax.tick_params(axis="both", labelsize=20)
    ax.grid(True, alpha=0.3)
    ax.set_facecolor("#f5f5f5")

    # Draw monitoring area boundary
    border = plt.Rectangle(
        (-half, -half), area_size, area_size,
        linewidth=2, edgecolor="black", facecolor="white", linestyle="--",
        zorder=0
    )
    ax.add_patch(border)

    legend_handles = []

    for did in drone_ids:
        if did not in paths or len(paths[did]) < 2:
            continue

        color = DRONE_COLORS.get(did, "#333333")
        pts = np.array(paths[did])

        # Flight path
        ax.plot(pts[:, 0], pts[:, 1], color=color, linewidth=2.0,
                alpha=0.7, zorder=3)

        # Velocity vectors (sampled every N points to avoid clutter)
        sample_interval = max(1, len(pts) // 20)
        for k in range(0, len(pts) - 1, sample_interval):
            dx = pts[k + 1, 0] - pts[k, 0]
            dy = pts[k + 1, 1] - pts[k, 1]
            speed = math.sqrt(dx * dx + dy * dy)
            if speed > 0.5:  # skip near-zero movement
                ax.annotate("", xy=(pts[k, 0] + dx, pts[k, 1] + dy),
                            xytext=(pts[k, 0], pts[k, 1]),
                            arrowprops=dict(arrowstyle="->", color=color,
                                            lw=1.5, alpha=0.6),
                            zorder=4)

        # Start marker
        ax.plot(pts[0, 0], pts[0, 1], marker="^", color=color, markersize=14,
                markeredgecolor="black", markeredgewidth=1.2, zorder=5)
        # End marker
        ax.plot(pts[-1, 0], pts[-1, 1], marker="s", color=color, markersize=11,
                markeredgecolor="black", markeredgewidth=1.2, zorder=5)

        # Mark the point where drone spent the most time (approximation of pbest)
        if timed_paths and did in timed_paths:
            entries = timed_paths[did]
            if len(entries) > 2:
                # Find position with longest dwell time
                max_dwell = 0
                best_pos = (pts[-1, 0], pts[-1, 1])
                for i in range(len(entries) - 1):
                    dwell = entries[i + 1][0] - entries[i][0]
                    if dwell > max_dwell:
                        max_dwell = dwell
                        best_pos = (entries[i][1], entries[i][2])
                ax.plot(best_pos[0], best_pos[1], marker="*", color=color,
                        markersize=18, markeredgecolor="black",
                        markeredgewidth=1.0, zorder=6)

        legend_handles.append(
            plt.Line2D([0], [0], color=color, linewidth=2.0,
                       label=f"Drone {did} trajectory")
        )

    # Add legend entries for markers
    legend_handles.append(plt.Line2D(
        [0], [0], marker="^", color="gray", linestyle="None",
        markeredgecolor="black", markersize=10, label="Start"))
    legend_handles.append(plt.Line2D(
        [0], [0], marker="s", color="gray", linestyle="None",
        markeredgecolor="black", markersize=8, label="End"))
    legend_handles.append(plt.Line2D(
        [0], [0], marker="*", color="gray", linestyle="None",
        markeredgecolor="black", markersize=14, label="Best dwell point"))

    # Plot targets
    draw_targets(ax, targets, legend_handles)

    ax.legend(handles=legend_handles, loc="upper left",
              bbox_to_anchor=(1.02, 1.0), fontsize=20, framealpha=0.9,
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
    parser.add_argument("--voronoi", action="store_true",
                        help="Generate Voronoi partition plot showing assigned regions per drone")
    parser.add_argument("--strips", action="store_true",
                        help="Generate lawnmower strip assignment plot")
    parser.add_argument("--snapshot-time", type=float, default=None,
                        help="Time (seconds) for Voronoi/force-field snapshot (default: last timestamp)")
    parser.add_argument("--force-field", action="store_true",
                        help="Generate APF force field visualization with repulsion zones")
    parser.add_argument("--pso-vectors", action="store_true",
                        help="Generate PSO velocity vector plot showing particle trajectories")
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

    targets_file = args.targets_file
    if not targets_file:
        auto_targets = os.path.join(data_dir, "targets.csv")
        if os.path.exists(auto_targets):
            targets_file = auto_targets
    targets = load_targets(targets_file)
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

    # Lawnmower strip plot
    if args.strips:
        strips_path = os.path.join(data_dir, "lawnmower_strips.png")
        plot_lawnmower_strips(
            paths, targets, args.area_size,
            strips_path, drone_ids=drone_ids,
            title=f"Lawnmower Strip Assignments — {exp_name}",
            dpi=args.dpi,
        )

    # Voronoi partition plot
    if args.voronoi:
        timed_paths = load_timed_positions(positions_csv)
        voronoi_path = os.path.join(data_dir, "voronoi_partitions.png")
        time_label = f" (t={args.snapshot_time:.0f}s)" if args.snapshot_time else ""
        plot_voronoi_partitions(
            paths, timed_paths, targets, args.area_size,
            voronoi_path, snapshot_time=args.snapshot_time,
            title=f"Voronoi Partitions — {exp_name}{time_label}",
            dpi=args.dpi,
        )

    # APF force field plot
    if args.force_field:
        timed_paths = load_timed_positions(positions_csv)
        ff_path = os.path.join(data_dir, "apf_force_field.png")
        time_label = f" (t={args.snapshot_time:.0f}s)" if args.snapshot_time else ""
        plot_force_field(
            paths, timed_paths, targets, args.area_size,
            ff_path, snapshot_time=args.snapshot_time,
            title=f"APF Force Field — {exp_name}{time_label}",
            dpi=args.dpi,
        )

    # PSO velocity vectors plot
    if args.pso_vectors:
        timed_paths = load_timed_positions(positions_csv)
        pso_path = os.path.join(data_dir, "pso_vectors.png")
        plot_pso_vectors(
            paths, timed_paths, targets, args.area_size,
            pso_path,
            title=f"PSO Particle Trajectories — {exp_name}",
            dpi=args.dpi,
        )

    print(f"Done — images saved to {data_dir}/")


if __name__ == "__main__":
    main()
