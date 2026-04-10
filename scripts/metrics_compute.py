#!/usr/bin/env python3
"""
Metrics Computation: Offline analysis of experiment log data.

Reads CSV files from data/logs/<experiment_id>/ and computes:
  - Coverage %: fraction of monitoring area within at least one drone's FOV
  - Detection latency: time from experiment start to first detection
  - Redundancy ratio: average number of drones covering each grid cell
  - Energy proxy: total distance traveled by all drones
  - Detection count: total detections over the experiment

Usage:
    python3 metrics_compute.py --experiment-id e1_adaptive_t01
    python3 metrics_compute.py --experiment-id e1_adaptive_t01 --area-size 40 --altitude 30
"""

import argparse
import csv
import json
import math
import os
import sys

import numpy as np


# Camera specs (from x500_mono_cam_down model)
CAMERA_HFOV_RAD = 1.74  # ~100 degrees


def load_csv(filepath: str) -> list[dict]:
    """Load a CSV file as a list of dicts."""
    if not os.path.exists(filepath):
        return []
    with open(filepath, "r") as f:
        return list(csv.DictReader(f))


def compute_fov_radius(altitude: float) -> float:
    """Compute the ground footprint radius for a downward camera at given altitude."""
    return altitude * math.tan(CAMERA_HFOV_RAD / 2)


def compute_coverage(positions: list[dict], area_size: float, altitude: float,
                     grid_resolution: float = 1.0) -> dict:
    """Compute coverage metrics from position data.

    Discretizes the monitoring area into a grid and checks which cells
    fall within at least one drone's camera FOV at each time step.

    Returns dict with:
      - coverage_percent: fraction of cells covered (time-averaged)
      - coverage_over_time: list of (elapsed_s, coverage_%) snapshots
      - redundancy_ratio: average number of drones covering each cell
    """
    if not positions:
        return {"coverage_percent": 0.0, "coverage_over_time": [], "redundancy_ratio": 0.0}

    half = area_size / 2
    fov_radius = compute_fov_radius(altitude)

    # Create grid
    xs = np.arange(-half, half, grid_resolution)
    ys = np.arange(-half, half, grid_resolution)
    grid_x, grid_y = np.meshgrid(xs, ys)
    n_cells = grid_x.size

    # Group positions by timestamp (rounded to 0.5s buckets)
    time_buckets: dict[float, list[tuple[float, float]]] = {}
    for row in positions:
        t = round(float(row["elapsed_s"]) * 2) / 2  # 0.5s buckets
        x = float(row["x_local"])
        y = float(row["y_local"])
        time_buckets.setdefault(t, []).append((x, y))

    # Compute coverage at each time step
    coverage_over_time = []
    all_coverage_counts = np.zeros_like(grid_x, dtype=float)
    n_steps = 0

    for t in sorted(time_buckets.keys()):
        drone_positions = time_buckets[t]
        step_coverage = np.zeros_like(grid_x, dtype=int)

        for dx, dy in drone_positions:
            dist = np.sqrt((grid_x - dx) ** 2 + (grid_y - dy) ** 2)
            step_coverage += (dist <= fov_radius).astype(int)

        covered = (step_coverage > 0).sum()
        coverage_pct = 100.0 * covered / n_cells
        coverage_over_time.append((t, coverage_pct))

        all_coverage_counts += step_coverage
        n_steps += 1

    # Time-averaged metrics
    avg_coverage = np.mean([c for _, c in coverage_over_time]) if coverage_over_time else 0.0
    avg_redundancy = float(all_coverage_counts.sum() / (n_cells * max(n_steps, 1)))

    return {
        "coverage_percent": round(avg_coverage, 2),
        "coverage_over_time": coverage_over_time,
        "redundancy_ratio": round(avg_redundancy, 3),
    }


def compute_energy(positions: list[dict]) -> dict:
    """Compute energy proxy: total distance traveled by all drones.

    Returns dict with:
      - total_distance_m: sum of Euclidean distances between consecutive positions
      - per_drone_distance: {drone_id: distance_m}
    """
    if not positions:
        return {"total_distance_m": 0.0, "per_drone_distance": {}}

    # Group by drone
    drone_tracks: dict[int, list[tuple[float, float]]] = {}
    for row in positions:
        did = int(row["drone_id"])
        x = float(row["x_local"])
        y = float(row["y_local"])
        drone_tracks.setdefault(did, []).append((x, y))

    per_drone = {}
    total = 0.0
    for did, track in drone_tracks.items():
        dist = 0.0
        for i in range(1, len(track)):
            dx = track[i][0] - track[i - 1][0]
            dy = track[i][1] - track[i - 1][1]
            dist += math.sqrt(dx * dx + dy * dy)
        per_drone[did] = round(dist, 2)
        total += dist

    return {
        "total_distance_m": round(total, 2),
        "per_drone_distance": per_drone,
    }


def compute_detection_metrics(detections: list[dict]) -> dict:
    """Compute detection-related metrics.

    Returns dict with:
      - total_detections: total number of detections
      - first_detection_s: elapsed time to first detection (or None)
      - detections_per_drone: {drone_id: count}
      - unique_classes: list of detected class names
    """
    if not detections:
        return {
            "total_detections": 0,
            "first_detection_s": None,
            "detections_per_drone": {},
            "unique_classes": [],
        }

    times = [float(row["elapsed_s"]) for row in detections]
    first = min(times)

    per_drone: dict[int, int] = {}
    classes = set()
    for row in detections:
        did = int(row["drone_id"])
        per_drone[did] = per_drone.get(did, 0) + 1
        classes.add(row["class_name"])

    return {
        "total_detections": len(detections),
        "first_detection_s": round(first, 2),
        "detections_per_drone": per_drone,
        "unique_classes": sorted(classes),
    }


def compute_coverage_during_event(positions: list[dict], mode_events: list[dict],
                                   area_size: float, altitude: float,
                                   grid_resolution: float = 1.0) -> dict:
    """Compute coverage percentage maintained while exploit mode is active.

    This metric quantifies how well the split-and-reform strategy preserves
    baseline coverage during exploitation events.

    Returns dict with:
      - coverage_during_exploit: average coverage % during EXPLOIT periods
      - coverage_during_explore: average coverage % during EXPLORE-only periods
      - exploit_duration_s: total time spent in exploit mode
      - num_exploit_events: number of exploit mode activations
    """
    if not positions or not mode_events:
        return {
            "coverage_during_exploit": None,
            "coverage_during_explore": None,
            "exploit_duration_s": 0.0,
            "num_exploit_events": 0,
        }

    half = area_size / 2
    fov_radius = compute_fov_radius(altitude)

    # Build exploit time intervals from mode events
    exploit_intervals = []  # list of (start, end) elapsed_s
    current_exploit_start = None
    num_exploit_events = 0

    for evt in sorted(mode_events, key=lambda e: float(e["elapsed_s"])):
        mode = evt["mode"]
        t = float(evt["elapsed_s"])
        if mode == "EXPLOIT" and current_exploit_start is None:
            current_exploit_start = t
            num_exploit_events += 1
        elif mode == "EXPLORE" and current_exploit_start is not None:
            exploit_intervals.append((current_exploit_start, t))
            current_exploit_start = None

    # Close any open exploit interval
    if current_exploit_start is not None:
        max_t = max(float(p["elapsed_s"]) for p in positions)
        exploit_intervals.append((current_exploit_start, max_t))

    if not exploit_intervals:
        return {
            "coverage_during_exploit": None,
            "coverage_during_explore": None,
            "exploit_duration_s": 0.0,
            "num_exploit_events": 0,
        }

    total_exploit_time = sum(end - start for start, end in exploit_intervals)

    # Create grid
    xs = np.arange(-half, half, grid_resolution)
    ys = np.arange(-half, half, grid_resolution)
    grid_x, grid_y = np.meshgrid(xs, ys)
    n_cells = grid_x.size

    # Group positions by time bucket
    time_buckets: dict[float, list[tuple[float, float]]] = {}
    for row in positions:
        t = round(float(row["elapsed_s"]) * 2) / 2
        x = float(row["x_local"])
        y = float(row["y_local"])
        time_buckets.setdefault(t, []).append((x, y))

    def _is_during_exploit(t: float) -> bool:
        for start, end in exploit_intervals:
            if start <= t <= end:
                return True
        return False

    exploit_coverages = []
    explore_coverages = []

    for t in sorted(time_buckets.keys()):
        drone_positions = time_buckets[t]
        step_coverage = np.zeros_like(grid_x, dtype=int)

        for dx, dy in drone_positions:
            dist = np.sqrt((grid_x - dx) ** 2 + (grid_y - dy) ** 2)
            step_coverage += (dist <= fov_radius).astype(int)

        covered = (step_coverage > 0).sum()
        coverage_pct = 100.0 * covered / n_cells

        if _is_during_exploit(t):
            exploit_coverages.append(coverage_pct)
        else:
            explore_coverages.append(coverage_pct)

    avg_exploit = np.mean(exploit_coverages) if exploit_coverages else None
    avg_explore = np.mean(explore_coverages) if explore_coverages else None

    return {
        "coverage_during_exploit": round(float(avg_exploit), 2) if avg_exploit is not None else None,
        "coverage_during_explore": round(float(avg_explore), 2) if avg_explore is not None else None,
        "exploit_duration_s": round(total_exploit_time, 2),
        "num_exploit_events": num_exploit_events,
    }


def compute_response_time(log_dir: str) -> dict:
    """Compute reconfiguration response time metrics from reconfig_events.csv.

    Returns dict with:
      - mean_reconfig_time_s: average time from detection to all drones arriving
      - mean_publish_delay_s: average time from detection to waypoint publish
      - min_reconfig_time_s / max_reconfig_time_s: extremes
      - num_reconfig_events: total events with arrival data
      - reconfig_events: list of per-event dicts
    """
    reconfig_path = os.path.join(log_dir, "reconfig_events.csv")
    events = load_csv(reconfig_path)

    if not events:
        return {
            "mean_reconfig_time_s": None,
            "mean_publish_delay_s": None,
            "min_reconfig_time_s": None,
            "max_reconfig_time_s": None,
            "num_reconfig_events": 0,
            "reconfig_events": [],
        }

    reconfig_times = []
    publish_delays = []
    event_summaries = []

    for evt in events:
        pub_delay = float(evt["publish_delay_s"]) if evt.get("publish_delay_s") else None
        reconfig_time = float(evt["reconfig_time_s"]) if evt.get("reconfig_time_s") else None

        if pub_delay is not None:
            publish_delays.append(pub_delay)
        if reconfig_time is not None:
            reconfig_times.append(reconfig_time)

        event_summaries.append({
            "event_class": evt.get("event_class", ""),
            "confidence": float(evt["confidence"]) if evt.get("confidence") else None,
            "reconfig_time_s": reconfig_time,
            "publish_delay_s": pub_delay,
            "reason": evt.get("reason", ""),
        })

    return {
        "mean_reconfig_time_s": round(np.mean(reconfig_times), 3) if reconfig_times else None,
        "mean_publish_delay_s": round(np.mean(publish_delays), 3) if publish_delays else None,
        "min_reconfig_time_s": round(min(reconfig_times), 3) if reconfig_times else None,
        "max_reconfig_time_s": round(max(reconfig_times), 3) if reconfig_times else None,
        "num_reconfig_events": len(reconfig_times),
        "reconfig_events": event_summaries,
    }


def compute_experiment_duration(positions: list[dict]) -> float:
    """Get experiment duration from position data."""
    if not positions:
        return 0.0
    times = [float(row["elapsed_s"]) for row in positions]
    return round(max(times) - min(times), 2)


def main():
    parser = argparse.ArgumentParser(description="Compute experiment metrics from CSV logs")
    parser.add_argument(
        "--experiment-id", type=str, required=True,
        help="Experiment identifier (subdirectory name)",
    )
    parser.add_argument(
        "--log-base", type=str,
        default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "data", "logs"),
        help="Base directory for logs",
    )
    parser.add_argument("--area-size", type=float, default=40.0, help="Monitoring area (m)")
    parser.add_argument("--altitude", type=float, default=30.0, help="Drone altitude (m)")
    parser.add_argument("--grid-resolution", type=float, default=2.0, help="Grid cell size (m)")
    args = parser.parse_args()

    log_dir = os.path.join(args.log_base, args.experiment_id)
    if not os.path.isdir(log_dir):
        print(f"ERROR: Log directory not found: {log_dir}")
        sys.exit(1)

    print(f"Loading data from: {log_dir}")
    positions = load_csv(os.path.join(log_dir, "positions.csv"))
    detections = load_csv(os.path.join(log_dir, "detections.csv"))
    mode_events = load_csv(os.path.join(log_dir, "swarm_mode.csv"))

    print(f"  Positions: {len(positions)} rows")
    print(f"  Detections: {len(detections)} rows")
    print(f"  Mode events: {len(mode_events)} rows")

    # Compute metrics
    duration = compute_experiment_duration(positions)
    coverage = compute_coverage(positions, args.area_size, args.altitude, args.grid_resolution)
    energy = compute_energy(positions)
    detection_metrics = compute_detection_metrics(detections)
    event_coverage = compute_coverage_during_event(
        positions, mode_events, args.area_size, args.altitude, args.grid_resolution
    )
    response_time = compute_response_time(log_dir)

    # Build summary
    summary = {
        "experiment_id": args.experiment_id,
        "duration_s": duration,
        "area_size_m": args.area_size,
        "altitude_m": args.altitude,
        "coverage_percent": coverage["coverage_percent"],
        "redundancy_ratio": coverage["redundancy_ratio"],
        "total_distance_m": energy["total_distance_m"],
        "per_drone_distance_m": energy["per_drone_distance"],
        "total_detections": detection_metrics["total_detections"],
        "first_detection_s": detection_metrics["first_detection_s"],
        "detections_per_drone": detection_metrics["detections_per_drone"],
        "unique_classes": detection_metrics["unique_classes"],
        "coverage_during_exploit": event_coverage["coverage_during_exploit"],
        "coverage_during_explore": event_coverage["coverage_during_explore"],
        "exploit_duration_s": event_coverage["exploit_duration_s"],
        "num_exploit_events": event_coverage["num_exploit_events"],
        "mean_reconfig_time_s": response_time["mean_reconfig_time_s"],
        "mean_publish_delay_s": response_time["mean_publish_delay_s"],
        "min_reconfig_time_s": response_time["min_reconfig_time_s"],
        "max_reconfig_time_s": response_time["max_reconfig_time_s"],
        "num_reconfig_events": response_time["num_reconfig_events"],
    }

    # Save summary
    summary_path = os.path.join(log_dir, "metrics_summary.json")
    # Don't include coverage_over_time in summary (too large)
    with open(summary_path, "w") as f:
        json.dump(summary, f, indent=2)

    # Save coverage time series separately
    cov_path = os.path.join(log_dir, "coverage_over_time.csv")
    with open(cov_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["elapsed_s", "coverage_percent"])
        for t, c in coverage["coverage_over_time"]:
            writer.writerow([f"{t:.1f}", f"{c:.2f}"])

    # Print results
    print("\n══════════════════════════════════════")
    print(f"  Experiment: {args.experiment_id}")
    print(f"  Duration: {duration:.1f}s")
    print(f"══════════════════════════════════════")
    print(f"  Coverage:      {coverage['coverage_percent']:.1f}%")
    print(f"  Redundancy:    {coverage['redundancy_ratio']:.3f}")
    print(f"  Total distance:{energy['total_distance_m']:.1f}m")
    print(f"  Detections:    {detection_metrics['total_detections']}")
    print(f"  First detect:  {detection_metrics['first_detection_s']}s")
    if event_coverage["coverage_during_exploit"] is not None:
        print(f"  Cov (exploit): {event_coverage['coverage_during_exploit']:.1f}%")
        print(f"  Cov (explore): {event_coverage['coverage_during_explore']:.1f}%")
        print(f"  Exploit time:  {event_coverage['exploit_duration_s']:.1f}s")
        print(f"  Exploit events:{event_coverage['num_exploit_events']}")
    print(f"══════════════════════════════════════")
    if response_time["mean_reconfig_time_s"] is not None:
        print(f"  Reconfig time: {response_time['mean_reconfig_time_s']:.3f}s (mean)")
        print(f"  Publish delay: {response_time['mean_publish_delay_s']:.3f}s (mean)")
        print(f"  Reconfig range: {response_time['min_reconfig_time_s']:.3f}–{response_time['max_reconfig_time_s']:.3f}s")
        print(f"  Reconfig events:{response_time['num_reconfig_events']}")
    print(f"  Saved: {summary_path}")
    print(f"  Saved: {cov_path}")


if __name__ == "__main__":
    main()
