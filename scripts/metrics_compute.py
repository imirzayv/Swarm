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

# ── Energy model (Abeywickrama et al., 2018) ────────────────────────────────
# Abeywickrama, H. V., Jayawickrama, B. A., He, Y., & Dutkiewicz, E. (2018).
# "Comprehensive Energy Consumption Model for Unmanned Aerial Vehicles, Based
#  on Empirical Studies of Battery Performance." IEEE Access 6:58383–58394.
#
# Reported DJI Matrice 100 numbers (Tables II–III, light-payload regime):
#   hover                 ≈ 142.8 W
#   forward flight @5 m/s ≈ 161.0 W    (ΔP_fwd = 18.2 W above hover)
#   communication load    ≈  10.0 W    (always-on 2.4 GHz telemetry)
#
# Total mission energy is decomposed as:
#   E_total = P_hover × T              # attitude hold, always paid
#           + ΔP_fwd   × (D / v_c)     # extra cost of translation
#           + P_comm   × T             # always-on telemetry
#   where D = total horizontal distance travelled (all drones summed),
#         v_c = cruise speed (5 m/s, matching the calibration point).
#
# The forward-flight term is what makes distance a first-class energy term —
# the old model was hover-only and any method that moved less got zero credit.
P_HOVER_W = 142.8
P_FWD_5MS_W = 161.0
V_CRUISE_MS = 5.0
DELTA_FWD_W = P_FWD_5MS_W - P_HOVER_W  # 18.2 W
P_COMM_W = 10.0

# Grid carbon intensity (g CO2 per kWh).
# Azerbaijan: IEA country profile, ~0.44 kg/kWh (2022).
# EU average: EEA, ~0.25 kg/kWh (2022).
CO2_AZERBAIJAN_G_PER_KWH = 440.0
CO2_EU_G_PER_KWH = 250.0

# Detection→target matching: a detection's (world_x, world_y) is accepted as
# "this target" if within this radius of the known target location.
TARGET_MATCH_RADIUS_M = 6.0


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


def compute_distance_phases(positions: list[dict],
                            steady_state_cutoff_s: float) -> dict:
    """Split total distance into steady-state and event-driven phases.

    Defines two non-overlapping windows of elapsed time:
      - steady_state: elapsed_s ∈ [0, cutoff)  — no events have spawned yet
      - event_phase:  elapsed_s ∈ [cutoff, ∞)  — event-response regime

    Each segment between consecutive position samples is assigned to whichever
    window contains its midpoint, which keeps the total exactly equal to
    sum(steady + event) for any cutoff. This split mirrors how the v2 paper
    reports distance: matched-baseline behavior in steady-state, plus the
    event-driven add-on. Without it, methods that move mostly during events
    can't be told apart from methods that move uniformly across the trial.
    """
    if not positions or steady_state_cutoff_s <= 0:
        return {
            "steady_state_distance_m": 0.0,
            "event_phase_distance_m": 0.0,
            "steady_state_cutoff_s": steady_state_cutoff_s,
        }

    drone_tracks: dict[int, list[tuple[float, float, float]]] = {}
    for row in positions:
        did = int(row["drone_id"])
        t = float(row["elapsed_s"])
        x = float(row["x_local"])
        y = float(row["y_local"])
        drone_tracks.setdefault(did, []).append((t, x, y))

    steady_total = 0.0
    event_total = 0.0
    for track in drone_tracks.values():
        track.sort(key=lambda p: p[0])
        for i in range(1, len(track)):
            t0, x0, y0 = track[i - 1]
            t1, x1, y1 = track[i]
            seg = math.hypot(x1 - x0, y1 - y0)
            mid_t = 0.5 * (t0 + t1)
            if mid_t < steady_state_cutoff_s:
                steady_total += seg
            else:
                event_total += seg

    return {
        "steady_state_distance_m": round(steady_total, 2),
        "event_phase_distance_m": round(event_total, 2),
        "steady_state_cutoff_s": steady_state_cutoff_s,
    }


def compute_energy_and_emissions(duration_s: float, n_drones: int,
                                 total_distance_m: float = 0.0) -> dict:
    """Mission energy (Wh) and CO2 equivalent (g) — Abeywickrama 2018 model.

    Decomposes energy into three additive terms:
      hover_wh    = P_hover × T × n_drones
      forward_wh  = ΔP_fwd × (D / v_cruise)        ← D = total_distance_m
      comm_wh     = P_comm × T × n_drones

    The forward-flight term converts horizontal flight distance into the extra
    propulsion work above hover. Methods that travel less to accomplish the
    same mission (the core claim of adaptive coverage) therefore win on Wh
    and CO2, which the prior hover-only model could not capture.

    CO2 is reported against two grid mixes so the paper can speak to both the
    deployment country (Azerbaijan) and the EU average baseline reviewers expect.
    """
    if duration_s <= 0 or n_drones <= 0:
        return {
            "hover_power_w": P_HOVER_W,
            "forward_power_w": P_FWD_5MS_W,
            "comm_power_w": P_COMM_W,
            "cruise_speed_ms": V_CRUISE_MS,
            "mission_duration_s": round(duration_s, 2),
            "total_distance_m": round(total_distance_m, 2),
            "n_drones": n_drones,
            "hover_energy_wh": 0.0,
            "forward_energy_wh": 0.0,
            "comm_energy_wh": 0.0,
            "energy_per_drone_wh": 0.0,
            "total_energy_wh": 0.0,
            "total_energy_kwh": 0.0,
            "co2_azerbaijan_g": 0.0,
            "co2_eu_g": 0.0,
        }

    hover_energy_wh = P_HOVER_W * duration_s * n_drones / 3600.0
    forward_energy_wh = DELTA_FWD_W * (max(total_distance_m, 0.0) / V_CRUISE_MS) / 3600.0
    comm_energy_wh = P_COMM_W * duration_s * n_drones / 3600.0

    total_energy_wh = hover_energy_wh + forward_energy_wh + comm_energy_wh
    energy_per_drone_wh = total_energy_wh / n_drones
    total_energy_kwh = total_energy_wh / 1000.0
    co2_az_g = total_energy_kwh * CO2_AZERBAIJAN_G_PER_KWH
    co2_eu_g = total_energy_kwh * CO2_EU_G_PER_KWH

    return {
        "hover_power_w": P_HOVER_W,
        "forward_power_w": P_FWD_5MS_W,
        "comm_power_w": P_COMM_W,
        "cruise_speed_ms": V_CRUISE_MS,
        "mission_duration_s": round(duration_s, 2),
        "total_distance_m": round(total_distance_m, 2),
        "n_drones": n_drones,
        "hover_energy_wh": round(hover_energy_wh, 3),
        "forward_energy_wh": round(forward_energy_wh, 3),
        "comm_energy_wh": round(comm_energy_wh, 3),
        "energy_per_drone_wh": round(energy_per_drone_wh, 3),
        "total_energy_wh": round(total_energy_wh, 3),
        "total_energy_kwh": round(total_energy_kwh, 5),
        "co2_azerbaijan_g": round(co2_az_g, 3),
        "co2_eu_g": round(co2_eu_g, 3),
    }


def compute_target_discovery_times(detections: list[dict],
                                   targets: list[dict],
                                   trial_duration_s: float,
                                   match_radius_m: float = TARGET_MATCH_RADIUS_M) -> dict:
    """Time-to-first and time-to-all for the ground-truth targets.

    For each target in `targets.csv`, scan `detections.csv` in time order and
    record the earliest detection whose `class_name` matches and whose
    `(world_x, world_y)` lies within `match_radius_m` of the target's location.
    Targets with no matching detection contribute `None` and are counted as
    "not found" — `time_to_all_targets_s` is only defined if every target was
    found, otherwise `None`.

    Returns:
      time_to_first_target_s: earliest discovery across all targets (or None)
      time_to_all_targets_s: latest discovery across all targets (or None if
          any target was never detected within the trial)
      targets_found: int — how many ground-truth targets were discovered
      targets_total: int — count of targets listed in targets.csv
      per_target_discovery: list of {name, class, found, first_detection_s}
    """
    if not targets:
        return {
            "time_to_first_target_s": None,
            "time_to_all_targets_s": None,
            "targets_found": 0,
            "targets_total": 0,
            "per_target_discovery": [],
            "match_radius_m": match_radius_m,
        }

    # Pre-sort detections by elapsed_s so first-match scan is O(n) per target.
    sorted_dets = sorted(
        (d for d in detections if d.get("world_x") and d.get("world_y")),
        key=lambda d: float(d["elapsed_s"]),
    )

    per_target = []
    discovery_times = []
    per_event_latencies = []  # detection-after-spawn, dynamic schedule only
    any_spawn_t_seen = False

    for tgt in targets:
        tx = float(tgt["x"])
        ty = float(tgt["y"])
        tclass = tgt["class"].strip().lower()
        # Manifest may carry a spawn_t (scheduled mode); 0/empty means t=0.
        spawn_raw = tgt.get("spawn_t", "")
        try:
            spawn_t = float(spawn_raw) if spawn_raw not in ("", None) else 0.0
        except ValueError:
            spawn_t = 0.0
        if spawn_t > 0:
            any_spawn_t_seen = True

        first_t = None
        for det in sorted_dets:
            if det["class_name"].strip().lower() != tclass:
                continue
            det_t = float(det["elapsed_s"])
            # Detections before the target was spawned can't be of this target,
            # even if a coincident lookalike sat at the same location.
            if det_t < spawn_t:
                continue
            dx = float(det["world_x"]) - tx
            dy = float(det["world_y"]) - ty
            if math.hypot(dx, dy) <= match_radius_m:
                first_t = det_t
                break

        latency = (first_t - spawn_t) if first_t is not None else None
        per_target.append({
            "name": tgt.get("name", ""),
            "class": tclass,
            "spawn_t_s": round(spawn_t, 3),
            "found": first_t is not None,
            "first_detection_s": round(first_t, 3) if first_t is not None else None,
            "latency_s": round(latency, 3) if latency is not None else None,
        })
        if first_t is not None:
            discovery_times.append(first_t)
            per_event_latencies.append(max(0.0, latency))

    targets_total = len(targets)
    targets_found = len(discovery_times)
    all_found = targets_found == targets_total and targets_found > 0
    mean_latency = (
        round(float(np.mean(per_event_latencies)), 3)
        if per_event_latencies else None
    )
    median_latency = (
        round(float(np.median(per_event_latencies)), 3)
        if per_event_latencies else None
    )
    max_latency = (
        round(float(np.max(per_event_latencies)), 3)
        if per_event_latencies else None
    )

    return {
        "time_to_first_target_s": round(min(discovery_times), 3) if discovery_times else None,
        "time_to_all_targets_s": round(max(discovery_times), 3) if all_found else None,
        "targets_found": targets_found,
        "targets_total": targets_total,
        "per_target_discovery": per_target,
        "per_event_latency_s": [round(l, 3) for l in per_event_latencies],
        "mean_event_latency_s": mean_latency,
        "median_event_latency_s": median_latency,
        "max_event_latency_s": max_latency,
        "schedule_aware": any_spawn_t_seen,
        "match_radius_m": match_radius_m,
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


def compute_target_separation(targets: list[dict]) -> dict:
    """Pairwise separation stats across ground-truth targets.

    `min_pairwise_target_distance_m` is the key post-hoc grouping variable:
    trials where the two closest targets are < detection_sigma apart fall in
    the "near" regime (where adaptive's density overlap actually concentrates
    drones), while trials above that threshold are the "far" regime where
    adaptive should behave like a lawnmower on coverage but still win on
    event-response latency.
    """
    if not targets or len(targets) < 2:
        return {
            "min_pairwise_target_distance_m": None,
            "mean_pairwise_target_distance_m": None,
            "max_pairwise_target_distance_m": None,
            "n_targets": len(targets) if targets else 0,
        }

    pts = []
    for t in targets:
        try:
            pts.append((float(t["x"]), float(t["y"])))
        except (KeyError, TypeError, ValueError):
            continue

    if len(pts) < 2:
        return {
            "min_pairwise_target_distance_m": None,
            "mean_pairwise_target_distance_m": None,
            "max_pairwise_target_distance_m": None,
            "n_targets": len(pts),
        }

    dists = []
    for i in range(len(pts)):
        for j in range(i + 1, len(pts)):
            dists.append(math.hypot(pts[i][0] - pts[j][0], pts[i][1] - pts[j][1]))

    return {
        "min_pairwise_target_distance_m": round(min(dists), 2),
        "mean_pairwise_target_distance_m": round(float(np.mean(dists)), 2),
        "max_pairwise_target_distance_m": round(max(dists), 2),
        "n_targets": len(pts),
    }


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
    parser.add_argument(
        "--steady-state-cutoff", type=float, default=20.0,
        help="Elapsed-time boundary (s) between steady-state and event phases. "
             "Default 20s matches the v2 schedule (first event at t=20).",
    )
    args = parser.parse_args()

    log_dir = os.path.join(args.log_base, args.experiment_id)
    if not os.path.isdir(log_dir):
        print(f"ERROR: Log directory not found: {log_dir}")
        sys.exit(1)

    print(f"Loading data from: {log_dir}")
    positions = load_csv(os.path.join(log_dir, "positions.csv"))
    detections = load_csv(os.path.join(log_dir, "detections.csv"))
    mode_events = load_csv(os.path.join(log_dir, "swarm_mode.csv"))
    targets = load_csv(os.path.join(log_dir, "targets.csv"))

    print(f"  Positions: {len(positions)} rows")
    print(f"  Detections: {len(detections)} rows")
    print(f"  Mode events: {len(mode_events)} rows")
    print(f"  Targets:    {len(targets)} rows")

    # Compute metrics
    duration = compute_experiment_duration(positions)
    coverage = compute_coverage(positions, args.area_size, args.altitude, args.grid_resolution)
    distance = compute_energy(positions)
    distance_phases = compute_distance_phases(positions, args.steady_state_cutoff)
    detection_metrics = compute_detection_metrics(detections)
    event_coverage = compute_coverage_during_event(
        positions, mode_events, args.area_size, args.altitude, args.grid_resolution
    )
    response_time = compute_response_time(log_dir)

    n_drones = len(distance["per_drone_distance"]) or 1
    energy = compute_energy_and_emissions(duration, n_drones, distance["total_distance_m"])
    discovery = compute_target_discovery_times(detections, targets, duration)

    # Inter-target separation — for the "close vs far" post-hoc analysis.
    # Computed once here so every metrics_summary.json carries the trial's
    # minimum and mean pairwise distance. Analysis scripts can group trials
    # into near/far strata without re-reading targets.csv.
    sep = compute_target_separation(targets)

    # Build summary
    summary = {
        "experiment_id": args.experiment_id,
        "duration_s": duration,
        "area_size_m": args.area_size,
        "altitude_m": args.altitude,
        "n_drones": n_drones,
        "coverage_percent": coverage["coverage_percent"],
        "redundancy_ratio": coverage["redundancy_ratio"],
        "total_distance_m": distance["total_distance_m"],
        "per_drone_distance_m": distance["per_drone_distance"],
        "steady_state_distance_m": distance_phases["steady_state_distance_m"],
        "event_phase_distance_m": distance_phases["event_phase_distance_m"],
        "steady_state_cutoff_s": distance_phases["steady_state_cutoff_s"],
        "hover_power_w": energy["hover_power_w"],
        "forward_power_w": energy["forward_power_w"],
        "comm_power_w": energy["comm_power_w"],
        "cruise_speed_ms": energy["cruise_speed_ms"],
        "hover_energy_wh": energy["hover_energy_wh"],
        "forward_energy_wh": energy["forward_energy_wh"],
        "comm_energy_wh": energy["comm_energy_wh"],
        "energy_per_drone_wh": energy["energy_per_drone_wh"],
        "total_energy_wh": energy["total_energy_wh"],
        "total_energy_kwh": energy["total_energy_kwh"],
        "co2_azerbaijan_g": energy["co2_azerbaijan_g"],
        "co2_eu_g": energy["co2_eu_g"],
        "min_pairwise_target_distance_m": sep["min_pairwise_target_distance_m"],
        "mean_pairwise_target_distance_m": sep["mean_pairwise_target_distance_m"],
        "max_pairwise_target_distance_m": sep["max_pairwise_target_distance_m"],
        "total_detections": detection_metrics["total_detections"],
        "first_detection_s": detection_metrics["first_detection_s"],
        "detections_per_drone": detection_metrics["detections_per_drone"],
        "unique_classes": detection_metrics["unique_classes"],
        "targets_total": discovery["targets_total"],
        "targets_found": discovery["targets_found"],
        "time_to_first_target_s": discovery["time_to_first_target_s"],
        "time_to_all_targets_s": discovery["time_to_all_targets_s"],
        "target_match_radius_m": discovery["match_radius_m"],
        "per_target_discovery": discovery["per_target_discovery"],
        "per_event_latency_s": discovery["per_event_latency_s"],
        "mean_event_latency_s": discovery["mean_event_latency_s"],
        "median_event_latency_s": discovery["median_event_latency_s"],
        "max_event_latency_s": discovery["max_event_latency_s"],
        "schedule_aware": discovery["schedule_aware"],
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
    print(f"  Total distance:{distance['total_distance_m']:.1f}m")
    print(f"   ↳ steady (<{distance_phases['steady_state_cutoff_s']:.0f}s): "
          f"{distance_phases['steady_state_distance_m']:.1f}m")
    print(f"   ↳ event   (≥{distance_phases['steady_state_cutoff_s']:.0f}s): "
          f"{distance_phases['event_phase_distance_m']:.1f}m")
    print(f"  Energy:        {energy['total_energy_wh']:.2f} Wh "
          f"(hover {energy['hover_energy_wh']:.2f} + fwd "
          f"{energy['forward_energy_wh']:.2f} + comm {energy['comm_energy_wh']:.2f})")
    print(f"  CO2 (AZ/EU):   {energy['co2_azerbaijan_g']:.2f} g / "
          f"{energy['co2_eu_g']:.2f} g")
    if sep["min_pairwise_target_distance_m"] is not None:
        print(f"  Target sep:    min={sep['min_pairwise_target_distance_m']:.1f}m "
              f"mean={sep['mean_pairwise_target_distance_m']:.1f}m "
              f"max={sep['max_pairwise_target_distance_m']:.1f}m")
    print(f"  Detections:    {detection_metrics['total_detections']}")
    print(f"  First detect:  {detection_metrics['first_detection_s']}s")
    print(f"  Targets found: {discovery['targets_found']}/{discovery['targets_total']}")
    ttf = discovery["time_to_first_target_s"]
    tta = discovery["time_to_all_targets_s"]
    print(f"  First target:  {ttf if ttf is not None else 'n/a'} s")
    print(f"  All targets:   {tta if tta is not None else 'n/a'} s")
    if discovery["schedule_aware"]:
        mel = discovery["mean_event_latency_s"]
        medl = discovery["median_event_latency_s"]
        maxl = discovery["max_event_latency_s"]
        per_lat = discovery["per_event_latency_s"]
        print(f"  Per-event lat: {per_lat} s")
        print(f"  Latency stats: mean={mel} median={medl} max={maxl} s "
              f"(detection − spawn)")
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
