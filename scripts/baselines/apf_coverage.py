#!/usr/bin/env python3
"""
Baseline: APF (Artificial Potential Field) — force-field coverage controller.

Each drone experiences virtual forces:
  - Repulsive from other drones (to spread out)
  - Attractive from uncovered areas (grid-based pull)
  - Attractive from detection locations (confidence-scaled)
  - Boundary repulsion at area edges

The net force vector determines each drone's next waypoint displacement.

Detection-reactive: detection locations become attractive force sources with
strength proportional to detection confidence.

Usage:
    source ~/Desktop/Swarm/ros2_ws/install/setup.bash
    python3 baselines/apf_coverage.py --drones 1 2 3 --area-size 200
"""

import argparse
import math
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

import rclpy
from rclpy.node import Node
from swarm_msgs.msg import Detection, TargetWaypoint
import numpy as np

from voronoi_utils import gps_to_xy, xy_to_gps


class APFCoverageNode(Node):
    """Artificial Potential Field coverage controller for drone swarm."""

    def __init__(self, drone_ids: list[int], area_size: float,
                 update_rate: float, altitude: float,
                 repulsion_gain: float, attraction_gain: float,
                 detection_gain: float, boundary_gain: float,
                 max_step: float, grid_resolution: float):
        super().__init__("apf_coverage_baseline")

        self.drone_ids = drone_ids
        self.altitude = altitude
        self.n = len(drone_ids)

        half = area_size / 2
        self.bounds = (-half, -half, half, half)
        self.area_size = area_size

        # APF parameters
        self.k_rep = repulsion_gain
        self.k_att = attraction_gain
        self.k_det = detection_gain
        self.k_bnd = boundary_gain
        self.max_step = max_step

        self.drone_positions: dict[int, tuple[float, float]] = {}

        # Coverage grid to track visited areas
        self.grid_res = grid_resolution
        nx = int(area_size / grid_resolution)
        ny = int(area_size / grid_resolution)
        self.coverage_grid = np.zeros((nx, ny), dtype=np.float32)
        self.grid_nx = nx
        self.grid_ny = ny

        # Detection events
        self.detections: list[tuple[float, float, float, float]] = []  # (x, y, confidence, timestamp)
        self.detection_decay = 30.0

        self.wp_publishers = {}
        self._subs = []

        for did in drone_ids:
            self._subs.append(self.create_subscription(
                TargetWaypoint, f"/drone{did}/position",
                lambda msg, d=did: self._position_cb(msg, d), 10,
            ))
            self._subs.append(self.create_subscription(
                Detection, f"/drone{did}/detection",
                lambda msg, d=did: self._detection_cb(msg, d), 10,
            ))
            self.wp_publishers[did] = self.create_publisher(
                TargetWaypoint, f"/drone{did}/target_waypoint", 10,
            )

        period = 1.0 / update_rate
        self.create_timer(period, self._apf_step)

        self.get_logger().info(
            f"APF coverage baseline ready — {self.n} drones, "
            f"area={area_size}m, k_rep={repulsion_gain}, k_att={attraction_gain}, "
            f"k_det={detection_gain}"
        )

    def _position_cb(self, msg: TargetWaypoint, drone_id: int):
        x, y = gps_to_xy(msg.latitude, msg.longitude)
        self.drone_positions[drone_id] = (x, y)
        self._update_coverage(x, y)

    def _detection_cb(self, msg: Detection, drone_id: int):
        if drone_id not in self.drone_positions:
            return
        drone_x, drone_y = self.drone_positions[drone_id]
        det_x = drone_x + msg.world_position[0]
        det_y = drone_y + msg.world_position[1]
        self.detections.append((det_x, det_y, msg.confidence, time.time()))
        self.get_logger().info(
            f"[APF Detection] {msg.class_name} ({msg.confidence:.2f}) "
            f"from drone {drone_id} at ({det_x:.1f}, {det_y:.1f})m"
        )

    def _update_coverage(self, x: float, y: float):
        """Mark grid cells within FOV as covered."""
        fov_radius = self.altitude * math.tan(1.74 / 2.0)
        x_min, y_min, _, _ = self.bounds
        gi = int((x - x_min) / self.grid_res)
        gj = int((y - y_min) / self.grid_res)
        r_cells = int(fov_radius / self.grid_res) + 1

        for di in range(-r_cells, r_cells + 1):
            for dj in range(-r_cells, r_cells + 1):
                ni, nj = gi + di, gj + dj
                if 0 <= ni < self.grid_nx and 0 <= nj < self.grid_ny:
                    cell_x = x_min + (ni + 0.5) * self.grid_res
                    cell_y = y_min + (nj + 0.5) * self.grid_res
                    dist = math.sqrt((x - cell_x) ** 2 + (y - cell_y) ** 2)
                    if dist <= fov_radius:
                        self.coverage_grid[ni, nj] = min(
                            self.coverage_grid[ni, nj] + 0.1, 1.0
                        )

    def _compute_forces(self, drone_id: int) -> np.ndarray:
        """Compute the net virtual force on a drone."""
        px, py = self.drone_positions[drone_id]
        pos = np.array([px, py])
        force = np.zeros(2)
        x_min, y_min, x_max, y_max = self.bounds

        # 1. Repulsive forces from other drones
        for other_id in self.drone_ids:
            if other_id == drone_id:
                continue
            if other_id not in self.drone_positions:
                continue
            ox, oy = self.drone_positions[other_id]
            diff = pos - np.array([ox, oy])
            dist = np.linalg.norm(diff)
            if dist < 1.0:
                dist = 1.0
            # Inverse-square repulsion
            force += self.k_rep * diff / (dist ** 3)

        # 2. Attractive force toward uncovered areas
        # Sample nearby uncovered cells and compute pull
        gi = int((px - x_min) / self.grid_res)
        gj = int((py - y_min) / self.grid_res)
        r = 8  # look-ahead radius in grid cells
        att_force = np.zeros(2)
        for di in range(-r, r + 1):
            for dj in range(-r, r + 1):
                ni, nj = gi + di, gj + dj
                if 0 <= ni < self.grid_nx and 0 <= nj < self.grid_ny:
                    uncovered = 1.0 - self.coverage_grid[ni, nj]
                    if uncovered < 0.1:
                        continue
                    cell_x = x_min + (ni + 0.5) * self.grid_res
                    cell_y = y_min + (nj + 0.5) * self.grid_res
                    diff = np.array([cell_x, cell_y]) - pos
                    dist = np.linalg.norm(diff)
                    if dist < 1.0:
                        dist = 1.0
                    att_force += uncovered * diff / (dist ** 2)
        force += self.k_att * att_force

        # 3. Attractive force from detection locations
        now = time.time()
        for det_x, det_y, conf, ts in self.detections:
            age = now - ts
            if age > 120.0:
                continue
            decay = 0.5 ** (age / self.detection_decay)
            diff = np.array([det_x, det_y]) - pos
            dist = np.linalg.norm(diff)
            if dist < 1.0:
                dist = 1.0
            force += self.k_det * conf * decay * diff / (dist ** 2)

        # 4. Boundary repulsion (push away from edges)
        margin = 5.0
        if px - x_min < margin:
            force[0] += self.k_bnd / max(px - x_min, 0.5) ** 2
        if x_max - px < margin:
            force[0] -= self.k_bnd / max(x_max - px, 0.5) ** 2
        if py - y_min < margin:
            force[1] += self.k_bnd / max(py - y_min, 0.5) ** 2
        if y_max - py < margin:
            force[1] -= self.k_bnd / max(y_max - py, 0.5) ** 2

        return force

    def _apf_step(self):
        """One APF iteration: compute forces, update positions."""
        if len(self.drone_positions) < self.n:
            return

        # Prune old detections
        now = time.time()
        self.detections = [(x, y, c, t) for x, y, c, t in self.detections
                           if now - t < 120.0]

        x_min, y_min, x_max, y_max = self.bounds

        for did in self.drone_ids:
            force = self._compute_forces(did)

            # Limit step size
            magnitude = np.linalg.norm(force)
            if magnitude > self.max_step:
                force = force * (self.max_step / magnitude)

            px, py = self.drone_positions[did]
            new_x = float(np.clip(px + force[0], x_min, x_max))
            new_y = float(np.clip(py + force[1], y_min, y_max))

            lat, lon = xy_to_gps(new_x, new_y)

            wp = TargetWaypoint()
            wp.drone_id = did
            wp.latitude = lat
            wp.longitude = lon
            wp.altitude = self.altitude
            wp.priority = 1 if self.detections else 0
            self.wp_publishers[did].publish(wp)


def main():
    parser = argparse.ArgumentParser(
        description="APF (Artificial Potential Field) coverage baseline controller"
    )
    parser.add_argument("--drones", type=int, nargs="+", default=[1, 2, 3])
    parser.add_argument("--area-size", type=float, default=200.0)
    parser.add_argument("--update-rate", type=float, default=0.5)
    parser.add_argument("--altitude", type=float, default=40.0)
    parser.add_argument("--repulsion-gain", type=float, default=500.0,
                        help="Repulsion gain from other drones")
    parser.add_argument("--attraction-gain", type=float, default=0.5,
                        help="Attraction gain toward uncovered areas")
    parser.add_argument("--detection-gain", type=float, default=20.0,
                        help="Attraction gain toward detections")
    parser.add_argument("--boundary-gain", type=float, default=50.0,
                        help="Boundary repulsion gain")
    parser.add_argument("--max-step", type=float, default=15.0,
                        help="Maximum displacement per step (meters)")
    parser.add_argument("--grid-resolution", type=float, default=5.0,
                        help="Coverage grid cell size (meters)")
    args = parser.parse_args()

    rclpy.init()
    node = APFCoverageNode(
        drone_ids=args.drones,
        area_size=args.area_size,
        update_rate=args.update_rate,
        altitude=args.altitude,
        repulsion_gain=args.repulsion_gain,
        attraction_gain=args.attraction_gain,
        detection_gain=args.detection_gain,
        boundary_gain=args.boundary_gain,
        max_step=args.max_step,
        grid_resolution=args.grid_resolution,
    )
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
