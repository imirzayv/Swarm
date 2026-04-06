#!/usr/bin/env python3
"""
Ablation Baseline: Binary Voronoi — Voronoi with binary (unweighted) density.

Same Voronoi coverage algorithm as the full adaptive system, but the density
function uses binary detection weights (1.0 for any detection, regardless of
confidence or class priority). This matches the approach used in existing
literature (e.g., CVT flood monitoring).

This ablation isolates the contribution of Novel Element 2:
confidence-weighted density function.

Usage:
    source ~/Desktop/Swarm/ros2_ws/install/setup.bash
    python3 baselines/binary_voronoi.py --drones 1 2 3 --area-size 200
"""

import argparse
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

import rclpy
from rclpy.node import Node
from swarm_msgs.msg import Detection, TargetWaypoint
import numpy as np

from voronoi_utils import (
    DensityMap,
    compute_bounded_voronoi,
    gps_to_xy,
    xy_to_gps,
    weighted_centroid,
    polygon_centroid,
)


class BinaryVoronoiNode(Node):
    """Voronoi coordinator with binary (unweighted) density — ablation baseline."""

    def __init__(self, drone_ids: list[int], area_size: float,
                 update_rate: float, altitude: float,
                 detection_weight: float, detection_sigma: float,
                 decay_half_life: float):
        super().__init__("binary_voronoi_baseline")

        self.drone_ids = drone_ids
        self.altitude = altitude

        half = area_size / 2
        self.bounds = (-half, -half, half, half)

        self.drone_positions: dict[int, tuple[float, float]] = {}

        # Binary density map: confidence_weighted=False
        # Any detection gets weight 1.0 regardless of confidence or class
        self.density_map = DensityMap(
            baseline=1.0,
            detection_sigma=detection_sigma,
            detection_weight=detection_weight,
            decay_half_life=decay_half_life,
            confidence_weighted=False,  # KEY DIFFERENCE: binary mode
        )

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
        self.create_timer(period, self._update_voronoi)

        self.get_logger().info(
            f"Binary Voronoi baseline ready — {len(drone_ids)} drones, "
            f"area={area_size}m (confidence_weighted=False)"
        )

    def _position_cb(self, msg: TargetWaypoint, drone_id: int):
        x, y = gps_to_xy(msg.latitude, msg.longitude)
        self.drone_positions[drone_id] = (x, y)

    def _detection_cb(self, msg: Detection, drone_id: int):
        if drone_id not in self.drone_positions:
            return

        drone_x, drone_y = self.drone_positions[drone_id]
        det_x = drone_x + msg.world_position[0]
        det_y = drone_y + msg.world_position[1]

        # Binary mode: confidence and class_priority are stored but ignored
        # in density evaluation (weight is always 1.0)
        self.density_map.add_detection(
            det_x, det_y,
            confidence=msg.confidence,
            class_priority=1.0,
            class_name=msg.class_name,
        )

        self.get_logger().info(
            f"[Binary Detection] {msg.class_name} ({msg.confidence:.2f}) "
            f"from drone {drone_id} at ({det_x:.1f}, {det_y:.1f})m — "
            f"weight=1.0 (binary)"
        )

    def _update_voronoi(self):
        """All drones do Voronoi coverage with binary density (no split-and-reform)."""
        if len(self.drone_positions) < len(self.drone_ids):
            return

        self.density_map.prune_expired(max_age=120.0)

        points = np.array([self.drone_positions[did] for did in self.drone_ids])
        cells = compute_bounded_voronoi(points, self.bounds)

        has_detections = len(self.density_map.detections) > 0

        for i, did in enumerate(self.drone_ids):
            if has_detections:
                cx, cy = weighted_centroid(cells[i], self.density_map)
            else:
                cx, cy = polygon_centroid(cells[i])

            x_min, y_min, x_max, y_max = self.bounds
            cx = float(np.clip(cx, x_min, x_max))
            cy = float(np.clip(cy, y_min, y_max))

            lat, lon = xy_to_gps(cx, cy)

            wp = TargetWaypoint()
            wp.drone_id = did
            wp.latitude = lat
            wp.longitude = lon
            wp.altitude = self.altitude
            wp.priority = 1 if has_detections else 0
            self.wp_publishers[did].publish(wp)


def main():
    parser = argparse.ArgumentParser(
        description="Binary Voronoi ablation baseline (no confidence weighting)"
    )
    parser.add_argument("--drones", type=int, nargs="+", default=[1, 2, 3])
    parser.add_argument("--area-size", type=float, default=200.0)
    parser.add_argument("--update-rate", type=float, default=0.5)
    parser.add_argument("--altitude", type=float, default=40.0)
    parser.add_argument("--detection-weight", type=float, default=10.0)
    parser.add_argument("--detection-sigma", type=float, default=15.0)
    parser.add_argument("--decay-half-life", type=float, default=30.0)
    args = parser.parse_args()

    rclpy.init()
    node = BinaryVoronoiNode(
        drone_ids=args.drones,
        area_size=args.area_size,
        update_rate=args.update_rate,
        altitude=args.altitude,
        detection_weight=args.detection_weight,
        detection_sigma=args.detection_sigma,
        decay_half_life=args.decay_half_life,
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
