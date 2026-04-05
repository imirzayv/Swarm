#!/usr/bin/env python3
"""
Voronoi Coordinator: Core adaptive area monitoring algorithm.

Subscribes to drone positions and detections, computes a weighted Voronoi
partition of the monitoring area, and publishes target waypoints.

Algorithm:
  1. Maintain a density map: uniform baseline + Gaussian bumps at detection sites
  2. Every update cycle: compute bounded Voronoi cells for current drone positions
  3. Find each cell's density-weighted centroid (attracts drones toward detections)
  4. Convert centroids to GPS and publish as TargetWaypoint messages
  5. Detection bumps decay over time, so drones gradually return to uniform coverage

Usage:
    source ~/Desktop/Swarm/ros2_ws/install/setup.bash
    python3 voronoi_coordinator.py --drones 1 2 3
    python3 voronoi_coordinator.py --drones 1 2 3 --area-size 50 --update-rate 0.5
"""

import argparse
import sys
import os

# Add scripts directory to path for voronoi_utils import
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

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


class VoronoiCoordinatorNode(Node):
    """ROS 2 node that computes adaptive Voronoi coverage and publishes waypoints."""

    def __init__(self, drone_ids: list[int], area_size: float,
                 update_rate: float, altitude: float,
                 detection_weight: float, detection_sigma: float,
                 decay_half_life: float, max_cluster: int):
        super().__init__("voronoi_coordinator")

        self.drone_ids = drone_ids
        self.altitude = altitude
        self.max_cluster = max_cluster

        # Monitoring area bounds centered on origin (symmetric)
        half = area_size / 2
        self.bounds = (-half, -half, half, half)

        # Drone positions in local XY (updated by position subscribers)
        self.drone_positions: dict[int, tuple[float, float]] = {}

        # Density map for adaptive reallocation
        self.density_map = DensityMap(
            baseline=1.0,
            detection_sigma=detection_sigma,
            detection_weight=detection_weight,
            decay_half_life=decay_half_life,
        )

        # Waypoint publishers
        self.wp_publishers = {}
        self._subs = []

        for did in drone_ids:
            # Subscribe to positions (published by waypoint_executor)
            sub_pos = self.create_subscription(
                TargetWaypoint,
                f"/drone{did}/position",
                lambda msg, d=did: self._position_callback(msg, d),
                10,
            )
            self._subs.append(sub_pos)

            # Subscribe to detections (published by detection_publisher)
            sub_det = self.create_subscription(
                Detection,
                f"/drone{did}/detection",
                lambda msg, d=did: self._detection_callback(msg, d),
                10,
            )
            self._subs.append(sub_det)

            # Publish waypoints
            pub = self.create_publisher(TargetWaypoint, f"/drone{did}/target_waypoint", 10)
            self.wp_publishers[did] = pub

        # Periodic Voronoi update
        period = 1.0 / update_rate
        self.create_timer(period, self._update_voronoi)

        self.get_logger().info(
            f"Voronoi coordinator ready — {len(drone_ids)} drones, "
            f"area={area_size}m, update={update_rate}Hz, "
            f"det_weight={detection_weight}, sigma={detection_sigma}m, "
            f"decay={decay_half_life}s, max_cluster={max_cluster}"
        )

    def _position_callback(self, msg: TargetWaypoint, drone_id: int):
        """Update drone's local XY position from GPS."""
        x, y = gps_to_xy(msg.latitude, msg.longitude)
        self.drone_positions[drone_id] = (x, y)

    def _detection_callback(self, msg: Detection, drone_id: int):
        """Add a detection event to the density map."""
        # The detection's world_position is relative to the drone.
        # Translate to global XY using the drone's current position.
        if drone_id not in self.drone_positions:
            self.get_logger().warn(
                f"[Drone {drone_id}] Detection received but no position known — skipping"
            )
            return

        drone_x, drone_y = self.drone_positions[drone_id]
        det_x = drone_x + msg.world_position[0]
        det_y = drone_y + msg.world_position[1]

        self.density_map.add_detection(det_x, det_y)
        self.get_logger().info(
            f"[Detection] {msg.class_name} ({msg.confidence:.2f}) from drone {drone_id} "
            f"at global ({det_x:.1f}, {det_y:.1f})m — "
            f"active detections: {len(self.density_map.detections)}"
        )

    def _update_voronoi(self):
        """Compute Voronoi partition and publish waypoints."""
        # Need positions from all drones
        if len(self.drone_positions) < len(self.drone_ids):
            return

        # Prune old detections
        self.density_map.prune_expired(max_age=120.0)

        # Current positions as array
        points = np.array([self.drone_positions[did] for did in self.drone_ids])

        # Compute bounded Voronoi cells
        cells = compute_bounded_voronoi(points, self.bounds)

        # Compute centroids (weighted if detections exist, uniform otherwise)
        has_detections = len(self.density_map.detections) > 0

        for i, did in enumerate(self.drone_ids):
            if has_detections:
                cx, cy = weighted_centroid(cells[i], self.density_map)
            else:
                cx, cy = polygon_centroid(cells[i])

            # Clip centroid to bounds
            x_min, y_min, x_max, y_max = self.bounds
            cx = float(np.clip(cx, x_min, x_max))
            cy = float(np.clip(cy, y_min, y_max))

            # Convert to GPS
            lat, lon = xy_to_gps(cx, cy)

            # Publish waypoint
            wp = TargetWaypoint()
            wp.drone_id = did
            wp.latitude = lat
            wp.longitude = lon
            wp.altitude = self.altitude
            wp.priority = 1 if has_detections else 0
            self.wp_publishers[did].publish(wp)


def main():
    parser = argparse.ArgumentParser(
        description="Voronoi-based adaptive area monitoring coordinator"
    )
    parser.add_argument(
        "--drones", type=int, nargs="+", default=[1, 2, 3],
        help="Drone IDs (default: 1 2 3)",
    )
    parser.add_argument(
        "--area-size", type=float, default=40.0,
        help="Monitoring area side length in meters (default: 40)",
    )
    parser.add_argument(
        "--update-rate", type=float, default=0.5,
        help="Voronoi update frequency in Hz (default: 0.5 = every 2s)",
    )
    parser.add_argument(
        "--altitude", type=float, default=30.0,
        help="Target altitude for waypoints in meters (default: 30)",
    )
    parser.add_argument(
        "--detection-weight", type=float, default=10.0,
        help="Weight of detection Gaussian bumps (default: 10)",
    )
    parser.add_argument(
        "--detection-sigma", type=float, default=5.0,
        help="Gaussian spread of detections in meters (default: 5)",
    )
    parser.add_argument(
        "--decay-half-life", type=float, default=30.0,
        help="Detection decay half-life in seconds (default: 30)",
    )
    parser.add_argument(
        "--max-cluster", type=int, default=2,
        help="Max drones converging on one detection (default: 2)",
    )
    args = parser.parse_args()

    rclpy.init()
    node = VoronoiCoordinatorNode(
        drone_ids=args.drones,
        area_size=args.area_size,
        update_rate=args.update_rate,
        altitude=args.altitude,
        detection_weight=args.detection_weight,
        detection_sigma=args.detection_sigma,
        decay_half_life=args.decay_half_life,
        max_cluster=args.max_cluster,
    )

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        node.get_logger().info("Coordinator shutting down")
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
