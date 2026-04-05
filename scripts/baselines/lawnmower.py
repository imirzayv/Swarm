#!/usr/bin/env python3
"""
Baseline: Lawnmower — drones sweep back-and-forth in assigned strips.

Divides the monitoring area into N vertical strips. Each drone flies a
back-and-forth lawnmower pattern within its strip. Loops forever.

Usage:
    source ~/Desktop/Swarm/ros2_ws/install/setup.bash
    python3 baselines/lawnmower.py --drones 1 2 3 --area-size 40 --altitude 30
"""

import argparse
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

import rclpy
from rclpy.node import Node
from swarm_msgs.msg import TargetWaypoint

from voronoi_utils import gps_to_xy, xy_to_gps


def generate_lawnmower_path(strip_x_min: float, strip_x_max: float,
                            y_min: float, y_max: float,
                            spacing: float = 8.0) -> list[tuple[float, float]]:
    """Generate a lawnmower path within a rectangular strip.

    Args:
        strip_x_min, strip_x_max: X bounds of this drone's strip.
        y_min, y_max: Y bounds of the monitoring area.
        spacing: Distance between sweeps in X direction.

    Returns:
        List of (x, y) waypoints forming the lawnmower pattern.
    """
    waypoints = []
    x = strip_x_min + spacing / 2
    forward = True  # sweep direction in Y

    while x < strip_x_max:
        if forward:
            waypoints.append((x, y_min + 1))
            waypoints.append((x, y_max - 1))
        else:
            waypoints.append((x, y_max - 1))
            waypoints.append((x, y_min + 1))
        forward = not forward
        x += spacing

    return waypoints


class LawnmowerNode(Node):
    """Publishes sequential lawnmower waypoints for each drone."""

    def __init__(self, drone_ids: list[int], area_size: float, altitude: float,
                 speed: float):
        super().__init__("lawnmower_baseline")

        self.drone_ids = drone_ids
        self.altitude = altitude

        half = area_size / 2
        n = len(drone_ids)
        strip_width = area_size / n

        # Generate paths per drone
        self.paths: dict[int, list[tuple[float, float]]] = {}
        self.path_idx: dict[int, int] = {}
        self.wp_publishers = {}

        for i, did in enumerate(drone_ids):
            x_min = -half + i * strip_width
            x_max = x_min + strip_width
            path = generate_lawnmower_path(x_min, x_max, -half, half)
            self.paths[did] = path
            self.path_idx[did] = 0

            self.wp_publishers[did] = self.create_publisher(
                TargetWaypoint, f"/drone{did}/target_waypoint", 10
            )
            self.get_logger().info(
                f"Drone {did}: strip x=[{x_min:.0f}, {x_max:.0f}]m, "
                f"{len(path)} waypoints"
            )

        # Subscribe to positions to detect arrival
        self._subs = []
        self._drone_pos: dict[int, tuple[float, float]] = {}
        for did in drone_ids:
            self._subs.append(self.create_subscription(
                TargetWaypoint, f"/drone{did}/position",
                lambda msg, d=did: self._position_cb(msg, d), 10,
            ))

        # Check for arrival and advance waypoints every 2 seconds
        self.create_timer(2.0, self._advance_waypoints)
        # Publish current targets every 3 seconds
        self.create_timer(3.0, self._publish_current)

        self.get_logger().info(f"Lawnmower baseline ready — {n} drones")

    def _position_cb(self, msg: TargetWaypoint, drone_id: int):
        x, y = gps_to_xy(msg.latitude, msg.longitude)
        self._drone_pos[drone_id] = (x, y)

    def _advance_waypoints(self):
        """Check if drones have reached current waypoint; advance if so."""
        arrival_threshold = 5.0  # meters

        for did in self.drone_ids:
            if did not in self._drone_pos:
                continue

            path = self.paths[did]
            idx = self.path_idx[did]
            target_x, target_y = path[idx]
            drone_x, drone_y = self._drone_pos[did]

            dist = ((drone_x - target_x) ** 2 + (drone_y - target_y) ** 2) ** 0.5
            if dist < arrival_threshold:
                # Advance to next waypoint (loop)
                self.path_idx[did] = (idx + 1) % len(path)
                next_x, next_y = path[self.path_idx[did]]
                self.get_logger().info(
                    f"[Drone {did}] Arrived at wp {idx}, advancing to {self.path_idx[did]} "
                    f"({next_x:.0f}, {next_y:.0f})m"
                )

    def _publish_current(self):
        """Publish current target waypoint for each drone."""
        for did in self.drone_ids:
            path = self.paths[did]
            idx = self.path_idx[did]
            x, y = path[idx]
            lat, lon = xy_to_gps(x, y)

            wp = TargetWaypoint()
            wp.drone_id = did
            wp.latitude = lat
            wp.longitude = lon
            wp.altitude = self.altitude
            wp.priority = 0
            self.wp_publishers[did].publish(wp)


def main():
    parser = argparse.ArgumentParser(description="Lawnmower baseline controller")
    parser.add_argument("--drones", type=int, nargs="+", default=[1, 2, 3])
    parser.add_argument("--area-size", type=float, default=40.0)
    parser.add_argument("--altitude", type=float, default=30.0)
    parser.add_argument("--speed", type=float, default=5.0, help="Drone speed (m/s)")
    args = parser.parse_args()

    rclpy.init()
    node = LawnmowerNode(args.drones, args.area_size, args.altitude, args.speed)
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
