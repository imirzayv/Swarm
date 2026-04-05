#!/usr/bin/env python3
"""
Baseline: Static Grid — drones hover at fixed grid positions.

Divides the monitoring area into N equal rectangular cells and assigns
each drone to hover at the center of its cell. No adaptation to detections.

Usage:
    source ~/Desktop/Swarm/ros2_ws/install/setup.bash
    python3 baselines/static_grid.py --drones 1 2 3 --area-size 40 --altitude 30
"""

import argparse
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

import rclpy
from rclpy.node import Node
from swarm_msgs.msg import TargetWaypoint

from voronoi_utils import xy_to_gps


def compute_grid_positions(n_drones: int, area_size: float) -> list[tuple[float, float]]:
    """Compute grid cell centers for N drones in a square area.

    Arranges drones in a grid pattern that covers the area as evenly as possible.
    """
    half = area_size / 2
    cols = math.ceil(math.sqrt(n_drones))
    rows = math.ceil(n_drones / cols)

    cell_w = area_size / cols
    cell_h = area_size / rows

    positions = []
    for i in range(n_drones):
        row = i // cols
        col = i % cols
        x = -half + cell_h * (row + 0.5)
        y = -half + cell_w * (col + 0.5)
        positions.append((x, y))

    return positions


class StaticGridNode(Node):
    """Publishes fixed grid waypoints at a low rate."""

    def __init__(self, drone_ids: list[int], area_size: float, altitude: float):
        super().__init__("static_grid_baseline")

        self.drone_ids = drone_ids
        self.altitude = altitude

        positions = compute_grid_positions(len(drone_ids), area_size)

        # Pre-compute GPS waypoints
        self.waypoints = {}
        self.wp_publishers = {}
        for i, did in enumerate(drone_ids):
            x, y = positions[i]
            lat, lon = xy_to_gps(x, y)
            self.waypoints[did] = (lat, lon)
            self.wp_publishers[did] = self.create_publisher(
                TargetWaypoint, f"/drone{did}/target_waypoint", 10
            )
            self.get_logger().info(
                f"Drone {did}: grid position ({x:.1f}, {y:.1f})m → "
                f"({lat:.6f}, {lon:.6f})"
            )

        # Publish waypoints every 5 seconds (drones just hold position)
        self.create_timer(5.0, self._publish_waypoints)
        self.get_logger().info(f"Static grid baseline ready — {len(drone_ids)} drones")

    def _publish_waypoints(self):
        for did in self.drone_ids:
            lat, lon = self.waypoints[did]
            wp = TargetWaypoint()
            wp.drone_id = did
            wp.latitude = lat
            wp.longitude = lon
            wp.altitude = self.altitude
            wp.priority = 0
            self.wp_publishers[did].publish(wp)


def main():
    parser = argparse.ArgumentParser(description="Static grid baseline controller")
    parser.add_argument("--drones", type=int, nargs="+", default=[1, 2, 3])
    parser.add_argument("--area-size", type=float, default=40.0)
    parser.add_argument("--altitude", type=float, default=30.0)
    args = parser.parse_args()

    rclpy.init()
    node = StaticGridNode(args.drones, args.area_size, args.altitude)
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
