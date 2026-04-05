#!/usr/bin/env python3
"""
Baseline: Random Waypoints — drones fly to random positions within the area.

Each drone picks a random point in the monitoring area, flies there,
and on arrival picks a new random point. No coordination between drones.

Usage:
    source ~/Desktop/Swarm/ros2_ws/install/setup.bash
    python3 baselines/random_waypoints.py --drones 1 2 3 --area-size 40 --altitude 30
"""

import argparse
import os
import random
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

import rclpy
from rclpy.node import Node
from swarm_msgs.msg import TargetWaypoint

from voronoi_utils import gps_to_xy, xy_to_gps


class RandomWaypointsNode(Node):
    """Publishes random waypoints; advances on arrival."""

    def __init__(self, drone_ids: list[int], area_size: float, altitude: float,
                 seed: int):
        super().__init__("random_waypoints_baseline")

        self.drone_ids = drone_ids
        self.altitude = altitude
        self.half = area_size / 2
        self.rng = random.Random(seed)

        # Current targets and publishers
        self.targets: dict[int, tuple[float, float]] = {}
        self.wp_publishers = {}
        self._drone_pos: dict[int, tuple[float, float]] = {}
        self._subs = []

        for did in drone_ids:
            # Initial random target
            self.targets[did] = self._random_point()

            self.wp_publishers[did] = self.create_publisher(
                TargetWaypoint, f"/drone{did}/target_waypoint", 10
            )

            self._subs.append(self.create_subscription(
                TargetWaypoint, f"/drone{did}/position",
                lambda msg, d=did: self._position_cb(msg, d), 10,
            ))

            x, y = self.targets[did]
            self.get_logger().info(f"Drone {did}: initial target ({x:.0f}, {y:.0f})m")

        # Check arrival and publish every 2 seconds
        self.create_timer(2.0, self._advance_and_publish)

        self.get_logger().info(
            f"Random waypoints baseline ready — {len(drone_ids)} drones, seed={seed}"
        )

    def _random_point(self) -> tuple[float, float]:
        """Generate a random point within the monitoring area."""
        x = self.rng.uniform(-self.half, self.half)
        y = self.rng.uniform(-self.half, self.half)
        return (x, y)

    def _position_cb(self, msg: TargetWaypoint, drone_id: int):
        x, y = gps_to_xy(msg.latitude, msg.longitude)
        self._drone_pos[drone_id] = (x, y)

    def _advance_and_publish(self):
        arrival_threshold = 5.0

        for did in self.drone_ids:
            # Check arrival
            if did in self._drone_pos:
                dx, dy = self._drone_pos[did]
                tx, ty = self.targets[did]
                dist = ((dx - tx) ** 2 + (dy - ty) ** 2) ** 0.5
                if dist < arrival_threshold:
                    new_target = self._random_point()
                    self.targets[did] = new_target
                    self.get_logger().info(
                        f"[Drone {did}] Arrived, new target: "
                        f"({new_target[0]:.0f}, {new_target[1]:.0f})m"
                    )

            # Publish current target
            x, y = self.targets[did]
            lat, lon = xy_to_gps(x, y)
            wp = TargetWaypoint()
            wp.drone_id = did
            wp.latitude = lat
            wp.longitude = lon
            wp.altitude = self.altitude
            wp.priority = 0
            self.wp_publishers[did].publish(wp)


def main():
    parser = argparse.ArgumentParser(description="Random waypoints baseline controller")
    parser.add_argument("--drones", type=int, nargs="+", default=[1, 2, 3])
    parser.add_argument("--area-size", type=float, default=40.0)
    parser.add_argument("--altitude", type=float, default=30.0)
    parser.add_argument("--seed", type=int, default=42, help="Random seed")
    args = parser.parse_args()

    rclpy.init()
    node = RandomWaypointsNode(args.drones, args.area_size, args.altitude, args.seed)
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
