#!/usr/bin/env python3
"""
Baseline: Lawnmower — drones sweep back-and-forth in assigned strips.

Divides the monitoring area into N vertical strips. Each drone flies a
back-and-forth lawnmower pattern within its strip, with sweep spacing
derived from camera HFOV and flight altitude so adjacent strips tile the
plane without wasted overlap. Runs until every drone has visited its last
waypoint, then publishes Bool(True) on /lawnmower/complete and shuts down.

Usage:
    source ~/Desktop/Swarm/ros2_ws/install/setup.bash
    python3 baselines/lawnmower.py --drones 1 2 3 --area-size 200 --altitude 40
"""

import argparse
import math
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import Bool
from swarm_msgs.msg import TargetWaypoint

from voronoi_utils import gps_to_xy, xy_to_gps


def compute_fov_spacing(altitude: float, hfov_rad: float,
                        overlap_factor: float = 0.95) -> float:
    """Sweep spacing that makes adjacent camera footprints tile the ground.

    Ground footprint width of a nadir camera is 2*h*tan(HFOV/2). Scaling
    it by overlap_factor<1 leaves a small overlap between strips so the
    seams are sealed without wasting flight time on wide re-coverage.
    """
    return 2.0 * altitude * math.tan(hfov_rad / 2.0) * overlap_factor


def generate_lawnmower_path(strip_x_min: float, strip_x_max: float,
                            y_min: float, y_max: float,
                            spacing: float) -> list[tuple[float, float]]:
    """Generate a lawnmower path within a rectangular strip.

    Adapts the number of sweeps so they exactly tile the strip width
    (rounding the count up, then spreading evenly). If the strip is
    narrower than one footprint, places a single centered sweep.
    """
    strip_width = strip_x_max - strip_x_min
    if strip_width <= spacing:
        x_positions = [0.5 * (strip_x_min + strip_x_max)]
    else:
        n_sweeps = max(1, int(math.ceil(strip_width / spacing)))
        actual_spacing = strip_width / n_sweeps
        x_positions = [
            strip_x_min + actual_spacing * (i + 0.5) for i in range(n_sweeps)
        ]

    waypoints: list[tuple[float, float]] = []
    forward = True
    for x in x_positions:
        if forward:
            waypoints.append((x, y_min + 1))
            waypoints.append((x, y_max - 1))
        else:
            waypoints.append((x, y_max - 1))
            waypoints.append((x, y_min + 1))
        forward = not forward

    return waypoints


def estimate_duration(paths: dict[int, list[tuple[float, float]]],
                      altitude: float, speed: float, vertical_speed: float) -> float:
    """Rough mission-time estimate (takeoff + longest drone's XY path)."""
    if speed <= 0:
        return 0.0
    longest = 0.0
    for path in paths.values():
        total = 0.0
        for a, b in zip(path[:-1], path[1:]):
            total += math.hypot(b[0] - a[0], b[1] - a[1])
        longest = max(longest, total)
    takeoff = altitude / max(vertical_speed, 0.1)
    return takeoff + longest / speed


class LawnmowerNode(Node):
    """Publishes sequential lawnmower waypoints for each drone."""

    def __init__(self, drone_ids: list[int], area_size: float, altitude: float,
                 speed: float, vertical_speed: float, hfov_rad: float,
                 overlap_factor: float):
        super().__init__("lawnmower_baseline")

        self.drone_ids = drone_ids
        self.altitude = altitude

        half = area_size / 2
        n = len(drone_ids)
        strip_width = area_size / n
        spacing = compute_fov_spacing(altitude, hfov_rad, overlap_factor)

        self.get_logger().info(
            f"FOV-aware spacing: altitude={altitude:.1f}m, "
            f"HFOV={math.degrees(hfov_rad):.1f}°, "
            f"overlap={overlap_factor:.2f} → spacing={spacing:.2f}m "
            f"(strip_width={strip_width:.1f}m)"
        )

        self.paths: dict[int, list[tuple[float, float]]] = {}
        self.path_idx: dict[int, int] = {}
        self.path_complete: dict[int, bool] = {}
        self.wp_publishers = {}

        for i, did in enumerate(drone_ids):
            x_min = -half + i * strip_width
            x_max = x_min + strip_width
            path = generate_lawnmower_path(x_min, x_max, -half, half, spacing)
            self.paths[did] = path
            self.path_idx[did] = 0
            self.path_complete[did] = False

            self.wp_publishers[did] = self.create_publisher(
                TargetWaypoint, f"/drone{did}/target_waypoint", 10
            )
            self.get_logger().info(
                f"Drone {did}: strip x=[{x_min:.1f}, {x_max:.1f}]m, "
                f"{len(path)} waypoints"
            )

        est = estimate_duration(self.paths, altitude, speed, vertical_speed)
        self.get_logger().info(
            f"Estimated mission duration: {est:.0f}s "
            f"(airspeed={speed:.1f} m/s, vertical={vertical_speed:.1f} m/s)"
        )

        # Latch completion so a late-joining subscriber still sees it.
        complete_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.complete_pub = self.create_publisher(
            Bool, "/lawnmower/complete", complete_qos
        )
        self._shutdown_requested = False

        self._subs = []
        self._drone_pos: dict[int, tuple[float, float]] = {}
        for did in drone_ids:
            self._subs.append(self.create_subscription(
                TargetWaypoint, f"/drone{did}/position",
                lambda msg, d=did: self._position_cb(msg, d), 10,
            ))

        self.create_timer(2.0, self._advance_waypoints)
        self.create_timer(3.0, self._publish_current)

        self.get_logger().info(f"Lawnmower baseline ready — {n} drones")

    def _position_cb(self, msg: TargetWaypoint, drone_id: int):
        x, y = gps_to_xy(msg.latitude, msg.longitude)
        self._drone_pos[drone_id] = (x, y)

    def _advance_waypoints(self):
        """Check if drones have reached current waypoint; advance or finish."""
        arrival_threshold = 5.0  # meters

        for did in self.drone_ids:
            if self.path_complete[did] or did not in self._drone_pos:
                continue

            path = self.paths[did]
            idx = self.path_idx[did]
            target_x, target_y = path[idx]
            drone_x, drone_y = self._drone_pos[did]

            dist = math.hypot(drone_x - target_x, drone_y - target_y)
            if dist < arrival_threshold:
                if idx + 1 >= len(path):
                    self.path_complete[did] = True
                    self.get_logger().info(
                        f"[Drone {did}] Path complete ({len(path)} waypoints)"
                    )
                else:
                    self.path_idx[did] = idx + 1
                    next_x, next_y = path[self.path_idx[did]]
                    self.get_logger().info(
                        f"[Drone {did}] Arrived at wp {idx}, "
                        f"advancing to {self.path_idx[did]} "
                        f"({next_x:.0f}, {next_y:.0f})m"
                    )

        if not self._shutdown_requested and all(self.path_complete.values()):
            self._signal_completion()

    def _signal_completion(self):
        self.get_logger().info(
            "All drones completed lawnmower path — signaling /lawnmower/complete"
        )
        msg = Bool()
        msg.data = True
        for _ in range(5):
            self.complete_pub.publish(msg)
            time.sleep(0.1)
        self._shutdown_requested = True
        # Give the publish a moment to flush before tearing down.
        self.create_timer(1.0, self._shutdown)

    def _shutdown(self):
        try:
            rclpy.shutdown()
        except Exception:
            pass

    def _publish_current(self):
        """Publish current target waypoint for each drone still in progress."""
        for did in self.drone_ids:
            if self.path_complete[did]:
                continue
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
    parser.add_argument("--speed", type=float, default=5.0,
                        help="Horizontal cruise airspeed (m/s); MPC_XY_CRUISE default is 5")
    parser.add_argument("--vertical-speed", type=float, default=1.0,
                        help="Vertical takeoff/climb speed (m/s); MPC_Z_VEL_MAX_UP default is 1")
    parser.add_argument("--hfov-rad", type=float, default=1.74,
                        help="Camera horizontal FOV in radians (x500_mono_cam_down default)")
    parser.add_argument("--overlap-factor", type=float, default=0.95,
                        help="Sweep spacing = footprint * overlap_factor; <1 leaves a small overlap")
    args = parser.parse_args()

    rclpy.init()
    node = LawnmowerNode(
        args.drones, args.area_size, args.altitude,
        args.speed, args.vertical_speed, args.hfov_rad, args.overlap_factor,
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
