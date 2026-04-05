#!/usr/bin/env python3
"""
Data Logger: ROS 2 node that records drone positions and detections to CSV files.

Subscribes to:
  /droneN/position       → positions.csv
  /droneN/detection      → detections.csv
  /droneN/target_waypoint → waypoints.csv

Output directory: data/logs/<experiment_id>/

Usage:
    source ~/Desktop/Swarm/ros2_ws/install/setup.bash
    python3 data_logger.py --drones 1 2 3 --experiment-id e1_adaptive_t01
    python3 data_logger.py --drones 1 2 3 --experiment-id test --duration 120
"""

import argparse
import csv
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import rclpy
from rclpy.node import Node
from swarm_msgs.msg import Detection, TargetWaypoint

from voronoi_utils import gps_to_xy


class DataLoggerNode(Node):
    """ROS 2 node that logs experiment data to CSV files."""

    def __init__(self, drone_ids: list[int], output_dir: str, duration: float):
        super().__init__("data_logger")

        self.drone_ids = drone_ids
        self.output_dir = output_dir
        self.start_time = time.time()
        self.duration = duration

        os.makedirs(output_dir, exist_ok=True)

        # Open CSV files
        self._pos_file = open(os.path.join(output_dir, "positions.csv"), "w", newline="")
        self._det_file = open(os.path.join(output_dir, "detections.csv"), "w", newline="")
        self._wp_file = open(os.path.join(output_dir, "waypoints.csv"), "w", newline="")

        self._pos_writer = csv.writer(self._pos_file)
        self._det_writer = csv.writer(self._det_file)
        self._wp_writer = csv.writer(self._wp_file)

        # Write headers
        self._pos_writer.writerow([
            "timestamp", "elapsed_s", "drone_id", "latitude", "longitude",
            "altitude", "x_local", "y_local",
        ])
        self._det_writer.writerow([
            "timestamp", "elapsed_s", "drone_id", "class_name", "confidence",
            "bbox_x1", "bbox_y1", "bbox_x2", "bbox_y2",
            "world_x", "world_y",
        ])
        self._wp_writer.writerow([
            "timestamp", "elapsed_s", "drone_id", "latitude", "longitude",
            "altitude", "priority", "x_local", "y_local",
        ])

        self._pos_count = 0
        self._det_count = 0
        self._wp_count = 0

        # Subscribe to all drone topics
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
            self._subs.append(self.create_subscription(
                TargetWaypoint, f"/drone{did}/target_waypoint",
                lambda msg, d=did: self._waypoint_cb(msg, d), 10,
            ))

        # Status timer (every 10s)
        self.create_timer(10.0, self._status_cb)

        # Duration timer (if set)
        if duration > 0:
            self.create_timer(duration, self._duration_expired)

        self.get_logger().info(
            f"Data logger ready — {len(drone_ids)} drones, "
            f"output: {output_dir}, "
            f"duration: {'unlimited' if duration <= 0 else f'{duration}s'}"
        )

    def _elapsed(self) -> float:
        return time.time() - self.start_time

    def _position_cb(self, msg: TargetWaypoint, drone_id: int):
        x, y = gps_to_xy(msg.latitude, msg.longitude)
        self._pos_writer.writerow([
            time.time(), f"{self._elapsed():.2f}", drone_id,
            f"{msg.latitude:.8f}", f"{msg.longitude:.8f}", f"{msg.altitude:.2f}",
            f"{x:.2f}", f"{y:.2f}",
        ])
        self._pos_count += 1

    def _detection_cb(self, msg: Detection, drone_id: int):
        self._det_writer.writerow([
            time.time(), f"{self._elapsed():.2f}", drone_id,
            msg.class_name, f"{msg.confidence:.4f}",
            f"{msg.bbox[0]:.1f}", f"{msg.bbox[1]:.1f}",
            f"{msg.bbox[2]:.1f}", f"{msg.bbox[3]:.1f}",
            f"{msg.world_position[0]:.2f}", f"{msg.world_position[1]:.2f}",
        ])
        self._det_count += 1

    def _waypoint_cb(self, msg: TargetWaypoint, drone_id: int):
        x, y = gps_to_xy(msg.latitude, msg.longitude)
        self._wp_writer.writerow([
            time.time(), f"{self._elapsed():.2f}", drone_id,
            f"{msg.latitude:.8f}", f"{msg.longitude:.8f}", f"{msg.altitude:.2f}",
            msg.priority, f"{x:.2f}", f"{y:.2f}",
        ])
        self._wp_count += 1

    def _status_cb(self):
        elapsed = self._elapsed()
        self.get_logger().info(
            f"[{elapsed:.0f}s] Logged: {self._pos_count} positions, "
            f"{self._det_count} detections, {self._wp_count} waypoints"
        )
        # Flush files periodically
        self._pos_file.flush()
        self._det_file.flush()
        self._wp_file.flush()

    def _duration_expired(self):
        self.get_logger().info(f"Duration ({self.duration}s) reached — stopping logger")
        self._close_files()
        raise SystemExit(0)

    def _close_files(self):
        self._pos_file.close()
        self._det_file.close()
        self._wp_file.close()
        self.get_logger().info(
            f"Final counts: {self._pos_count} positions, "
            f"{self._det_count} detections, {self._wp_count} waypoints"
        )
        self.get_logger().info(f"Data saved to: {self.output_dir}")

    def destroy_node(self):
        self._close_files()
        super().destroy_node()


def main():
    parser = argparse.ArgumentParser(description="Log experiment data to CSV")
    parser.add_argument(
        "--drones", type=int, nargs="+", default=[1, 2, 3],
        help="Drone IDs (default: 1 2 3)",
    )
    parser.add_argument(
        "--experiment-id", type=str, default="test",
        help="Experiment identifier for output directory name",
    )
    parser.add_argument(
        "--output-base", type=str,
        default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "data", "logs"),
        help="Base directory for log output",
    )
    parser.add_argument(
        "--duration", type=float, default=0,
        help="Auto-stop after N seconds (0 = run until Ctrl+C)",
    )
    args = parser.parse_args()

    output_dir = os.path.join(args.output_base, args.experiment_id)

    rclpy.init()
    node = DataLoggerNode(
        drone_ids=args.drones,
        output_dir=output_dir,
        duration=args.duration,
    )

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
