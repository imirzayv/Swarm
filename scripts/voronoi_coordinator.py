#!/usr/bin/env python3
"""
Voronoi Coordinator: Core adaptive area monitoring algorithm.

Subscribes to drone positions and detections, computes a confidence-weighted
Voronoi partition of the monitoring area, and publishes target waypoints.

Novel elements implemented:
  1. Confidence x class_priority weighted density function
  2. Class-specific formation responses (person/vehicle/fire)
  3. Dual-mode exploration/exploitation with split-and-reform

Algorithm:
  EXPLORE mode (default):
    - All drones participate in weighted Voronoi coverage
    - Density map = uniform base + Gaussian bumps weighted by (conf x priority)
    - Every update cycle: compute bounded Voronoi -> publish centroids as waypoints

  EXPLOIT mode (triggered by high-confidence detection):
    - K nearest drones assigned to exploit set -> fly class-specific formation
    - Remaining drones continue Voronoi coverage (explore set)
    - Reform after exploit_timeout_s or when confidence drops below threshold

Usage:
    source ~/Desktop/Swarm/ros2_ws/install/setup.bash
    python3 voronoi_coordinator.py --drones 1 2 3
    python3 voronoi_coordinator.py --drones 1 2 3 4 5 --area-size 200
"""

import argparse
import csv
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import rclpy
from rclpy.node import Node
from swarm_msgs.msg import Detection, TargetWaypoint, SwarmMode
import numpy as np

from voronoi_utils import (
    DensityMap,
    compute_bounded_voronoi,
    gps_to_xy,
    xy_to_gps,
    weighted_centroid,
    polygon_centroid,
)
from response_selector import ResponseSelector


class ExploitEvent:
    """Tracks an active exploitation event."""

    def __init__(self, class_name: str, det_x: float, det_y: float,
                 confidence: float, exploit_drone_ids: list[int],
                 formation_waypoints: dict[int, tuple[float, float]],
                 timeout: float, t_detect: float, t_publish: float):
        self.class_name = class_name
        self.det_x = det_x
        self.det_y = det_y
        self.confidence = confidence
        self.exploit_drone_ids = exploit_drone_ids
        self.formation_waypoints = formation_waypoints
        self.start_time = time.time()
        self.timeout = timeout
        self.last_confidence = confidence
        self.t_detect = t_detect
        self.t_publish = t_publish
        self.t_arrive: float | None = None
        self.arrival_checked = False

    def is_expired(self, confidence_drop_threshold: float) -> bool:
        """Check if exploit event should end (timeout or confidence drop)."""
        elapsed = time.time() - self.start_time
        if elapsed >= self.timeout:
            return True
        if self.last_confidence < confidence_drop_threshold:
            return True
        return False


class VoronoiCoordinatorNode(Node):
    """ROS 2 node that computes adaptive Voronoi coverage with dual-mode logic."""

    def __init__(self, drone_ids: list[int], area_size: float,
                 update_rate: float, altitude: float,
                 detection_weight: float, detection_sigma: float,
                 decay_half_life: float, max_cluster: int,
                 config_path: str | None = None,
                 exploit_confidence_threshold: float = 0.4,
                 exploit_timeout: float = 30.0,
                 exploit_confidence_drop: float = 0.2,
                 min_explore_drones: int = 1):
        super().__init__("voronoi_coordinator")

        self.drone_ids = drone_ids
        self.altitude = altitude
        self.max_cluster = max_cluster
        self.exploit_confidence_threshold = exploit_confidence_threshold
        self.exploit_timeout = exploit_timeout
        self.exploit_confidence_drop = exploit_confidence_drop
        self.min_explore_drones = min_explore_drones

        # Monitoring area bounds centered on origin
        half = area_size / 2
        self.bounds = (-half, -half, half, half)

        # Drone positions in local XY
        self.drone_positions: dict[int, tuple[float, float]] = {}

        # Density map for adaptive reallocation (confidence-weighted)
        self.density_map = DensityMap(
            baseline=1.0,
            detection_sigma=detection_sigma,
            detection_weight=detection_weight,
            decay_half_life=decay_half_life,
            confidence_weighted=True,
        )

        # Response selector for class-specific formations
        self.response_selector = ResponseSelector(config_path=config_path)

        # Active exploit event (None = all drones exploring)
        self.active_exploit: ExploitEvent | None = None
        self.arrival_threshold = 5.0  # meters — consider drone "arrived" if within this

        # Reconfiguration log (for E4 response time analysis)
        self._reconfig_log: list[dict] = []

        # Waypoint publishers
        self.wp_publishers = {}
        self._subs = []

        # SwarmMode publisher
        self.mode_publisher = self.create_publisher(SwarmMode, "/swarm/mode", 10)

        for did in drone_ids:
            sub_pos = self.create_subscription(
                TargetWaypoint,
                f"/drone{did}/position",
                lambda msg, d=did: self._position_callback(msg, d),
                10,
            )
            self._subs.append(sub_pos)

            sub_det = self.create_subscription(
                Detection,
                f"/drone{did}/detection",
                lambda msg, d=did: self._detection_callback(msg, d),
                10,
            )
            self._subs.append(sub_det)

            pub = self.create_publisher(TargetWaypoint, f"/drone{did}/target_waypoint", 10)
            self.wp_publishers[did] = pub

        # Periodic Voronoi update
        period = 1.0 / update_rate
        self.create_timer(period, self._update_voronoi)

        self.get_logger().info(
            f"Voronoi coordinator ready — {len(drone_ids)} drones, "
            f"area={area_size}m, update={update_rate}Hz, "
            f"det_weight={detection_weight}, sigma={detection_sigma}m, "
            f"decay={decay_half_life}s, max_cluster={max_cluster}, "
            f"exploit_threshold={exploit_confidence_threshold}, "
            f"exploit_timeout={exploit_timeout}s"
        )

    def _position_callback(self, msg: TargetWaypoint, drone_id: int):
        """Update drone's local XY position from GPS."""
        x, y = gps_to_xy(msg.latitude, msg.longitude)
        self.drone_positions[drone_id] = (x, y)

    def _detection_callback(self, msg: Detection, drone_id: int):
        """Add a detection event to the density map and check for exploit trigger."""
        if drone_id not in self.drone_positions:
            self.get_logger().warn(
                f"[Drone {drone_id}] Detection received but no position known — skipping"
            )
            return

        drone_x, drone_y = self.drone_positions[drone_id]
        det_x = drone_x + msg.world_position[0]
        det_y = drone_y + msg.world_position[1]

        # Get class priority
        class_priority = self.response_selector.get_priority(msg.class_name)

        # Add to density map with confidence and class priority
        self.density_map.add_detection(
            det_x, det_y,
            confidence=msg.confidence,
            class_priority=class_priority,
            class_name=msg.class_name,
        )

        self.get_logger().info(
            f"[Detection] {msg.class_name} ({msg.confidence:.2f}, priority={class_priority}) "
            f"from drone {drone_id} at global ({det_x:.1f}, {det_y:.1f})m — "
            f"active detections: {len(self.density_map.detections)}"
        )

        # Check if this detection should trigger exploit mode
        t_detect = time.time()
        if (self.active_exploit is None
                and msg.confidence >= self.exploit_confidence_threshold):
            self._enter_exploit_mode(msg.class_name, det_x, det_y, msg.confidence, t_detect)
        elif self.active_exploit is not None:
            # Update confidence of active exploit event if same area
            dist_to_event = ((det_x - self.active_exploit.det_x) ** 2 +
                             (det_y - self.active_exploit.det_y) ** 2) ** 0.5
            if dist_to_event < 20.0:  # within 20m of current exploit event
                self.active_exploit.last_confidence = max(
                    self.active_exploit.last_confidence, msg.confidence
                )

    def _enter_exploit_mode(self, class_name: str, det_x: float, det_y: float,
                            confidence: float, t_detect: float):
        """Assign nearest drones to exploit formation, rest continue exploring."""
        exploit_ids = self.response_selector.select_exploit_drones(
            class_name, det_x, det_y, self.drone_positions,
            min_explore=self.min_explore_drones,
        )

        if not exploit_ids:
            self.get_logger().warn(
                f"Cannot enter exploit mode — not enough drones (need to keep "
                f"{self.min_explore_drones} exploring)"
            )
            return

        formation_wps = self.response_selector.compute_formation(
            class_name, det_x, det_y, exploit_ids,
        )

        t_publish = time.time()

        self.active_exploit = ExploitEvent(
            class_name=class_name,
            det_x=det_x,
            det_y=det_y,
            confidence=confidence,
            exploit_drone_ids=exploit_ids,
            formation_waypoints=formation_wps,
            timeout=self.exploit_timeout,
            t_detect=t_detect,
            t_publish=t_publish,
        )

        explore_ids = [d for d in self.drone_ids if d not in exploit_ids]

        self.get_logger().info(
            f"[EXPLOIT] {class_name} at ({det_x:.1f}, {det_y:.1f})m conf={confidence:.2f} — "
            f"exploit drones: {exploit_ids}, explore drones: {explore_ids}"
        )

        # Publish SwarmMode
        self._publish_mode(SwarmMode.MODE_EXPLOIT, exploit_ids, class_name,
                           det_x, det_y, confidence)

    def _exit_exploit_mode(self, reason: str):
        """Return all drones to explore mode and log reconfiguration timing."""
        evt = self.active_exploit
        if evt is not None:
            reconfig_entry = {
                "timestamp": time.time(),
                "event_class": evt.class_name,
                "event_x": evt.det_x,
                "event_y": evt.det_y,
                "confidence": evt.confidence,
                "t_detect": evt.t_detect,
                "t_publish": evt.t_publish,
                "t_arrive": evt.t_arrive,
                "reconfig_time_s": (evt.t_arrive - evt.t_detect) if evt.t_arrive else None,
                "publish_delay_s": evt.t_publish - evt.t_detect,
                "exploit_drones": evt.exploit_drone_ids,
                "reason": reason,
            }
            self._reconfig_log.append(reconfig_entry)

            arrive_str = f"{evt.t_arrive - evt.t_detect:.2f}s" if evt.t_arrive else "N/A"
            self.get_logger().info(
                f"[RECONFIG] {evt.class_name} — publish_delay={evt.t_publish - evt.t_detect:.3f}s, "
                f"arrive_delay={arrive_str}, reason={reason}"
            )

        self.get_logger().info(
            f"[EXPLORE] Returning to explore mode — reason: {reason}"
        )
        self.active_exploit = None
        self._publish_mode(SwarmMode.MODE_EXPLORE, [], "", 0.0, 0.0, 0.0)

    def _publish_mode(self, mode: int, exploit_ids: list[int], event_class: str,
                      event_x: float, event_y: float, confidence: float):
        """Publish SwarmMode message."""
        msg = SwarmMode()
        msg.mode = mode
        msg.exploit_drone_ids = [int(d) for d in exploit_ids]
        msg.event_class = event_class
        msg.event_position = [event_x, event_y, 0.0]
        msg.event_confidence = confidence
        msg.stamp = self.get_clock().now().to_msg()
        self.mode_publisher.publish(msg)

    def _update_voronoi(self):
        """Compute Voronoi partition and publish waypoints (dual-mode)."""
        if len(self.drone_positions) < len(self.drone_ids):
            return

        # Prune old detections
        self.density_map.prune_expired(max_age=120.0)

        # Check if exploit drones have arrived at formation positions
        if (self.active_exploit is not None
                and not self.active_exploit.arrival_checked):
            all_arrived = True
            for did in self.active_exploit.exploit_drone_ids:
                if did not in self.drone_positions or did not in self.active_exploit.formation_waypoints:
                    all_arrived = False
                    break
                px, py = self.drone_positions[did]
                tx, ty = self.active_exploit.formation_waypoints[did]
                dist = ((px - tx) ** 2 + (py - ty) ** 2) ** 0.5
                if dist > self.arrival_threshold:
                    all_arrived = False
                    break
            if all_arrived:
                self.active_exploit.t_arrive = time.time()
                self.active_exploit.arrival_checked = True
                reconfig_time = self.active_exploit.t_arrive - self.active_exploit.t_detect
                self.get_logger().info(
                    f"[ARRIVE] All exploit drones arrived — reconfig_time={reconfig_time:.2f}s"
                )

        # Check if exploit event has expired
        if self.active_exploit is not None:
            if self.active_exploit.is_expired(self.exploit_confidence_drop):
                elapsed = time.time() - self.active_exploit.start_time
                if elapsed >= self.active_exploit.timeout:
                    self._exit_exploit_mode("timeout")
                else:
                    self._exit_exploit_mode("confidence_drop")

        # Determine which drones are exploring vs exploiting
        if self.active_exploit is not None:
            exploit_ids = set(self.active_exploit.exploit_drone_ids)
            explore_ids = [d for d in self.drone_ids if d not in exploit_ids]
        else:
            exploit_ids = set()
            explore_ids = list(self.drone_ids)

        # ── Exploit drones: fly class-specific formation ────────────────────
        if self.active_exploit is not None:
            for did in self.active_exploit.exploit_drone_ids:
                if did in self.active_exploit.formation_waypoints:
                    tx, ty = self.active_exploit.formation_waypoints[did]
                    lat, lon = xy_to_gps(tx, ty)
                    wp = TargetWaypoint()
                    wp.drone_id = did
                    wp.latitude = lat
                    wp.longitude = lon
                    wp.altitude = self.altitude
                    wp.priority = 2  # high priority for exploit
                    self.wp_publishers[did].publish(wp)

        # ── Explore drones: weighted Voronoi coverage ───────────────────────
        if len(explore_ids) == 0:
            return

        points = np.array([self.drone_positions[did] for did in explore_ids])
        cells = compute_bounded_voronoi(points, self.bounds)

        has_detections = len(self.density_map.detections) > 0

        for i, did in enumerate(explore_ids):
            if has_detections:
                cx, cy = weighted_centroid(cells[i], self.density_map)
            else:
                cx, cy = polygon_centroid(cells[i])

            # Clip centroid to bounds
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
        description="Voronoi-based adaptive area monitoring coordinator"
    )
    parser.add_argument(
        "--drones", type=int, nargs="+", default=[1, 2, 3],
        help="Drone IDs (default: 1 2 3)",
    )
    parser.add_argument(
        "--area-size", type=float, default=200.0,
        help="Monitoring area side length in meters (default: 200)",
    )
    parser.add_argument(
        "--update-rate", type=float, default=0.5,
        help="Voronoi update frequency in Hz (default: 0.5 = every 2s)",
    )
    parser.add_argument(
        "--altitude", type=float, default=40.0,
        help="Target altitude for waypoints in meters (default: 40)",
    )
    parser.add_argument(
        "--detection-weight", type=float, default=10.0,
        help="Weight of detection Gaussian bumps (default: 10)",
    )
    parser.add_argument(
        "--detection-sigma", type=float, default=15.0,
        help="Gaussian spread of detections in meters (default: 15)",
    )
    parser.add_argument(
        "--decay-half-life", type=float, default=30.0,
        help="Detection decay half-life in seconds (default: 30)",
    )
    parser.add_argument(
        "--max-cluster", type=int, default=2,
        help="Max drones converging on one detection (default: 2)",
    )
    parser.add_argument(
        "--config", type=str, default=None,
        help="Path to adaptive_params.yaml config file",
    )
    parser.add_argument(
        "--exploit-threshold", type=float, default=0.4,
        help="Min confidence to trigger exploit mode (default: 0.4)",
    )
    parser.add_argument(
        "--exploit-timeout", type=float, default=30.0,
        help="Exploit mode timeout in seconds (default: 30)",
    )
    parser.add_argument(
        "--exploit-confidence-drop", type=float, default=0.2,
        help="Reform if confidence drops below this (default: 0.2)",
    )
    parser.add_argument(
        "--min-explore-drones", type=int, default=1,
        help="Minimum drones that must remain exploring (default: 1)",
    )
    args = parser.parse_args()

    # Try to find config file automatically
    config_path = args.config
    if config_path is None:
        script_dir = os.path.dirname(os.path.abspath(__file__))
        project_dir = os.path.dirname(script_dir)
        default_config = os.path.join(project_dir, "config", "adaptive_params.yaml")
        if os.path.exists(default_config):
            config_path = default_config

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
        config_path=config_path,
        exploit_confidence_threshold=args.exploit_threshold,
        exploit_timeout=args.exploit_timeout,
        exploit_confidence_drop=args.exploit_confidence_drop,
        min_explore_drones=args.min_explore_drones,
    )

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        node.get_logger().info("Coordinator shutting down")
    finally:
        # Save reconfiguration log if any events recorded
        if node._reconfig_log:
            script_dir = os.path.dirname(os.path.abspath(__file__))
            project_dir = os.path.dirname(script_dir)
            reconfig_dir = os.path.join(project_dir, "data", "logs")
            os.makedirs(reconfig_dir, exist_ok=True)
            reconfig_path = os.path.join(reconfig_dir, "reconfig_events.csv")
            write_header = not os.path.exists(reconfig_path)
            with open(reconfig_path, "a", newline="") as f:
                writer = csv.writer(f)
                if write_header:
                    writer.writerow([
                        "timestamp", "event_class", "event_x", "event_y",
                        "confidence", "t_detect", "t_publish", "t_arrive",
                        "reconfig_time_s", "publish_delay_s",
                        "exploit_drones", "reason",
                    ])
                for entry in node._reconfig_log:
                    writer.writerow([
                        f"{entry['timestamp']:.3f}",
                        entry["event_class"],
                        f"{entry['event_x']:.2f}",
                        f"{entry['event_y']:.2f}",
                        f"{entry['confidence']:.4f}",
                        f"{entry['t_detect']:.3f}",
                        f"{entry['t_publish']:.3f}",
                        f"{entry['t_arrive']:.3f}" if entry["t_arrive"] else "",
                        f"{entry['reconfig_time_s']:.3f}" if entry["reconfig_time_s"] else "",
                        f"{entry['publish_delay_s']:.3f}",
                        ";".join(str(d) for d in entry["exploit_drones"]),
                        entry["reason"],
                    ])
            node.get_logger().info(
                f"Saved {len(node._reconfig_log)} reconfig events to {reconfig_path}"
            )

        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
