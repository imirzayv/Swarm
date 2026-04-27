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
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import Bool
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


class ConfirmedTarget:
    """A persistent record of a physical target inferred from detections.

    Identity is (class_name, spatial cluster). Detections from any drone
    within `merge_radius` of this target's centroid and matching class are
    folded in — they update the running mean position and bump the
    detection count instead of creating a new entry. A target is treated
    as `confirmed` once `n_detections` reaches `min_confirmations`; below
    that it is a noisy *candidate* (filters single-frame YOLO false
    positives).
    """

    def __init__(self, x: float, y: float, class_name: str,
                 confidence: float, t_now: float):
        self.x = x
        self.y = y
        self.class_name = class_name
        self.peak_confidence = confidence
        self.first_seen_t = t_now
        self.last_seen_t = t_now
        self.n_detections = 1
        self.confirmed_at_t: float | None = None
        # Wall-clock time before which this target should not retrigger
        # an exploit. Refreshed each time an exploit on this target ends.
        self.cooldown_until_t: float = 0.0

    def update(self, x: float, y: float, confidence: float, t_now: float):
        # Running mean — gives more stable position estimate as evidence grows.
        n = self.n_detections
        self.x = (self.x * n + x) / (n + 1)
        self.y = (self.y * n + y) / (n + 1)
        self.peak_confidence = max(self.peak_confidence, confidence)
        self.last_seen_t = t_now
        self.n_detections += 1


class TargetRegistry:
    """Persistent list of unique physical targets seen across all drones.

    A detection from drone B at the same world location as drone A's
    earlier detection updates the existing entry rather than creating a
    new one. This is the cross-drone deduplication the swarm needs to
    avoid double-counting the same target.

    Confirmation gating (`min_confirmations`) protects against
    single-frame false positives — a candidate must be reinforced K times
    before it counts toward `confirmed_count()`. K=3 is a sensible
    default for YOLO-class detectors.
    """

    def __init__(self, merge_radius: float = 8.0,
                 min_confirmations: int = 3):
        self.merge_radius = merge_radius
        self.min_confirmations = min_confirmations
        self.targets: list[ConfirmedTarget] = []
        self.last_confirmed_t: float = 0.0

    def ingest(self, x: float, y: float, class_name: str,
               confidence: float) -> tuple[ConfirmedTarget, bool, bool]:
        """Associate a new detection with an existing target or register it.

        Returns: (target, is_new_candidate, just_confirmed)
        """
        t_now = time.time()
        # Find nearest same-class candidate within merge_radius.
        best, best_d = None, float("inf")
        for tgt in self.targets:
            if tgt.class_name != class_name:
                continue
            d = math.hypot(x - tgt.x, y - tgt.y)
            if d < best_d and d <= self.merge_radius:
                best, best_d = tgt, d
        if best is not None:
            was_confirmed = best.confirmed_at_t is not None
            best.update(x, y, confidence, t_now)
            just_confirmed = False
            if (not was_confirmed
                    and best.n_detections >= self.min_confirmations):
                best.confirmed_at_t = t_now
                self.last_confirmed_t = t_now
                just_confirmed = True
            return best, False, just_confirmed
        # New candidate.
        tgt = ConfirmedTarget(x, y, class_name, confidence, t_now)
        # Edge case: min_confirmations <= 1 ⇒ confirmed on first sighting.
        if self.min_confirmations <= 1:
            tgt.confirmed_at_t = t_now
            self.last_confirmed_t = t_now
            self.targets.append(tgt)
            return tgt, True, True
        self.targets.append(tgt)
        return tgt, True, False

    def confirmed(self) -> list[ConfirmedTarget]:
        return [t for t in self.targets if t.confirmed_at_t is not None]

    def confirmed_count(self) -> int:
        return sum(1 for t in self.targets if t.confirmed_at_t is not None)

    def quiet_seconds(self) -> float:
        """Time since last *new* confirmation. Large value ⇒ swarm is
        flying without finding anything new — likely done."""
        if self.last_confirmed_t <= 0:
            return float("inf")
        return time.time() - self.last_confirmed_t


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
                 min_explore_drones: int = 1,
                 max_concurrent_exploits: int = 2,
                 exploit_merge_radius: float = 20.0,
                 target_merge_radius: float = 8.0,
                 min_confirmations: int = 3,
                 confirmed_repulsion_sigma: float = 25.0,
                 confirmed_repulsion_weight: float = 1.0,
                 cooldown_after_exploit_s: float = 60.0,
                 expected_targets: int | None = None,
                 completion_quiet_period_s: float = 0.0):
        super().__init__("voronoi_coordinator")

        self.drone_ids = drone_ids
        self.altitude = altitude
        self.max_cluster = max_cluster
        self.exploit_confidence_threshold = exploit_confidence_threshold
        self.exploit_timeout = exploit_timeout
        self.exploit_confidence_drop = exploit_confidence_drop
        self.min_explore_drones = min_explore_drones
        self.max_concurrent_exploits = max_concurrent_exploits
        self.exploit_merge_radius = exploit_merge_radius
        self.confirmed_repulsion_sigma = confirmed_repulsion_sigma
        self.confirmed_repulsion_weight = confirmed_repulsion_weight
        self.cooldown_after_exploit_s = cooldown_after_exploit_s
        self.expected_targets = expected_targets
        self.completion_quiet_period_s = completion_quiet_period_s

        # Cross-drone target identity: same class within merge_radius is
        # treated as one physical target. Confirmation gating below
        # min_confirmations filters single-frame false positives.
        self.target_registry = TargetRegistry(
            merge_radius=target_merge_radius,
            min_confirmations=min_confirmations,
        )

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

        # Concurrent exploit events (empty list = all drones exploring).
        # Each entry tracks its own drones, formation, confidence, and timeout.
        # A new event is accepted only if (1) it doesn't overlap an existing
        # one within exploit_merge_radius, (2) there are enough free drones
        # left to form it while keeping min_explore_drones exploring, and
        # (3) len(active_exploits) < max_concurrent_exploits.
        self.active_exploits: list[ExploitEvent] = []
        self.arrival_threshold = 5.0  # meters — consider drone "arrived" if within this

        # Reconfiguration log (for E4 response time analysis)
        self._reconfig_log: list[dict] = []

        # Waypoint publishers
        self.wp_publishers = {}
        self._subs = []

        # SwarmMode publisher
        self.mode_publisher = self.create_publisher(SwarmMode, "/swarm/mode", 10)

        # Completion publisher — same QoS as lawnmower's /lawnmower/complete
        # so the data logger and run_experiment.sh treat them identically.
        complete_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.complete_pub = self.create_publisher(
            Bool, "/adaptive/complete", complete_qos
        )
        self._complete_signaled = False

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
            f"exploit_timeout={exploit_timeout}s, "
            f"max_concurrent={max_concurrent_exploits}, "
            f"min_explore={min_explore_drones}"
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

        # Cross-drone deduplication: associate this detection with an
        # existing target (same class, within target_merge_radius) if any,
        # otherwise register a new candidate. The registry's running mean
        # also stabilises the target's position estimate as more drones
        # observe it.
        tgt, is_new_candidate, just_confirmed = self.target_registry.ingest(
            det_x, det_y, msg.class_name, msg.confidence,
        )
        if just_confirmed:
            self.get_logger().info(
                f"[CONFIRMED] {msg.class_name} #"
                f"{self.target_registry.confirmed_count()} at "
                f"({tgt.x:.1f}, {tgt.y:.1f})m after {tgt.n_detections} detections"
            )

        # Get class priority
        class_priority = self.response_selector.get_priority(msg.class_name)

        # Density map writes are gated on confirmation status. Once a
        # target is confirmed, fresh detections on it carry no new
        # information for the explore policy — the registry already
        # has the position. Without this gate, drones hovering on a
        # confirmed target keep stacking bumps; the inner zero-out
        # absorbs the centre but the bumps' tails leak attractive
        # density into the surrounding ring and pull explore drones
        # back. Exploit trigger and registry ingest are unaffected,
        # so detection counts and exploit decisions still update.
        # Coupled to the repulsion flag so an ablation run
        # (`--confirmed-repulsion-weight 0`) restores the original
        # write-everything behaviour.
        suppress_write = (
            self.confirmed_repulsion_weight > 0
            and tgt.confirmed_at_t is not None
            and not just_confirmed
        )
        if not suppress_write:
            self.density_map.add_detection(
                det_x, det_y,
                confidence=msg.confidence,
                class_priority=class_priority,
                class_name=msg.class_name,
            )

        self.get_logger().info(
            f"[Detection] {msg.class_name} ({msg.confidence:.2f}, priority={class_priority}) "
            f"from drone {drone_id} at global ({det_x:.1f}, {det_y:.1f})m — "
            f"registry: {self.target_registry.confirmed_count()} confirmed / "
            f"{len(self.target_registry.targets)} candidates "
            f"({'new' if is_new_candidate else 'merged'}"
            f"{', density-suppressed' if suppress_write else ''})"
        )

        # Check if this detection should (a) reinforce an existing exploit or
        # (b) trigger a new concurrent exploit.
        t_detect = time.time()

        # Find the nearest active exploit; if within merge_radius, refresh it.
        nearest_evt = None
        nearest_dist = float("inf")
        for evt in self.active_exploits:
            d = ((det_x - evt.det_x) ** 2 + (det_y - evt.det_y) ** 2) ** 0.5
            if d < nearest_dist:
                nearest_dist = d
                nearest_evt = evt

        if nearest_evt is not None and nearest_dist < self.exploit_merge_radius:
            nearest_evt.last_confidence = max(nearest_evt.last_confidence,
                                              msg.confidence)
            return

        # Accept a new concurrent exploit only if confidence crosses threshold
        # AND the admission policy (drone budget + concurrency cap) permits it.
        if msg.confidence >= self.exploit_confidence_threshold:
            # Per-target cooldown: skip retrigger on a target whose
            # exploit recently ended. The 30s timeout already gave the
            # swarm time at this target; the cooldown reserves the next
            # window for actually exploring elsewhere.
            if (tgt.confirmed_at_t is not None
                    and t_detect < tgt.cooldown_until_t):
                self.get_logger().debug(
                    f"[Detection] {msg.class_name} at "
                    f"({det_x:.1f},{det_y:.1f})m in cooldown "
                    f"({tgt.cooldown_until_t - t_detect:.1f}s left) — ignoring"
                )
                return
            if len(self.active_exploits) >= self.max_concurrent_exploits:
                self.get_logger().debug(
                    f"[Detection] Concurrency cap reached "
                    f"({self.max_concurrent_exploits}) — ignoring"
                )
                return
            self._enter_exploit_mode(msg.class_name, det_x, det_y,
                                     msg.confidence, t_detect)

    def _busy_drone_ids(self) -> set[int]:
        """Drones currently assigned to an active exploit event."""
        busy: set[int] = set()
        for evt in self.active_exploits:
            busy.update(evt.exploit_drone_ids)
        return busy

    def _enter_exploit_mode(self, class_name: str, det_x: float, det_y: float,
                            confidence: float, t_detect: float):
        """Assign nearest *free* drones to exploit formation.

        Free drones = all drones minus those already assigned to another
        active exploit. The selector is given a reduced `drone_positions`
        dict so it can't steal drones from a concurrent event. The overall
        `min_explore_drones` floor is enforced across all events combined.
        """
        busy = self._busy_drone_ids()
        free_positions = {
            did: pos for did, pos in self.drone_positions.items()
            if did not in busy
        }
        # Total drones minus what's already busy must still leave min_explore
        # after this new event. Equivalently: min_explore counts against the
        # whole swarm, not each event.
        already_exploring = len(busy)
        available_for_exploit = (
            len(self.drone_ids) - already_exploring - self.min_explore_drones
        )
        if available_for_exploit <= 0:
            self.get_logger().warn(
                f"Cannot enter exploit mode — {already_exploring} drones already "
                f"exploiting, must keep {self.min_explore_drones} exploring"
            )
            return

        # Temporarily shrink min_explore to 0 for the selector (we enforced
        # the global floor above) so it hands us up to `n_needed` free drones.
        exploit_ids = self.response_selector.select_exploit_drones(
            class_name, det_x, det_y, free_positions, min_explore=0,
        )
        # Cap at what the global floor allows
        exploit_ids = exploit_ids[:available_for_exploit]

        if not exploit_ids:
            self.get_logger().warn(
                f"Cannot enter exploit mode — no free drones for {class_name}"
            )
            return

        formation_wps = self.response_selector.compute_formation(
            class_name, det_x, det_y, exploit_ids,
        )

        t_publish = time.time()

        evt = ExploitEvent(
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
        self.active_exploits.append(evt)

        explore_ids = [d for d in self.drone_ids if d not in self._busy_drone_ids()]

        self.get_logger().info(
            f"[EXPLOIT+] {class_name} at ({det_x:.1f}, {det_y:.1f})m "
            f"conf={confidence:.2f} — assigned drones: {exploit_ids}, "
            f"explore: {explore_ids}, concurrent events: {len(self.active_exploits)}"
        )

        # Aggregate SwarmMode publish — exploit_ids is union across events
        all_exploit_ids = sorted(self._busy_drone_ids())
        self._publish_mode(SwarmMode.MODE_EXPLOIT, all_exploit_ids, class_name,
                           det_x, det_y, confidence)

    def _exit_one_exploit(self, evt: ExploitEvent, reason: str):
        """Retire a single exploit event; its drones rejoin exploration."""
        # Stamp a cooldown on the matching confirmed target so a fresh
        # detection from a still-hovering drone can't immediately
        # retrigger exploit on the same point. Without this, the
        # exploit/explore cycle collapses to a 30s+1s loop and explore
        # drones never get time to relocate.
        if self.cooldown_after_exploit_s > 0:
            best, best_d = None, float("inf")
            for tgt in self.target_registry.confirmed():
                if tgt.class_name != evt.class_name:
                    continue
                d = math.hypot(tgt.x - evt.det_x, tgt.y - evt.det_y)
                if d < best_d and d <= self.exploit_merge_radius:
                    best, best_d = tgt, d
            if best is not None:
                best.cooldown_until_t = (
                    time.time() + self.cooldown_after_exploit_s
                )

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
            f"arrive_delay={arrive_str}, reason={reason}, "
            f"remaining events: {len(self.active_exploits) - 1}"
        )

        self.active_exploits.remove(evt)

        # Publish updated mode. If no events remain, broadcast EXPLORE.
        if self.active_exploits:
            survivors = sorted(self._busy_drone_ids())
            head = self.active_exploits[0]
            self._publish_mode(SwarmMode.MODE_EXPLOIT, survivors, head.class_name,
                               head.det_x, head.det_y, head.last_confidence)
        else:
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

    def _explore_density(self, x: float, y: float) -> float:
        """Density seen by *explore* drones.

        Inside `confirmed_repulsion_sigma` of any confirmed target, the
        density collapses to baseline — the area is treated as already
        explored, regardless of how many fresh detections are piling
        onto it. A weighted Gaussian subtraction is not enough here:
        when exploit drones hover on a stationary target they emit
        detections at multiple Hz, so the bump at the target grows
        unboundedly and a single subtracted Gaussian gets dwarfed.

        Exploit drones use formation waypoints directly and do not
        consult this function, so their behaviour is unchanged.

        Set `confirmed_repulsion_weight=0` to disable (ablation mode).
        """
        d = self.density_map(x, y)
        if not self.target_registry.targets:
            return d
        sigma = self.confirmed_repulsion_sigma
        if sigma <= 0 or self.confirmed_repulsion_weight <= 0:
            return d
        sigma_sq = sigma * sigma
        for tgt in self.target_registry.confirmed():
            dx = x - tgt.x
            dy = y - tgt.y
            if dx * dx + dy * dy < sigma_sq:
                return self.density_map.baseline
        return d

    def _check_completion(self):
        """Publish /adaptive/complete once the search is done.

        Two completion criteria, either of which fires:
          1. confirmed_count >= expected_targets (oracle-N stopping)
          2. quiet_seconds >= completion_quiet_period_s (heuristic
             stopping when expected_targets is None)

        Mirrors lawnmower's _signal_completion: latched QoS, multiple
        publishes for reliability, then a one-shot timer that calls
        rclpy.shutdown().
        """
        if self._complete_signaled:
            return
        confirmed = self.target_registry.confirmed_count()
        triggered_by = None
        if (self.expected_targets is not None
                and confirmed >= self.expected_targets):
            triggered_by = f"all {self.expected_targets} targets confirmed"
        elif (self.completion_quiet_period_s > 0
              and confirmed > 0
              and self.target_registry.quiet_seconds()
                  >= self.completion_quiet_period_s):
            triggered_by = (
                f"{self.completion_quiet_period_s:.0f}s without new "
                f"confirmation ({confirmed} confirmed)"
            )
        if triggered_by is None:
            return
        self.get_logger().info(
            f"[COMPLETE] {triggered_by} — signaling /adaptive/complete"
        )
        msg = Bool()
        msg.data = True
        for _ in range(5):
            self.complete_pub.publish(msg)
            time.sleep(0.1)
        self._complete_signaled = True
        # Defer shutdown by one tick so the publish flushes.
        self.create_timer(1.0, self._shutdown_after_complete)

    def _shutdown_after_complete(self):
        try:
            rclpy.shutdown()
        except Exception:
            pass

    def _update_voronoi(self):
        """Compute Voronoi partition and publish waypoints (dual-mode)."""
        if len(self.drone_positions) < len(self.drone_ids):
            return

        # Cheap in this loop — every Voronoi tick we also poll the
        # completion criteria. Avoids a separate timer.
        self._check_completion()

        # Prune old detections
        self.density_map.prune_expired(max_age=120.0)

        # ── Arrival check + expiry sweep over every active exploit ──────────
        # Iterate over a copy so _exit_one_exploit can mutate the list.
        for evt in list(self.active_exploits):
            if not evt.arrival_checked:
                all_arrived = True
                for did in evt.exploit_drone_ids:
                    if did not in self.drone_positions or did not in evt.formation_waypoints:
                        all_arrived = False
                        break
                    px, py = self.drone_positions[did]
                    tx, ty = evt.formation_waypoints[did]
                    dist = ((px - tx) ** 2 + (py - ty) ** 2) ** 0.5
                    if dist > self.arrival_threshold:
                        all_arrived = False
                        break
                if all_arrived:
                    evt.t_arrive = time.time()
                    evt.arrival_checked = True
                    reconfig_time = evt.t_arrive - evt.t_detect
                    self.get_logger().info(
                        f"[ARRIVE] {evt.class_name} drones arrived "
                        f"— reconfig_time={reconfig_time:.2f}s"
                    )

            if evt.is_expired(self.exploit_confidence_drop):
                elapsed = time.time() - evt.start_time
                reason = "timeout" if elapsed >= evt.timeout else "confidence_drop"
                self._exit_one_exploit(evt, reason)

        # Determine which drones are exploring vs exploiting (after expiry sweep)
        exploit_ids = self._busy_drone_ids()
        explore_ids = [d for d in self.drone_ids if d not in exploit_ids]

        # ── Exploit drones: fly class-specific formations ───────────────────
        for evt in self.active_exploits:
            for did in evt.exploit_drone_ids:
                if did in evt.formation_waypoints:
                    tx, ty = evt.formation_waypoints[did]
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
                # Explore drones see the *repulsed* density: confirmed
                # targets are landmarks to avoid, not attractors.
                cx, cy = weighted_centroid(cells[i], self._explore_density)
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
    parser.add_argument(
        "--max-concurrent-exploits", type=int, default=2,
        help="Maximum simultaneously active exploit events (default: 2)",
    )
    parser.add_argument(
        "--exploit-merge-radius", type=float, default=20.0,
        help="Detections within this distance of an active event reinforce "
             "it instead of opening a new one (default: 20m)",
    )
    parser.add_argument(
        "--target-merge-radius", type=float, default=8.0,
        help="Cross-drone target dedup radius — detections of the same "
             "class within this distance are treated as one physical "
             "target by the registry (default: 8m)",
    )
    parser.add_argument(
        "--min-confirmations", type=int, default=3,
        help="Detections required before a candidate is counted as a "
             "confirmed target (default: 3 — filters single-frame YOLO "
             "false positives)",
    )
    parser.add_argument(
        "--confirmed-repulsion-sigma", type=float, default=25.0,
        help="Radius (m) over which confirmed targets repel explore-mode "
             "drones from their density map (default: 25)",
    )
    parser.add_argument(
        "--confirmed-repulsion-weight", type=float, default=1.0,
        help="Enables hard zero-out repulsion when >0 (any positive "
             "value); set to 0 to disable confirmed-target repulsion "
             "entirely as an ablation. Magnitude is no longer a "
             "Gaussian scale — within `--confirmed-repulsion-sigma` of "
             "a confirmed target the explore-mode density collapses to "
             "baseline regardless of weight (default: 1.0)",
    )
    parser.add_argument(
        "--cooldown-after-exploit-s", type=float, default=60.0,
        help="After an exploit on a confirmed target ends, ignore "
             "detections of that same target for this many seconds — "
             "lets the swarm actually explore instead of retriggering "
             "on a still-visible target. Set to 0 to disable "
             "(default: 60)",
    )
    parser.add_argument(
        "--expected-targets", type=int, default=None,
        help="If set, /adaptive/complete fires once the registry has this "
             "many confirmed targets (oracle-N stopping). Pass the trial "
             "target count from run_experiment.sh.",
    )
    parser.add_argument(
        "--completion-quiet-period", type=float, default=0.0,
        help="If >0, /adaptive/complete also fires when no new target has "
             "been confirmed for this many seconds (heuristic stopping "
             "when expected_targets is None). Default 0 disables.",
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
        max_concurrent_exploits=args.max_concurrent_exploits,
        exploit_merge_radius=args.exploit_merge_radius,
        target_merge_radius=args.target_merge_radius,
        min_confirmations=args.min_confirmations,
        confirmed_repulsion_sigma=args.confirmed_repulsion_sigma,
        confirmed_repulsion_weight=args.confirmed_repulsion_weight,
        cooldown_after_exploit_s=args.cooldown_after_exploit_s,
        expected_targets=args.expected_targets,
        completion_quiet_period_s=args.completion_quiet_period,
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
