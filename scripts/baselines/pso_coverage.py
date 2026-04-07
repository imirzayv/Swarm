#!/usr/bin/env python3
"""
Baseline: PSO (Particle Swarm Optimization) — bio-inspired coverage controller.

Each drone is a particle with position and velocity. Fitness is evaluated based
on distance to uncovered grid cells plus an optional detection bonus. Velocities
are updated using standard PSO equations:

    v_i(t+1) = w*v_i(t) + c1*r1*(pbest_i - x_i) + c2*r2*(gbest - x_i)
    x_i(t+1) = x_i(t) + v_i(t+1)

Usage:
    source ~/Desktop/Swarm/ros2_ws/install/setup.bash
    python3 baselines/pso_coverage.py --drones 1 2 3 --area-size 200
    python3 baselines/pso_coverage.py --drones 1 2 3 --area-size 200 --no-detections
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


class PSOCoverageNode(Node):
    """PSO-based coverage controller for drone swarm."""

    def __init__(self, drone_ids: list[int], area_size: float,
                 update_rate: float, altitude: float,
                 inertia: float, cognitive: float, social: float,
                 max_velocity: float, grid_resolution: float,
                 use_detections: bool = True):
        super().__init__("pso_coverage_baseline")

        self.drone_ids = drone_ids
        self.altitude = altitude
        self.n = len(drone_ids)
        self.use_detections = use_detections

        half = area_size / 2
        self.bounds = (-half, -half, half, half)
        self.area_size = area_size

        # PSO parameters
        self.w = inertia
        self.c1 = cognitive
        self.c2 = social
        self.v_max = max_velocity

        # Particle state — initialize velocities RANDOMLY so particles move
        self.rng = np.random.default_rng(42)
        self.velocities: dict[int, np.ndarray] = {
            did: self.rng.uniform(-max_velocity, max_velocity, 2)
            for did in drone_ids
        }
        self.drone_positions: dict[int, tuple[float, float]] = {}
        self.pbest_pos: dict[int, np.ndarray] = {}
        self.pbest_fitness: dict[int, float] = {did: -np.inf for did in drone_ids}
        self.gbest_pos: np.ndarray | None = None
        self.gbest_fitness: float = -np.inf

        # Coverage grid for fitness evaluation
        self.grid_res = grid_resolution
        nx = int(area_size / grid_resolution)
        ny = int(area_size / grid_resolution)
        self.coverage_grid = np.zeros((nx, ny), dtype=np.float32)
        self.grid_nx = nx
        self.grid_ny = ny

        # Detection events (optional)
        self.detections: list[tuple[float, float, float, float]] = []
        self.detection_decay = 30.0

        self.wp_publishers = {}
        self._subs = []

        for did in drone_ids:
            self._subs.append(self.create_subscription(
                TargetWaypoint, f"/drone{did}/position",
                lambda msg, d=did: self._position_cb(msg, d), 10,
            ))
            if self.use_detections:
                self._subs.append(self.create_subscription(
                    Detection, f"/drone{did}/detection",
                    lambda msg, d=did: self._detection_cb(msg, d), 10,
                ))
            self.wp_publishers[did] = self.create_publisher(
                TargetWaypoint, f"/drone{did}/target_waypoint", 10,
            )

        period = 1.0 / update_rate
        self.create_timer(period, self._pso_step)

        det_mode = "ON" if use_detections else "OFF"
        self.get_logger().info(
            f"PSO coverage baseline ready — {self.n} drones, "
            f"area={area_size}m, w={inertia}, c1={cognitive}, c2={social}, "
            f"detections={det_mode}"
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
            f"[PSO Detection] {msg.class_name} ({msg.confidence:.2f}) "
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

    def _evaluate_fitness(self, x: float, y: float, drone_id: int) -> float:
        """Evaluate fitness at position (x, y).

        Higher fitness = more uncovered area nearby + detection bonus.
        Penalizes proximity to other drones to encourage spreading.
        """
        x_min, y_min, _, _ = self.bounds
        fitness = 0.0

        # Component 1: attraction to uncovered cells over the FULL grid
        # Sample sparsely to keep it fast but break symmetry
        step = max(1, self.grid_nx // 20)
        for ni in range(0, self.grid_nx, step):
            for nj in range(0, self.grid_ny, step):
                uncovered = 1.0 - self.coverage_grid[ni, nj]
                if uncovered < 0.05:
                    continue
                cell_x = x_min + (ni + 0.5) * self.grid_res
                cell_y = y_min + (nj + 0.5) * self.grid_res
                dist = math.sqrt((x - cell_x) ** 2 + (y - cell_y) ** 2) + 1.0
                fitness += uncovered / dist

        # Component 2: penalty for being close to other drones (spread out)
        for other_id in self.drone_ids:
            if other_id == drone_id or other_id not in self.drone_positions:
                continue
            ox, oy = self.drone_positions[other_id]
            dist = math.sqrt((x - ox) ** 2 + (y - oy) ** 2) + 1.0
            fitness -= 50.0 / dist

        # Component 3: detection bonus (optional)
        if self.use_detections:
            now = time.time()
            for det_x, det_y, conf, ts in self.detections:
                age = now - ts
                if age > 120.0:
                    continue
                decay = 0.5 ** (age / self.detection_decay)
                dist = math.sqrt((x - det_x) ** 2 + (y - det_y) ** 2) + 1.0
                fitness += 10.0 * conf * decay / dist

        return fitness

    def _pso_step(self):
        """One PSO iteration: evaluate fitness, update velocities and positions."""
        if len(self.drone_positions) < self.n:
            return

        # Prune old detections
        if self.use_detections:
            now = time.time()
            self.detections = [(x, y, c, t) for x, y, c, t in self.detections
                               if now - t < 120.0]

        x_min, y_min, x_max, y_max = self.bounds

        # Evaluate fitness and update bests
        for did in self.drone_ids:
            px, py = self.drone_positions[did]
            pos = np.array([px, py])

            fitness = self._evaluate_fitness(px, py, did)

            # Initialize pbest on first evaluation
            if did not in self.pbest_pos:
                self.pbest_pos[did] = pos.copy()
                self.pbest_fitness[did] = fitness

            # Update personal best
            if fitness > self.pbest_fitness[did]:
                self.pbest_fitness[did] = fitness
                self.pbest_pos[did] = pos.copy()

            # Update global best
            if fitness > self.gbest_fitness:
                self.gbest_fitness = fitness
                self.gbest_pos = pos.copy()

        # Fallback gbest if still None
        if self.gbest_pos is None:
            px, py = self.drone_positions[self.drone_ids[0]]
            self.gbest_pos = np.array([px, py])

        # Update velocities and compute new target positions
        for did in self.drone_ids:
            px, py = self.drone_positions[did]
            pos = np.array([px, py])

            r1 = self.rng.random(2)
            r2 = self.rng.random(2)

            v = self.velocities[did]
            v_new = (self.w * v
                     + self.c1 * r1 * (self.pbest_pos[did] - pos)
                     + self.c2 * r2 * (self.gbest_pos - pos))

            # Add small random perturbation to avoid stagnation
            v_new += self.rng.uniform(-1.0, 1.0, 2)

            # Clamp velocity
            speed = np.linalg.norm(v_new)
            if speed > self.v_max:
                v_new = v_new * (self.v_max / speed)

            self.velocities[did] = v_new

            new_pos = pos + v_new
            new_pos[0] = np.clip(new_pos[0], x_min, x_max)
            new_pos[1] = np.clip(new_pos[1], y_min, y_max)

            lat, lon = xy_to_gps(float(new_pos[0]), float(new_pos[1]))

            wp = TargetWaypoint()
            wp.drone_id = did
            wp.latitude = lat
            wp.longitude = lon
            wp.altitude = self.altitude
            wp.priority = 0
            self.wp_publishers[did].publish(wp)


def main():
    parser = argparse.ArgumentParser(
        description="PSO coverage baseline controller"
    )
    parser.add_argument("--drones", type=int, nargs="+", default=[1, 2, 3])
    parser.add_argument("--area-size", type=float, default=200.0)
    parser.add_argument("--update-rate", type=float, default=0.5)
    parser.add_argument("--altitude", type=float, default=40.0)
    parser.add_argument("--inertia", type=float, default=0.7,
                        help="PSO inertia weight w")
    parser.add_argument("--cognitive", type=float, default=1.5,
                        help="PSO cognitive coefficient c1")
    parser.add_argument("--social", type=float, default=1.5,
                        help="PSO social coefficient c2")
    parser.add_argument("--max-velocity", type=float, default=15.0,
                        help="Maximum particle velocity (m/step)")
    parser.add_argument("--grid-resolution", type=float, default=5.0,
                        help="Coverage grid cell size (meters)")
    parser.add_argument("--no-detections", action="store_true",
                        help="Disable detection influence on waypoint generation")
    args = parser.parse_args()

    rclpy.init()
    node = PSOCoverageNode(
        drone_ids=args.drones,
        area_size=args.area_size,
        update_rate=args.update_rate,
        altitude=args.altitude,
        inertia=args.inertia,
        cognitive=args.cognitive,
        social=args.social,
        max_velocity=args.max_velocity,
        grid_resolution=args.grid_resolution,
        use_detections=not args.no_detections,
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
