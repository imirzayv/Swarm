"""
Response Selector: Class-specific swarm formation generator.

Given a detection class, confidence, and current drone positions, computes
formation waypoints for the assigned exploit drones:
  - person  -> tight cluster (2 drones, 3m radius around detection)
  - vehicle -> tracking chain (2 drones, 8m separation along heading)
  - fire    -> wide perimeter (3 drones, 10m radius equilateral triangle)

Usage:
    from response_selector import ResponseSelector
    selector = ResponseSelector(config_path="config/adaptive_params.yaml")
    waypoints = selector.compute_formation("person", det_x, det_y, drone_positions)
"""

import math
import os

import yaml


# Default formation configs (used if no YAML config loaded)
DEFAULT_FORMATIONS = {
    "person": {"type": "cluster", "num_drones": 2, "radius": 3.0},
    "vehicle": {"type": "chain", "num_drones": 2, "separation": 8.0},
    "car": {"type": "chain", "num_drones": 2, "separation": 8.0},
    "truck": {"type": "chain", "num_drones": 2, "separation": 8.0},
    "fire": {"type": "perimeter", "num_drones": 3, "radius": 10.0},
}

DEFAULT_FALLBACK = {"type": "cluster", "num_drones": 2, "radius": 5.0}

DEFAULT_PRIORITIES = {
    "person": 0.9,
    "vehicle": 0.6,
    "car": 0.6,
    "truck": 0.6,
    "fire": 1.0,
}


class ResponseSelector:
    """Selects and computes class-specific swarm formations."""

    def __init__(self, config_path: str | None = None):
        self.formations = dict(DEFAULT_FORMATIONS)
        self.default_formation = dict(DEFAULT_FALLBACK)
        self.class_priorities = dict(DEFAULT_PRIORITIES)

        if config_path and os.path.exists(config_path):
            with open(config_path, "r") as f:
                cfg = yaml.safe_load(f)
            if cfg.get("formations"):
                self.formations = cfg["formations"]
            if cfg.get("default_formation"):
                self.default_formation = cfg["default_formation"]
            if cfg.get("class_priorities"):
                self.class_priorities = cfg["class_priorities"]

    def get_priority(self, class_name: str) -> float:
        """Get priority weight for a detection class."""
        return self.class_priorities.get(class_name, 0.5)

    def get_formation_config(self, class_name: str) -> dict:
        """Get formation config for a class, falling back to default."""
        return self.formations.get(class_name, self.default_formation)

    def get_num_exploit_drones(self, class_name: str) -> int:
        """How many drones this class needs for its formation."""
        cfg = self.get_formation_config(class_name)
        return cfg.get("num_drones", 2)

    def select_exploit_drones(self, class_name: str, det_x: float, det_y: float,
                              drone_positions: dict[int, tuple[float, float]],
                              min_explore: int = 1) -> list[int]:
        """Select the nearest N drones for exploitation, keeping min_explore exploring.

        Args:
            class_name: detected object class
            det_x, det_y: detection position in local XY
            drone_positions: {drone_id: (x, y)} for all drones
            min_explore: minimum drones that must remain exploring

        Returns:
            List of drone IDs assigned to exploit (may be fewer than requested
            if not enough drones available after reserving min_explore).
        """
        n_needed = self.get_num_exploit_drones(class_name)
        n_available = len(drone_positions)

        # Ensure we keep at least min_explore drones exploring
        max_exploit = max(0, n_available - min_explore)
        n_assign = min(n_needed, max_exploit)

        if n_assign == 0:
            return []

        # Sort drones by distance to detection
        distances = []
        for did, (dx, dy) in drone_positions.items():
            dist = math.sqrt((dx - det_x) ** 2 + (dy - det_y) ** 2)
            distances.append((dist, did))
        distances.sort()

        return [did for _, did in distances[:n_assign]]

    def compute_formation(self, class_name: str, det_x: float, det_y: float,
                          exploit_drone_ids: list[int]) -> dict[int, tuple[float, float]]:
        """Compute formation waypoints for exploit drones around a detection.

        Args:
            class_name: detection class
            det_x, det_y: detection position in local XY
            exploit_drone_ids: drone IDs assigned to this formation

        Returns:
            {drone_id: (target_x, target_y)} formation waypoints.
        """
        cfg = self.get_formation_config(class_name)
        formation_type = cfg.get("type", "cluster")

        if formation_type == "cluster":
            return self._cluster_formation(det_x, det_y, exploit_drone_ids,
                                           cfg.get("radius", 3.0))
        elif formation_type == "chain":
            return self._chain_formation(det_x, det_y, exploit_drone_ids,
                                         cfg.get("separation", 8.0))
        elif formation_type == "perimeter":
            return self._perimeter_formation(det_x, det_y, exploit_drone_ids,
                                             cfg.get("radius", 10.0))
        else:
            return self._cluster_formation(det_x, det_y, exploit_drone_ids,
                                           cfg.get("radius", 5.0))

    def _cluster_formation(self, cx: float, cy: float,
                           drone_ids: list[int],
                           radius: float) -> dict[int, tuple[float, float]]:
        """Tight cluster: drones evenly spaced on a circle around the detection."""
        waypoints = {}
        n = len(drone_ids)
        for i, did in enumerate(drone_ids):
            angle = 2 * math.pi * i / n
            x = cx + radius * math.cos(angle)
            y = cy + radius * math.sin(angle)
            waypoints[did] = (x, y)
        return waypoints

    def _chain_formation(self, cx: float, cy: float,
                         drone_ids: list[int],
                         separation: float) -> dict[int, tuple[float, float]]:
        """Tracking chain: drones lined up with separation along the X axis."""
        waypoints = {}
        n = len(drone_ids)
        total_span = separation * (n - 1) if n > 1 else 0
        start_offset = -total_span / 2
        for i, did in enumerate(drone_ids):
            x = cx + start_offset + i * separation
            y = cy
            waypoints[did] = (x, y)
        return waypoints

    def _perimeter_formation(self, cx: float, cy: float,
                             drone_ids: list[int],
                             radius: float) -> dict[int, tuple[float, float]]:
        """Wide perimeter: drones on vertices of a regular polygon around detection."""
        waypoints = {}
        n = len(drone_ids)
        for i, did in enumerate(drone_ids):
            angle = 2 * math.pi * i / n - math.pi / 2  # start from top
            x = cx + radius * math.cos(angle)
            y = cy + radius * math.sin(angle)
            waypoints[did] = (x, y)
        return waypoints
