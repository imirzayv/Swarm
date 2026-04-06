"""
Voronoi coverage utilities for adaptive area monitoring.

Provides:
  - Bounded Voronoi cell computation (clipped to rectangular monitoring area)
  - Lloyd's relaxation (move generators to cell centroids)
  - Weighted centroid computation (density-weighted for adaptive reallocation)
  - GPS <-> local XY coordinate conversion (flat-earth approximation)
  - Gaussian density map for detection-triggered reallocation
"""

import math
import time
from dataclasses import dataclass, field

import numpy as np
from scipy.spatial import Voronoi


# PX4 SITL default home position
PX4_HOME_LAT = 47.397742
PX4_HOME_LON = 8.545594

# Earth radius approximation for flat-earth conversion
METERS_PER_DEG_LAT = 111_320.0
METERS_PER_DEG_LON = METERS_PER_DEG_LAT * math.cos(math.radians(PX4_HOME_LAT))


# ── GPS <-> Local XY conversion ─────────────────────────────────────────────

def gps_to_xy(lat: float, lon: float,
              home_lat: float = PX4_HOME_LAT,
              home_lon: float = PX4_HOME_LON) -> tuple[float, float]:
    """Convert GPS coordinates to local XY (meters) relative to home position.

    X = North, Y = East (NED-like, but Z ignored here).
    """
    x = (lat - home_lat) * METERS_PER_DEG_LAT
    y = (lon - home_lon) * METERS_PER_DEG_LON
    return x, y


def xy_to_gps(x: float, y: float,
              home_lat: float = PX4_HOME_LAT,
              home_lon: float = PX4_HOME_LON) -> tuple[float, float]:
    """Convert local XY (meters) to GPS coordinates.

    X = North offset, Y = East offset.
    """
    lat = home_lat + x / METERS_PER_DEG_LAT
    lon = home_lon + y / METERS_PER_DEG_LON
    return lat, lon


# ── Bounded Voronoi ─────────────────────────────────────────────────────────

def clip_polygon_to_rect(polygon: np.ndarray,
                         bounds: tuple[float, float, float, float]) -> np.ndarray:
    """Clip a convex polygon to a rectangle using Sutherland-Hodgman algorithm.

    Args:
        polygon: Nx2 array of vertices.
        bounds: (x_min, y_min, x_max, y_max).

    Returns:
        Clipped polygon as Nx2 array. May be empty if fully outside.
    """
    x_min, y_min, x_max, y_max = bounds

    # Clip edges: left, right, bottom, top
    clip_edges = [
        (lambda p: p[0] >= x_min, lambda p, q: _intersect_x(p, q, x_min)),
        (lambda p: p[0] <= x_max, lambda p, q: _intersect_x(p, q, x_max)),
        (lambda p: p[1] >= y_min, lambda p, q: _intersect_y(p, q, y_min)),
        (lambda p: p[1] <= y_max, lambda p, q: _intersect_y(p, q, y_max)),
    ]

    output = list(polygon)
    for inside_fn, intersect_fn in clip_edges:
        if len(output) == 0:
            return np.empty((0, 2))
        inp = output
        output = []
        for i in range(len(inp)):
            current = inp[i]
            previous = inp[i - 1]
            if inside_fn(current):
                if not inside_fn(previous):
                    output.append(intersect_fn(previous, current))
                output.append(current)
            elif inside_fn(previous):
                output.append(intersect_fn(previous, current))

    return np.array(output) if output else np.empty((0, 2))


def _intersect_x(p, q, x):
    """Intersection of line segment p->q with vertical line x."""
    t = (x - p[0]) / (q[0] - p[0]) if q[0] != p[0] else 0
    return np.array([x, p[1] + t * (q[1] - p[1])])


def _intersect_y(p, q, y):
    """Intersection of line segment p->q with horizontal line y."""
    t = (y - p[1]) / (q[1] - p[1]) if q[1] != p[1] else 0
    return np.array([p[0] + t * (q[0] - p[0]), y])


def compute_bounded_voronoi(points: np.ndarray,
                            bounds: tuple[float, float, float, float]
                            ) -> list[np.ndarray]:
    """Compute Voronoi cells clipped to a rectangular boundary.

    Args:
        points: Nx2 array of generator positions.
        bounds: (x_min, y_min, x_max, y_max) monitoring area.

    Returns:
        List of Nx2 polygon arrays, one per generator (same order as points).
    """
    x_min, y_min, x_max, y_max = bounds
    # Add mirror points far outside to ensure all cells are bounded
    margin = 4 * max(x_max - x_min, y_max - y_min)
    mirror_points = np.array([
        [x_min - margin, (y_min + y_max) / 2],
        [x_max + margin, (y_min + y_max) / 2],
        [(x_min + x_max) / 2, y_min - margin],
        [(x_min + x_max) / 2, y_max + margin],
    ])

    all_points = np.vstack([points, mirror_points])
    vor = Voronoi(all_points)

    cells = []
    for i in range(len(points)):
        region_idx = vor.point_region[i]
        region = vor.regions[region_idx]
        if -1 in region:
            # Unbounded region — shouldn't happen with mirror points, but fallback
            cells.append(np.array([[x_min, y_min], [x_max, y_min],
                                   [x_max, y_max], [x_min, y_max]]))
            continue
        vertices = vor.vertices[region]
        clipped = clip_polygon_to_rect(vertices, bounds)
        if len(clipped) < 3:
            # Degenerate — give a small square around the point
            px, py = points[i]
            cells.append(np.array([[px - 0.5, py - 0.5], [px + 0.5, py - 0.5],
                                   [px + 0.5, py + 0.5], [px - 0.5, py + 0.5]]))
        else:
            cells.append(clipped)

    return cells


# ── Centroid computation ─────────────────────────────────────────────────────

def polygon_centroid(vertices: np.ndarray) -> tuple[float, float]:
    """Compute the centroid of a simple polygon using the shoelace formula."""
    n = len(vertices)
    if n == 0:
        return 0.0, 0.0

    # Close the polygon
    xs = vertices[:, 0]
    ys = vertices[:, 1]
    xs_next = np.roll(xs, -1)
    ys_next = np.roll(ys, -1)

    cross = xs * ys_next - xs_next * ys
    area = 0.5 * np.sum(cross)

    if abs(area) < 1e-10:
        # Degenerate polygon — return mean of vertices
        return float(np.mean(xs)), float(np.mean(ys))

    cx = np.sum((xs + xs_next) * cross) / (6 * area)
    cy = np.sum((ys + ys_next) * cross) / (6 * area)

    return float(cx), float(cy)


def weighted_centroid(vertices: np.ndarray,
                      density_fn,
                      n_samples: int = 200) -> tuple[float, float]:
    """Compute density-weighted centroid of a polygon via Monte Carlo sampling.

    Args:
        vertices: Polygon vertices (Nx2).
        density_fn: Callable(x, y) -> float, density at point (x, y).
        n_samples: Number of random samples inside the polygon.

    Returns:
        (cx, cy) weighted centroid.
    """
    if len(vertices) < 3:
        return polygon_centroid(vertices)

    # Bounding box
    x_min, y_min = vertices.min(axis=0)
    x_max, y_max = vertices.max(axis=0)

    # Sample random points inside the polygon
    rng = np.random.default_rng()
    samples_x = []
    samples_y = []
    weights = []

    attempts = 0
    max_attempts = n_samples * 10
    while len(samples_x) < n_samples and attempts < max_attempts:
        batch = 500
        xs = rng.uniform(x_min, x_max, batch)
        ys = rng.uniform(y_min, y_max, batch)
        for x, y in zip(xs, ys):
            if _point_in_polygon(x, y, vertices):
                w = density_fn(x, y)
                samples_x.append(x)
                samples_y.append(y)
                weights.append(w)
                if len(samples_x) >= n_samples:
                    break
        attempts += batch

    if not samples_x:
        return polygon_centroid(vertices)

    weights = np.array(weights)
    total_w = weights.sum()
    if total_w < 1e-10:
        return polygon_centroid(vertices)

    cx = np.dot(weights, samples_x) / total_w
    cy = np.dot(weights, samples_y) / total_w

    return float(cx), float(cy)


def _point_in_polygon(x: float, y: float, vertices: np.ndarray) -> bool:
    """Ray casting algorithm for point-in-polygon test."""
    n = len(vertices)
    inside = False
    j = n - 1
    for i in range(n):
        xi, yi = vertices[i]
        xj, yj = vertices[j]
        if ((yi > y) != (yj > y)) and (x < (xj - xi) * (y - yi) / (yj - yi) + xi):
            inside = not inside
        j = i
    return inside


# ── Lloyd's relaxation ───────────────────────────────────────────────────────

def lloyd_step(points: np.ndarray,
               bounds: tuple[float, float, float, float],
               density_fn=None) -> np.ndarray:
    """One step of Lloyd's relaxation: move each point to its Voronoi cell centroid.

    Args:
        points: Nx2 array of generator positions.
        bounds: (x_min, y_min, x_max, y_max).
        density_fn: Optional callable(x, y) -> float. If None, uses uniform density.

    Returns:
        Nx2 array of updated positions (clipped to bounds).
    """
    cells = compute_bounded_voronoi(points, bounds)
    new_points = np.zeros_like(points)

    for i, cell in enumerate(cells):
        if density_fn is not None:
            cx, cy = weighted_centroid(cell, density_fn)
        else:
            cx, cy = polygon_centroid(cell)

        # Clip to bounds
        x_min, y_min, x_max, y_max = bounds
        cx = np.clip(cx, x_min, x_max)
        cy = np.clip(cy, y_min, y_max)
        new_points[i] = [cx, cy]

    return new_points


# ── Density map ──────────────────────────────────────────────────────────────

@dataclass
class DetectionEvent:
    """A detection event that creates a Gaussian bump in the density map."""
    x: float          # local X position (meters)
    y: float          # local Y position (meters)
    timestamp: float  # time.time() when detected
    weight: float = 1.0
    confidence: float = 1.0    # YOLO detection confidence
    class_priority: float = 1.0  # class-based priority weight
    class_name: str = ""       # detection class name


@dataclass
class DensityMap:
    """Density function: uniform baseline + Gaussian bumps at detection locations.

    Supports two weighting modes:
      - confidence_weighted=True (default): weight = confidence x class_priority
      - confidence_weighted=False: binary mode (weight = 1.0 for any detection)
    """
    baseline: float = 1.0
    detection_sigma: float = 5.0    # Gaussian spread in meters
    detection_weight: float = 10.0  # Peak weight of detection bump
    decay_half_life: float = 30.0   # Seconds for bump to decay to half weight
    confidence_weighted: bool = True  # False = binary density (ablation mode)
    detections: list[DetectionEvent] = field(default_factory=list)

    def add_detection(self, x: float, y: float, weight: float = 1.0,
                      confidence: float = 1.0, class_priority: float = 1.0,
                      class_name: str = ""):
        """Add a new detection event with confidence and class priority."""
        self.detections.append(DetectionEvent(
            x=x, y=y, timestamp=time.time(), weight=weight,
            confidence=confidence, class_priority=class_priority,
            class_name=class_name,
        ))

    def prune_expired(self, max_age: float = 120.0):
        """Remove detections older than max_age seconds."""
        now = time.time()
        self.detections = [d for d in self.detections if now - d.timestamp < max_age]

    def __call__(self, x: float, y: float) -> float:
        """Evaluate density at point (x, y).

        When confidence_weighted=True:
          density = baseline + sum(det_weight * conf * priority * decay * gaussian)
        When confidence_weighted=False (binary mode):
          density = baseline + sum(det_weight * 1.0 * decay * gaussian)
        """
        density = self.baseline
        now = time.time()

        for det in self.detections:
            age = now - det.timestamp
            # Exponential decay
            decay = 0.5 ** (age / self.decay_half_life) if self.decay_half_life > 0 else 1.0
            # Gaussian bump
            dist_sq = (x - det.x) ** 2 + (y - det.y) ** 2
            gaussian = math.exp(-dist_sq / (2 * self.detection_sigma ** 2))

            if self.confidence_weighted:
                # Novel: confidence x class_priority weighting
                w = det.confidence * det.class_priority
            else:
                # Binary mode (ablation): any detection has weight 1.0
                w = 1.0

            density += self.detection_weight * det.weight * w * decay * gaussian

        return density
