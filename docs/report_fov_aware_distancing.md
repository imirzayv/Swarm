# FOV-Aware Distancing Report
## Where Camera Field-of-View Drives Geometric Decisions

---

## 1. The Camera Specification

All FOV-aware logic in the codebase derives from a single hardware constant
defined in `detection_publisher.py` (and echoed in `metrics_compute.py`):

```python
CAMERA_HFOV_RAD = 1.74   # ≈ 100° horizontal FOV
CAMERA_WIDTH    = 1280   # pixels
CAMERA_HEIGHT   = 960    # pixels
```

This matches the Gazebo `x500_mono_cam_down` SDF model — a nadir-pointing
(straight-down) camera mounted under the drone body. From this one number,
the system derives the ground footprint at any given altitude:

```
footprint_width = 2 × altitude × tan(HFOV / 2)
                = 2 × altitude × tan(0.87)
                ≈ altitude × 2.27   (meters, for a nadir camera)
```

At the standard experiment altitude of 40 m, one drone's camera covers
approximately **90.8 m** of ground width. At 60 m (used in the full
experiment battery), it covers approximately **136 m**.

---

## 2. Where FOV-Aware Distancing Is Applied

FOV awareness appears in three distinct places:

| Location | Phase | Purpose |
|---|---|---|
| `detection_publisher.py` — `bbox_to_world_offset()` | **Perception** | Project a pixel coordinate onto the ground using the camera's focal length (derived from HFOV) |
| `baselines/lawnmower.py` — `compute_fov_spacing()` | **Path planning** | Space lawnmower sweep lines so adjacent camera footprints tile the ground exactly |
| `waypoint_executor.py` — altitude layering | **Flight control** | Separate drones onto altitude layers spaced to avoid both collisions and excessive FOV overlap |
| `metrics_compute.py` — coverage grid | **Postprocessing** | Determine which grid cells each drone's camera covered at each timestamp |
| `run_experiment.sh` — lawnmower duration cap | **Orchestration** | Compute a safe timeout for lawnmower trials based on FOV-derived sweep count |

---

## 3. Phase 1 — Perception: Ground Projection (`detection_publisher.py`)

**File:** [scripts/detection_publisher.py](../scripts/detection_publisher.py),
lines 162–196 (`bbox_to_world_offset`)

**What it does:** Converts a detected bounding-box center pixel `(cx, cy)` to
a ground-plane position in the ENU world frame.

**How HFOV is used:**

The focal length `fx` (pixels per unit) is back-computed from the known HFOV:

```python
fx = (img_w / 2.0) / math.tan(CAMERA_HFOV_RAD / 2.0)
fy = fx   # square pixels assumed
```

A ray is then built from the pixel through the camera centre:

```python
ray_cam = [(cx - W/2) / fx,
           (cy - H/2) / fy,
           1.0]
```

The ray is rotated through three frames:

```
camera frame  ──R_BODY_CAM──►  body frame  ──R(q_body_to_ned)──►  NED frame
```

Then intersected with the ground plane at the drone's current AGL altitude:

```python
t          = altitude / ray_ned[2]   # ray parameter at ground
north_off  = t * ray_ned[0]
east_off   = t * ray_ned[1]
```

**Why this matters for the system:** Without the HFOV-based focal length,
the scale of the projected ground position would be wrong — detections would
appear tens of meters away from where the target actually is. This
world-position offset is the only geometric link between the camera image and
the Voronoi coordinator's coordinate system.

**Attitude sync:** The attitude quaternion used in the projection is
SLERP-interpolated between the two bracketing samples from
`/droneN/attitude`. This prevents projection errors caused by drone roll/pitch
during turns — a tilted camera footprint maps differently onto the ground than
a level one.

---

## 4. Phase 2 — Path Planning: Lawnmower Sweep Spacing (`lawnmower.py`)

**File:** [scripts/baselines/lawnmower.py](../scripts/baselines/lawnmower.py),
lines 33–41 (`compute_fov_spacing`) and lines 44–74 (`generate_lawnmower_path`)

**What it does:** Calculates the lateral spacing between parallel sweep lines
so that adjacent camera footprints tile the ground without leaving uncovered
seams and without wasting time on excessive overlap.

**The formula:**

```python
def compute_fov_spacing(altitude: float, hfov_rad: float,
                        overlap_factor: float = 0.95) -> float:
    return 2.0 * altitude * math.tan(hfov_rad / 2.0) * overlap_factor
```

`2 × altitude × tan(HFOV/2)` is the full ground footprint width of the nadir
camera. Multiplying by `overlap_factor=0.95` contracts this by 5%, leaving a
thin overlap seam between adjacent strips. This seam ensures:
- No ground is left uncovered at the strip boundaries
- The overlap is narrow enough that it does not significantly inflate total
  flight time

**How it drives waypoint generation:**

The monitoring area is divided into N vertical strips (one per drone). Within
each strip, `generate_lawnmower_path()` computes how many parallel sweeps are
needed:

```python
n_sweeps = max(1, math.ceil(strip_width / spacing))
actual_spacing = strip_width / n_sweeps
```

Rounding up `n_sweeps` and dividing the strip evenly prevents the last sweep
from being narrower than the others.

**Example at experiment geometry (400 m area, 60 m altitude, 3 drones):**

```
footprint = 2 × 60 × tan(0.87) ≈ 136.3 m
spacing   = 136.3 × 0.95      ≈ 129.5 m

strip_width = 400 / 3 ≈ 133.3 m
n_sweeps    = ceil(133.3 / 129.5) = 1 sweep per strip
```

Each drone makes a single end-to-end pass down its strip, and adjacent
footprints overlap by ~3 m at the boundaries.

**Which module uses it:** Only the `lawnmower` baseline controller. The
adaptive Voronoi controller uses a density-weighted centroid approach and does
not need predetermined sweep lines — its "distancing" is emergent from the
Voronoi partition rather than derived from FOV geometry directly.

---

## 5. Phase 3 — Flight Control: Altitude Layering (`waypoint_executor.py`)

**File:** [scripts/waypoint_executor.py](../scripts/waypoint_executor.py),
lines 165–168

**What it does:** Assigns each drone its own horizontal altitude layer,
separated by 1.5 m per drone ID, so drones can cross XY paths without risk
of collision.

```python
drone_altitude = altitude + (drone_id - 1) * 1.5
```

**The FOV connection (from the inline comment):**

> 1.5 m between layers is well outside the x500's rotor footprint and keeps
> FOV overlap small enough for detection.

The 1.5 m vertical separation is chosen with the camera FOV in mind. At 40 m
altitude, a 1.5 m altitude difference between two drones produces only a
~3.4 m difference in their ground footprint widths — negligible for coverage
purposes, but meaningful for collision avoidance. If layers were 10 m apart,
drones at lower altitudes would have significantly smaller footprints,
distorting the coverage uniformity that the coordinator assumes.

**Important consequence for the detection publisher:** The position callback
in `detection_publisher.py` stores the live AGL altitude per drone from
`/droneN/position`, which carries `relative_altitude_m` (height above
takeoff). The waypoint executor deliberately publishes AGL rather than AMSL
because the ground-projection formula requires height above ground, not height
above sea level.

---

## 6. Phase 4 — Postprocessing: Coverage Computation (`metrics_compute.py`)

**File:** [scripts/metrics_compute.py](../scripts/metrics_compute.py)

**What it does:** After a trial, reads `positions.csv` and determines which
fraction of the monitoring area was covered at each timestep. Each drone's
camera footprint is modelled as a circle on the ground:

```python
CAMERA_HFOV_RAD = 1.74
footprint_radius = altitude * math.tan(CAMERA_HFOV_RAD / 2.0)
```

For each drone position sample, the circular footprint is rasterised onto a
grid. A cell is "covered" if any drone's circle overlaps it at any point
during the trial. The redundancy ratio is the average number of drone circles
that overlap each covered cell — a direct measure of wasted double-coverage.

**Why this must match the planner:** If the metrics used a different HFOV than
the lawnmower planner, the planner would claim 95% coverage by design while
the metrics would report a different number. Using the same constant ensures
that "coverage" is self-consistent across planning and evaluation.

---

## 7. Orchestration: Lawnmower Duration Cap (`run_experiment.sh`)

**File:** [scripts/run_experiment.sh](../scripts/run_experiment.sh),
lines 218–227

**What it does:** Computes a safety-cap duration for the data logger when
running the lawnmower baseline, since lawnmower terminates on
`/lawnmower/complete` rather than a wall-clock timeout.

```bash
LOGGER_DURATION=$(python3 -c "
import math
a, h, s = $AREA_SIZE, $ALTITUDE, 5.0
hfov = 1.74
footprint = 2*h*math.tan(hfov/2)*0.95
n_sweeps = max(1, math.ceil((a/$NUM_DRONES) / footprint))
length = n_sweeps*a + max(0, n_sweeps-1)*footprint
print(int(max(1200, 4 * (h + length/s))))
")
```

This is the same FOV-based sweep count calculation as in the lawnmower node
itself, used here to estimate total path length and therefore worst-case
mission time. The `× 4` multiplier and `min 1200 s` floor give a wide safety
margin to catch simulator slowdowns or edge cases.

---

## 8. Summary: Data Flow of FOV Information

```
Camera SDF model (x500_mono_cam_down)
    ↓  100° HFOV specification
CAMERA_HFOV_RAD = 1.74  (defined once, shared across files)
    │
    ├─► detection_publisher.py
    │       • computes focal length: fx = (W/2) / tan(HFOV/2)
    │       • enables pixel→ground projection for all detections
    │       • used at RUNTIME on every processed camera frame
    │
    ├─► lawnmower.py
    │       • computes sweep spacing: 2h·tan(HFOV/2)·0.95
    │       • drives waypoint generation for boustrophedon paths
    │       • used at STARTUP when the lawnmower node initialises
    │
    ├─► waypoint_executor.py
    │       • altitude layering (1.5 m) sized to keep FOV overlap small
    │       • applied at TAKEOFF, permanent for the trial
    │
    ├─► metrics_compute.py
    │       • footprint_radius = altitude · tan(HFOV/2)
    │       • rasterised onto grid for coverage % and redundancy computation
    │       • used OFFLINE after the trial completes
    │
    └─► run_experiment.sh
            • duration safety cap for lawnmower trials
            • one-shot shell calculation before node launch
```

FOV information flows in one direction: from the hardware specification
constant outward into geometry calculations. No module feeds computed
footprint sizes back to any other module — the constant is the single source
of truth.
