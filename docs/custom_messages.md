# Custom ROS 2 Messages — `swarm_msgs`

The `swarm_msgs` package defines three custom message types used throughout the
adaptive area monitoring system. All nodes communicate exclusively through these
messages, making controllers interchangeable via the experiment pipeline.

**Package location:** `ros2_ws/src/swarm_msgs/`

**Build:**
```bash
cd ~/Desktop/Swarm/ros2_ws
colcon build --packages-select swarm_msgs
source install/setup.bash
```

**Verify:**
```bash
ros2 interface show swarm_msgs/msg/Detection
ros2 interface show swarm_msgs/msg/TargetWaypoint
ros2 interface show swarm_msgs/msg/SwarmMode
```

---

## 1. Detection.msg

**File:** `ros2_ws/src/swarm_msgs/msg/Detection.msg`

**Purpose:** Published by `detection_publisher.py` when YOLOv8 detects an object
in a drone's camera feed. Contains the detection class, confidence, bounding box,
and estimated world position relative to the drone.

**Definition:**
```
uint8 drone_id
string class_name
float32 confidence
float32[4] bbox           # [x1, y1, x2, y2] in pixels
float64[3] world_position # [x, y, z] estimated ground position in local frame (meters)
builtin_interfaces/Time stamp
```

**Fields:**

| Field | Type | Description |
|-------|------|-------------|
| `drone_id` | uint8 | ID of the drone that made the detection (1-based) |
| `class_name` | string | YOLO/COCO class name (e.g., `car`, `truck`, `person`) |
| `confidence` | float32 | Detection confidence score (0.0 to 1.0) |
| `bbox` | float32[4] | Bounding box in pixel coordinates: [x1, y1, x2, y2] |
| `world_position` | float64[3] | Estimated ground position offset from drone in meters [x, y, z]. X = forward (north), Y = right (east), Z = down. Computed from bbox center using camera FOV and drone altitude. |
| `stamp` | Time | ROS 2 timestamp of the detection |

**Topic pattern:** `/droneN/detection` (e.g., `/drone1/detection`, `/drone2/detection`)

**Publisher:** `scripts/detection_publisher.py`

**Subscribers:**
- `scripts/voronoi_coordinator.py` — uses class, confidence, and position to update the density map
- `scripts/baselines/binary_voronoi.py` — uses position only (binary weighting)
- `scripts/baselines/all_converge.py` — uses class, confidence, and position (same as full system)
- `scripts/data_logger.py` — logs all fields to `detections.csv`

**World position estimation:**
The `world_position` field contains the *drone-relative* ground offset, not global
coordinates. The detection publisher estimates this from the bounding box center
pixel using the camera's horizontal FOV (100 deg) and the drone's altitude:

```
ground_half_width = altitude * tan(HFOV / 2)
x_offset = -normalized_cy * ground_half_height   # image Y → north
y_offset =  normalized_cx * ground_half_width     # image X → east
```

Downstream nodes (coordinator, logger) must add the drone's current position to
get global coordinates: `global = drone_pos + world_position`.

**Example `ros2 topic echo`:**
```yaml
drone_id: 1
class_name: car
confidence: 0.7200000286102295
bbox:
- 580.0
- 412.0
- 695.0
- 498.0
world_position:
- 3.2
- -5.1
- 0.0
stamp:
  sec: 1712345678
  nanosec: 123456789
```

---

## 2. TargetWaypoint.msg

**File:** `ros2_ws/src/swarm_msgs/msg/TargetWaypoint.msg`

**Purpose:** Used for two roles:
1. **Command waypoints:** Published by controllers (coordinator or baselines) to
   tell drones where to fly.
2. **Position reports:** Published by `waypoint_executor.py` to report each
   drone's current GPS position.

**Definition:**
```
uint8 drone_id
float64 latitude
float64 longitude
float64 altitude
uint8 priority            # 0 = normal coverage, 1 = detection response, 2 = exploit formation
```

**Fields:**

| Field | Type | Description |
|-------|------|-------------|
| `drone_id` | uint8 | Drone ID (1-based) |
| `latitude` | float64 | WGS84 latitude in degrees |
| `longitude` | float64 | WGS84 longitude in degrees |
| `altitude` | float64 | Altitude in meters (AMSL) |
| `priority` | uint8 | 0 = normal/explore, 1 = detection-triggered, 2 = exploit formation |

**Priority levels:**

| Value | Meaning | Set by |
|-------|---------|--------|
| 0 | Normal coverage (no detections active) | All controllers |
| 1 | Detection-influenced Voronoi centroid | Coordinator (explore drones during exploit) |
| 2 | Exploit formation waypoint | Coordinator (exploit drones) |

**Topic patterns:**

| Topic | Direction | Publisher | Subscriber |
|-------|-----------|-----------|------------|
| `/droneN/target_waypoint` | Command | Controller | `waypoint_executor.py` |
| `/droneN/position` | Report | `waypoint_executor.py` | Controller, `data_logger.py` |

**GPS reference frame:**
All coordinates use the PX4 SITL default home position as the reference:
- Home latitude: `47.397742`
- Home longitude: `8.545594`

Local XY (meters) is converted to/from GPS using flat-earth approximation in
`voronoi_utils.py` (`xy_to_gps()` / `gps_to_xy()`).

**Example `ros2 topic echo`:**
```yaml
drone_id: 2
latitude: 47.39792
longitude: 8.54538
altitude: 40.0
priority: 1
```

---

## 3. SwarmMode.msg

**File:** `ros2_ws/src/swarm_msgs/msg/SwarmMode.msg`

**Purpose:** Published by `voronoi_coordinator.py` to announce swarm mode
transitions between exploration and exploitation. Used by `data_logger.py` to
log mode events for the coverage-during-event metric.

**Definition:**
```
uint8 MODE_EXPLORE=0
uint8 MODE_EXPLOIT=1

uint8 mode                    # EXPLORE or EXPLOIT
uint8[] exploit_drone_ids     # drones assigned to exploitation
string event_class            # class that triggered exploit mode (e.g., "person", "vehicle", "fire")
float64[3] event_position     # [x, y, z] of the triggering detection in local frame
float32 event_confidence      # confidence of the triggering detection
builtin_interfaces/Time stamp
```

**Fields:**

| Field | Type | Description |
|-------|------|-------------|
| `mode` | uint8 | `0` = EXPLORE (all drones covering), `1` = EXPLOIT (split formation active) |
| `exploit_drone_ids` | uint8[] | Variable-length array of drone IDs assigned to the exploit formation. Empty when mode = EXPLORE. |
| `event_class` | string | YOLO class name of the detection that triggered exploit mode (e.g., `person`, `car`, `fire`). Empty when mode = EXPLORE. |
| `event_position` | float64[3] | Global XY position of the triggering detection [x, y, z] in meters. Zero when mode = EXPLORE. |
| `event_confidence` | float32 | Confidence of the triggering detection (0.0-1.0). Zero when mode = EXPLORE. |
| `stamp` | Time | ROS 2 timestamp of the mode transition |

**Constants:**

| Constant | Value | Description |
|----------|-------|-------------|
| `MODE_EXPLORE` | 0 | All drones in Voronoi coverage mode |
| `MODE_EXPLOIT` | 1 | Some drones split into class-specific formation |

**Topic:** `/swarm/mode` (single global topic, not per-drone)

**Publisher:** `scripts/voronoi_coordinator.py`

**Subscribers:**
- `scripts/data_logger.py` — logs to `swarm_mode.csv` for coverage-during-event analysis

**Mode transitions:**

```
                   high-confidence detection
    EXPLORE  ──────────────────────────────────►  EXPLOIT
       ▲                                              │
       │         timeout OR confidence drop           │
       └──────────────────────────────────────────────┘
```

**Exploit mode triggers when:**
- A detection has confidence >= `exploit_confidence_threshold` (default: 0.4)
- No exploit event is already active

**Exploit mode ends when (whichever comes first):**
- `exploit_timeout_s` seconds have elapsed (default: 30s)
- Detection confidence drops below `exploit_confidence_drop` (default: 0.2)

**Example — entering exploit mode:**
```yaml
mode: 1
exploit_drone_ids: [1, 3]
event_class: person
event_position: [45.2, -30.8, 0.0]
event_confidence: 0.8199999928474426
stamp:
  sec: 1712345700
  nanosec: 500000000
```

**Example — returning to explore mode:**
```yaml
mode: 0
exploit_drone_ids: []
event_class: ''
event_position: [0.0, 0.0, 0.0]
event_confidence: 0.0
stamp:
  sec: 1712345730
  nanosec: 100000000
```

---

## Message Flow Diagram

```
detection_publisher.py                voronoi_coordinator.py
  │                                     │           │
  │  Detection.msg                      │           │  SwarmMode.msg
  ├──► /droneN/detection ──────────────►│           ├──► /swarm/mode ──────► data_logger.py
  │                                     │           │
  │                                     │  TargetWaypoint.msg
  │                                     ├──► /droneN/target_waypoint ──► waypoint_executor.py
  │                                     │                                    │
  │                                     │        TargetWaypoint.msg          │
  │                                     │◄── /droneN/position ◄──────────────┤
  │                                                                          │
  │                                                                          │
  └─────────────────────────────────────────────────────────────► data_logger.py
                                                                  (logs all topics)
```

---

## Data Logged Per Message Type

| Message | Log file | Columns |
|---------|----------|---------|
| Detection | `detections.csv` | timestamp, elapsed_s, drone_id, class_name, confidence, bbox (x1,y1,x2,y2), world_x, world_y |
| TargetWaypoint (position) | `positions.csv` | timestamp, elapsed_s, drone_id, lat, lon, altitude, x_local, y_local |
| TargetWaypoint (waypoint) | `waypoints.csv` | timestamp, elapsed_s, drone_id, lat, lon, altitude, priority, x_local, y_local |
| SwarmMode | `swarm_mode.csv` | timestamp, elapsed_s, mode, exploit_drone_ids, event_class, event_x, event_y, event_confidence |

---

## How Each Novel Element Uses These Messages

### Novel Element 1: Multi-Class Detection -> Heterogeneous Response
- **Detection.msg** `class_name` field determines which formation is triggered
- **TargetWaypoint.msg** `priority=2` for exploit formation waypoints
- **SwarmMode.msg** `event_class` records which class triggered the exploit

### Novel Element 2: Confidence-Weighted Density Function
- **Detection.msg** `confidence` field is multiplied by class priority to weight
  the Voronoi density map Gaussian bumps
- Without this (binary_voronoi ablation), all detections get weight 1.0

### Novel Element 3: Dual-Mode Exploration/Exploitation
- **SwarmMode.msg** announces mode transitions (EXPLORE <-> EXPLOIT)
- **SwarmMode.msg** `exploit_drone_ids` identifies which drones are in formation
- Explore drones get **TargetWaypoint** with `priority=0/1` (Voronoi centroids)
- Exploit drones get **TargetWaypoint** with `priority=2` (formation waypoints)
