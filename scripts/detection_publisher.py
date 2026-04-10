#!/usr/bin/env python3
"""
Detection Publisher: YOLOv8 inference on ROS 2 camera feeds with Detection message output.

Architecture (attitude-aware, queue + processor thread):
  - Two ROS subscriptions per drone: camera image + attitude quaternion
  - Both callbacks are lightweight — they just push the message into a per-drone
    deque (image_queue / attitude_queue) tagged with its header timestamp.
  - A single background processor thread:
        1. Pops the oldest image from a drone's image queue
        2. Looks up the attitude that matches the image's stamp
           (SLERP between the bracketing attitude samples; nearest-neighbour
            fallback within `--max-attitude-age`)
        3. Runs YOLOv8 inference on the image
        4. Projects each bbox center onto the ground plane in the world
           (NED) frame using the synced quaternion + drone altitude
        5. Publishes a swarm_msgs/Detection with world-frame offsets

Time synchronisation strategy (between image and attitude streams):
  - Each Image header.stamp is treated as the moment the frame was captured.
  - Each QuaternionStamped header.stamp is treated as the moment the attitude
    was sampled.
  - For each image we search the attitude buffer for the newest sample with
    stamp <= image_stamp ("before") and the oldest sample with stamp > image_stamp
    ("after"). If both exist, we SLERP between them at the image timestamp.
    If only "before" exists we briefly wait for an "after" (`--max-sync-wait`)
    before falling back to the nearest sample within `--max-attitude-age`.

Usage:
    source ~/Desktop/Swarm/ros2_ws/install/setup.bash
    python3 detection_publisher.py --drones 1 2 3
    python3 detection_publisher.py --drones 1 --show --altitude 30

Prerequisites:
    pip install ultralytics opencv-python-headless numpy
    Camera bridge running:
      ros2 launch ~/Desktop/Swarm/ros2_ws/src/swarm_bringup/launch/camera_bridge.launch.py
    Waypoint executor running (publishes /droneN/attitude)
"""

import argparse
import math
import threading
import time
from collections import deque

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from geometry_msgs.msg import QuaternionStamped
from swarm_msgs.msg import Detection
from ultralytics import YOLO


# Camera specs from x500_mono_cam_down model.sdf
CAMERA_HFOV_RAD = 1.74  # ~100 degrees horizontal FOV
CAMERA_WIDTH = 1280
CAMERA_HEIGHT = 960


# ── Camera mount: downward-facing ──────────────────────────────────────────
# Image-frame camera convention:
#     x_cam = right (image column index increases)
#     y_cam = down  (image row index increases)
#     z_cam = forward (into the scene, i.e. away from the lens)
#
# For a camera mounted underneath the drone looking straight down with the
# image x-axis aligned with the body-right axis:
#     body_x (forward) = -y_cam   (top of image is forward)
#     body_y (right)   = +x_cam   (right of image is right)
#     body_z (down)    = +z_cam   (camera optical axis points down in body)
R_BODY_CAM = np.array([
    [0.0, -1.0, 0.0],
    [1.0,  0.0, 0.0],
    [0.0,  0.0, 1.0],
])


def stamp_to_seconds(stamp) -> float:
    """builtin_interfaces/Time → float seconds."""
    return stamp.sec + stamp.nanosec * 1e-9


def quat_normalize(q):
    n = math.sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3])
    if n < 1e-12:
        return (1.0, 0.0, 0.0, 0.0)
    return (q[0] / n, q[1] / n, q[2] / n, q[3] / n)


def quat_slerp(q0, q1, t):
    """Spherical linear interpolation between two unit quaternions (w, x, y, z)."""
    q0 = quat_normalize(q0)
    q1 = quat_normalize(q1)
    dot = q0[0] * q1[0] + q0[1] * q1[1] + q0[2] * q1[2] + q0[3] * q1[3]
    # Take the shorter arc
    if dot < 0.0:
        q1 = (-q1[0], -q1[1], -q1[2], -q1[3])
        dot = -dot
    if dot > 0.9995:
        # Quaternions are nearly parallel — fall back to linear interp + normalize
        result = (
            q0[0] + t * (q1[0] - q0[0]),
            q0[1] + t * (q1[1] - q0[1]),
            q0[2] + t * (q1[2] - q0[2]),
            q0[3] + t * (q1[3] - q0[3]),
        )
        return quat_normalize(result)
    theta_0 = math.acos(dot)
    theta = theta_0 * t
    sin_theta = math.sin(theta)
    sin_theta_0 = math.sin(theta_0)
    s0 = math.cos(theta) - dot * sin_theta / sin_theta_0
    s1 = sin_theta / sin_theta_0
    return (
        s0 * q0[0] + s1 * q1[0],
        s0 * q0[1] + s1 * q1[1],
        s0 * q0[2] + s1 * q1[2],
        s0 * q0[3] + s1 * q1[3],
    )


def quat_to_rotmat(q) -> np.ndarray:
    """Convert (w, x, y, z) to a 3x3 rotation matrix using v' = q ⊗ v ⊗ q*."""
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)],
        [2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)],
    ])


def bbox_to_world_offset(cx_pixel: float, cy_pixel: float,
                         img_w: int, img_h: int,
                         altitude: float,
                         q_ned_body) -> tuple[float, float, float]:
    """
    Project a bbox-center pixel to a ground point (NED z=0) using the drone's
    attitude. Returns (north_offset, east_offset, 0.0) in meters relative to
    the drone's current XY position in the world (NED) frame.

    Parameters
    ----------
    cx_pixel, cy_pixel : pixel coordinates of the bbox center
    img_w, img_h       : image dimensions (px)
    altitude           : drone height above ground (m, positive)
    q_ned_body         : MAVSDK / PX4 attitude quaternion (w, x, y, z) — represents
                         the rotation from NED earth frame to FRD body frame
                         (PX4 vehicle_attitude convention).
    """
    # Pinhole intrinsics from horizontal FOV (square pixels)
    fx = (img_w / 2.0) / math.tan(CAMERA_HFOV_RAD / 2.0)
    fy = fx

    # Pixel ray in camera frame (z forward into scene)
    ray_cam = np.array([
        (cx_pixel - img_w / 2.0) / fx,
        (cy_pixel - img_h / 2.0) / fy,
        1.0,
    ])

    # Camera frame → body (FRD) frame
    ray_body = R_BODY_CAM @ ray_cam

    # Body → world. PX4's vehicle_attitude.q is the rotation NED → body, so
    # the matrix R(q) maps NED vectors to body vectors. We need the inverse
    # (R^T) to lift our body-frame ray into the world (NED) frame.
    R_body_ned = quat_to_rotmat(q_ned_body)
    R_ned_body = R_body_ned.T
    ray_ned = R_ned_body @ ray_body

    # Camera origin in NED, relative to the ground intersection target:
    #   the drone sits at NED z = -altitude (down is +z, ground is z=0).
    #   ray_ned points from the camera toward the scene; for a downward-tilted
    #   camera ray_ned[2] should be > 0 (pointing down).
    if ray_ned[2] <= 1e-6:
        # Ray is horizontal or pointing up — no ground intersection in the
        # forward half-space. Return zero offset.
        return (0.0, 0.0, 0.0)
    t = altitude / ray_ned[2]
    north_off = t * ray_ned[0]
    east_off = t * ray_ned[1]
    return (north_off, east_off, 0.0)


# ── Queue items ────────────────────────────────────────────────────────────
class _ImageItem:
    __slots__ = ("stamp", "frame", "header_stamp")

    def __init__(self, stamp: float, frame: np.ndarray, header_stamp):
        self.stamp = stamp                # float seconds (for sync math)
        self.frame = frame                # BGR np.ndarray
        self.header_stamp = header_stamp  # original builtin_interfaces/Time


class _AttitudeItem:
    __slots__ = ("stamp", "q")

    def __init__(self, stamp: float, q: tuple):
        self.stamp = stamp
        self.q = q  # (w, x, y, z)


class DetectionPublisherNode(Node):
    """ROS 2 node — queue-based YOLO inference with attitude-synced projection."""

    def __init__(self, drone_ids: list[int], model_path: str,
                 show_preview: bool, confidence: float, altitude: float,
                 frame_skip: int, max_attitude_age: float,
                 max_sync_wait: float):
        super().__init__("detection_publisher")

        self.show_preview = show_preview
        self.confidence = confidence
        self.altitude = altitude
        self.drone_ids = drone_ids
        self.frame_skip = max(1, frame_skip)
        self.max_attitude_age = max_attitude_age
        self.max_sync_wait = max_sync_wait

        self.frame_count = {d: 0 for d in drone_ids}
        self.detection_count = {d: 0 for d in drone_ids}
        self.dropped_no_attitude = {d: 0 for d in drone_ids}

        # Per-drone queues
        self._image_queues: dict[int, deque] = {
            d: deque(maxlen=8) for d in drone_ids
        }
        self._attitude_queues: dict[int, deque] = {
            d: deque(maxlen=256) for d in drone_ids
        }
        self._image_locks = {d: threading.Lock() for d in drone_ids}
        self._attitude_locks = {d: threading.Lock() for d in drone_ids}
        self._image_event = threading.Event()
        self._stop_event = threading.Event()

        # Load YOLO model
        self.get_logger().info(f"Loading YOLO model: {model_path}")
        self.model = YOLO(model_path)
        self.get_logger().info("YOLO model loaded")

        # QoS for camera bridge
        cam_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.det_publishers = {}
        self.subscriptions_list = []
        for drone_id in drone_ids:
            pub = self.create_publisher(Detection, f"/drone{drone_id}/detection", 10)
            self.det_publishers[drone_id] = pub

            img_topic = f"/drone{drone_id}/camera/image_raw"
            sub_img = self.create_subscription(
                Image, img_topic,
                lambda msg, did=drone_id: self._image_callback(msg, did),
                cam_qos,
            )
            self.subscriptions_list.append(sub_img)

            att_topic = f"/drone{drone_id}/attitude"
            sub_att = self.create_subscription(
                QuaternionStamped, att_topic,
                lambda msg, did=drone_id: self._attitude_callback(msg, did),
                50,
            )
            self.subscriptions_list.append(sub_att)

            self.get_logger().info(
                f"Drone {drone_id}: {img_topic} + {att_topic} → /drone{drone_id}/detection"
            )

        # Spawn the processor thread
        self._processor_thread = threading.Thread(
            target=self._processor_loop,
            name="detection_processor",
            daemon=True,
        )
        self._processor_thread.start()

        self.get_logger().info(
            f"Detection publisher ready — {len(drone_ids)} drone(s), "
            f"conf={self.confidence}, altitude={self.altitude}m, "
            f"frame_skip={self.frame_skip}, "
            f"max_attitude_age={self.max_attitude_age}s, "
            f"max_sync_wait={self.max_sync_wait}s"
        )

    # ── ROS callbacks (lightweight: enqueue only) ──────────────────────────

    def _image_callback(self, msg: Image, drone_id: int):
        self.frame_count[drone_id] += 1
        if self.frame_count[drone_id] % self.frame_skip != 0:
            return

        try:
            frame = self._ros_image_to_cv2(msg)
        except ValueError as e:
            self.get_logger().warn(f"[Drone {drone_id}] Image error: {e}")
            return

        item = _ImageItem(
            stamp=stamp_to_seconds(msg.header.stamp),
            frame=frame,
            header_stamp=msg.header.stamp,
        )
        with self._image_locks[drone_id]:
            self._image_queues[drone_id].append(item)
        self._image_event.set()

    def _attitude_callback(self, msg: QuaternionStamped, drone_id: int):
        item = _AttitudeItem(
            stamp=stamp_to_seconds(msg.header.stamp),
            q=(msg.quaternion.w, msg.quaternion.x, msg.quaternion.y, msg.quaternion.z),
        )
        with self._attitude_locks[drone_id]:
            self._attitude_queues[drone_id].append(item)

    # ── Processor thread ───────────────────────────────────────────────────

    def _processor_loop(self):
        """Drain the image queues, run YOLO, and publish detections."""
        while not self._stop_event.is_set():
            if not self._image_event.wait(timeout=0.5):
                continue
            self._image_event.clear()

            # Drain across drones until no work remains in this pass
            any_processed = True
            while any_processed and not self._stop_event.is_set():
                any_processed = False
                for drone_id in self.drone_ids:
                    if self._process_one(drone_id):
                        any_processed = True

    def _process_one(self, drone_id: int) -> bool:
        """Process one image from `drone_id`. Returns True if work was done."""
        with self._image_locks[drone_id]:
            if not self._image_queues[drone_id]:
                return False
            image_item = self._image_queues[drone_id].popleft()

        q_synced = self._lookup_attitude(drone_id, image_item.stamp)
        if q_synced is None:
            self.dropped_no_attitude[drone_id] += 1
            if self.dropped_no_attitude[drone_id] % 10 == 1:
                self.get_logger().warn(
                    f"[Drone {drone_id}] No attitude available for image stamp "
                    f"{image_item.stamp:.3f} (dropped: "
                    f"{self.dropped_no_attitude[drone_id]})"
                )
            return True

        frame = image_item.frame
        results = self.model(frame, conf=self.confidence, verbose=False)
        detections = results[0].boxes
        img_h, img_w = frame.shape[:2]

        if len(detections) > 0:
            self.detection_count[drone_id] += len(detections)
            self.get_logger().info(
                f"[Drone {drone_id}] t={image_item.stamp:.3f} — "
                f"{len(detections)} detection(s)"
            )

            for box in detections:
                cls_id = int(box.cls[0])
                cls_name = self.model.names[cls_id]
                conf = float(box.conf[0])
                x1, y1, x2, y2 = box.xyxy[0].tolist()

                cx = (x1 + x2) / 2
                cy = (y1 + y2) / 2
                north_off, east_off, _ = bbox_to_world_offset(
                    cx, cy, img_w, img_h, self.altitude, q_synced
                )

                det_msg = Detection()
                det_msg.drone_id = drone_id
                det_msg.class_name = cls_name
                det_msg.confidence = conf
                det_msg.bbox = [float(x1), float(y1), float(x2), float(y2)]
                det_msg.world_position = [float(north_off), float(east_off), 0.0]
                # Preserve the original capture timestamp so downstream nodes
                # can reason about latency.
                det_msg.stamp = image_item.header_stamp

                self.det_publishers[drone_id].publish(det_msg)

                self.get_logger().info(
                    f"  → {cls_name} ({conf:.2f}) "
                    f"bbox=[{x1:.0f},{y1:.0f},{x2:.0f},{y2:.0f}] "
                    f"offset_NE=({north_off:.1f},{east_off:.1f})m"
                )

        if self.show_preview:
            annotated = results[0].plot()
            cv2.imshow(f"Drone {drone_id} - YOLOv8", annotated)
            cv2.waitKey(1)

        return True

    def _lookup_attitude(self, drone_id: int, image_stamp: float):
        """
        Find a quaternion synchronized to `image_stamp`.

        Strategy:
          1. SLERP between the two attitude samples bracketing the image stamp
             (newest "before" and oldest "after").
          2. If only "before" exists, briefly wait (`max_sync_wait`) for a
             newer sample so we can interpolate.
          3. Fall back to the nearest sample if it is within `max_attitude_age`.
          4. Otherwise drop the image.
        """
        deadline = time.time() + self.max_sync_wait
        while True:
            with self._attitude_locks[drone_id]:
                queue_snapshot = list(self._attitude_queues[drone_id])

            before = None
            after = None
            for item in queue_snapshot:
                if item.stamp <= image_stamp:
                    if before is None or item.stamp > before.stamp:
                        before = item
                else:
                    if after is None or item.stamp < after.stamp:
                        after = item

            if before is not None and after is not None:
                span = after.stamp - before.stamp
                if span <= 1e-9:
                    return before.q
                t = (image_stamp - before.stamp) / span
                t = max(0.0, min(1.0, t))
                return quat_slerp(before.q, after.q, t)

            # Only one side available — try to wait for the other before
            # falling back to nearest-neighbour.
            if time.time() < deadline:
                time.sleep(0.005)
                continue

            if before is not None:
                age = image_stamp - before.stamp
                if age <= self.max_attitude_age:
                    return before.q
                return None
            if after is not None:
                gap = after.stamp - image_stamp
                if gap <= self.max_attitude_age:
                    return after.q
                return None
            return None

    # ── Helpers ────────────────────────────────────────────────────────────

    def shutdown(self):
        self._stop_event.set()
        self._image_event.set()
        if self._processor_thread.is_alive():
            self._processor_thread.join(timeout=1.0)

    def _ros_image_to_cv2(self, msg: Image) -> np.ndarray:
        """Convert sensor_msgs/Image to BGR numpy array."""
        encoding = msg.encoding.lower()
        h, w = msg.height, msg.width

        if encoding in ("rgb8", "8uc3"):
            frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w, 3)
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        elif encoding == "bgr8":
            frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w, 3)
        elif encoding in ("rgba8", "8uc4"):
            frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w, 4)
            frame = cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)
        elif encoding in ("bgra8",):
            frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w, 4)
            frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
        elif encoding in ("mono8", "8uc1"):
            frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w)
            frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
        else:
            raise ValueError(f"Unsupported encoding: {msg.encoding}")

        return frame


def main():
    parser = argparse.ArgumentParser(
        description="YOLOv8 detection publisher (queue + attitude-synced projection)"
    )
    parser.add_argument("--drones", type=int, nargs="+", default=[1],
                        help="Drone IDs to monitor (default: 1)")
    parser.add_argument("--model", type=str, default="yolov8n.pt",
                        help="YOLO model (default: yolov8n.pt)")
    parser.add_argument("--show", action="store_true",
                        help="Show live annotated preview window")
    parser.add_argument("--confidence", type=float, default=0.25,
                        help="Detection confidence threshold (default: 0.25)")
    parser.add_argument("--altitude", type=float, default=30.0,
                        help="Drone height above ground in meters (default: 30)")
    parser.add_argument("--frame-skip", type=int, default=5,
                        help="Process every Nth frame (default: 5)")
    parser.add_argument("--max-attitude-age", type=float, default=0.1,
                        help="Max age (s) of an attitude sample to accept "
                             "as nearest-neighbour (default: 0.1)")
    parser.add_argument("--max-sync-wait", type=float, default=0.05,
                        help="Max time (s) to wait for a bracketing "
                             "attitude sample before falling back (default: 0.05)")
    args = parser.parse_args()

    rclpy.init()
    node = DetectionPublisherNode(
        drone_ids=args.drones,
        model_path=args.model,
        show_preview=args.show,
        confidence=args.confidence,
        altitude=args.altitude,
        frame_skip=args.frame_skip,
        max_attitude_age=args.max_attitude_age,
        max_sync_wait=args.max_sync_wait,
    )

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n=== Detection Summary ===")
        for d in args.drones:
            print(
                f"  Drone {d}: {node.frame_count[d]} frames, "
                f"{node.detection_count[d]} detections, "
                f"{node.dropped_no_attitude[d]} dropped (no attitude)"
            )
    finally:
        node.shutdown()
        if args.show:
            cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
