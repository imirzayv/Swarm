#!/usr/bin/env python3
"""
Detection Publisher: YOLOv8 inference on ROS 2 camera feeds with Detection message output.

Refactored from yolo_detector.py (Day 3). Adds:
  - Publishes swarm_msgs/msg/Detection on /droneN/detection
  - Estimates world position from bbox center using drone altitude + camera FOV
  - Accepts --altitude parameter for position estimation

Usage:
    # Source the workspace first:
    #   source ~/Desktop/Swarm/ros2_ws/install/setup.bash

    python3 detection_publisher.py --drones 1 2 3
    python3 detection_publisher.py --drones 1 --show --altitude 30

Prerequisites:
    pip install ultralytics opencv-python-headless numpy
    Camera bridge running:
      ros2 launch ~/Desktop/Swarm/ros2_ws/src/swarm_bringup/launch/camera_bridge.launch.py
"""

import argparse
import math

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from swarm_msgs.msg import Detection
from ultralytics import YOLO


# Camera specs from x500_mono_cam_down model.sdf
CAMERA_HFOV_RAD = 1.74  # ~100 degrees horizontal FOV
CAMERA_WIDTH = 1280
CAMERA_HEIGHT = 960


def bbox_center_to_local_xy(cx_pixel: float, cy_pixel: float,
                            img_w: int, img_h: int,
                            altitude: float) -> tuple[float, float]:
    """
    Estimate ground position (x, y) in drone-local frame from a bbox center pixel.

    For a downward-facing camera:
      - Image center = directly below the drone (0, 0)
      - Pixel offset from center maps to ground offset via altitude and FOV

    Returns (x_offset, y_offset) in meters relative to the drone's position.
    """
    # Ground footprint half-widths at the given altitude
    half_ground_w = altitude * math.tan(CAMERA_HFOV_RAD / 2)
    # Vertical FOV derived from aspect ratio
    vfov = 2 * math.atan(math.tan(CAMERA_HFOV_RAD / 2) * img_h / img_w)
    half_ground_h = altitude * math.tan(vfov / 2)

    # Normalized pixel offset from image center: [-1, 1]
    nx = (cx_pixel - img_w / 2) / (img_w / 2)
    ny = (cy_pixel - img_h / 2) / (img_h / 2)

    # Map to ground offsets (x = forward, y = left in NED-like frame)
    x_offset = -ny * half_ground_h  # negative because image y increases downward
    y_offset = nx * half_ground_w

    return x_offset, y_offset


class DetectionPublisherNode(Node):
    """ROS 2 node: YOLOv8 inference + Detection message publishing."""

    def __init__(self, drone_ids: list[int], model_path: str,
                 show_preview: bool, confidence: float, altitude: float):
        super().__init__("detection_publisher")

        self.show_preview = show_preview
        self.confidence = confidence
        self.altitude = altitude
        self.drone_ids = drone_ids
        self.frame_count = {d: 0 for d in drone_ids}
        self.detection_count = {d: 0 for d in drone_ids}

        # Load YOLO model
        self.get_logger().info(f"Loading YOLO model: {model_path}")
        self.model = YOLO(model_path)
        self.get_logger().info("YOLO model loaded")

        # QoS matching Gazebo camera bridge
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Create a publisher and subscriber per drone
        self.det_publishers = {}
        self.subscriptions_list = []
        for drone_id in drone_ids:
            # Publisher: /droneN/detection
            pub = self.create_publisher(Detection, f"/drone{drone_id}/detection", 10)
            self.det_publishers[drone_id] = pub

            # Subscriber: /droneN/camera/image_raw
            topic = f"/drone{drone_id}/camera/image_raw"
            sub = self.create_subscription(
                Image, topic,
                lambda msg, did=drone_id: self._image_callback(msg, did),
                qos,
            )
            self.subscriptions_list.append(sub)
            self.get_logger().info(f"Drone {drone_id}: {topic} → /drone{drone_id}/detection")

        self.get_logger().info(
            f"Detection publisher ready — {len(drone_ids)} drone(s), "
            f"conf={self.confidence}, altitude={self.altitude}m"
        )

    def _image_callback(self, msg: Image, drone_id: int):
        """Run YOLO inference and publish Detection messages."""
        self.frame_count[drone_id] += 1

        # Process every 5th frame
        if self.frame_count[drone_id] % 5 != 0:
            return

        try:
            frame = self._ros_image_to_cv2(msg)
        except ValueError as e:
            self.get_logger().warn(f"[Drone {drone_id}] Image error: {e}")
            return

        results = self.model(frame, conf=self.confidence, verbose=False)
        detections = results[0].boxes
        img_h, img_w = frame.shape[:2]

        if len(detections) > 0:
            self.detection_count[drone_id] += len(detections)
            self.get_logger().info(
                f"[Drone {drone_id}] Frame #{self.frame_count[drone_id]} — "
                f"{len(detections)} detection(s):"
            )

            for box in detections:
                cls_id = int(box.cls[0])
                cls_name = self.model.names[cls_id]
                conf = float(box.conf[0])
                x1, y1, x2, y2 = box.xyxy[0].tolist()

                # Estimate world position from bbox center
                cx = (x1 + x2) / 2
                cy = (y1 + y2) / 2
                x_off, y_off = bbox_center_to_local_xy(
                    cx, cy, img_w, img_h, self.altitude
                )

                # Build and publish Detection message
                det_msg = Detection()
                det_msg.drone_id = drone_id
                det_msg.class_name = cls_name
                det_msg.confidence = conf
                det_msg.bbox = [float(x1), float(y1), float(x2), float(y2)]
                det_msg.world_position = [x_off, y_off, 0.0]
                det_msg.stamp = self.get_clock().now().to_msg()

                self.det_publishers[drone_id].publish(det_msg)

                self.get_logger().info(
                    f"  → {cls_name} ({conf:.2f}) "
                    f"bbox=[{x1:.0f},{y1:.0f},{x2:.0f},{y2:.0f}] "
                    f"pos=({x_off:.1f}, {y_off:.1f})m"
                )
        else:
            if self.frame_count[drone_id] % 50 == 0:
                self.get_logger().info(
                    f"[Drone {drone_id}] Frame #{self.frame_count[drone_id]} — "
                    f"no detections (total: {self.detection_count[drone_id]})"
                )

        if self.show_preview:
            annotated = results[0].plot()
            cv2.imshow(f"Drone {drone_id} - YOLOv8", annotated)
            cv2.waitKey(1)

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
        description="YOLOv8 detection publisher for drone camera feeds"
    )
    parser.add_argument(
        "--drones", type=int, nargs="+", default=[1],
        help="Drone IDs to monitor (default: 1)",
    )
    parser.add_argument(
        "--model", type=str, default="yolov8n.pt",
        help="YOLO model (default: yolov8n.pt)",
    )
    parser.add_argument(
        "--show", action="store_true",
        help="Show live annotated preview window",
    )
    parser.add_argument(
        "--confidence", type=float, default=0.25,
        help="Detection confidence threshold (default: 0.25)",
    )
    parser.add_argument(
        "--altitude", type=float, default=30.0,
        help="Assumed drone altitude in meters for position estimation (default: 30)",
    )
    args = parser.parse_args()

    rclpy.init()
    node = DetectionPublisherNode(
        drone_ids=args.drones,
        model_path=args.model,
        show_preview=args.show,
        confidence=args.confidence,
        altitude=args.altitude,
    )

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n=== Detection Summary ===")
        for d in args.drones:
            print(
                f"  Drone {d}: {node.frame_count[d]} frames, "
                f"{node.detection_count[d]} detections"
            )
    finally:
        if args.show:
            cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
