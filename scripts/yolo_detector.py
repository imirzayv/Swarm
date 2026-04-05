#!/usr/bin/env python3
"""
Day 3 Task: YOLOv8 inference on ROS 2 camera topic streams.

Subscribes to drone camera image topics (bridged from Gazebo via ros_gz_bridge),
runs YOLOv8 detection on each frame, and prints results to the terminal.
Optionally displays an annotated live preview window.

Usage:
    # Detect on drone 1 only (default):
    python3 yolo_detector.py

    # Detect on drones 1, 2, 3:
    python3 yolo_detector.py --drones 1 2 3

    # With live OpenCV preview window:
    python3 yolo_detector.py --drones 1 --show

    # Use a specific YOLO model:
    python3 yolo_detector.py --model yolov8s.pt

Prerequisites:
    pip install ultralytics opencv-python-headless numpy
    # ROS 2 Humble sourced
    # Camera bridge running:
    #   ros2 launch ~/Desktop/Swarm/ros2_ws/src/swarm_bringup/launch/camera_bridge.launch.py
"""

import argparse
import sys
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from ultralytics import YOLO


class YOLODetectorNode(Node):
    """ROS 2 node that runs YOLOv8 on drone camera feeds."""

    def __init__(self, drone_ids: list[int], model_path: str, show_preview: bool,
                 confidence: float):
        super().__init__("yolo_detector")

        self.show_preview = show_preview
        self.confidence = confidence
        self.frame_count = {d: 0 for d in drone_ids}
        self.detection_count = {d: 0 for d in drone_ids}

        # Load YOLO model (downloads automatically if not present)
        self.get_logger().info(f"Loading YOLO model: {model_path}")
        self.model = YOLO(model_path)
        self.get_logger().info("YOLO model loaded successfully")

        # Use BEST_EFFORT QoS to match Gazebo camera bridge output
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Subscribe to each drone's camera topic
        self.subscriptions_list = []
        for drone_id in drone_ids:
            topic = f"/drone{drone_id}/camera/image_raw"
            sub = self.create_subscription(
                Image,
                topic,
                lambda msg, did=drone_id: self._image_callback(msg, did),
                qos,
            )
            self.subscriptions_list.append(sub)
            self.get_logger().info(f"Subscribed to {topic}")

        self.get_logger().info(
            f"YOLOv8 detector ready — monitoring {len(drone_ids)} drone(s) "
            f"[confidence threshold: {self.confidence}]"
        )

    def _image_callback(self, msg: Image, drone_id: int):
        """Convert ROS Image to numpy, run YOLO, print detections."""
        self.frame_count[drone_id] += 1

        # Only process every 5th frame to avoid overloading
        if self.frame_count[drone_id] % 5 != 0:
            return

        # Convert ROS Image message to OpenCV numpy array
        try:
            frame = self._ros_image_to_cv2(msg)
        except ValueError as e:
            self.get_logger().warn(f"[Drone {drone_id}] Image conversion error: {e}")
            return

        # Run YOLOv8 inference
        results = self.model(frame, conf=self.confidence, verbose=False)

        # Process detections
        detections = results[0].boxes
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
                self.get_logger().info(
                    f"  → {cls_name} ({conf:.2f}) at "
                    f"[{x1:.0f}, {y1:.0f}, {x2:.0f}, {y2:.0f}]"
                )
        else:
            # Print a status line every 50 frames even with no detections
            if self.frame_count[drone_id] % 50 == 0:
                self.get_logger().info(
                    f"[Drone {drone_id}] Frame #{self.frame_count[drone_id]} — "
                    f"no detections (total so far: {self.detection_count[drone_id]})"
                )

        # Optional: show annotated frame in an OpenCV window
        if self.show_preview:
            annotated = results[0].plot()
            window_name = f"Drone {drone_id} - YOLOv8"
            cv2.imshow(window_name, annotated)
            cv2.waitKey(1)

    def _ros_image_to_cv2(self, msg: Image) -> np.ndarray:
        """Convert a sensor_msgs/Image to a BGR numpy array for OpenCV/YOLO."""
        encoding = msg.encoding.lower()
        h, w = msg.height, msg.width

        if encoding in ("rgb8", "8uc3"):
            # 3-channel RGB → convert to BGR for OpenCV
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
        description="YOLOv8 object detection on ROS 2 drone camera feeds"
    )
    parser.add_argument(
        "--drones", type=int, nargs="+", default=[1],
        help="Drone IDs to monitor (default: 1)",
    )
    parser.add_argument(
        "--model", type=str, default="yolov8n.pt",
        help="YOLO model to use (default: yolov8n.pt — auto-downloads)",
    )
    parser.add_argument(
        "--show", action="store_true",
        help="Show live annotated preview window (requires display)",
    )
    parser.add_argument(
        "--confidence", type=float, default=0.25,
        help="Detection confidence threshold (default: 0.25)",
    )
    args = parser.parse_args()

    rclpy.init()
    node = YOLODetectorNode(
        drone_ids=args.drones,
        model_path=args.model,
        show_preview=args.show,
        confidence=args.confidence,
    )

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Print summary on exit
        print("\n=== Detection Summary ===")
        for d in args.drones:
            print(
                f"  Drone {d}: {node.frame_count[d]} frames processed, "
                f"{node.detection_count[d]} total detections"
            )
    finally:
        if args.show:
            cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
