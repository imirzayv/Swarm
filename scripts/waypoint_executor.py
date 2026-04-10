#!/usr/bin/env python3
"""
Waypoint Executor: MAVSDK-based ROS 2 node that flies drones to commanded waypoints.

Subscribes to /droneN/target_waypoint (TargetWaypoint) and calls MAVSDK goto_location().
Publishes current GPS position to /droneN/position (TargetWaypoint with priority=0) at 2 Hz.
Publishes current attitude to /droneN/attitude (QuaternionStamped) at 30 Hz.

On startup, arms all drones and takes off to --altitude meters.

Usage:
    source ~/Desktop/Swarm/ros2_ws/install/setup.bash
    python3 waypoint_executor.py --drones 1 2 3
    python3 waypoint_executor.py --drones 1 2 3 --altitude 25

Architecture:
    - Main thread: asyncio event loop for MAVSDK (async drone control)
    - Background thread: rclpy spin (ROS 2 callbacks)
    - Waypoint callbacks queue commands; asyncio tasks consume them
"""

import argparse
import asyncio
import threading
from collections import defaultdict

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from mavsdk import System
from geometry_msgs.msg import QuaternionStamped
from swarm_msgs.msg import TargetWaypoint


class WaypointExecutorNode(Node):
    """ROS 2 node that receives waypoints and forwards them to MAVSDK."""

    def __init__(self, drone_ids: list[int], altitude: float):
        super().__init__("waypoint_executor")

        self.drone_ids = drone_ids
        self.default_altitude = altitude

        # Thread-safe queue: drone_id -> latest waypoint
        self._wp_lock = threading.Lock()
        self._pending_wp: dict[int, TargetWaypoint] = {}

        # Latest known positions (updated by asyncio telemetry tasks)
        self._pos_lock = threading.Lock()
        self._positions: dict[int, tuple[float, float, float]] = {}

        # Latest known attitudes (updated by asyncio telemetry tasks)
        self._att_lock = threading.Lock()
        self._attitudes: dict[int, tuple[float, float, float, float]] = {}  # (w,x,y,z)

        # QoS for publishing positions
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # Subscribers and publishers per drone
        self.pos_publishers = {}
        self.att_publishers = {}
        self._subs = []
        for did in drone_ids:
            # Subscribe to waypoint commands
            sub = self.create_subscription(
                TargetWaypoint,
                f"/drone{did}/target_waypoint",
                lambda msg, d=did: self._waypoint_callback(msg, d),
                10,
            )
            self._subs.append(sub)

            # Publish current position
            pub = self.create_publisher(TargetWaypoint, f"/drone{did}/position", qos)
            self.pos_publishers[did] = pub

            # Publish current attitude
            att_pub = self.create_publisher(
                QuaternionStamped, f"/drone{did}/attitude", 50
            )
            self.att_publishers[did] = att_pub

            self.get_logger().info(
                f"Drone {did}: sub /drone{did}/target_waypoint, "
                f"pub /drone{did}/position + /drone{did}/attitude"
            )

        # Timer to publish positions at 2 Hz
        self.create_timer(0.5, self._publish_positions)
        # Timer to publish attitudes at 30 Hz
        self.create_timer(1.0 / 30.0, self._publish_attitudes)

        self.get_logger().info(
            f"Waypoint executor ready — {len(drone_ids)} drone(s), "
            f"takeoff altitude={altitude}m"
        )

    def _waypoint_callback(self, msg: TargetWaypoint, drone_id: int):
        """Store the latest waypoint for a drone (consumed by asyncio loop)."""
        with self._wp_lock:
            self._pending_wp[drone_id] = msg
        self.get_logger().info(
            f"[Drone {drone_id}] New waypoint: "
            f"({msg.latitude:.6f}, {msg.longitude:.6f}, {msg.altitude:.1f}m) "
            f"priority={msg.priority}"
        )

    def get_pending_waypoint(self, drone_id: int) -> TargetWaypoint | None:
        """Pop the pending waypoint for a drone (called from asyncio)."""
        with self._wp_lock:
            return self._pending_wp.pop(drone_id, None)

    def update_position(self, drone_id: int, lat: float, lon: float, alt: float):
        """Update cached position (called from asyncio telemetry task)."""
        with self._pos_lock:
            self._positions[drone_id] = (lat, lon, alt)

    def update_attitude(self, drone_id: int,
                        w: float, x: float, y: float, z: float):
        """Update cached attitude quaternion (called from asyncio telemetry task)."""
        with self._att_lock:
            self._attitudes[drone_id] = (w, x, y, z)

    def _publish_positions(self):
        """Timer callback: publish all known positions at 2 Hz."""
        with self._pos_lock:
            positions = dict(self._positions)

        for did, (lat, lon, alt) in positions.items():
            msg = TargetWaypoint()
            msg.drone_id = did
            msg.latitude = lat
            msg.longitude = lon
            msg.altitude = alt
            msg.priority = 0
            self.pos_publishers[did].publish(msg)

    def _publish_attitudes(self):
        """Timer callback: publish all known attitudes at 30 Hz."""
        with self._att_lock:
            attitudes = dict(self._attitudes)

        now = self.get_clock().now().to_msg()
        for did, (w, x, y, z) in attitudes.items():
            msg = QuaternionStamped()
            msg.header.stamp = now
            msg.header.frame_id = "map"
            msg.quaternion.w = w
            msg.quaternion.x = x
            msg.quaternion.y = y
            msg.quaternion.z = z
            self.att_publishers[did].publish(msg)


async def drone_task(node: WaypointExecutorNode, drone_id: int, altitude: float):
    """Async task: connect to one drone, arm, takeoff, then follow waypoints."""

    udp_port = 14540 + drone_id
    grpc_port = 50040 + drone_id

    drone = System(port=grpc_port)
    node.get_logger().info(
        f"[Drone {drone_id}] Connecting on udp://:{udp_port} (gRPC: {grpc_port})..."
    )
    await drone.connect(system_address=f"udp://:{udp_port}")

    # Wait for connection
    async for state in drone.core.connection_state():
        if state.is_connected:
            node.get_logger().info(f"[Drone {drone_id}] Connected")
            break

    # Wait for GPS
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            node.get_logger().info(f"[Drone {drone_id}] GPS OK")
            break

    # Arm and takeoff
    node.get_logger().info(f"[Drone {drone_id}] Arming...")
    await drone.action.arm()
    await drone.action.set_takeoff_altitude(altitude)
    node.get_logger().info(f"[Drone {drone_id}] Taking off to {altitude}m...")
    await drone.action.takeoff()
    await asyncio.sleep(10)
    node.get_logger().info(f"[Drone {drone_id}] Airborne")

    # Start telemetry tasks (position + attitude)
    asyncio.create_task(telemetry_task(node, drone, drone_id))
    asyncio.create_task(attitude_telemetry_task(node, drone, drone_id))

    # Main loop: check for waypoints every 0.5s
    while True:
        wp = node.get_pending_waypoint(drone_id)
        if wp is not None:
            node.get_logger().info(
                f"[Drone {drone_id}] Executing waypoint → "
                f"({wp.latitude:.6f}, {wp.longitude:.6f}, {wp.altitude:.1f}m)"
            )
            await drone.action.goto_location(
                wp.latitude, wp.longitude, wp.altitude, float("nan")
            )
        await asyncio.sleep(0.5)


async def telemetry_task(node: WaypointExecutorNode, drone: System, drone_id: int):
    """Stream position telemetry and update the node's position cache."""
    async for pos in drone.telemetry.position():
        node.update_position(
            drone_id,
            pos.latitude_deg,
            pos.longitude_deg,
            pos.absolute_altitude_m,
        )


async def attitude_telemetry_task(node: WaypointExecutorNode,
                                  drone: System, drone_id: int):
    """Stream attitude quaternion telemetry and update the node's cache."""
    async for q in drone.telemetry.attitude_quaternion():
        node.update_attitude(drone_id, q.w, q.x, q.y, q.z)


def ros_spin_thread(node: WaypointExecutorNode):
    """Run rclpy.spin in a background thread."""
    rclpy.spin(node)


async def async_main(node: WaypointExecutorNode, drone_ids: list[int], altitude: float):
    """Launch all drone tasks concurrently."""
    tasks = []
    for did in drone_ids:
        tasks.append(asyncio.create_task(drone_task(node, did, altitude)))
        await asyncio.sleep(2)  # stagger connections
    await asyncio.gather(*tasks)


def main():
    parser = argparse.ArgumentParser(
        description="MAVSDK waypoint executor with ROS 2 interface"
    )
    parser.add_argument(
        "--drones", type=int, nargs="+", default=[1],
        help="Drone IDs to control (default: 1)",
    )
    parser.add_argument(
        "--altitude", type=float, default=30.0,
        help="Takeoff altitude in meters (default: 30)",
    )
    args = parser.parse_args()

    rclpy.init()
    node = WaypointExecutorNode(drone_ids=args.drones, altitude=args.altitude)

    # Spin ROS 2 in a background thread
    spin_thread = threading.Thread(target=ros_spin_thread, args=(node,), daemon=True)
    spin_thread.start()

    try:
        asyncio.run(async_main(node, args.drones, args.altitude))
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down waypoint executor")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
