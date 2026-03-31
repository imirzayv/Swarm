#!/usr/bin/env python3
"""
Day 2 Task: Command 3 drones to take off and fly to different waypoints using MAVSDK.
Each drone connects on a different port corresponding to its PX4 SITL instance.

Usage:
    pip install mavsdk
    python3 multi_drone_waypoints.py
"""

import asyncio
from mavsdk import System


async def run_drone(instance_id: int, target_lat: float, target_lon: float, target_alt: float):
    """Connect to a single drone, arm, take off, and fly to a waypoint."""

    udp_port = 14540 + instance_id
    grpc_port = 50040 + instance_id

    # Each System() starts its own mavsdk_server on a unique gRPC port
    drone = System(port=grpc_port)

    print(f"[Drone {instance_id}] Connecting on udp://:{udp_port} (gRPC: {grpc_port})...")
    await drone.connect(system_address=f"udp://:{udp_port}")

    # Wait for connection
    print(f"[Drone {instance_id}] Waiting for connection...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"[Drone {instance_id}] Connected!")
            break

    # Wait for GPS fix
    print(f"[Drone {instance_id}] Waiting for global position estimate...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print(f"[Drone {instance_id}] Global position OK")
            break

    # Arm and take off
    print(f"[Drone {instance_id}] Arming...")
    await drone.action.arm()

    print(f"[Drone {instance_id}] Taking off...")
    await drone.action.takeoff()
    await asyncio.sleep(10)

    # Fly to waypoint
    print(f"[Drone {instance_id}] Flying to waypoint ({target_lat}, {target_lon}, {target_alt}m)...")
    await drone.action.goto_location(target_lat, target_lon, target_alt, 0.0)
    await asyncio.sleep(15)

    # Hover for observation
    print(f"[Drone {instance_id}] Arrived. Hovering for 10 seconds...")
    await asyncio.sleep(10)

    # Return to launch
    print(f"[Drone {instance_id}] Returning to launch...")
    await drone.action.return_to_launch()

    print(f"[Drone {instance_id}] Mission complete.")


async def main():
    # Default PX4 SITL home: 47.397742, 8.545594
    # Waypoints are small offsets from home
    waypoints = [
        (47.3978, 8.5456, 30.0),   # Drone 0 — north
        (47.3975, 8.5461, 40.0),   # Drone 1 — northeast
        (47.3972, 8.5450, 35.0),   # Drone 2 — west
    ]

    # Launch drones sequentially with a small delay
    # (starting all at once can overwhelm mavsdk_server spawning)
    tasks = []
    for i, (lat, lon, alt) in enumerate(waypoints, start=1):
        task = asyncio.create_task(run_drone(i, lat, lon, alt))
        tasks.append(task)
        await asyncio.sleep(2)  # stagger connections

    await asyncio.gather(*tasks)


if __name__ == "__main__":
    asyncio.run(main())
