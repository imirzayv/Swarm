#!/bin/bash
# Day 3 Runner: Multi-drone CV detection pipeline
#
# This script orchestrates the full Day 3 demo:
#   1. Launches 3 drones with downward cameras (launch_multi_drone.sh)
#   2. Starts the Gazebo↔ROS 2 camera bridge
#   3. Spawns target objects in the Gazebo world
#   4. Sends drones to hover over the target area
#   5. Runs YOLOv8 detection on drone camera feeds
#
# Usage:
#   ./run_day3.sh          # full pipeline
#   ./run_day3.sh --no-sim # skip drone/Gazebo launch (already running)
#
# Prerequisites:
#   - PX4 built (make px4_sitl)
#   - ROS 2 Humble sourced
#   - MicroXRCEAgent running: MicroXRCEAgent udp4 -p 8888
#   - pip install mavsdk ultralytics opencv-python-headless

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LAUNCH_DIR="$HOME/Desktop/Swarm/ros2_ws/src/swarm_bringup/launch"
SKIP_SIM=false

if [ "$1" = "--no-sim" ]; then
    SKIP_SIM=true
    echo "Skipping simulation launch (assuming drones already running)"
fi

echo "╔═══════════════════════════════════════════════════════════════╗"
echo "║           Day 3: Multi-Drone CV Detection Pipeline           ║"
echo "╚═══════════════════════════════════════════════════════════════╝"
echo ""

# ── Step 1: Launch drones ────────────────────────────────────────────────────
if [ "$SKIP_SIM" = false ]; then
    echo "──── Step 1/5: Launching 3 drones with downward cameras ────"
    bash "$SCRIPT_DIR/launch_multi_drone.sh" 3 x500_mono_cam_down
    echo ""
    echo "Waiting 15 seconds for all drones to stabilize..."
    sleep 15
else
    echo "──── Step 1/5: Skipped (--no-sim) ────"
fi

# ── Step 2: Start camera bridge ─────────────────────────────────────────────
echo ""
echo "──── Step 2/5: Starting Gazebo → ROS 2 camera bridge ────"
gnome-terminal --title="Camera Bridge" -- bash -c "
    source /opt/ros/humble/setup.bash
    ros2 launch '$LAUNCH_DIR/camera_bridge.launch.py'
    exec bash
"
echo "Camera bridge started. Waiting 5 seconds..."
sleep 5

# ── Step 3: Spawn target objects ─────────────────────────────────────────────
echo ""
echo "──── Step 3/5: Spawning target objects in Gazebo ────"
bash "$SCRIPT_DIR/spawn_targets.sh"
echo ""
sleep 2

# ── Step 4: Fly drones over target area ──────────────────────────────────────
echo ""
echo "──── Step 4/5: Sending drones to hover over targets ────"
gnome-terminal --title="Drone Waypoints" -- bash -c "
    cd '$SCRIPT_DIR'
    python3 multi_drone_waypoints.py
    exec bash
"
echo "Waypoint mission started. Waiting 30 seconds for drones to reach altitude..."
sleep 30

# ── Step 5: Run YOLOv8 detection ─────────────────────────────────────────────
echo ""
echo "──── Step 5/5: Running YOLOv8 detection on all 3 drones ────"
echo ""
echo "Starting detection with live preview..."
echo "Press Ctrl+C to stop and see the detection summary."
echo ""

python3 "$SCRIPT_DIR/yolo_detector.py" --drones 1 2 3 --show --confidence 0.3
