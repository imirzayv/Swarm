#!/bin/bash
# Day 7: Full adaptive area monitoring pipeline.
#
# Launches all nodes for the Voronoi-based adaptive coverage system:
#   1. Multi-drone simulation (PX4 SITL + Gazebo)
#   2. Camera bridge (Gazebo → ROS 2)
#   3. Spawn target objects
#   4. Waypoint executor (MAVSDK control + position publishing)
#   5. Detection publisher (YOLOv8 inference + Detection messages)
#   6. Voronoi coordinator (adaptive coverage algorithm)
#
# Usage:
#   ./run_adaptive.sh                    # full pipeline, 3 drones
#   ./run_adaptive.sh --no-sim           # skip drone launch (already running)
#   ./run_adaptive.sh --drones 5         # use 5 drones
#   ./run_adaptive.sh --no-sim --no-spawn # skip sim + spawn (targets already placed)

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
LAUNCH_DIR="$PROJECT_DIR/ros2_ws/src/swarm_bringup/launch"
ROS2_WS="$PROJECT_DIR/ros2_ws"

# ── Defaults ─────────────────────────────────────────────────────────────────
NUM_DRONES=3
ALTITUDE=30
SKIP_SIM=false
SKIP_SPAWN=false
DETECTION_CONFIDENCE=0.15
AREA_SIZE=40
DETECTION_WEIGHT=10.0
DETECTION_SIGMA=5.0
DECAY_HALF_LIFE=30.0

# ── Parse arguments ──────────────────────────────────────────────────────────
while [[ $# -gt 0 ]]; do
    case $1 in
        --no-sim)     SKIP_SIM=true; shift ;;
        --no-spawn)   SKIP_SPAWN=true; shift ;;
        --drones)     NUM_DRONES=$2; shift 2 ;;
        --altitude)   ALTITUDE=$2; shift 2 ;;
        --confidence) DETECTION_CONFIDENCE=$2; shift 2 ;;
        --area-size)  AREA_SIZE=$2; shift 2 ;;
        *) echo "Unknown option: $1"; exit 1 ;;
    esac
done

# Build drone ID list: "1 2 3" for NUM_DRONES=3
DRONE_IDS=$(seq -s ' ' 1 $NUM_DRONES)

# ── Environment setup ────────────────────────────────────────────────────────
# Activate conda swarm env + ROS 2 + workspace
SETUP_CMD="export PATH=/home/ilham/anaconda3/envs/swarm/bin:\$PATH && source /opt/ros/humble/setup.bash && source $ROS2_WS/install/setup.bash"

echo "╔══════════════════════════════════════════════════════════════════╗"
echo "║        Adaptive Area Monitoring — Full Pipeline Launch          ║"
echo "╠══════════════════════════════════════════════════════════════════╣"
echo "║  Drones: $NUM_DRONES  |  Altitude: ${ALTITUDE}m  |  Area: ${AREA_SIZE}m       ║"
echo "║  Detection confidence: $DETECTION_CONFIDENCE                              ║"
echo "║  Detection weight: $DETECTION_WEIGHT  |  Sigma: ${DETECTION_SIGMA}m  |  Decay: ${DECAY_HALF_LIFE}s    ║"
echo "╚══════════════════════════════════════════════════════════════════╝"
echo ""

# ── Step 1: Launch drones ────────────────────────────────────────────────────
if [ "$SKIP_SIM" = false ]; then
    echo "──── Step 1/6: Launching $NUM_DRONES drones with downward cameras ────"
    bash "$SCRIPT_DIR/launch_multi_drone.sh" "$NUM_DRONES" x500_mono_cam_down
    echo ""
    echo "Waiting 15s for drones to stabilize..."
    sleep 15
else
    echo "──── Step 1/6: Skipped (--no-sim) ────"
fi

# ── Step 2: Camera bridge ───────────────────────────────────────────────────
echo ""
echo "──── Step 2/6: Starting camera bridge ────"
gnome-terminal --title="Camera Bridge" -- bash -c "
    $SETUP_CMD
    ros2 launch '$LAUNCH_DIR/camera_bridge.launch.py'
    exec bash
"
sleep 5

# ── Step 3: Spawn targets ───────────────────────────────────────────────────
if [ "$SKIP_SPAWN" = false ]; then
    echo ""
    echo "──── Step 3/6: Spawning target objects ────"
    bash "$SCRIPT_DIR/spawn_targets.sh"
    sleep 2
else
    echo ""
    echo "──── Step 3/6: Skipped (--no-spawn) ────"
fi

# ── Step 4: Waypoint executor ───────────────────────────────────────────────
echo ""
echo "──── Step 4/6: Starting waypoint executor (arm + takeoff + follow) ────"
gnome-terminal --title="Waypoint Executor" -- bash -c "
    $SETUP_CMD
    cd '$PROJECT_DIR'
    python3 scripts/waypoint_executor.py --drones $DRONE_IDS --altitude $ALTITUDE
    exec bash
"
echo "Waiting 25s for all drones to take off..."
sleep 25

# ── Step 5: Detection publisher ─────────────────────────────────────────────
echo ""
echo "──── Step 5/6: Starting YOLOv8 detection publisher ────"
gnome-terminal --title="Detection Publisher" -- bash -c "
    $SETUP_CMD
    cd '$PROJECT_DIR'
    python3 scripts/detection_publisher.py --drones $DRONE_IDS --confidence $DETECTION_CONFIDENCE --altitude $ALTITUDE
    exec bash
"
sleep 3

# ── Step 6: Voronoi coordinator ─────────────────────────────────────────────
echo ""
echo "──── Step 6/6: Starting Voronoi coordinator ────"
gnome-terminal --title="Voronoi Coordinator" -- bash -c "
    $SETUP_CMD
    cd '$PROJECT_DIR'
    python3 scripts/voronoi_coordinator.py \
        --drones $DRONE_IDS \
        --area-size $AREA_SIZE \
        --altitude $ALTITUDE \
        --detection-weight $DETECTION_WEIGHT \
        --detection-sigma $DETECTION_SIGMA \
        --decay-half-life $DECAY_HALF_LIFE
    exec bash
"

# ── Summary ──────────────────────────────────────────────────────────────────
echo ""
echo "╔══════════════════════════════════════════════════════════════════╗"
echo "║                   All nodes launched!                           ║"
echo "╠══════════════════════════════════════════════════════════════════╣"
echo "║  Terminals running:                                            ║"
echo "║    - Camera Bridge                                             ║"
echo "║    - Waypoint Executor (drones armed + airborne)               ║"
echo "║    - Detection Publisher (YOLOv8 on camera feeds)              ║"
echo "║    - Voronoi Coordinator (adaptive coverage)                   ║"
echo "╠══════════════════════════════════════════════════════════════════╣"
echo "║  Monitor topics:                                               ║"
echo "║    ros2 topic echo /drone1/detection                           ║"
echo "║    ros2 topic echo /drone1/target_waypoint                     ║"
echo "║    ros2 topic echo /drone1/position                            ║"
echo "╠══════════════════════════════════════════════════════════════════╣"
echo "║  Expected behavior:                                            ║"
echo "║    1. Drones spread to Voronoi centroids (uniform coverage)    ║"
echo "║    2. On detection: nearby drones converge toward target       ║"
echo "║    3. After decay (~${DECAY_HALF_LIFE}s): drones relax back to coverage    ║"
echo "╚══════════════════════════════════════════════════════════════════╝"
