#!/bin/bash
# Run a single experiment trial with a specified method.
#
# Launches the simulation (or skips if --no-sim), runs the specified controller,
# logs data, and computes metrics.
#
# Usage:
#   ./run_experiment.sh --method adaptive --drones 3 --targets 5 --duration 180 --trial 1
#   ./run_experiment.sh --method static --drones 3 --targets 5 --duration 180 --trial 1 --no-sim
#   ./run_experiment.sh --method lawnmower --drones 3 --targets 5 --duration 180 --trial 3
#   ./run_experiment.sh --method random --drones 3 --targets 5 --duration 180 --trial 1 --seed 42

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
LAUNCH_DIR="$PROJECT_DIR/ros2_ws/src/swarm_bringup/launch"
ROS2_WS="$PROJECT_DIR/ros2_ws"

# ── Defaults ─────────────────────────────────────────────────────────────────
METHOD="adaptive"
NUM_DRONES=3
NUM_TARGETS=5
DURATION=180
TRIAL=1
ALTITUDE=30
AREA_SIZE=40
CONFIDENCE=0.15
SEED=42
SKIP_SIM=false
OUTPUT_BASE="$PROJECT_DIR/data/logs"

# ── Parse arguments ──────────────────────────────────────────────────────────
while [[ $# -gt 0 ]]; do
    case $1 in
        --method)     METHOD=$2; shift 2 ;;
        --drones)     NUM_DRONES=$2; shift 2 ;;
        --targets)    NUM_TARGETS=$2; shift 2 ;;
        --duration)   DURATION=$2; shift 2 ;;
        --trial)      TRIAL=$2; shift 2 ;;
        --altitude)   ALTITUDE=$2; shift 2 ;;
        --area-size)  AREA_SIZE=$2; shift 2 ;;
        --confidence) CONFIDENCE=$2; shift 2 ;;
        --seed)       SEED=$2; shift 2 ;;
        --no-sim)     SKIP_SIM=true; shift ;;
        --output-base) OUTPUT_BASE=$2; shift 2 ;;
        *) echo "Unknown option: $1"; exit 1 ;;
    esac
done

EXPERIMENT_ID="${METHOD}_d${NUM_DRONES}_t${NUM_TARGETS}_trial$(printf '%02d' $TRIAL)"
OUTPUT_DIR="$OUTPUT_BASE/$EXPERIMENT_ID"
DRONE_IDS=$(seq -s ' ' 1 $NUM_DRONES)

# Environment setup command
SETUP_CMD="export PATH=/home/ilham/anaconda3/envs/swarm/bin:\$PATH && source /opt/ros/humble/setup.bash && source $ROS2_WS/install/setup.bash"

echo "══════════════════════════════════════════════════════════════"
echo "  Experiment: $EXPERIMENT_ID"
echo "  Method: $METHOD | Drones: $NUM_DRONES | Targets: $NUM_TARGETS"
echo "  Duration: ${DURATION}s | Trial: $TRIAL | Seed: $SEED"
echo "  Output: $OUTPUT_DIR"
echo "══════════════════════════════════════════════════════════════"

# ── Cleanup function ─────────────────────────────────────────────────────────
cleanup() {
    echo ""
    echo "Cleaning up experiment processes..."
    # Kill nodes we started (by title)
    for title in "EXP-CamBridge" "EXP-WPExecutor" "EXP-DetPub" "EXP-Controller" "EXP-Logger"; do
        pkill -f "gnome-terminal.*$title" 2>/dev/null || true
    done
    # Kill Python nodes by script name
    pkill -f "data_logger.py.*$EXPERIMENT_ID" 2>/dev/null || true
    pkill -f "waypoint_executor.py" 2>/dev/null || true
    pkill -f "detection_publisher.py" 2>/dev/null || true
    pkill -f "voronoi_coordinator.py" 2>/dev/null || true
    pkill -f "static_grid.py" 2>/dev/null || true
    pkill -f "lawnmower.py" 2>/dev/null || true
    pkill -f "random_waypoints.py" 2>/dev/null || true
    echo "Cleanup done."
}
trap cleanup EXIT

# ── Step 1: Launch simulation ────────────────────────────────────────────────
if [ "$SKIP_SIM" = false ]; then
    echo "[1/6] Launching $NUM_DRONES drones..."
    bash "$SCRIPT_DIR/launch_multi_drone.sh" "$NUM_DRONES" x500_mono_cam_down
    sleep 15
else
    echo "[1/6] Skipped (--no-sim)"
fi

# ── Step 2: Camera bridge ───────────────────────────────────────────────────
echo "[2/6] Starting camera bridge..."
gnome-terminal --title="EXP-CamBridge" -- bash -c "
    $SETUP_CMD
    ros2 launch '$LAUNCH_DIR/camera_bridge.launch.py'
    exec bash
"
sleep 5

# ── Step 3: Spawn targets ───────────────────────────────────────────────────
echo "[3/6] Spawning $NUM_TARGETS targets (seed=$SEED)..."
bash "$SCRIPT_DIR/spawn_targets.sh"
sleep 2

# ── Step 4: Waypoint executor ───────────────────────────────────────────────
echo "[4/6] Starting waypoint executor..."
gnome-terminal --title="EXP-WPExecutor" -- bash -c "
    $SETUP_CMD
    cd '$PROJECT_DIR'
    python3 scripts/waypoint_executor.py --drones $DRONE_IDS --altitude $ALTITUDE
    exec bash
"
sleep 25

# ── Step 5: Detection publisher ─────────────────────────────────────────────
echo "[5/6] Starting detection publisher..."
gnome-terminal --title="EXP-DetPub" -- bash -c "
    $SETUP_CMD
    cd '$PROJECT_DIR'
    python3 scripts/detection_publisher.py --drones $DRONE_IDS --confidence $CONFIDENCE --altitude $ALTITUDE
    exec bash
"
sleep 3

# ── Step 6: Controller + Logger ─────────────────────────────────────────────
echo "[6/6] Starting $METHOD controller + data logger (${DURATION}s)..."

# Start the controller based on method
case $METHOD in
    adaptive)
        gnome-terminal --title="EXP-Controller" -- bash -c "
            $SETUP_CMD
            cd '$PROJECT_DIR'
            python3 scripts/voronoi_coordinator.py --drones $DRONE_IDS --area-size $AREA_SIZE --altitude $ALTITUDE
            exec bash
        "
        ;;
    static)
        gnome-terminal --title="EXP-Controller" -- bash -c "
            $SETUP_CMD
            cd '$PROJECT_DIR'
            python3 scripts/baselines/static_grid.py --drones $DRONE_IDS --area-size $AREA_SIZE --altitude $ALTITUDE
            exec bash
        "
        ;;
    lawnmower)
        gnome-terminal --title="EXP-Controller" -- bash -c "
            $SETUP_CMD
            cd '$PROJECT_DIR'
            python3 scripts/baselines/lawnmower.py --drones $DRONE_IDS --area-size $AREA_SIZE --altitude $ALTITUDE
            exec bash
        "
        ;;
    random)
        gnome-terminal --title="EXP-Controller" -- bash -c "
            $SETUP_CMD
            cd '$PROJECT_DIR'
            python3 scripts/baselines/random_waypoints.py --drones $DRONE_IDS --area-size $AREA_SIZE --altitude $ALTITUDE --seed $SEED
            exec bash
        "
        ;;
    *)
        echo "ERROR: Unknown method: $METHOD"
        echo "Valid methods: adaptive, static, lawnmower, random"
        exit 1
        ;;
esac
sleep 2

# Start data logger with duration limit (will auto-stop)
eval "$SETUP_CMD"
cd "$PROJECT_DIR"
echo "Logging for ${DURATION}s → $OUTPUT_DIR"
python3 scripts/data_logger.py \
    --drones $DRONE_IDS \
    --experiment-id "$EXPERIMENT_ID" \
    --output-base "$OUTPUT_BASE" \
    --duration "$DURATION"

# ── Compute metrics ──────────────────────────────────────────────────────────
echo ""
echo "Computing metrics..."
python3 scripts/metrics_compute.py \
    --experiment-id "$EXPERIMENT_ID" \
    --log-base "$OUTPUT_BASE" \
    --area-size "$AREA_SIZE" \
    --altitude "$ALTITUDE"

echo ""
echo "══════════════════════════════════════════════════════════════"
echo "  Trial complete: $EXPERIMENT_ID"
echo "  Results: $OUTPUT_DIR/metrics_summary.json"
echo "══════════════════════════════════════════════════════════════"
