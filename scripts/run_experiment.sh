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
#   ./run_experiment.sh --method binary_voronoi --drones 3 --targets 5 --duration 180 --trial 1
#   ./run_experiment.sh --method all_converge --drones 3 --targets 5 --duration 180 --trial 1
#   ./run_experiment.sh --method adaptive --drones 3 --targets 5 --classes person,vehicle,fire

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
ALTITUDE=40
AREA_SIZE=200
CONFIDENCE=0.15
SEED=42
SKIP_SIM=false
NO_DETECTIONS=false
OUTPUT_BASE="$PROJECT_DIR/data/logs"
TARGET_CLASSES="person,vehicle,fire"
SCHEDULE=""        # if non-empty, comma-separated spawn times → dynamic spawning

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
        --no-detections) NO_DETECTIONS=true; shift ;;
        --output-base) OUTPUT_BASE=$2; shift 2 ;;
        --classes)    TARGET_CLASSES=$2; shift 2 ;;
        --schedule)   SCHEDULE=$2; shift 2 ;;
        *) echo "Unknown option: $1"; exit 1 ;;
    esac
done

# When --schedule is provided, the spawn times drive the target count.
if [ -n "$SCHEDULE" ]; then
    IFS=',' read -ra _SCHED_TIMES <<< "$SCHEDULE"
    NUM_TARGETS=${#_SCHED_TIMES[@]}
fi

EXPERIMENT_ID="${METHOD}_d${NUM_DRONES}_t${NUM_TARGETS}_trial$(printf '%02d' $TRIAL)"
OUTPUT_DIR="$OUTPUT_BASE/$EXPERIMENT_ID"
DRONE_IDS=$(seq -s ' ' 1 $NUM_DRONES)
mkdir -p "$OUTPUT_DIR"

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
    pkill -f "binary_voronoi.py" 2>/dev/null || true
    pkill -f "all_converge.py" 2>/dev/null || true
    pkill -f "pso_coverage.py" 2>/dev/null || true
    pkill -f "apf_coverage.py" 2>/dev/null || true
    pkill -f "spawn_targets.sh.*$EXPERIMENT_ID" 2>/dev/null || true
    pkill -f "spawn_targets.sh.*--schedule" 2>/dev/null || true
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
echo "[2/6] Starting camera bridge for $NUM_DRONES drones..."
# Generate camera bridge YAML dynamically based on drone count
BRIDGE_YAML="$PROJECT_DIR/data/camera_bridge_dynamic.yaml"
mkdir -p "$(dirname "$BRIDGE_YAML")"
> "$BRIDGE_YAML"
GZ_WORLD="swarm_default"
for i in $(seq 1 $NUM_DRONES); do
    cat >> "$BRIDGE_YAML" <<YAML
- ros_topic_name: /drone${i}/camera/image_raw
  gz_topic_name: /world/${GZ_WORLD}/model/x500_mono_cam_down_${i}/link/camera_link/sensor/imager/image
  ros_type_name: sensor_msgs/msg/Image
  gz_type_name: gz.msgs.Image
  direction: GZ_TO_ROS
  lazy: false

- ros_topic_name: /drone${i}/camera/camera_info
  gz_topic_name: /world/${GZ_WORLD}/model/x500_mono_cam_down_${i}/link/camera_link/sensor/imager/camera_info
  ros_type_name: sensor_msgs/msg/CameraInfo
  gz_type_name: gz.msgs.CameraInfo
  direction: GZ_TO_ROS
  lazy: false

YAML
done

gnome-terminal --title="EXP-CamBridge" -- bash -c "
    $SETUP_CMD
    ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:='$BRIDGE_YAML'
    exec bash
"
sleep 5

# ── Step 3: Spawn targets ───────────────────────────────────────────────────
# In static mode all targets appear before the drones take off. In schedule
# mode the spawns are deferred until just before the data logger starts so
# their wall-clock times line up with the trial timeline.
if [ -z "$SCHEDULE" ]; then
    echo "[3/6] Spawning $NUM_TARGETS targets (classes: $TARGET_CLASSES) inside ${AREA_SIZE}m area..."
    bash "$SCRIPT_DIR/spawn_targets.sh" \
        --world "$GZ_WORLD" \
        --count "$NUM_TARGETS" \
        --classes "$TARGET_CLASSES" \
        --random \
        --seed "$SEED" \
        --area-size "$AREA_SIZE" \
        --output-csv "$OUTPUT_DIR/targets.csv"
    sleep 2
else
    echo "[3/6] Deferring target spawning — schedule [$SCHEDULE] runs alongside the data logger"
fi

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

# Per-method data-logger duration. Lawnmower overrides this below to a
# safety cap because it stops on /lawnmower/complete, not on a wall clock.
LOGGER_DURATION="$DURATION"

# Start the controller based on method
case $METHOD in
    adaptive)
        # Adaptive runs until /adaptive/complete fires (all expected
        # targets confirmed by the registry). The user-provided
        # --duration only acts as a safety cap on the data logger,
        # mirroring how lawnmower works.
        LOGGER_DURATION=$(python3 -c "print(int(max(1500, 4 * $DURATION)))")
        echo "  Adaptive: run-until-complete (safety cap ${LOGGER_DURATION}s, expected_targets=$NUM_TARGETS)"
        gnome-terminal --title="EXP-Controller" -- bash -c "
            $SETUP_CMD
            cd '$PROJECT_DIR'
            python3 scripts/voronoi_coordinator.py \
                --drones $DRONE_IDS \
                --area-size $AREA_SIZE \
                --altitude $ALTITUDE \
                --expected-targets $NUM_TARGETS
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
        # Lawnmower ignores the user-provided --duration and instead runs
        # until every drone finishes its FOV-sized sweep pattern. The logger
        # stops on /lawnmower/complete; the value below is only a safety cap
        # (~4x the analytic estimate, minimum 20 min) to guard against hangs.
        LOGGER_DURATION=$(python3 -c "
import math
a, h, s = $AREA_SIZE, $ALTITUDE, 5.0
hfov = 1.74
footprint = 2*h*math.tan(hfov/2)*0.95
n_sweeps = max(1, math.ceil((a/$NUM_DRONES) / footprint))
length = n_sweeps*a + max(0, n_sweeps-1)*footprint
print(int(max(1200, 4 * (h + length/s))))
")
        echo "  Lawnmower: run-until-complete (safety cap ${LOGGER_DURATION}s)"
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
    binary_voronoi)
        gnome-terminal --title="EXP-Controller" -- bash -c "
            $SETUP_CMD
            cd '$PROJECT_DIR'
            python3 scripts/baselines/binary_voronoi.py --drones $DRONE_IDS --area-size $AREA_SIZE --altitude $ALTITUDE
            exec bash
        "
        ;;
    all_converge)
        gnome-terminal --title="EXP-Controller" -- bash -c "
            $SETUP_CMD
            cd '$PROJECT_DIR'
            python3 scripts/baselines/all_converge.py --drones $DRONE_IDS --area-size $AREA_SIZE --altitude $ALTITUDE
            exec bash
        "
        ;;
    pso)
        DET_FLAG=""
        [ "$NO_DETECTIONS" = true ] && DET_FLAG="--no-detections"
        gnome-terminal --title="EXP-Controller" -- bash -c "
            $SETUP_CMD
            cd '$PROJECT_DIR'
            python3 scripts/baselines/pso_coverage.py --drones $DRONE_IDS --area-size $AREA_SIZE --altitude $ALTITUDE $DET_FLAG
            exec bash
        "
        ;;
    apf)
        DET_FLAG=""
        [ "$NO_DETECTIONS" = true ] && DET_FLAG="--no-detections"
        gnome-terminal --title="EXP-Controller" -- bash -c "
            $SETUP_CMD
            cd '$PROJECT_DIR'
            python3 scripts/baselines/apf_coverage.py --drones $DRONE_IDS --area-size $AREA_SIZE --altitude $ALTITUDE $DET_FLAG
            exec bash
        "
        ;;
    *)
        echo "ERROR: Unknown method: $METHOD"
        echo "Valid methods: adaptive, static, lawnmower, random, binary_voronoi, all_converge, pso, apf"
        exit 1
        ;;
esac
sleep 2

# In schedule mode, kick off the deferred spawner now so its t=0 lines up
# with the data logger's t=0. Track the PID so cleanup can reap it.
SPAWN_BG_PID=""
if [ -n "$SCHEDULE" ]; then
    echo "Launching scheduled target spawner [$SCHEDULE] in background..."
    bash "$SCRIPT_DIR/spawn_targets.sh" \
        --world "$GZ_WORLD" \
        --schedule "$SCHEDULE" \
        --classes "$TARGET_CLASSES" \
        --seed "$SEED" \
        --area-size "$AREA_SIZE" \
        --output-csv "$OUTPUT_DIR/targets.csv" &
    SPAWN_BG_PID=$!
fi

# Start data logger with duration limit (will auto-stop)
eval "$SETUP_CMD"
cd "$PROJECT_DIR"
echo "Logging for ${LOGGER_DURATION}s → $OUTPUT_DIR"
python3 scripts/data_logger.py \
    --drones $DRONE_IDS \
    --experiment-id "$EXPERIMENT_ID" \
    --output-base "$OUTPUT_BASE" \
    --duration "$LOGGER_DURATION"

# Reap the background spawner so a partial schedule doesn't leak.
if [ -n "$SPAWN_BG_PID" ]; then
    kill "$SPAWN_BG_PID" 2>/dev/null || true
    wait "$SPAWN_BG_PID" 2>/dev/null || true
fi

# ── Compute metrics ──────────────────────────────────────────────────────────
echo ""
echo "Computing metrics..."
python3 scripts/metrics_compute.py \
    --experiment-id "$EXPERIMENT_ID" \
    --log-base "$OUTPUT_BASE" \
    --area-size "$AREA_SIZE" \
    --altitude "$ALTITUDE"

# ── Plot paths ──────────────────────────────────────────────────────────────
echo ""
echo "Generating path plots..."
python3 scripts/plot_paths.py \
    --experiment-id "$EXPERIMENT_ID" \
    --log-base "$OUTPUT_BASE" \
    --area-size "$AREA_SIZE" \
    --altitude "$ALTITUDE" \
    --targets-file "$OUTPUT_DIR/targets.csv"

echo ""
echo "══════════════════════════════════════════════════════════════"
echo "  Trial complete: $EXPERIMENT_ID"
echo "  Results: $OUTPUT_DIR/metrics_summary.json"
echo "  Plots:   $OUTPUT_DIR/paths_combined.png"
echo "══════════════════════════════════════════════════════════════"
