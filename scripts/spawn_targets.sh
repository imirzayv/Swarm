#!/bin/bash
# Spawn target objects into a running Gazebo world for drone detection.
#
# Uses realistic 3D models from Gazebo Fuel that are recognizable by
# YOLOv8 (COCO dataset classes: car, truck, bus).
#
# Models are downloaded from Gazebo Fuel on first run and cached in ~/.gz/fuel/.
#
# Usage:
#   ./spawn_targets.sh                    # spawn default 5 targets
#   ./spawn_targets.sh <world_name>       # specify world name
#   ./spawn_targets.sh swarm_default 10   # spawn 10 targets
#
# Requires: gz sim running (launched by launch_multi_drone.sh)

WORLD=${1:-swarm_default}
NUM_TARGETS=${2:-5}

echo "=== Spawning $NUM_TARGETS target objects into Gazebo world: $WORLD ==="

# ── Download models from Gazebo Fuel (cached after first download) ──────────
FUEL_BASE="https://fuel.gazebosim.org/1.0/OpenRobotics/models"

download_if_needed() {
    local model_name=$1
    local cache_dir="$HOME/.gz/fuel/fuel.gazebosim.org/openrobotics/models/$(echo "$model_name" | tr '[:upper:]' '[:lower:]')"
    if [ ! -d "$cache_dir" ]; then
        echo "[Download] $model_name from Gazebo Fuel..."
        gz fuel download -u "$FUEL_BASE/$model_name" > /dev/null 2>&1
    fi
}

echo "Checking Fuel model cache..."
download_if_needed "SUV"
download_if_needed "Hatchback red"
download_if_needed "Hatchback blue"
download_if_needed "Pickup"
download_if_needed "Prius Hybrid"
download_if_needed "Bus"
download_if_needed "Hatchback"
echo "Models ready."

# ── Fuel model URIs (Gazebo resolves these from the local cache) ────────────
MODELS=(
    "https://fuel.gazebosim.org/1.0/OpenRobotics/models/SUV"
    "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Hatchback red"
    "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Hatchback blue"
    "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Pickup"
    "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Prius Hybrid"
    "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Bus"
    "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Hatchback"
)

# Human-readable names for logging
MODEL_NAMES=(
    "suv"
    "hatchback_red"
    "hatchback_blue"
    "pickup"
    "prius"
    "bus"
    "hatchback"
)

# ── Predefined target positions ─────────────────────────────────────────────
# Spread across a 200x200m monitoring area (-100 to +100).
# Targets are far from each other (~40-60m apart) and from the drone
# start point (origin 0,0). Positions are in local XY (meters).
# Yaw values (radians) to vary orientation for more realistic appearance.
POS_X=(  70  -80   50  -60   85  -75  -20   40  -50   65 )
POS_Y=( -60   70   80  -50  -30   20  -85   45   55  -75 )
POS_YAW=( 0  1.57 0.78 2.35 1.05 3.14 0.52 1.83 2.62 0.39 )

# ── Temp directory for SDF files (cleaned up on exit) ───────────────────────
TMPDIR=$(mktemp -d)
trap "rm -rf $TMPDIR" EXIT

# ── Spawn helper ────────────────────────────────────────────────────────────
spawn_fuel_model() {
    local NAME=$1
    local X=$2
    local Y=$3
    local Z=$4
    local YAW=$5
    local FUEL_URI=$6

    echo "[Spawn] $NAME at ($X, $Y, $Z) yaw=$YAW"

    # Create a minimal SDF that includes the Fuel model
    local TMPFILE="$TMPDIR/${NAME}.sdf"
    cat > "$TMPFILE" <<SDEOF
<?xml version="1.0"?>
<sdf version="1.9">
  <include>
    <uri>$FUEL_URI</uri>
    <static>true</static>
  </include>
</sdf>
SDEOF

    gz service -s /world/${WORLD}/create \
        --reqtype gz.msgs.EntityFactory \
        --reptype gz.msgs.Boolean \
        --timeout 5000 \
        --req "sdf_filename: '${TMPFILE}', name: '${NAME}', pose: {position: {x: ${X}, y: ${Y}, z: ${Z}}, orientation: {x: 0, y: 0, z: $(echo "s($YAW/2)" | bc -l), w: $(echo "c($YAW/2)" | bc -l)}}"
}

# ── Spawn targets ──────────────────────────────────────────────────────────
NUM_MODELS=${#MODELS[@]}

for i in $(seq 0 $((NUM_TARGETS - 1))); do
    model_idx=$((i % NUM_MODELS))
    target_name="target_${MODEL_NAMES[$model_idx]}_$((i + 1))"
    spawn_fuel_model \
        "$target_name" \
        "${POS_X[$i]}" \
        "${POS_Y[$i]}" \
        "0.1" \
        "${POS_YAW[$i]}" \
        "${MODELS[$model_idx]}"
    sleep 1
done

echo ""
echo "=== Done! $NUM_TARGETS target objects spawned ==="
echo "COCO classes: car, truck, bus (detectable by YOLOv8)"
echo "Fly drones overhead and check /droneN/camera/image_raw for detections."
