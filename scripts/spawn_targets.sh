#!/bin/bash
# Spawn target objects into a running Gazebo world for drone detection.
#
# Supports multi-class spawning: person, vehicle, fire models.
# Models are downloaded from Gazebo Fuel on first run and cached.
#
# Usage:
#   ./spawn_targets.sh                                      # default 5 vehicle targets
#   ./spawn_targets.sh --count 10                           # spawn 10 targets
#   ./spawn_targets.sh --classes person,vehicle,fire         # mixed-class targets
#   ./spawn_targets.sh --count 5 --classes person,vehicle --random --seed 42
#   ./spawn_targets.sh --world swarm_default --count 8 --classes fire

set -e

# ── Defaults ─────────────────────────────────────────────────────────────────
WORLD="swarm_default"
NUM_TARGETS=5
CLASSES="vehicle"
RANDOM_PLACEMENT=false
SEED=42

# ── Parse arguments ──────────────────────────────────────────────────────────
while [[ $# -gt 0 ]]; do
    case $1 in
        --world)    WORLD=$2; shift 2 ;;
        --count)    NUM_TARGETS=$2; shift 2 ;;
        --classes)  CLASSES=$2; shift 2 ;;
        --random)   RANDOM_PLACEMENT=true; shift ;;
        --seed)     SEED=$2; shift 2 ;;
        *)
            # Legacy positional args: $1=world $2=count
            if [ -z "${LEGACY_WORLD_SET:-}" ]; then
                WORLD=$1; LEGACY_WORLD_SET=1; shift
            elif [ -z "${LEGACY_COUNT_SET:-}" ]; then
                NUM_TARGETS=$1; LEGACY_COUNT_SET=1; shift
            else
                echo "Unknown option: $1"; exit 1
            fi
            ;;
    esac
done

# Parse comma-separated classes into array
IFS=',' read -ra CLASS_LIST <<< "$CLASSES"

echo "=== Spawning $NUM_TARGETS targets into Gazebo world: $WORLD ==="
echo "    Classes: ${CLASS_LIST[*]}"
echo "    Random placement: $RANDOM_PLACEMENT (seed=$SEED)"

# ── Download models from Gazebo Fuel ─────────────────────────────────────────
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

# Vehicle models
download_if_needed "SUV"
download_if_needed "Hatchback red"
download_if_needed "Hatchback blue"
download_if_needed "Pickup"
download_if_needed "Hatchback"

# Person models
download_if_needed "Rescue Randy"
download_if_needed "Casual female"

echo "Models ready."

# ── Model pools per class ──────────────────────────────────────────────────
# Vehicle models (COCO classes: car, truck)
VEHICLE_URIS=(
    "https://fuel.gazebosim.org/1.0/OpenRobotics/models/SUV"
    "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Hatchback red"
    "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Hatchback blue"
    "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Pickup"
    "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Hatchback"
)
VEHICLE_NAMES=("suv" "hatchback_red" "hatchback_blue" "pickup" "hatchback")

# Person models (COCO class: person)
PERSON_URIS=(
    "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Rescue Randy"
    "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Casual female"
)
PERSON_NAMES=("rescue_randy" "casual_female")

# Fire model: red/orange ground patch (SDF inline)
# No Fuel model needed — we create a simple visual marker

# ── Pre-defined positions (used when --random is not set) ──────────────────
# Spread across 200x200m area (-100 to +100)
PREDEF_X=(  70  -80   50  -60   85  -75  -20   40  -50   65 )
PREDEF_Y=( -60   70   80  -50  -30   20  -85   45   55  -75 )
PREDEF_YAW=( 0  1.57 0.78 2.35 1.05 3.14 0.52 1.83 2.62 0.39 )

# ── Seeded random number generator (portable via awk) ──────────────────────
random_coord() {
    # Generate a random float in [-90, 90] using awk with seed
    local idx=$1
    awk -v seed="$((SEED + idx))" 'BEGIN { srand(seed); printf "%.1f", (rand() * 180) - 90 }'
}

random_yaw() {
    local idx=$1
    awk -v seed="$((SEED + idx + 1000))" 'BEGIN { srand(seed); printf "%.2f", rand() * 6.28 }'
}

# ── Temp directory for SDF files ───────────────────────────────────────────
TMPDIR=$(mktemp -d)
trap "rm -rf $TMPDIR" EXIT

# ── Spawn helper: Fuel model ──────────────────────────────────────────────
spawn_fuel_model() {
    local NAME=$1
    local X=$2
    local Y=$3
    local Z=$4
    local YAW=$5
    local FUEL_URI=$6

    echo "[Spawn] $NAME at ($X, $Y, $Z) yaw=$YAW"

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

# ── Spawn helper: Fire patch (inline SDF) ─────────────────────────────────
spawn_fire_patch() {
    local NAME=$1
    local X=$2
    local Y=$3

    echo "[Spawn] $NAME (fire patch) at ($X, $Y, 0.01)"

    local TMPFILE="$TMPDIR/${NAME}.sdf"
    cat > "$TMPFILE" <<SDEOF
<?xml version="1.0"?>
<sdf version="1.9">
  <model name="$NAME">
    <static>true</static>
    <link name="link">
      <visual name="visual">
        <geometry>
          <box>
            <size>3 3 0.02</size>
          </box>
        </geometry>
        <material>
          <ambient>1.0 0.3 0.0 1</ambient>
          <diffuse>1.0 0.2 0.0 1</diffuse>
          <emissive>0.8 0.15 0.0 1</emissive>
        </material>
      </visual>
    </link>
  </model>
</sdf>
SDEOF

    gz service -s /world/${WORLD}/create \
        --reqtype gz.msgs.EntityFactory \
        --reptype gz.msgs.Boolean \
        --timeout 5000 \
        --req "sdf_filename: '${TMPFILE}', name: '${NAME}', pose: {position: {x: ${X}, y: ${Y}, z: 0.01}}"
}

# ── Spawn targets ──────────────────────────────────────────────────────────
NUM_CLASSES=${#CLASS_LIST[@]}
VEHICLE_IDX=0
PERSON_IDX=0

for i in $(seq 0 $((NUM_TARGETS - 1))); do
    # Determine class for this target (round-robin across requested classes)
    class_idx=$((i % NUM_CLASSES))
    TARGET_CLASS="${CLASS_LIST[$class_idx]}"

    # Determine position
    if [ "$RANDOM_PLACEMENT" = true ]; then
        POS_X=$(random_coord $((i * 2)))
        POS_Y=$(random_coord $((i * 2 + 1)))
        YAW=$(random_yaw $i)
    else
        pos_idx=$((i % ${#PREDEF_X[@]}))
        POS_X="${PREDEF_X[$pos_idx]}"
        POS_Y="${PREDEF_Y[$pos_idx]}"
        YAW="${PREDEF_YAW[$pos_idx]}"
    fi

    case "$TARGET_CLASS" in
        vehicle|car|truck)
            model_idx=$((VEHICLE_IDX % ${#VEHICLE_URIS[@]}))
            target_name="target_vehicle_${VEHICLE_NAMES[$model_idx]}_$((i + 1))"
            spawn_fuel_model "$target_name" "$POS_X" "$POS_Y" "0.1" "$YAW" "${VEHICLE_URIS[$model_idx]}"
            VEHICLE_IDX=$((VEHICLE_IDX + 1))
            ;;
        person)
            model_idx=$((PERSON_IDX % ${#PERSON_URIS[@]}))
            target_name="target_person_${PERSON_NAMES[$model_idx]}_$((i + 1))"
            spawn_fuel_model "$target_name" "$POS_X" "$POS_Y" "0.0" "$YAW" "${PERSON_URIS[$model_idx]}"
            PERSON_IDX=$((PERSON_IDX + 1))
            ;;
        fire)
            target_name="target_fire_patch_$((i + 1))"
            spawn_fire_patch "$target_name" "$POS_X" "$POS_Y"
            ;;
        *)
            echo "WARNING: Unknown class '$TARGET_CLASS' — defaulting to vehicle"
            model_idx=$((VEHICLE_IDX % ${#VEHICLE_URIS[@]}))
            target_name="target_vehicle_${VEHICLE_NAMES[$model_idx]}_$((i + 1))"
            spawn_fuel_model "$target_name" "$POS_X" "$POS_Y" "0.1" "$YAW" "${VEHICLE_URIS[$model_idx]}"
            VEHICLE_IDX=$((VEHICLE_IDX + 1))
            ;;
    esac

    sleep 1
done

echo ""
echo "=== Done! $NUM_TARGETS targets spawned (classes: ${CLASS_LIST[*]}) ==="
