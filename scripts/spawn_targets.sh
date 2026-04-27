#!/bin/bash
# Spawn colored target primitives into a running Gazebo world for HSV detection.
#
# Targets are flat-shaded SDF primitives whose RGB matches the HSV ranges in
# scripts/detection_publisher.py:
#   person  → pure red    box (1.0, 0.0, 0.0)   H  0–10,  S≥150, V≥100
#   vehicle → pure blue   box (0.0, 0.0, 1.0)   H 100–140, S≥150, V≥100
#   fire    → pure orange patch (1.0, 0.5, 0.0) H 10–25,  S≥150, V≥150
#
# Fuel models (SUV, Rescue Randy, etc.) were removed because their textured
# appearance does not pass HSV thresholding — see docs/10_day_readme.md.
#
# Usage:
#   ./spawn_targets.sh                                      # default 5 vehicles
#   ./spawn_targets.sh --count 10                           # 10 targets
#   ./spawn_targets.sh --classes person,vehicle,fire        # mixed classes
#   ./spawn_targets.sh --count 5 --classes person,vehicle --random --seed 42
#   ./spawn_targets.sh --world swarm_default --count 8 --classes fire

set -e

# ── Defaults ─────────────────────────────────────────────────────────────────
WORLD="swarm_default"
NUM_TARGETS=5
CLASSES="vehicle"
RANDOM_PLACEMENT=false
SEED=42
AREA_SIZE=100            # side length (m) of the monitoring square targets go into
OUTPUT_CSV=""            # optional manifest: name,x,y,class,spawn_t
SCHEDULE=""              # comma-separated spawn times (s), e.g. "20,50,80"

# ── Parse arguments ──────────────────────────────────────────────────────────
while [[ $# -gt 0 ]]; do
    case $1 in
        --world)      WORLD=$2; shift 2 ;;
        --count)      NUM_TARGETS=$2; shift 2 ;;
        --classes)    CLASSES=$2; shift 2 ;;
        --random)     RANDOM_PLACEMENT=true; shift ;;
        --seed)       SEED=$2; shift 2 ;;
        --area-size)  AREA_SIZE=$2; shift 2 ;;
        --output-csv) OUTPUT_CSV=$2; shift 2 ;;
        --schedule)   SCHEDULE=$2; shift 2 ;;
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

IFS=',' read -ra CLASS_LIST <<< "$CLASSES"

# Schedule mode: spawn times drive target count and force random placement.
SCHED_TIMES=()
if [ -n "$SCHEDULE" ]; then
    IFS=',' read -ra SCHED_TIMES <<< "$SCHEDULE"
    NUM_TARGETS=${#SCHED_TIMES[@]}
    RANDOM_PLACEMENT=true
    SCHEDULE_START_NS=$(date +%s%N)
fi

# Keep targets inside 80% of the monitoring area so drones can actually see them.
HALF_SPAN=$(awk -v a="$AREA_SIZE" 'BEGIN {printf "%.3f", a*0.4}')
# Scale the legacy PREDEF table (which was sized for a ~180 m area) down to AREA_SIZE.
PREDEF_SCALE=$(awk -v h="$HALF_SPAN" 'BEGIN {printf "%.4f", h/85.0}')

echo "=== Spawning $NUM_TARGETS colored primitives into Gazebo world: $WORLD ==="
echo "    Classes:     ${CLASS_LIST[*]}"
echo "    Area size:   ${AREA_SIZE} m (targets confined to ±${HALF_SPAN} m)"
echo "    Placement:   random=$RANDOM_PLACEMENT seed=$SEED"
if [ -n "$SCHEDULE" ]; then
    echo "    Schedule:    [$SCHEDULE] relative seconds from script start"
fi
[ -n "$OUTPUT_CSV" ] && echo "    Manifest:    $OUTPUT_CSV"

if [ -n "$OUTPUT_CSV" ]; then
    mkdir -p "$(dirname "$OUTPUT_CSV")"
    echo "name,x,y,class,spawn_t" > "$OUTPUT_CSV"
fi

# ── Pre-defined positions (used when --random is not set) ──────────────────
PREDEF_X=(  70  -80   50  -60   85  -75  -20   40  -50   65 )
PREDEF_Y=( -60   70   80  -50  -30   20  -85   45   55  -75 )
PREDEF_YAW=( 0  1.57 0.78 2.35 1.05 3.14 0.52 1.83 2.62 0.39 )

random_coord() {
    local idx=$1
    awk -v seed="$((SEED + idx))" -v h="$HALF_SPAN" \
        'BEGIN { srand(seed); printf "%.1f", (rand() * 2 * h) - h }'
}

random_yaw() {
    local idx=$1
    awk -v seed="$((SEED + idx + 1000))" 'BEGIN { srand(seed); printf "%.2f", rand() * 6.28 }'
}

TMPDIR=$(mktemp -d)
trap "rm -rf $TMPDIR" EXIT

# ── Spawn helper: colored primitive ────────────────────────────────────────
# Writes an inline SDF describing a static box with flat RGB material and
# calls gz service /world/<W>/create. All three classes flow through here —
# only the dimensions and RGB differ.
spawn_colored_box() {
    local NAME=$1
    local X=$2
    local Y=$3
    local Z=$4
    local YAW=$5
    local SIZE_X=$6
    local SIZE_Y=$7
    local SIZE_Z=$8
    local R=$9
    local G=${10}
    local B=${11}

    echo "[Spawn] $NAME at ($X, $Y, $Z) size=${SIZE_X}x${SIZE_Y}x${SIZE_Z} rgb=($R,$G,$B)"

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
            <size>$SIZE_X $SIZE_Y $SIZE_Z</size>
          </box>
        </geometry>
        <material>
          <ambient>$R $G $B 1</ambient>
          <diffuse>$R $G $B 1</diffuse>
          <specular>0.1 0.1 0.1 1</specular>
          <emissive>$(awk -v v=$R 'BEGIN{printf "%.2f", v*0.5}') $(awk -v v=$G 'BEGIN{printf "%.2f", v*0.5}') $(awk -v v=$B 'BEGIN{printf "%.2f", v*0.5}') 1</emissive>
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
        --req "sdf_filename: '${TMPFILE}', name: '${NAME}', pose: {position: {x: ${X}, y: ${Y}, z: ${Z}}, orientation: {x: 0, y: 0, z: $(echo "s($YAW/2)" | bc -l), w: $(echo "c($YAW/2)" | bc -l)}}"
}

# ── Spawn targets ──────────────────────────────────────────────────────────
NUM_CLASSES=${#CLASS_LIST[@]}

for i in $(seq 0 $((NUM_TARGETS - 1))); do
    class_idx=$((i % NUM_CLASSES))
    TARGET_CLASS="${CLASS_LIST[$class_idx]}"

    if [ "$RANDOM_PLACEMENT" = true ]; then
        POS_X=$(random_coord $((i * 2)))
        POS_Y=$(random_coord $((i * 2 + 1)))
        YAW=$(random_yaw $i)
    else
        pos_idx=$((i % ${#PREDEF_X[@]}))
        POS_X=$(awk -v v="${PREDEF_X[$pos_idx]}" -v s="$PREDEF_SCALE" 'BEGIN {printf "%.1f", v*s}')
        POS_Y=$(awk -v v="${PREDEF_Y[$pos_idx]}" -v s="$PREDEF_SCALE" 'BEGIN {printf "%.1f", v*s}')
        YAW="${PREDEF_YAW[$pos_idx]}"
    fi

    # All classes use the same flat slab primitive — only RGB differs.
    # A thin ground patch is detected reliably by HSV from nadir view
    # (always visible when drone flies overhead) and eliminates the
    # perspective issues that 1.5–1.8 m tall boxes introduced.
    SLAB_SIZE_X="3.0"
    SLAB_SIZE_Y="3.0"
    SLAB_SIZE_Z="0.1"
    SLAB_Z="0.05"

    case "$TARGET_CLASS" in
        person)
            target_name="target_person_$((i + 1))"
            R="1.0"; G="0.0"; B="0.0"
            manifest_class="person"
            ;;
        vehicle)
            target_name="target_vehicle_$((i + 1))"
            R="0.0"; G="0.0"; B="1.0"
            manifest_class="vehicle"
            ;;
        fire)
            target_name="target_fire_$((i + 1))"
            R="1.0"; G="0.5"; B="0.0"
            manifest_class="fire"
            ;;
        *)
            echo "WARNING: Unknown class '$TARGET_CLASS' — defaulting to vehicle"
            target_name="target_vehicle_$((i + 1))"
            R="0.0"; G="0.0"; B="1.0"
            manifest_class="vehicle"
            ;;
    esac

    # Scheduled mode: wait until the target's absolute spawn time before firing.
    SPAWN_T="0"
    if [ -n "$SCHEDULE" ]; then
        SPAWN_T="${SCHED_TIMES[$i]}"
        NOW_NS=$(date +%s%N)
        TARGET_NS=$(awk -v t="$SPAWN_T" 'BEGIN{printf "%.0f", t*1000000000}')
        WAIT_NS=$((SCHEDULE_START_NS + TARGET_NS - NOW_NS))
        if [ "$WAIT_NS" -gt 0 ]; then
            WAIT_S=$(awk -v n="$WAIT_NS" 'BEGIN{printf "%.3f", n/1000000000}')
            echo "[Schedule] Waiting ${WAIT_S}s until t=${SPAWN_T}s (target ${target_name})"
            sleep "$WAIT_S"
        fi
    fi

    spawn_colored_box "$target_name" "$POS_X" "$POS_Y" "$SLAB_Z" "$YAW" \
        "$SLAB_SIZE_X" "$SLAB_SIZE_Y" "$SLAB_SIZE_Z" "$R" "$G" "$B"

    if [ -n "$OUTPUT_CSV" ]; then
        echo "${target_name},${POS_X},${POS_Y},${manifest_class},${SPAWN_T}" >> "$OUTPUT_CSV"
    fi

    # Static mode pacing between spawns; schedule mode times itself.
    [ -z "$SCHEDULE" ] && sleep 0.5
done

echo ""
echo "=== Done! $NUM_TARGETS targets spawned (classes: ${CLASS_LIST[*]}) ==="
