#!/bin/bash
# Day 3 Task: Spawn colored target objects into a running Gazebo world.
#
# These are simple colored box/cylinder/sphere primitives placed on the ground
# so drones can detect them from above with YOLOv8.
#
# Usage:
#   ./spawn_targets.sh              # spawns default set of targets
#   ./spawn_targets.sh <world_name> # specify Gazebo world name (default: "default")
#
# Requires: gz sim running (launched by launch_multi_drone.sh)

WORLD=${1:-default}

echo "=== Spawning target objects into Gazebo world: $WORLD ==="

# Helper: spawn a model from an SDF file via gz service
spawn_model() {
    local NAME=$1
    local X=$2
    local Y=$3
    local Z=$4
    local SDF_FILE=$5

    echo "[Spawn] $NAME at ($X, $Y, $Z)"

    gz service -s /world/${WORLD}/create \
        --reqtype gz.msgs.EntityFactory \
        --reptype gz.msgs.Boolean \
        --timeout 5000 \
        --req "sdf_filename: '${SDF_FILE}', name: '${NAME}', pose: {position: {x: ${X}, y: ${Y}, z: ${Z}}}"
}

# Create a temp directory for SDF files
TMPDIR=$(mktemp -d)
trap "rm -rf $TMPDIR" EXIT

# ── Target 1: Red box (simulates a person / vehicle from above) ──────────────
cat > "$TMPDIR/red_box.sdf" <<'EOF'
<?xml version="1.0"?>
<sdf version="1.9">
  <model name="red_box">
    <static>true</static>
    <link name="link">
      <visual name="visual">
        <geometry><box><size>1.0 0.5 0.8</size></box></geometry>
        <material>
          <ambient>0.8 0.1 0.1 1</ambient>
          <diffuse>0.9 0.1 0.1 1</diffuse>
        </material>
      </visual>
      <collision name="collision">
        <geometry><box><size>1.0 0.5 0.8</size></box></geometry>
      </collision>
    </link>
  </model>
</sdf>
EOF

# ── Target 2: Blue cylinder ──────────────────────────────────────────────────
cat > "$TMPDIR/blue_cylinder.sdf" <<'EOF'
<?xml version="1.0"?>
<sdf version="1.9">
  <model name="blue_cylinder">
    <static>true</static>
    <link name="link">
      <visual name="visual">
        <geometry><cylinder><radius>0.4</radius><length>1.0</length></cylinder></geometry>
        <material>
          <ambient>0.1 0.1 0.8 1</ambient>
          <diffuse>0.1 0.1 0.9 1</diffuse>
        </material>
      </visual>
      <collision name="collision">
        <geometry><cylinder><radius>0.4</radius><length>1.0</length></cylinder></geometry>
      </collision>
    </link>
  </model>
</sdf>
EOF

# ── Target 3: Green sphere ──────────────────────────────────────────────────
cat > "$TMPDIR/green_sphere.sdf" <<'EOF'
<?xml version="1.0"?>
<sdf version="1.9">
  <model name="green_sphere">
    <static>true</static>
    <link name="link">
      <visual name="visual">
        <geometry><sphere><radius>0.5</radius></sphere></geometry>
        <material>
          <ambient>0.1 0.7 0.1 1</ambient>
          <diffuse>0.1 0.8 0.1 1</diffuse>
        </material>
      </visual>
      <collision name="collision">
        <geometry><sphere><radius>0.5</radius></sphere></geometry>
      </collision>
    </link>
  </model>
</sdf>
EOF

# ── Target 4: Yellow box (wider, like a vehicle) ────────────────────────────
cat > "$TMPDIR/yellow_box.sdf" <<'EOF'
<?xml version="1.0"?>
<sdf version="1.9">
  <model name="yellow_box">
    <static>true</static>
    <link name="link">
      <visual name="visual">
        <geometry><box><size>1.5 0.7 0.6</size></box></geometry>
        <material>
          <ambient>0.8 0.8 0.1 1</ambient>
          <diffuse>0.9 0.9 0.1 1</diffuse>
        </material>
      </visual>
      <collision name="collision">
        <geometry><box><size>1.5 0.7 0.6</size></box></geometry>
      </collision>
    </link>
  </model>
</sdf>
EOF

# ── Target 5: White cylinder (tall, like a pole/person) ─────────────────────
cat > "$TMPDIR/white_cylinder.sdf" <<'EOF'
<?xml version="1.0"?>
<sdf version="1.9">
  <model name="white_cylinder">
    <static>true</static>
    <link name="link">
      <visual name="visual">
        <geometry><cylinder><radius>0.25</radius><length>1.8</length></cylinder></geometry>
        <material>
          <ambient>0.9 0.9 0.9 1</ambient>
          <diffuse>1.0 1.0 1.0 1</diffuse>
        </material>
      </visual>
      <collision name="collision">
        <geometry><cylinder><radius>0.25</radius><length>1.8</length></cylinder></geometry>
      </collision>
    </link>
  </model>
</sdf>
EOF

# ── Spawn all targets at different positions ─────────────────────────────────
# Positions chosen so drones at ~30m altitude with downward camera can see them
# Spread across a ~20x20m area near the default PX4 home position (origin)

spawn_model "target_red_box"        5    3   0.4  "$TMPDIR/red_box.sdf"
spawn_model "target_blue_cylinder" -4    6   0.5  "$TMPDIR/blue_cylinder.sdf"
spawn_model "target_green_sphere"   8   -5   0.5  "$TMPDIR/green_sphere.sdf"
spawn_model "target_yellow_box"    -3   -4   0.3  "$TMPDIR/yellow_box.sdf"
spawn_model "target_white_cylinder" 2    8   0.9  "$TMPDIR/white_cylinder.sdf"

echo ""
echo "=== Done! 5 target objects spawned ==="
echo "Objects: red_box, blue_cylinder, green_sphere, yellow_box, white_cylinder"
echo "Fly drones overhead and check /droneN/camera/image_raw for detections."
