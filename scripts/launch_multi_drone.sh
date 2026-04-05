#!/bin/bash
# Launch multiple PX4 SITL drones per the official PX4 multi-vehicle docs:
# https://docs.px4.io/main/en/sim_gazebo_gz/multi_vehicle_simulation
#
# Usage: ./launch_multi_drone.sh [num_drones] [model]
# Examples:
#   ./launch_multi_drone.sh           → 3x x500
#   ./launch_multi_drone.sh 2 x500_mono_cam_down  → 2x x500 with downward camera

NUM_DRONES=${1:-3}
MODEL=${2:-x500_mono_cam_down}
PX4_DIR="$HOME/Desktop/Swarm/firmware"
SPACING=3  # meters between drones (Y axis)

# ── Validation ────────────────────────────────────────────────────────────────
if [ ! -d "$PX4_DIR" ]; then
    echo "ERROR: PX4 directory not found at $PX4_DIR"
    exit 1
fi

if [ ! -f "$PX4_DIR/build/px4_sitl_default/bin/px4" ]; then
    echo "ERROR: PX4 not built. Run: cd $PX4_DIR && make px4_sitl"
    exit 1
fi

# ── Clean up stale processes ──────────────────────────────────────────────────
echo "Cleaning up old px4/gz processes..."
pkill -f "bin/px4" 2>/dev/null
pkill -f "gz sim" 2>/dev/null
sleep 2

echo "=== Launching $NUM_DRONES x $MODEL drones ==="
echo ""

# ── Drone 1 (instance 1): starts Gazebo + first drone ────────────────────────
# Per docs: first terminal does NOT use PX4_GZ_STANDALONE
# Uses PX4_SIM_MODEL=gz_<model> (with gz_ prefix) — this is what triggers
# the correct airframe and gz_bridge spawn logic in px4-rc.gzsim
echo "[Drone 1] Starting Gazebo + first drone..."
gnome-terminal --title="Drone-1 (Gazebo)" -- bash -c "
    cd '$PX4_DIR'
    PX4_SYS_AUTOSTART=4001 \
    PX4_GZ_MODEL_POSE='0,0,0,0,0,0' \
    PX4_SIM_MODEL=gz_${MODEL} \
    ./build/px4_sitl_default/bin/px4 -i 1
    exec bash
"

echo "Waiting 25 seconds for Gazebo + Drone 1 to initialize..."
echo "(Watch 'Drone-1' terminal — wait for 'Ready for takeoff!')"
sleep 25

# ── Drones 2..N: connect to the already-running Gazebo ───────────────────────
# Per docs: add PX4_GZ_STANDALONE=1 so they don't try to launch a new Gazebo
for i in $(seq 2 $NUM_DRONES); do
    Y_POS=$(( (i - 1) * SPACING ))
    DDS_PORT=$((8888 + i - 1))
    MAVLINK_PORT=$((14540 + i - 1))

    echo "[Drone $i] Spawning at (0, $Y_POS, 0) | MAVLink: $MAVLINK_PORT | DDS: $DDS_PORT"

    gnome-terminal --title="Drone-$i" -- bash -c "
        cd '$PX4_DIR'
        PX4_GZ_STANDALONE=1 \
        PX4_SYS_AUTOSTART=4001 \
        PX4_GZ_MODEL_POSE='0,${Y_POS},0,0,0,0' \
        PX4_SIM_MODEL=gz_${MODEL} \
        PX4_UXRCE_DDS_PORT=${DDS_PORT} \
        ./build/px4_sitl_default/bin/px4 -i $i
        exec bash
    "

    echo "Waiting 20 seconds before next drone..."
    sleep 20
done

# ── Summary ───────────────────────────────────────────────────────────────────
echo ""
echo "=== All $NUM_DRONES drones launched ==="
echo ""
echo "MAVSDK / MAVROS ports:"
for i in $(seq 1 $NUM_DRONES); do
    echo "  Drone $i → udp://:$((14540 + i - 1))"
done
echo ""
echo "ROS 2 (Micro XRCE-DDS) — run in a new terminal:"
echo "  MicroXRCEAgent udp4 -p 8888"
echo "  Topics: /px4_1/fmu/..., /px4_2/fmu/..., etc."
echo ""
echo "Python (MAVSDK) ports:"
for i in $(seq 1 $NUM_DRONES); do
    echo "  Drone $i → System(port=$((50040 + i - 1))) + connect('udp://:$((14540 + i - 1))')"
done
