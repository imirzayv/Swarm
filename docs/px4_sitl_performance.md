# PX4 SITL Performance: GPU, Headless & CPU Starvation

## Contents
1. [GPU Access for Gazebo](#1-gpu-access-for-gazebo)
2. [Running PX4 Headless](#2-running-px4-headless)
3. [Overcoming CPU Starvation](#3-overcoming-cpu-starvation)

---

## 1. GPU Access for Gazebo

Gazebo Harmonic uses the **OGRE2** renderer which requires a properly exposed GPU.
You have an RTX 4060 Laptop — NVIDIA drivers are installed (verified). The issue on
laptops is Optimus: the display runs through Intel iGPU by default, and Gazebo may
silently fall back to software rendering.

### Verify which renderer Gazebo is using

```bash
gz sim --verbose 2>&1 | grep -Ei "render|ogre|opengl|vulkan"
```

If you see `OGRE2` and `NVIDIA` — you're good. If you see `llvmpipe` or `Mesa` —
Gazebo is on the CPU and this is what causes slowdowns with multiple drones.

### Force NVIDIA GPU (Optimus/Laptop fix)

Add these exports **before** launching PX4 SITL, either in your terminal or `~/.bashrc`:

```bash
# Force Gazebo onto the NVIDIA dGPU (Optimus)
export __NV_PRIME_RENDER_OFFLOAD=1
export __GLX_VENDOR_LIBRARY_NAME=nvidia
export __VK_LAYER_NV_optimus=NVIDIA_only

# Force OGRE2 renderer (GPU-accelerated, default in Harmonic but explicit is safer)
export PX4_GZ_SIM_RENDER_ENGINE=ogre2
```

Then run your launch script normally. To make permanent:

```bash
echo 'export __NV_PRIME_RENDER_OFFLOAD=1' >> ~/.bashrc
echo 'export __GLX_VENDOR_LIBRARY_NAME=nvidia' >> ~/.bashrc
echo 'export PX4_GZ_SIM_RENDER_ENGINE=ogre2' >> ~/.bashrc
source ~/.bashrc
```

### Verify GPU is being used after launch

```bash
# While Gazebo is running:
nvidia-smi
# GPU-Util % should be > 0 when Gazebo is rendering
```

---

## 2. Running PX4 Headless

Headless mode disables the Gazebo **GUI** (3D window) but keeps the physics server
running. This is the single biggest resource saving when running 3+ drones, as OGRE2
rendering consumes significant CPU+GPU even when you're not watching.

### How to enable headless

Set `HEADLESS=1` before the first drone (which starts the Gazebo server):

```bash
# Terminal 1 — Gazebo server only, no GUI window
HEADLESS=1 \
PX4_SYS_AUTOSTART=4001 \
PX4_SIM_MODEL=gz_x500 \
./build/px4_sitl_default/bin/px4 -i 1
```

Subsequent drones with `PX4_GZ_STANDALONE=1` are unaffected — they never touch the GUI.

### Re-attach the GUI later (optional)

```bash
gz sim -g
```

This opens the Gazebo GUI and connects to the already-running physics server.

### Headless via the launch script

```bash
HEADLESS=1 ./scripts/launch_multi_drone.sh 3
```

The script passes `HEADLESS` through to the first drone's terminal automatically since
it inherits the environment.

---

## 3. Overcoming CPU Starvation

When running 3+ drones, each PX4 instance + Gazebo physics compete for CPU cores.
The sensor timeout / accel-missing errors are almost always CPU starvation, not
genuine sensor failures.

### Root cause

PX4 SITL uses **lockstep simulation**: PX4 waits for a sensor update from Gazebo,
processes it, then signals Gazebo to advance. If the CPU can't deliver the next
physics step in time, PX4 times out and reports missing sensors.

### Fix 1: Slow down the simulation clock (most effective)

After all drones are up, reduce the real-time factor so physics runs slower than
wall-clock time, giving every PX4 instance enough CPU budget:

```bash
# 0.5 = simulation runs at half real-time speed
# Increase to 1.0 once stable, decrease if still timing out
gz service -s /world/default/set_physics \
  --reqtype gz.msgs.Physics \
  --reptype gz.msgs.Boolean \
  --timeout 5000 \
  --req "real_time_factor: 0.5"
```

### Fix 2: Increase spawn delay between drones

Each new drone stresses the physics engine during its initialization. The launch script
already waits 20–25 seconds between spawns. Increase to 30s on slow machines by editing
`SPACING` at the top of [launch_multi_drone.sh](../scripts/launch_multi_drone.sh).

### Fix 3: Lock Gazebo to your performance cores

On laptops, Linux may schedule Gazebo on efficiency cores. Pin it to P-cores:

```bash
GZ_PID=$(pgrep -f "gz sim")
taskset -cp 0-7 $GZ_PID   # adjust range to your CPU's P-core count
```

### Fix 4: Reduce camera update rates

Each simulated camera adds CPU+GPU load per frame. See
[drone_cameras.md](drone_cameras.md) for how to lower the camera `<update_rate>`.

### Fix 5: Run headless

Disabling the GUI saves ~1–2 CPU cores worth of render thread load. See
[section 2](#2-running-px4-headless) above.

---

*Companion docs: [drone_cameras.md](drone_cameras.md) · [quick_start_guide.md](../quick_start_guide.md)*
