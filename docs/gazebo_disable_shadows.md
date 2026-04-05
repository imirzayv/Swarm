# Disabling Shadows in Gazebo Harmonic for PX4 SITL

Drone shadows cast onto the ground plane cause false positive detections in
YOLOv8 (the shadow silhouette gets classified as objects like "kite" or
"airplane"). Disabling shadows eliminates this noise.

---

## How It Works

PX4 SITL loads a Gazebo world SDF at startup. The world file controls scene
rendering settings including shadows. We use a custom world file with shadows
disabled and symlink it into PX4's worlds directory.

### Files

| File | Purpose |
|------|---------|
| `worlds/default.sdf` | Custom world — copy of PX4's `default.sdf` with shadows off |
| `scripts/launch_multi_drone.sh` | Symlinks our world into PX4 and sets `PX4_GZ_WORLD` |

### What Changed in the World SDF

Two modifications from the original `firmware/Tools/simulation/gz/worlds/default.sdf`:

**1. Scene-level shadows (line 16):**
```xml
<scene>
  ...
  <shadows>false</shadows>   <!-- was: true -->
</scene>
```

**2. Sun light shadow casting (line 69):**
```xml
<light name="sunUTC" type="directional">
  ...
  <cast_shadows>false</cast_shadows>   <!-- was: true -->
</light>
```

**3. World name (line 3):**
```xml
<world name="swarm_default">   <!-- was: default -->
```

Both shadow settings must be disabled. `<shadows>` controls the renderer's
global shadow pass, `<cast_shadows>` controls whether the specific light
source produces shadows. The world name must match the SDF filename so PX4's
`gz_bridge` can find the running world.

---

## Why Not Just Edit PX4's Default World?

PX4 firmware is a git submodule. Editing files inside `firmware/` means:
- `git submodule update` would revert the change
- `git status` shows the submodule as dirty

Instead, we keep our world in `worlds/` (tracked in our repo) and symlink it
into PX4's worlds directory at launch time.

---

## How the Launch Script Loads It

`launch_multi_drone.sh` does three things:

```bash
# 1. Symlink our world into PX4's worlds directory
ln -sf "$PROJECT_DIR/worlds/default.sdf" \
       "$PX4_DIR/Tools/simulation/gz/worlds/swarm_default.sdf"

# 2. Set the world name env var (passed to each PX4 instance)
export PX4_GZ_WORLD="swarm_default"

# 3. First drone starts Gazebo with the custom world
PX4_GZ_WORLD='swarm_default' ./build/px4_sitl_default/bin/px4 -i 1
```

### Why Symlink Instead of Setting PX4_GZ_WORLDS?

PX4's startup script (`px4-rc.gzsim`) sources `gz_env.sh` which **overrides**
`PX4_GZ_WORLDS` back to `firmware/Tools/simulation/gz/worlds/`. Setting this
env var externally has no effect — it gets clobbered before the world is loaded.

The symlink approach works because:
1. `gz_env.sh` sets `PX4_GZ_WORLDS` to PX4's worlds directory
2. Our symlink is already there: `swarm_default.sdf -> our worlds/default.sdf`
3. PX4 loads `${PX4_GZ_WORLDS}/swarm_default.sdf` — which is our file

---

## Target Spawning

Since the world name changed from `default` to `swarm_default`, the spawn
script must use the new name for the `gz service` endpoint:

```bash
# spawn_targets.sh uses:
gz service -s /world/swarm_default/create ...
```

This is handled by `WORLD=${1:-swarm_default}` in `spawn_targets.sh`.

---

## Verification

After launching the simulation:

1. **Visual check:** Fly a drone over the ground plane — no shadow should
   appear beneath it.

2. **Detection check:** Run `detection_publisher.py` — false positives from
   drone shadows should be eliminated.

3. **World name check:**
   ```bash
   gz topic -l | grep world
   # Should show: /world/swarm_default/...
   ```
