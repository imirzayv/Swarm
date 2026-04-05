# Spawning Realistic Targets from Gazebo Fuel

## What is Gazebo Fuel?

[Gazebo Fuel](https://app.gazebosim.org) is a cloud-hosted repository of 3D
models, worlds, and plugins for Gazebo simulations. It is maintained by
Open Robotics and the Gazebo community. Think of it as a package registry
(like npm or PyPI) but for simulation assets.

Key features:
- **Free and open** — most models are Creative Commons or Apache licensed
- **Thousands of models** — vehicles, buildings, people, furniture, terrain
- **Automatic caching** — models download once to `~/.gz/fuel/` and are reused
- **URI-based referencing** — include models in SDF files by URL, no manual
  file management needed

Browse models at: https://app.gazebosim.org/fuel/models

### Fuel CLI

The `gz fuel` command manages model downloads:

```bash
# Download a model
gz fuel download -u "https://fuel.gazebosim.org/1.0/OpenRobotics/models/SUV"

# List cached models
ls ~/.gz/fuel/fuel.gazebosim.org/openrobotics/models/

# Models are cached at:
# ~/.gz/fuel/fuel.gazebosim.org/<owner>/models/<model_name>/<version>/
```

### Using Fuel Models in SDF

Reference a Fuel model directly by its URL:

```xml
<include>
  <uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/SUV</uri>
  <pose>5 3 0 0 0 0</pose>
  <static>true</static>
</include>
```

Gazebo resolves the URI from the local cache. If not cached, it downloads
automatically on first use.

---

## Why Fuel Models Instead of Primitives?

The original `spawn_targets.sh` used colored geometric shapes (boxes,
cylinders, spheres). These caused two problems:

1. **YOLOv8 cannot detect them** — YOLO is trained on the COCO dataset
   (80 real-world classes). Colored primitives don't match any class, so
   YOLO guesses the closest match ("kite", "suitcase") with low confidence,
   producing unreliable false positives.

2. **Not realistic** — for a paper on area monitoring, targets should
   resemble objects a real drone would need to detect (vehicles, people).

**Solution:** Use textured 3D vehicle models from Gazebo Fuel. These render
realistically from a top-down camera view and match COCO classes that YOLOv8
is trained to detect: `car`, `truck`, `bus`.

---

## Available Models

Models used in `spawn_targets.sh`, all from `OpenRobotics` on Fuel:

| Model | Fuel Name | COCO Class | Notes |
|-------|-----------|------------|-------|
| SUV | `SUV` | car | Large vehicle, easy to detect |
| Red Hatchback | `Hatchback red` | car | Compact, distinctive color |
| Blue Hatchback | `Hatchback blue` | car | Compact, distinctive color |
| Pickup Truck | `Pickup` | truck | Larger footprint |
| Prius | `Prius Hybrid` | car | Mid-size sedan |
| Bus | `Bus` | bus | Very large, high confidence |
| Hatchback | `Hatchback` | car | Silver/grey variant |

All models are static (no physics), textured with realistic materials, and
properly scaled for Gazebo.

---

## How spawn_targets.sh Works

### 1. Download Phase

On first run, the script downloads any missing models from Fuel:

```bash
download_if_needed() {
    local cache_dir="$HOME/.gz/fuel/.../$(echo "$model_name" | tr '[:upper:]' '[:lower:]')"
    if [ ! -d "$cache_dir" ]; then
        gz fuel download -u "$FUEL_BASE/$model_name" > /dev/null 2>&1
    fi
}
```

Subsequent runs skip the download (cached in `~/.gz/fuel/`).

### 2. Spawn Phase

For each target, the script:

1. Creates a temporary SDF file that `<include>`s the Fuel model URI
2. Calls `gz service /world/<world>/create` with the SDF file path, target
   name, position, and orientation
3. Cleans up the temporary file

```bash
# Example: spawn an SUV at position (5, 3) with 0 yaw
spawn_fuel_model "target_suv_1" 5 3 0.1 0 \
    "https://fuel.gazebosim.org/1.0/OpenRobotics/models/SUV"
```

### 3. Orientation

Each target gets a yaw rotation (converted to quaternion for the `gz service`
pose message) so vehicles face different directions. This makes the scene
more realistic and tests detection from multiple viewing angles.

---

## Usage

```bash
# Spawn 5 targets (default)
./scripts/spawn_targets.sh

# Spawn 5 targets in a specific world
./scripts/spawn_targets.sh swarm_default

# Spawn 10 targets
./scripts/spawn_targets.sh swarm_default 10
```

### Target Positions

Predefined positions spread across the monitoring area (local XY in meters):

| Index | X | Y | Model |
|-------|-----|------|-------|
| 1 | 70 | -60 | SUV |
| 2 | -80 | 70 | Hatchback red |
| 3 | 50 | 80 | Hatchback blue |
| 4 | -60 | -50 | Pickup |
| 5 | 85 | -30 | Prius |
| 6 | -75 | 20 | Bus |
| 7 | -20 | -85 | Hatchback |
| 8 | 40 | 45 | SUV |
| 9 | -50 | 55 | Hatchback red |
| 10 | 65 | -75 | Hatchback blue |

When spawning fewer than 10, only the first N positions are used.
When spawning more than 7, models are cycled (modulo).

---

## Adding New Models

To add a new model from Fuel:

1. **Find the model** at https://app.gazebosim.org/fuel/models

2. **Add the download** in `spawn_targets.sh`:
   ```bash
   download_if_needed "Model Name"
   ```

3. **Add to the MODELS array**:
   ```bash
   MODELS+=("https://fuel.gazebosim.org/1.0/OpenRobotics/models/Model Name")
   MODEL_NAMES+=("model_name")
   ```

4. **Add positions** to the `POS_X` and `POS_Y` arrays if increasing total
   target count.

### Finding COCO-Compatible Models

Search Fuel for objects matching COCO's 80 classes. Good candidates for
aerial monitoring:

- **Vehicles:** `car`, `truck`, `bus`, `motorcycle`, `bicycle`
- **People:** COCO has `person` — search Fuel for "person", "pedestrian"
- **Outdoor objects:** `bench`, `fire hydrant`, `stop sign`

Check the COCO class list at: https://cocodataset.org/#explore

---

## Troubleshooting

| Problem | Cause | Fix |
|---------|-------|-----|
| `Download failed` | No internet or Fuel server down | Check network; models cache after first download |
| Model spawns underground | Z position too low | Increase Z to 0.1 or higher |
| Model not visible | Scale too small | Check model SDF for `<scale>` values |
| `Service call timed out` | Gazebo not running or wrong world name | Verify `gz topic -l` shows world topics |
| YOLO still detects as wrong class | Altitude too high, target too small | Lower altitude or increase confidence threshold |
