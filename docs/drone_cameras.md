# Drone Cameras: Attach, Bridge & Visualize

## Contents
1. [Attaching Cameras to Drones](#1-attaching-cameras-to-drones)
2. [Bridging Gazebo Camera Topics to ROS 2](#2-bridging-gazebo-camera-topics-to-ros-2)
3. [Viewing Feeds with Foxglove](#3-viewing-feeds-with-foxglove)
4. [Viewing Feeds with RViz2 / rqt](#4-viewing-feeds-with-rviz2--rqt)

---

## 1. Attaching Cameras to Drones

PX4 ships ready-made camera variants of the x500. The easiest approach is to use
these directly — no SDF editing needed.

### Option A: Use a built-in camera model (recommended)

Simply change the model argument in the launch script:

| Model | Camera orientation | Use case |
|-------|--------------------|----------|
| `x500_mono_cam` | Forward-facing | Target tracking, navigation |
| `x500_mono_cam_down` | Downward-facing (nadir) | Mapping, coverage, agriculture |
| `x500_depth` | Depth camera (RGBD) | 3D reconstruction, obstacle avoidance |
| `x500_vision` | Vision + optical flow | Accurate indoor positioning |

```bash
# 3 drones with downward cameras
./scripts/launch_multi_drone.sh 3 x500_mono_cam_down
```

The camera is automatically part of the model and publishes to Gazebo transport topics.

---

### Option B: Understand what's inside

The `x500_mono_cam` model is a thin wrapper that merges `x500` with `mono_cam`:

```xml
<!-- firmware/Tools/simulation/gz/models/x500_mono_cam/model.sdf -->
<model name='x500_mono_cam'>
  <include merge='true'><uri>x500</uri></include>
  <include merge='true'>
    <uri>model://mono_cam</uri>
    <pose>.12 .03 .242 0 0 0</pose>   <!-- forward, slightly right, above center -->
  </include>
  <joint name="CameraJoint" type="fixed">
    <parent>base_link</parent>
    <child>camera_link</child>
  </joint>
</model>
```

The `mono_cam` sensor (`firmware/Tools/simulation/gz/models/mono_cam/model.sdf`):

```xml
<sensor name="camera" type="camera">
  <gz_frame_id>camera_link</gz_frame_id>
  <camera>
    <horizontal_fov>1.74</horizontal_fov>   <!-- ~100 degrees -->
    <image>
      <width>1280</width>
      <height>960</height>
    </image>
    <clip><near>0.1</near><far>3000</far></clip>
  </camera>
  <always_on>1</always_on>
  <update_rate>30</update_rate>
</sensor>
```

`x500_mono_cam_down` is identical but with `<pose>0 0 .10 0 1.5707 0</pose>` — the
`1.5707` (π/2) rotation on the pitch axis points the lens straight down.

---

### Option C: Create a custom camera model (lower resolution / FPS)

Useful when CPU is saturated — halving resolution and FPS reduces render cost ~4×.

```bash
cp -r ~/Desktop/Swarm/firmware/Tools/simulation/gz/models/x500_mono_cam \
      ~/Desktop/Swarm/firmware/Tools/simulation/gz/models/x500_cam_lite
```

Edit `x500_cam_lite/model.sdf` and change the `mono_cam` include URI to a new model,
or edit the sensor directly in the copied `mono_cam` model:

```xml
<!-- reduce from 1280x960@30 to 640x480@15 -->
<image>
  <width>640</width>
  <height>480</height>
</image>
<update_rate>15</update_rate>
```

Then launch with:
```bash
./scripts/launch_multi_drone.sh 3 x500_cam_lite
```

---

## 2. Bridging Gazebo Camera Topics to ROS 2

Camera images are published on **Gazebo transport**, not on PX4/XRCE-DDS. The
MicroXRCEAgent only carries flight data. To get images into ROS 2 you need
`ros_gz_bridge`.

### Install

```bash
sudo apt install ros-humble-ros-gz-bridge
```

### Find the exact Gazebo topic name

```bash
gz topic -l | grep image
gz topic -e -t /world/default/model/x500_mono_cam_down_1/link/camera_link/sensor/imager/image
```

Topics follow this pattern:
```
/world/default/model/x500_mono_cam_down_1/link/camera_link/sensor/camera/image
/world/default/model/x500_mono_cam_down_2/link/camera_link/sensor/camera/image
/world/default/model/x500_mono_cam_down_3/link/camera_link/sensor/camera/image
```

> The suffix `_1`, `_2`, `_3` is the PX4 instance number set by `-i` in the launch
> script.

### Bridge a single drone manually

```bash
source /opt/ros/humble/setup.bash

ros2 run ros_gz_bridge parameter_bridge \
  /world/default/model/x500_mono_cam_down_1/link/camera_link/sensor/camera/image@sensor_msgs/msg/Image[gz.msgs.Image
```

### Bridge all drones via launch file

[ros2_ws/src/swarm_bringup/launch/camera_bridge.launch.py](../ros2_ws/src/swarm_bringup/launch/camera_bridge.launch.py)
bridges all drones at once and remaps the long Gazebo paths to short ROS 2 topics:

```
/drone1/camera/image_raw
/drone2/camera/image_raw
/drone3/camera/image_raw
```

Run it:
```bash
source /opt/ros/humble/setup.bash
ros2 launch ~/Desktop/Swarm/ros2_ws/src/swarm_bringup/launch/camera_bridge.launch.py
```

Verify:
```bash
ros2 topic list | grep drone
ros2 topic hz /drone1/camera/image_raw   # expect ~30 Hz
```

---

## 3. Viewing Feeds with Foxglove

Foxglove Studio can connect directly to your ROS 2 topics via WebSocket — no extra
tooling needed beyond `foxglove_bridge`.

### Install

```bash
sudo apt install ros-humble-foxglove-bridge
```

### Start the bridge

```bash
source /opt/ros/humble/setup.bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
```

You'll see:
```
[foxglove_bridge] WebSocket server listening on ws://0.0.0.0:8765
```

### Connect Foxglove Studio

1. Open [Foxglove Studio](https://studio.foxglove.dev) (desktop app or browser)
2. **Open Connection** → **Foxglove WebSocket**
3. URL: `ws://localhost:8765`
4. Add an **Image** panel → set topic to `/drone1/camera/image_raw`

> Foxglove sees all active ROS 2 topics automatically. Make sure the `camera_bridge`
> launch is also running so the image topics exist.

### View multiple cameras side by side

Add one Image panel per drone. Foxglove lets you tile panels in a grid layout — useful
for monitoring all drones simultaneously.

---

## 4. Viewing Feeds with RViz2 / rqt

Alternative viewers if you prefer to stay in ROS 2 native tools.

### rqt_image_view

```bash
sudo apt install ros-humble-rqt-image-view   # if not installed
ros2 run rqt_image_view rqt_image_view
# Select topic from the dropdown: /drone1/camera/image_raw
```


If i remember correctly, if you have humble and GZ_VERSION=harmonic you have to:

Delete previous gz_sim installation
Install gz_sim using: sudo apt-get install ros-humble-ros-gzharmonic
Source everything again
And it should work.

### RViz2

```bash
rviz2
# Panels → Add → Image
# Set "Image Topic" to /drone1/camera/image_raw
```

### Quick Python subscriber (verify data is flowing)

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

rclpy.init()
node = Node('cam_check')
node.create_subscription(
    Image,
    '/drone1/camera/image_raw',
    lambda msg: print(f"[drone1] {msg.width}x{msg.height} {msg.encoding}"),
    10
)
rclpy.spin(node)
```

---

*Companion docs: [px4_sitl_performance.md](px4_sitl_performance.md) · [quick_start_guide.md](../quick_start_guide.md)*
