"""
Bridge Gazebo camera topics to ROS 2 for all drones using a YAML config.
Topics bridged:
  /drone1/camera/image_raw  (+ camera_info)
  /drone2/camera/image_raw  (+ camera_info)
  /drone3/camera/image_raw  (+ camera_info)

Usage:
  ros2 launch ~/Desktop/Swarm/ros2_ws/src/swarm_bringup/launch/camera_bridge.launch.py
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node

# Absolute path — safe whether launched by file path or via package
CONFIG = os.path.join(
    os.path.dirname(os.path.realpath(__file__)), "..", "config", "camera_bridge.yaml"
)


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="camera_bridge",
            parameters=[{
                "config_file": CONFIG,
                # Match Gazebo's BEST_EFFORT QoS so messages aren't dropped
                "use_sim_time": True,
            }],
            output="screen",
        )
    ])
