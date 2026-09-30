#!/usr/bin/env python
"""Launch the vehicle_local_position listener in its own terminal.

Automated version of week 2 task 3. Run `ros2 topic echo
/fmu/out/vehicle_local_position_v1` by hand first (see week-2-px4-sitl-gazebo.md)
so you can compare the raw CLI output to what this node logs.

Usage:
  ros2 launch week2_px4_sitl listener.launch.py

Requires PX4 SITL + the Micro XRCE-DDS Agent already running
(see px4_sitl.launch.py in this package).
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='week2_px4_sitl',
            executable='listener',
            name='local_position_listener',
            prefix='gnome-terminal --'
        ),
    ])
