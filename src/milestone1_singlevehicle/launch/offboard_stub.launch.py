#!/usr/bin/env python
"""Launch the week 3 offboard control stub node in its own terminal.

Requires PX4 SITL + the Micro XRCE-DDS Agent already running — see
week2_px4_sitl's px4_sitl.launch.py.

Usage:
  ros2 launch milestone1_singlevehicle offboard_stub.launch.py
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='milestone1_singlevehicle',
            executable='offboard_stub',
            name='offboard_control_stub',
            prefix='gnome-terminal --'
        ),
    ])
