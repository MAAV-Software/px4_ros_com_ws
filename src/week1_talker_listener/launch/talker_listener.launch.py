#!/usr/bin/env python
"""Launch the week 1 talker and listener, each in its own terminal.

Mirrors the terminal-launch pattern used by the archived offboard packages
(see archive/src/px4_multi_vehicle_offboard/px4_single_plan/launch/offboard_waypoint.launch.py):
each node gets its own gnome-terminal window via the `prefix` argument, so you
can watch it directly instead of both nodes' logs interleaving in one process.

Usage:
  ros2 launch week1_talker_listener talker_listener.launch.py
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='week1_talker_listener',
            executable='talker',
            name='talker',
            prefix='gnome-terminal --'
        ),
        Node(
            package='week1_talker_listener',
            executable='listener',
            name='listener',
            prefix='gnome-terminal --'
        ),
    ])
