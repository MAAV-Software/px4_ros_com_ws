#!/usr/bin/env python
"""Launch the week 4 offboard control stub, targeting one vehicle instance.

Requires a 2-instance PX4 SITL setup + the Micro XRCE-DDS Agent already
running (see week-4-git-workflow-multivehicle.md task 2).

Usage:
  ros2 launch week4_multivehicle offboard_stub.launch.py instance:=1
  ros2 launch week4_multivehicle offboard_stub.launch.py instance:=0
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    instance_arg = DeclareLaunchArgument(
        'instance', default_value='1',
        description='Vehicle instance to target (0 or 1 in a 2-instance setup)')

    return LaunchDescription([
        instance_arg,
        Node(
            package='week4_multivehicle',
            executable='offboard_stub',
            name='offboard_control_stub',
            # launch arguments are strings by default; the node's
            # vehicle_instance parameter is declared as an int, so cast it here.
            parameters=[{'vehicle_instance': ParameterValue(LaunchConfiguration('instance'), value_type=int)}],
            prefix='gnome-terminal --'
        ),
    ])
