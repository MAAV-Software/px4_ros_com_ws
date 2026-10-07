#!/usr/bin/env python
"""Launch the week 4 offboard control reference solution, targeting one vehicle instance.

Requires PX4 SITL + the Micro XRCE-DDS Agent already
running (see week-4-git-workflow-multivehicle.md task 2).

Usage:
  ros2 launch week4_multivehicle offboard_solution.launch.py instance:=1
  ros2 launch week4_multivehicle offboard_solution.launch.py instance:=0
  ros2 launch week4_multivehicle offboard_solution.launch.py instance:=0 path:=/path/to/waypoints.yaml
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    instance_arg = DeclareLaunchArgument(
        'instance', default_value='1',
        description='Vehicle instance to target (0 or 1 in a 2-instance setup)')

    # Resolve the installed YAML so launch works from any current directory.
    path_arg = DeclareLaunchArgument(
        'path',
        default_value=PathJoinSubstitution([
            FindPackageShare('week4_multivehicle'), 'resource',
            'milestone1_path_t1_t7.yaml',
        ]),
        description='Waypoint YAML file (yaw in radians)')

    return LaunchDescription([
        instance_arg,
        path_arg,
        Node(
            package='week4_multivehicle',
            executable='offboard_solution',
            name='offboard_control_solution',
            parameters=[{
                'vehicle_instance': ParameterValue(LaunchConfiguration('instance'), value_type=int),
                'waypoints_file': ParameterValue(LaunchConfiguration('path'), value_type=str),
            }],
            output='screen'
        ),
    ])
