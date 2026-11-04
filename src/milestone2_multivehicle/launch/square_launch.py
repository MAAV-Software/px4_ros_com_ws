#!/usr/bin/env python
"""Launch the two milestone 2 offboard nodes on the square path: climb, a 5 m square, then descend.

Same as offboard_solution.launch.py (and takes the same launch_sitl, headless
and path arguments), with a different default waypoint file.

Usage:
  ros2 launch milestone2_multivehicle square_launch.py [launch_sitl:=false] [headless:=false]
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    path_arg = DeclareLaunchArgument(
        'path',
        default_value=PathJoinSubstitution([
            FindPackageShare('milestone2_multivehicle'), 'resource', 'milestone2_square.yaml']),
        description='Waypoint YAML file (yaw in radians)')
    launch_sitl_arg = DeclareLaunchArgument('launch_sitl', default_value='true')
    headless_arg = DeclareLaunchArgument('headless', default_value='true')

    main_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('milestone2_multivehicle'), 'offboard_solution.launch.py')),
        launch_arguments={
            'path': LaunchConfiguration('path'),
            'launch_sitl': LaunchConfiguration('launch_sitl'),
            'headless': LaunchConfiguration('headless'),
        }.items())

    return LaunchDescription([path_arg, launch_sitl_arg, headless_arg, main_launch])
