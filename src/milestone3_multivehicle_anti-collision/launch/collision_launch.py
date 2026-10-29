#!/usr/bin/env python
"""Two predetermined offset-pass routes that exercise the 5 m yield rule."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package = 'milestone3_multivehicle'
    paths = [DeclareLaunchArgument(
        f'path_{i + 1}', default_value=PathJoinSubstitution([
            FindPackageShare(package), 'resource', f'milestone3_collision_{i}.yaml']))
        for i in range(2)]
    main_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory(package), 'offboard_solution.launch.py')),
        launch_arguments={f'path_{i + 1}': LaunchConfiguration(f'path_{i + 1}')
                          for i in range(2)}.items())
    return LaunchDescription([*paths, main_launch])
