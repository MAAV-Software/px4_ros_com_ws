#!/usr/bin/env python
"""Four predetermined diagonal convergence routes for SITL stress testing."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package = 'milestone4_multivehicle'
    paths = [DeclareLaunchArgument(
        f'path_{i + 1}', default_value=PathJoinSubstitution([
            FindPackageShare(package), 'resource', f'milestone4_headon_{i}.yaml']))
        for i in range(4)]
    main_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory(package), 'offboard_solution.launch.py')),
        launch_arguments={f'path_{i + 1}': LaunchConfiguration(f'path_{i + 1}')
                          for i in range(4)}.items())
    return LaunchDescription([*paths, main_launch])
