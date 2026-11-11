#!/usr/bin/env python
"""Launch one milestone 2 vehicle on the up/down path: climb to 5 m, then descend.

Same as offboard_solution.launch.py, and takes the same path, topic_prefix and
system_id arguments, with a different default waypoint file. It starts no
simulator, so it runs unchanged in SITL and on a real drone.

Usage:
  ros2 launch milestone2_multivehicle updown_launch.py [topic_prefix:=/px4_1 system_id:=2]
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

PACKAGE = 'milestone2_multivehicle'


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument(
            'path',
            default_value=PathJoinSubstitution([FindPackageShare(PACKAGE), 'resource', 'milestone2_updown.yaml']),
            description='Waypoint YAML file (yaw in radians)'),
        DeclareLaunchArgument('topic_prefix', default_value='',
                              description="Prefix for this vehicle's PX4 topics, e.g. /px4_1; "
                                          'empty for the bare /fmu topics'),
        DeclareLaunchArgument('system_id', default_value='1',
                              description="This vehicle's MAV_SYS_ID, which its commands are sent to"),
    ]
    main_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory(PACKAGE), 'offboard_solution.launch.py')),
        launch_arguments={name: LaunchConfiguration(name)
                          for name in ('path', 'topic_prefix', 'system_id')}.items())
    return LaunchDescription([*arguments, main_launch])
