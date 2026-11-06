#!/usr/bin/env python
"""Launch the milestone 1 offboard node on the climb-and-descend path.

Same as offboard_solution.launch.py (and takes the same launch_sitl, headless,
topic_prefix, relative_start, record, bag_dir and path arguments), with a different default waypoint file.

Usage:
  ros2 launch milestone1_singlevehicle updown_launch.py [launch_sitl:=false] [headless:=false]
  # Real fleet drone (PX4 namespace px4_1), SITL/agent already running:
  ros2 launch milestone1_singlevehicle updown_launch.py launch_sitl:=false topic_prefix:=/px4_1
  # ...and record a rosbag of the flight to ~/flight_bags/flight_<time>:
  ros2 launch milestone1_singlevehicle updown_launch.py launch_sitl:=false topic_prefix:=/px4_1 record:=true
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    path_arg = DeclareLaunchArgument(
        'path',
        default_value=PathJoinSubstitution([
            FindPackageShare('milestone1_singlevehicle'), 'resource', 'milestone1_updown.yaml']),
        description='Waypoint YAML file (yaw in radians)')
    launch_sitl_arg = DeclareLaunchArgument('launch_sitl', default_value='true')
    headless_arg = DeclareLaunchArgument('headless', default_value='true')
    topic_prefix_arg = DeclareLaunchArgument('topic_prefix', default_value='')
    relative_start_arg = DeclareLaunchArgument('relative_start', default_value='false')
    record_arg = DeclareLaunchArgument('record', default_value='false')
    bag_dir_arg = DeclareLaunchArgument(
        'bag_dir', default_value=PathJoinSubstitution([EnvironmentVariable('HOME'), 'flight_bags']))

    main_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('milestone1_singlevehicle'), 'offboard_solution.launch.py')),
        launch_arguments={
            'path': LaunchConfiguration('path'),
            'launch_sitl': LaunchConfiguration('launch_sitl'),
            'headless': LaunchConfiguration('headless'),
            'topic_prefix': LaunchConfiguration('topic_prefix'),
            'relative_start': LaunchConfiguration('relative_start'),
            'record': LaunchConfiguration('record'),
            'bag_dir': LaunchConfiguration('bag_dir'),
        }.items())

    return LaunchDescription([path_arg, launch_sitl_arg, headless_arg, topic_prefix_arg, relative_start_arg, record_arg, bag_dir_arg, main_launch])
