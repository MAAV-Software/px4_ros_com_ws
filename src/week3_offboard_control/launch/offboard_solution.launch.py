#!/usr/bin/env python
"""Launch the week 3 offboard control reference solution, optionally bringing
up PX4 SITL + the Micro XRCE-DDS Agent (week2_px4_sitl) first — each in its
own terminal.

Usage:
  # SITL/agent not running yet — bring them up too, then the solution node:
  ros2 launch week3_offboard_control offboard_solution.launch.py

  # SITL/agent already running (e.g. you brought them up by hand per the
  # "Before starting" step) — just the solution node, so you don't spin up a
  # second conflicting SITL/Gazebo instance:
  ros2 launch week3_offboard_control offboard_solution.launch.py launch_sitl:=false
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# PX4 SITL + Gazebo can take a while to build and boot, especially the first
# time — give it a head start before the offboard node begins streaming
# setpoints. If it still isn't armed/flying, PX4 wasn't up in time; just
# rerun this launch file (with launch_sitl:=false, since SITL is up by now).
SITL_STARTUP_DELAY = 20.0


def generate_launch_description():
    launch_sitl_arg = DeclareLaunchArgument(
        'launch_sitl', default_value='true',
        description='Also launch PX4 SITL + the XRCE agent (week2_px4_sitl). '
                     'Set to false if they are already running.')
    launch_sitl = LaunchConfiguration('launch_sitl')

    sitl_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('week2_px4_sitl'), 'px4_sitl.launch.py')),
        condition=IfCondition(launch_sitl))

    # We launched SITL ourselves — wait for it to boot before streaming setpoints.
    delayed_offboard_node = TimerAction(
        period=SITL_STARTUP_DELAY,
        actions=[
            Node(
                package='week3_offboard_control',
                executable='offboard_solution',
                name='offboard_control_solution',
                prefix='gnome-terminal --'),
        ],
        condition=IfCondition(launch_sitl))

    # SITL was already running — start immediately, no delay needed.
    immediate_offboard_node = Node(
        package='week3_offboard_control',
        executable='offboard_solution',
        name='offboard_control_solution',
        prefix='gnome-terminal --',
        condition=UnlessCondition(launch_sitl))

    return LaunchDescription([
        launch_sitl_arg,
        sitl_launch,
        delayed_offboard_node,
        immediate_offboard_node,
    ])
