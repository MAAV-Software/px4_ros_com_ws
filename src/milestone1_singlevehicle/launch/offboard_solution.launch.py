#!/usr/bin/env python
"""Launch the milestone 1 offboard control node, optionally bringing up PX4
SITL + the Micro XRCE-DDS Agent (px4_sitl.launch.py) first. Gazebo runs
headless (no GUI window) unless headless:=false.

The node is started only once PX4 reports a valid position
a valid local position estimate (m1_wait_ready), not after a fixed delay, so it works the
same whether SITL was just started, is already running, or is a real drone.

Usage:
  # SITL/agent not running yet — bring them up too, then the solution node:
  ros2 launch milestone1_singlevehicle offboard_solution.launch.py

  # SITL/agent already running (or a real drone over the XRCE agent) — just
  # the solution node, so you don't spin up a second conflicting instance:
  ros2 launch milestone1_singlevehicle offboard_solution.launch.py launch_sitl:=false

  # Show the Gazebo GUI instead of running headless:
  ros2 launch milestone1_singlevehicle offboard_solution.launch.py headless:=false

  # Use a different waypoint YAML (yaw in radians):
  ros2 launch milestone1_singlevehicle offboard_solution.launch.py path:=/path/to/waypoints.yaml
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription,
    LogInfo, RegisterEventHandler)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

# How long to wait for PX4 to become ready. The first start after a build
# (PX4 + Gazebo compile) can take minutes.
READY_TIMEOUT_S = 300


def generate_launch_description():
    launch_sitl_arg = DeclareLaunchArgument(
        'launch_sitl', default_value='true',
        description='Also launch PX4 SITL + the XRCE agent (px4_sitl.launch.py). '
                     'Set to false if they are already running.')

    headless_arg = DeclareLaunchArgument(
        'headless', default_value='true',
        description='Run Gazebo without its GUI window. Only used when launch_sitl is true.')

    # Resolve the installed YAML so launch works from any current directory.
    path_arg = DeclareLaunchArgument(
        'path',
        default_value=PathJoinSubstitution([
            FindPackageShare('milestone1_singlevehicle'), 'resource',
            'milestone1_path.yaml',
        ]),
        description='Waypoint YAML file (yaw in radians)')

    sitl_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('milestone1_singlevehicle'), 'px4_sitl.launch.py')),
        launch_arguments={'headless': LaunchConfiguration('headless')}.items(),
        condition=IfCondition(LaunchConfiguration('launch_sitl')))

    # Exits 0 once PX4 is ready (immediately, if it already is), 1 on timeout.
    wait_ready = Node(
        package='milestone1_singlevehicle',
        executable='m1_wait_ready',
        name='m1_wait_ready',
        arguments=['--timeout', str(READY_TIMEOUT_S)],
        output='screen')

    offboard_node = Node(
        package='milestone1_singlevehicle',
        executable='offboard_solution',
        name='offboard_control_solution',
        parameters=[{
            'waypoints_file': ParameterValue(LaunchConfiguration('path'), value_type=str),
        }],
        output='screen')

    def start_node_when_ready(event, context):
        if event.returncode == 0:
            return [offboard_node]
        return [LogInfo(msg='PX4 never became ready; not starting the offboard node.'),
                EmitEvent(event=Shutdown(reason='PX4 not ready'))]

    return LaunchDescription([
        launch_sitl_arg,
        headless_arg,
        path_arg,
        sitl_launch,
        wait_ready,
        RegisterEventHandler(OnProcessExit(
            target_action=wait_ready, on_exit=start_node_when_ready)),
    ])
