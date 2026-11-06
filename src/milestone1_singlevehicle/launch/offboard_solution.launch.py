#!/usr/bin/env python
"""Launch the milestone 1 offboard control node, optionally bringing up PX4
SITL + the Micro XRCE-DDS Agent (px4_sitl.launch.py) first. Gazebo runs
headless (no GUI window) unless headless:=false.

The node is started only once PX4 reports a valid local position estimate
(m1_wait_ready), not after a fixed delay, so it works the same whether SITL was
just started, is already running, or is a real drone. The node itself also
refuses to arm until the drone is within max_start_offset_m of the first
waypoint (see documentation/hitl-runbook.md).

Usage:
  # SITL/agent not running yet — bring them up too, then the solution node:
  ros2 launch milestone1_singlevehicle offboard_solution.launch.py

  # SITL/agent already running (or a real drone over the XRCE agent) — just
  # the solution node, so you don't spin up a second conflicting instance:
  ros2 launch milestone1_singlevehicle offboard_solution.launch.py launch_sitl:=false

  # Real drone whose PX4 uses a namespace (e.g. fleet drone1 -> px4_1):
  ros2 launch milestone1_singlevehicle offboard_solution.launch.py launch_sitl:=false topic_prefix:=/px4_1

  # Real drone, 10 s warning before it arms, no start-position check:
  ros2 launch milestone1_singlevehicle offboard_solution.launch.py launch_sitl:=false arm_delay_s:=10 max_start_offset_m:=-1

  # Show the Gazebo GUI instead of running headless:
  ros2 launch milestone1_singlevehicle offboard_solution.launch.py headless:=false

  # Use a different waypoint YAML (yaw in radians):
  ros2 launch milestone1_singlevehicle offboard_solution.launch.py path:=/path/to/waypoints.yaml
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, EmitEvent, ExecuteProcess, IncludeLaunchDescription,
    LogInfo, RegisterEventHandler)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution, PythonExpression)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

# How long to wait for PX4 to become ready. The first start after a build
# (PX4 + Gazebo compile) can take minutes.
READY_TIMEOUT_S = 300

# Topics recorded when record:=true (names under <topic_prefix>/fmu/). Enough to
# reconstruct a flight afterwards: what the node commanded, what PX4 estimated,
# and the state of the arming/failsafe/RC/battery inputs.
RECORD_TOPICS = [
    'in/offboard_control_mode', 'in/trajectory_setpoint', 'in/vehicle_command',
    'out/vehicle_command_ack_v1', 'out/vehicle_local_position_v1', 'out/vehicle_attitude',
    'out/vehicle_odometry', 'out/vehicle_global_position', 'out/vehicle_gps_position',
    'out/vehicle_control_mode', 'out/vehicle_status_v4', 'out/vehicle_land_detected',
    'out/failsafe_flags', 'out/estimator_status', 'out/estimator_status_flags',
    'out/estimator_gps_status', 'out/battery_status_v1', 'out/input_rc',
    'out/manual_control_setpoint', 'out/sensor_combined',
]


def generate_launch_description():
    launch_sitl_arg = DeclareLaunchArgument(
        'launch_sitl', default_value='true',
        description='Also launch PX4 SITL + the XRCE agent (px4_sitl.launch.py). '
                     'Set to false if they are already running.')

    headless_arg = DeclareLaunchArgument(
        'headless', default_value='true',
        description='Run Gazebo without its GUI window. Only used when launch_sitl is true.')

    topic_prefix_arg = DeclareLaunchArgument(
        'topic_prefix', default_value='',
        description='Prefix for the PX4 topics, e.g. /px4_1 for /px4_1/fmu/... '
                    '(empty for the bare /fmu/... topics of instance 0 / SITL).')

    arm_delay_arg = DeclareLaunchArgument(
        'arm_delay_s', default_value='0.0',
        description='Extra seconds to wait before the node first tries to arm (warning time on a real drone).')

    max_offset_arg = DeclareLaunchArgument(
        'max_start_offset_m', default_value='2.0',
        description='Refuse to arm if the drone is further than this (x/y metres) from the first '
                    'waypoint. Negative disables the check.')

    relative_start_arg = DeclareLaunchArgument(
        'relative_start', default_value='false',
        description='Fly the path relative to where the drone is when it arms (a "-5 m" climb is 5 m '
                    'above the start), instead of to absolute EKF-origin coordinates. Skips the '
                    'max_start_offset_m check.')

    record_arg = DeclareLaunchArgument(
        'record', default_value='false',
        description='Record the flight to a rosbag (see RECORD_TOPICS) from before PX4-ready until '
                    'the launch ends.')

    bag_dir_arg = DeclareLaunchArgument(
        'bag_dir', default_value=PathJoinSubstitution([EnvironmentVariable('HOME'), 'flight_bags']),
        description='Directory the rosbag is written into (one timestamped folder per launch). '
                    'Only used when record is true.')

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

    # A small write cache (bytes) keeps the bag current on disk, so most of it survives a
    # power cut (the default cache would hold ~100 MB in RAM and lose it).
    recorder = ExecuteProcess(
        cmd=['ros2', 'bag', 'record',
             '-o', [LaunchConfiguration('bag_dir'), '/flight_',
                    PythonExpression(["__import__('time').strftime('%Y%m%d_%H%M%S')"])],
             '--storage', 'sqlite3', '--max-cache-size', '1000000',
             '--topics'] + [[LaunchConfiguration('topic_prefix'), '/fmu/' + t] for t in RECORD_TOPICS],
        output='log',
        condition=IfCondition(LaunchConfiguration('record')))

    # Exits 0 once PX4 is ready (immediately, if it already is), 1 on timeout.
    wait_ready = Node(
        package='milestone1_singlevehicle',
        executable='m1_wait_ready',
        name='m1_wait_ready',
        arguments=['--timeout', str(READY_TIMEOUT_S),
                   # One '=' argument so an empty prefix can't swallow or drop a value.
                   ['--topic-prefix=', LaunchConfiguration('topic_prefix')]],
        output='screen')

    offboard_node = Node(
        package='milestone1_singlevehicle',
        executable='offboard_solution',
        name='offboard_control_solution',
        parameters=[{
            'waypoints_file': ParameterValue(LaunchConfiguration('path'), value_type=str),
            'topic_prefix': ParameterValue(LaunchConfiguration('topic_prefix'), value_type=str),
            'arm_delay_s': ParameterValue(LaunchConfiguration('arm_delay_s'), value_type=float),
            'max_start_offset_m': ParameterValue(LaunchConfiguration('max_start_offset_m'), value_type=float),
            'relative_start': ParameterValue(LaunchConfiguration('relative_start'), value_type=bool),
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
        topic_prefix_arg,
        arm_delay_arg,
        max_offset_arg,
        relative_start_arg,
        record_arg,
        bag_dir_arg,
        path_arg,
        sitl_launch,
        recorder,
        wait_ready,
        RegisterEventHandler(OnProcessExit(
            target_action=wait_ready, on_exit=start_node_when_ready)),
    ])
