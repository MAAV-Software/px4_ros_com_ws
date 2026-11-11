#!/usr/bin/env python
"""Fly one vehicle's milestone 2 route: wait until its PX4 is ready, then start its controller.

The same file runs in SITL and on real drones, and starts no simulator. In SITL, the
web launcher's Fleet tab starts the PX4 instances (or run px4_sitl.launch.py by hand).
On a real drone, the XRCE-DDS link has to be up first (see the HITL runbook).

For several vehicles, run it once per vehicle, each with its own topic_prefix and
system_id. The web launcher does that for every drone ticked on the phone, and only
sends Go once every one of them has passed its start checks.

  ros2 launch milestone2_multivehicle offboard_solution.launch.py
  ros2 launch milestone2_multivehicle offboard_solution.launch.py topic_prefix:=/px4_1 system_id:=2
"""
import math
import os

import yaml

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, EmitEvent, LogInfo, OpaqueFunction, RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from milestone2_multivehicle.vehicle import resolve_vehicle

PACKAGE = 'milestone2_multivehicle'
READY_TIMEOUT_S = 300


def check_waypoints(path):
    """Refuse a bad file before anything can arm."""
    with open(path) as stream:
        data = yaml.safe_load(stream)
    points = data.get('waypoints') if isinstance(data, dict) else None
    if not isinstance(points, list) or not points:
        raise ValueError(f'{path}: waypoints must be a non-empty list')
    for point in points:
        if not isinstance(point, dict) or not {'pos', 'yaw', 'hold_s'} <= point.keys():
            raise ValueError(f'{path}: each waypoint requires pos, yaw and hold_s')
        pos = tuple(float(v) for v in point['pos'])
        yaw, hold = float(point['yaw']), float(point['hold_s'])
        if len(pos) != 3 or not all(math.isfinite(v) for v in (*pos, yaw, hold)) or hold < 0:
            raise ValueError(f'{path}: expected three finite coordinates, finite yaw and nonnegative hold_s')


def launch_vehicle(context):
    def value(name):
        return LaunchConfiguration(name).perform(context)

    system_id = int(value('system_id'))
    if not 1 <= system_id <= 255:
        raise ValueError('system_id must be the vehicle MAV_SYS_ID, from 1 to 255')
    # An empty prefix means the bare /fmu topics of instance 0 / a single drone.
    vehicle = resolve_vehicle(system_id - 1, value('topic_prefix') or '/', system_id)
    path = os.path.abspath(os.path.expanduser(value('path')))
    check_waypoints(path)

    params = {
        'vehicle_instance': system_id - 1,
        'topic_namespace': vehicle.namespace or '/',
        'target_system_id': vehicle.system_id,
    }
    waiter = Node(
        package=PACKAGE, executable='m2_wait_ready', name=f'm2_wait_ready_{system_id}',
        parameters=[params], arguments=['--timeout', str(READY_TIMEOUT_S)], output='screen')
    controller = Node(
        package=PACKAGE, executable='offboard_solution', name=f'offboard_control_{system_id}',
        parameters=[{**params, 'waypoints_file': path}], output='screen')

    def when_ready(event, context):
        if context.is_shutdown:
            return []
        if event.returncode != 0:
            return [LogInfo(msg='PX4 not ready; the mission will not start.'),
                    EmitEvent(event=Shutdown(reason='PX4 readiness failed'))]
        return [LogInfo(msg='PX4 ready; starting the mission.'), controller]

    # Registered before the waiter starts, so a PX4 that is already ready isn't missed.
    return [RegisterEventHandler(OnProcessExit(target_action=waiter, on_exit=when_ready)), waiter]


def declare_vehicle_arguments(default_path):
    """The arguments every milestone 2 launch file takes, so the wrappers match this one."""
    return [
        DeclareLaunchArgument('path', default_value=default_path,
                              description='Waypoint YAML file (yaw in radians)'),
        DeclareLaunchArgument('topic_prefix', default_value='',
                              description='Prefix for this vehicle\'s PX4 topics, e.g. /px4_1; '
                                          'empty for the bare /fmu topics'),
        DeclareLaunchArgument('system_id', default_value='1',
                              description='This vehicle\'s MAV_SYS_ID, which its commands are sent to'),
    ]


def generate_launch_description():
    return LaunchDescription([
        *declare_vehicle_arguments(PathJoinSubstitution([
            FindPackageShare(PACKAGE), 'resource', 'milestone2_path.yaml'])),
        OpaqueFunction(function=launch_vehicle),
    ])
