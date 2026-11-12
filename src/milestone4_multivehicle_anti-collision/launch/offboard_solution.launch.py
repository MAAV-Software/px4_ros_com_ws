#!/usr/bin/env python
"""Launch up to four waypoint controllers with priority-based anti-collision after PX4 readiness.

Optional SITL starts up to four x500s and one agent. With launch_sitl:=false, use
already-running SITL or an existing configured fleet. path selects a common
local-NED route; path_1 through path_4 can override it independently.
"""
from itertools import combinations
import math
import os

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription,
    LogInfo, OpaqueFunction, RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from milestone4_multivehicle.vehicle import resolve_vehicle, validate_pair

PACKAGE = 'milestone4_multivehicle'
READY_TIMEOUT_S = 300


def launch_fleet(context):
    def value(name):
        return LaunchConfiguration(name).perform(context)

    count = int(value('vehicle_count'))
    if count not in (2, 3, 4):
        raise ValueError('vehicle_count must be 2, 3 or 4')
    launch_sitl = value('launch_sitl').lower() in ('true', '1')
    vehicles = [resolve_vehicle(i, value(f'namespace_{i + 1}'),
                                int(value(f'system_id_{i + 1}'))) for i in range(count)]
    for first, second in combinations(vehicles, 2):
        validate_pair(first, second)
    if launch_sitl and [v.system_id for v in vehicles] != list(range(1, count + 1)):
        raise ValueError('Built-in SITL requires system_id_N:=N for every vehicle')
    # Check ALL paths before any controller can arm either vehicle.
    paths = [os.path.abspath(os.path.expanduser(value(f'path_{i + 1}')))
             for i in range(count)]
    for path in paths:
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

    origins = [(float(value(f'spawn_x_{i + 1}')), float(value(f'spawn_y_{i + 1}')))
               for i in range(count)]
    if not all(math.isfinite(v) for origin in origins for v in origin):
        raise ValueError('Spawn/origin XY must be finite')
    actions = []
    if launch_sitl:
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory(PACKAGE), 'px4_sitl.launch.py')),
            launch_arguments={
                'headless': value('headless'), 'px4_dir': value('px4_dir'),
                'vehicle_count': str(count),
                **{f'namespace_{i + 1}': v.namespace or '/' for i, v in enumerate(vehicles)},
                **{f'spawn_{axis}_{i + 1}': value(f'spawn_{axis}_{i + 1}')
                   for i in range(count) for axis in ('x', 'y')},
            }.items()))

    controllers, waiters = [], []
    for i, vehicle in enumerate(vehicles):
        params = {
            'vehicle_instance': i,
            'vehicle_count': count,
            'topic_namespace': vehicle.namespace or '/',
            'target_system_id': vehicle.system_id,
        }
        waiters.append(Node(
            package=PACKAGE, executable='m4_wait_ready', name=f'm4_wait_ready_{i}',
            parameters=[params], arguments=['--timeout', str(READY_TIMEOUT_S)], output='screen'))
        controllers.append(Node(
            package=PACKAGE, executable='offboard_solution', name=f'offboard_control_{i}',
            parameters=[{**params, 'waypoints_file': paths[i],
                         'initial_global_x': origins[i][0],
                         'initial_global_y': origins[i][1]}], output='screen'))

    ready = set()
    failed = False

    def when_ready(index):
        def callback(event, context):
            nonlocal failed
            if context.is_shutdown or failed:
                return []
            if event.returncode != 0:
                failed = True
                return [LogInfo(msg=f'Vehicle {index} not ready; no mission will start.'),
                        EmitEvent(event=Shutdown(reason='PX4 readiness failed'))]
            ready.add(index)
            if len(ready) == count:
                return [LogInfo(msg='All vehicles ready; starting missions with priority-based anti-collision.'),
                        *controllers]
            return []
        return callback

    # Register handlers before starting processes, including already-ready hardware.
    actions.extend(RegisterEventHandler(OnProcessExit(
        target_action=waiter, on_exit=when_ready(i))) for i, waiter in enumerate(waiters))
    actions.extend(waiters)
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('launch_sitl', default_value='true', choices=['true', 'false'],
                              description='Start the PX4 fleet and the agent; false for existing SITL/hardware'),
        DeclareLaunchArgument('headless', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('px4_dir', default_value=os.path.expanduser('~/PX4-Autopilot')),
        DeclareLaunchArgument('path', default_value=PathJoinSubstitution([
            FindPackageShare(PACKAGE), 'resource', 'milestone4_path.yaml'])),
        DeclareLaunchArgument('path_1', default_value=LaunchConfiguration('path')),
        DeclareLaunchArgument('path_2', default_value=LaunchConfiguration('path')),
        DeclareLaunchArgument('namespace_1', default_value='/'),
        DeclareLaunchArgument('namespace_2', default_value='/px4_1'),
        DeclareLaunchArgument('system_id_1', default_value='1'),
        DeclareLaunchArgument('system_id_2', default_value='2'),
        DeclareLaunchArgument('spawn_x_1', default_value='0.0'),
        DeclareLaunchArgument('spawn_y_1', default_value='0.0'),
        DeclareLaunchArgument('spawn_x_2', default_value='20.0'),
        DeclareLaunchArgument('spawn_y_2', default_value='0.0'),
        DeclareLaunchArgument('vehicle_count', default_value='4', choices=['2', '3', '4']),
        DeclareLaunchArgument('namespace_3', default_value='/px4_2'),
        DeclareLaunchArgument('spawn_x_3', default_value='20.0'),
        DeclareLaunchArgument('spawn_y_3', default_value='20.0'),
        DeclareLaunchArgument('path_3', default_value=LaunchConfiguration('path')),
        DeclareLaunchArgument('system_id_3', default_value='3'),
        DeclareLaunchArgument('namespace_4', default_value='/px4_3'),
        DeclareLaunchArgument('spawn_x_4', default_value='0.0'),
        DeclareLaunchArgument('spawn_y_4', default_value='20.0'),
        DeclareLaunchArgument('path_4', default_value=LaunchConfiguration('path')),
        DeclareLaunchArgument('system_id_4', default_value='4'),
        OpaqueFunction(function=launch_fleet),
    ])
