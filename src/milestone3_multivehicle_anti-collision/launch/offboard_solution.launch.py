#!/usr/bin/env python
"""Launch two waypoint controllers with priority-based anti-collision after PX4 readiness.

Optional SITL starts two x500s and one agent. With launch_sitl:=false, use
already-running SITL or two configured airframes. path selects a common
local-NED route; path_1 and path_2 can override it independently.
"""
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

from milestone3_multivehicle.vehicle import resolve_vehicle, validate_pair

PACKAGE = 'milestone3_multivehicle'
READY_TIMEOUT_S = 300


def launch_pair(context):
    def value(name):
        return LaunchConfiguration(name).perform(context)

    launch_sitl = value('launch_sitl').lower() in ('true', '1')
    vehicles = [resolve_vehicle(i, value(f'namespace_{i + 1}'),
                                int(value(f'system_id_{i + 1}'))) for i in range(2)]
    validate_pair(*vehicles)
    if launch_sitl and [v.system_id for v in vehicles] != [1, 2]:
        raise ValueError('Built-in SITL uses system_id_1:=1 and system_id_2:=2')
    # Check BOTH paths before any controller can arm either vehicle.
    paths = [os.path.abspath(os.path.expanduser(value(f'path_{i + 1}')))
             for i in range(2)]
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
               for i in range(2)]
    if not all(math.isfinite(v) for origin in origins for v in origin):
        raise ValueError('Spawn/origin XY must be finite')
    actions = []
    if launch_sitl:
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory(PACKAGE), 'px4_sitl.launch.py')),
            launch_arguments={
                'headless': value('headless'), 'px4_dir': value('px4_dir'),
                'namespace_1': vehicles[0].namespace or '/',
                'namespace_2': vehicles[1].namespace or '/',
                **{f'spawn_{axis}_{i + 1}': value(f'spawn_{axis}_{i + 1}')
                   for i in range(2) for axis in ('x', 'y')},
            }.items()))

    controllers, waiters = [], []
    for i, vehicle in enumerate(vehicles):
        params = {
            'vehicle_instance': i,
            'topic_namespace': vehicle.namespace or '/',
            'target_system_id': vehicle.system_id,
        }
        waiters.append(Node(
            package=PACKAGE, executable='m3_wait_ready', name=f'm3_wait_ready_{i}',
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
                return [LogInfo(msg=f'Vehicle {index} not ready; neither mission will start.'),
                        EmitEvent(event=Shutdown(reason='PX4 readiness failed'))]
            ready.add(index)
            if len(ready) == 2:
                return [LogInfo(msg='Both vehicles ready; starting missions with priority-based anti-collision.'),
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
                              description='Start two PX4s and the agent; false for existing SITL/hardware'),
        DeclareLaunchArgument('headless', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('px4_dir', default_value=os.path.expanduser('~/PX4-Autopilot')),
        DeclareLaunchArgument('path', default_value=PathJoinSubstitution([
            FindPackageShare(PACKAGE), 'resource', 'milestone3_path.yaml'])),
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
        OpaqueFunction(function=launch_pair),
    ])
