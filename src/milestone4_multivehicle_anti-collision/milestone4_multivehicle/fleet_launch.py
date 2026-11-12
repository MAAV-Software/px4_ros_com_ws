"""Shared by milestone 4's launch files: one vehicle of a four-drone fleet per launch.

Each launch file names its routes and the world origins they were planned from; this
checks the arguments, waits for that PX4, then starts its controller.
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

from milestone4_multivehicle.vehicle import resolve_vehicle

PACKAGE = 'milestone4_multivehicle'
READY_TIMEOUT_S = 300
MAX_VEHICLES = 4


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

    count = int(value('vehicle_count'))
    if count not in (2, 3, 4):
        raise ValueError('vehicle_count must be 2, 3 or 4')
    system_id = int(value('system_id'))
    if not 1 <= system_id <= count:
        raise ValueError(f'the collision test flies {count} vehicles: system_id must be 1 to {count}')
    instance = system_id - 1
    # An empty prefix means the bare /fmu topics of instance 0 / a single drone.
    vehicle = resolve_vehicle(instance, value('topic_prefix') or '/', system_id)
    path = os.path.abspath(os.path.expanduser(value(f'path_{instance + 1}')))
    check_waypoints(path)
    origin = (float(value(f'spawn_x_{instance + 1}')), float(value(f'spawn_y_{instance + 1}')))
    if not all(math.isfinite(v) for v in origin):
        raise ValueError('spawn x and y must be finite')

    policy = value('avoidance')
    if policy not in ('orca', 'priority'):
        raise ValueError("avoidance must be 'orca' or 'priority'")
    max_speed = float(value('max_speed'))
    if not (math.isfinite(max_speed) and max_speed > 0):
        raise ValueError('max_speed must be a positive number of m/s')

    params = {
        'vehicle_instance': instance,
        'vehicle_count': count,
        'topic_namespace': vehicle.namespace or '/',
        'target_system_id': vehicle.system_id,
    }
    waiter = Node(
        package=PACKAGE, executable='m4_wait_ready', name=f'm4_wait_ready_{instance}',
        parameters=[params], arguments=['--timeout', str(READY_TIMEOUT_S)], output='screen')
    controller = Node(
        package=PACKAGE, executable='offboard_solution', name=f'offboard_control_{instance}',
        parameters=[{**params, 'waypoints_file': path,
                     'initial_global_x': origin[0], 'initial_global_y': origin[1],
                     'avoidance': policy, 'max_speed': max_speed}],
        output='screen')

    def when_ready(event, context):
        if context.is_shutdown:
            return []
        if event.returncode != 0:
            return [LogInfo(msg='PX4 not ready; the mission will not start.'),
                    EmitEvent(event=Shutdown(reason='PX4 readiness failed'))]
        return [LogInfo(msg=f'PX4 ready; starting vehicle {instance} with {policy} anti-collision.'),
                controller]

    # Registered before the waiter starts, so a PX4 that is already ready isn't missed.
    return [RegisterEventHandler(OnProcessExit(target_action=waiter, on_exit=when_ready)), waiter]


def fleet_launch_description(route_prefix, origins):
    """One vehicle of a fleet flying resource/<route_prefix>_N.yaml from world origins[N]."""
    def route(instance):
        return PathJoinSubstitution([FindPackageShare(PACKAGE), 'resource', f'{route_prefix}_{instance}.yaml'])

    per_vehicle = []
    for i, (east, north) in enumerate(origins):
        per_vehicle += [
            DeclareLaunchArgument(f'path_{i + 1}', default_value=route(i),
                                  description=f'Route for system_id {i + 1} (vehicle {i})'),
            DeclareLaunchArgument(f'spawn_x_{i + 1}', default_value=str(east),
                                  description=f'World east of vehicle {i}, where its EKF origin is (m)'),
            DeclareLaunchArgument(f'spawn_y_{i + 1}', default_value=str(north),
                                  description=f'World north of vehicle {i} (m)'),
        ]
    return LaunchDescription([
        DeclareLaunchArgument('topic_prefix', default_value='',
                              description="Prefix for this vehicle's PX4 topics, e.g. /px4_1; "
                                          'empty for the bare /fmu topics'),
        DeclareLaunchArgument('system_id', default_value='1',
                              description="This vehicle's MAV_SYS_ID; with avoidance:=priority, higher ones have priority"),
        DeclareLaunchArgument('avoidance', default_value='orca', choices=['orca', 'priority'],
                              description='orca: every drone sidesteps; priority: lower drones brake, '
                                          'climb 5 m and wait. Give every drone the same value'),
        DeclareLaunchArgument('max_speed', default_value='1.0',
                              description="ORCA's top horizontal speed (m/s); match the drones' "
                                          'MPC_XY_VEL_MAX'),
        DeclareLaunchArgument('vehicle_count', default_value=str(MAX_VEHICLES), choices=['2', '3', '4'],
                              description='Drones in the test; give every drone the same value'),
        *per_vehicle,
        OpaqueFunction(function=launch_vehicle),
    ])
