#!/usr/bin/env python
"""Milestone 4's collision test for one vehicle: four diagonal head-on routes that converge at (10, 10).

Run it once per vehicle, with topic_prefix and system_id saying which PX4 it flies.
system_id N is vehicle N-1. The higher the instance the higher the priority: vehicle 3
never yields, and each lower vehicle brakes, climbs 5 m and waits while a higher one
is within 5 m.

Each vehicle takes its own route (path_N) and its own world origin (spawn_x_N,
spawn_y_N), so every drone can be given the same file and the same arguments. The web
launcher does that for every drone ticked on the phone, adding each drone's
topic_prefix and system_id, and sends Go once all have passed their start checks.

It starts no simulator, so it runs unchanged in SITL and on real drones. The
controllers trade positions on /milestone4/global_positions, so their ROS graphs have
to see each other. The default origins are the 20 m square px4_sitl.launch.py spawns:
(0,0), (20,0), (20,20), (0,20), world east/north. The origins must be where each
drone really started, or the routes will not converge.

  ros2 launch milestone4_multivehicle collision_launch.py system_id:=1
  ros2 launch milestone4_multivehicle collision_launch.py topic_prefix:=/px4_1 system_id:=2
  ros2 launch milestone4_multivehicle collision_launch.py topic_prefix:=/px4_2 system_id:=3
  ros2 launch milestone4_multivehicle collision_launch.py topic_prefix:=/px4_3 system_id:=4
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
DEFAULT_ORIGINS = ((0.0, 0.0), (20.0, 0.0), (20.0, 20.0), (0.0, 20.0))


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
                     'initial_global_x': origin[0], 'initial_global_y': origin[1]}],
        output='screen')

    def when_ready(event, context):
        if context.is_shutdown:
            return []
        if event.returncode != 0:
            return [LogInfo(msg='PX4 not ready; the mission will not start.'),
                    EmitEvent(event=Shutdown(reason='PX4 readiness failed'))]
        return [LogInfo(msg=f'PX4 ready; starting vehicle {instance} with priority-based anti-collision.'),
                controller]

    # Registered before the waiter starts, so a PX4 that is already ready isn't missed.
    return [RegisterEventHandler(OnProcessExit(target_action=waiter, on_exit=when_ready)), waiter]


def generate_launch_description():
    def route(instance):
        return PathJoinSubstitution([FindPackageShare(PACKAGE), 'resource', f'milestone4_headon_{instance}.yaml'])

    per_vehicle = []
    for i, (east, north) in enumerate(DEFAULT_ORIGINS):
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
                              description="This vehicle's MAV_SYS_ID; higher ones have priority"),
        DeclareLaunchArgument('vehicle_count', default_value=str(MAX_VEHICLES), choices=['2', '3', '4'],
                              description='Drones in the test; give every drone the same value'),
        *per_vehicle,
        OpaqueFunction(function=launch_vehicle),
    ])
