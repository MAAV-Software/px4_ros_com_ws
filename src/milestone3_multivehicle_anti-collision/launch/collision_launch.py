#!/usr/bin/env python
"""Milestone 3's collision test for one vehicle: two offset-pass routes that exercise the 5 m yield rule.

Run it once per vehicle, with topic_prefix and system_id saying which PX4 it flies:

- system_id 1 is vehicle 0. It hovers at its origin for 35 s and yields: it brakes,
  climbs 5 m and waits while vehicle 1 is within 5 m.
- system_id 2 is vehicle 1, which has priority. It starts 20 m east and flies west,
  passing 3 m north of vehicle 0.

Each vehicle takes its own route (path_1 or path_2) and its own world origin (spawn_x/y_1
or spawn_x/y_2), so both drones can be given the same file and the same arguments. The
web launcher does that for every drone ticked on the phone, adding each drone's
topic_prefix and system_id, and sends Go once both have passed their start checks.

It starts no simulator, so it runs unchanged in SITL and on real drones. The two
controllers trade positions on /milestone3/global_positions, so their ROS graphs have to
see each other. In SITL they share one machine. The spawn origins match the web
launcher's SITL drones (and px4_sitl.launch.py): the second 20 m east of the first.

  ros2 launch milestone3_multivehicle collision_launch.py system_id:=1
  ros2 launch milestone3_multivehicle collision_launch.py topic_prefix:=/px4_1 system_id:=2
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

from milestone3_multivehicle.vehicle import resolve_vehicle

PACKAGE = 'milestone3_multivehicle'
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
    if system_id not in (1, 2):
        raise ValueError('the collision test flies two vehicles: system_id must be 1 or 2')
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
        'topic_namespace': vehicle.namespace or '/',
        'target_system_id': vehicle.system_id,
    }
    waiter = Node(
        package=PACKAGE, executable='m3_wait_ready', name=f'm3_wait_ready_{instance}',
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
        return PathJoinSubstitution([FindPackageShare(PACKAGE), 'resource', f'milestone3_collision_{instance}.yaml'])

    return LaunchDescription([
        DeclareLaunchArgument('topic_prefix', default_value='',
                              description="Prefix for this vehicle's PX4 topics, e.g. /px4_1; "
                                          'empty for the bare /fmu topics'),
        DeclareLaunchArgument('system_id', default_value='1',
                              description="This vehicle's MAV_SYS_ID: 1 hovers and yields, 2 passes"),
        DeclareLaunchArgument('path_1', default_value=route(0), description='Route for system_id 1 (vehicle 0)'),
        DeclareLaunchArgument('path_2', default_value=route(1), description='Route for system_id 2 (vehicle 1)'),
        DeclareLaunchArgument('spawn_x_1', default_value='0.0',
                              description='World east of vehicle 0, where its EKF origin is (m)'),
        DeclareLaunchArgument('spawn_y_1', default_value='0.0', description='World north of vehicle 0 (m)'),
        DeclareLaunchArgument('spawn_x_2', default_value='20.0',
                              description='World east of vehicle 1, where its EKF origin is (m)'),
        DeclareLaunchArgument('spawn_y_2', default_value='0.0', description='World north of vehicle 1 (m)'),
        OpaqueFunction(function=launch_vehicle),
    ])
