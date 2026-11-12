#!/usr/bin/env python
"""Start up to four PX4/Gazebo x500s and one Micro XRCE-DDS Agent.

Build PX4 first (make px4_sitl). Instance 0 starts the world, followed by
instances 1..N-1 after each preceding instance publishes a valid local position.
Spawn XY and fixed controller origins must match; defaults form a 20 m square.
"""
from itertools import combinations
import math
import os
from pathlib import Path
import subprocess

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, EmitEvent, ExecuteProcess, LogInfo,
    OpaqueFunction, RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from milestone4_multivehicle.vehicle import resolve_vehicle, validate_pair

PACKAGE = 'milestone4_multivehicle'
SITL_PARAMS = {
    'MPC_XY_VEL_MAX': '0.5',
    'MPC_Z_VEL_MAX_UP': '1',
    'MPC_Z_VEL_MAX_DN': '1',
    'MPC_YAWRAUTO_MAX': '45',
}


def start_sitl(context):
    def value(name):
        return LaunchConfiguration(name).perform(context)

    count = int(value('vehicle_count'))
    if count not in (2, 3, 4):
        raise ValueError('vehicle_count must be 2, 3 or 4')
    px4_dir = Path(value('px4_dir')).expanduser().resolve()
    if not os.access(px4_dir / 'build/px4_sitl_default/bin/px4', os.X_OK):
        raise RuntimeError(f'Build PX4 first: cd {px4_dir} && make px4_sitl')
    # Do not join another run's vehicles or reuse an unrelated agent/world.
    for name in ('px4', 'MicroXRCEAgent'):
        if subprocess.run(['pgrep', '-x', name], stdout=subprocess.DEVNULL).returncode == 0:
            raise RuntimeError(f'{name} is already running; use launch_sitl:=false or stop that run')
    if subprocess.run(['pgrep', '-f', r'(^|/| )gz sim( |$)|gz-sim.*server'],
                      stdout=subprocess.DEVNULL).returncode == 0:
        raise RuntimeError('Gazebo is already running; stop that world before starting this run')

    vehicles = [resolve_vehicle(i, value(f'namespace_{i + 1}')) for i in range(count)]
    for first, second in combinations(vehicles, 2):
        validate_pair(first, second)
    origins = [(float(value(f'spawn_x_{i + 1}')), float(value(f'spawn_y_{i + 1}')))
               for i in range(count)]
    if not all(math.isfinite(v) for origin in origins for v in origin):
        raise ValueError('Spawn/origin XY must be finite')
    env = {'PX4_SYS_AUTOSTART': '4001', 'PX4_SIM_MODEL': 'gz_x500',
           'PX4_GZ_WORLD': 'default', 'PX4_GZ_MODEL_NAME': '',
           'PX4_UXRCE_DDS_PORT': '8888',
           'HEADLESS': '1' if value('headless') == 'true' else ''}
    env.update({f'PX4_PARAM_{name}': v for name, v in SITL_PARAMS.items()})
    processes = []
    for i, vehicle in enumerate(vehicles):
        processes.append(Node(
            package=PACKAGE, executable='m4_sitl_process', name=f'm4_px4_{i}',
            arguments=['--px4-dir', str(px4_dir), '--instance', str(i)],
            additional_env={**env,
                            'PX4_UXRCE_DDS_NS': vehicle.namespace.lstrip('/'),
                            'PX4_GZ_MODEL_POSE': f'{origins[i][0]:g},{origins[i][1]:g},0',
                            'PX4_GZ_STANDALONE': '' if i == 0 else '1'},
            output='log'))
    agent = ExecuteProcess(cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'], output='log')
    waiters = [Node(
        package=PACKAGE, executable='m4_wait_ready', name=f'm4_wait_world_{i}',
        parameters=[{'vehicle_instance': i, 'topic_namespace': vehicles[i].namespace or '/'}],
        arguments=['--timeout', '300'], output='screen') for i in range(count - 1)]

    def start_next(index):
        def callback(event, context):
            if context.is_shutdown:
                return []
            if event.returncode == 0:
                following = [processes[index + 1]]
                if index + 1 < len(waiters):
                    following.append(waiters[index + 1])
                return following
            return [LogInfo(msg=f'PX4 {index} not ready; remaining instances will not start.'),
                    EmitEvent(event=Shutdown(reason='SITL startup failed'))]
        return callback

    def process_exited(event, context):
        if context.is_shutdown:
            return []
        return [EmitEvent(event=Shutdown(reason='PX4 or XRCE agent exited'))]

    return [
        *[RegisterEventHandler(OnProcessExit(target_action=w, on_exit=start_next(i)))
          for i, w in enumerate(waiters)],
        *[RegisterEventHandler(OnProcessExit(target_action=p, on_exit=process_exited))
          for p in [agent, *processes]],
        agent, processes[0], waiters[0],
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('px4_dir', default_value=os.path.expanduser('~/PX4-Autopilot')),
        DeclareLaunchArgument('namespace_1', default_value='/'),
        DeclareLaunchArgument('namespace_2', default_value='/px4_1'),
        DeclareLaunchArgument('spawn_x_1', default_value='0.0'),
        DeclareLaunchArgument('spawn_y_1', default_value='0.0'),
        DeclareLaunchArgument('spawn_x_2', default_value='20.0'),
        DeclareLaunchArgument('spawn_y_2', default_value='0.0'),
        DeclareLaunchArgument('vehicle_count', default_value='4', choices=['2', '3', '4']),
        DeclareLaunchArgument('namespace_3', default_value='/px4_2'),
        DeclareLaunchArgument('spawn_x_3', default_value='20.0'),
        DeclareLaunchArgument('spawn_y_3', default_value='20.0'),
        DeclareLaunchArgument('namespace_4', default_value='/px4_3'),
        DeclareLaunchArgument('spawn_x_4', default_value='0.0'),
        DeclareLaunchArgument('spawn_y_4', default_value='20.0'),
        OpaqueFunction(function=start_sitl),
    ])
