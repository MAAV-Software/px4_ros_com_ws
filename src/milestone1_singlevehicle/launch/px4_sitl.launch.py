#!/usr/bin/env python
"""Launch the Micro XRCE-DDS Agent and PX4 SITL (Gazebo, x500).

Adapted from week2_px4_sitl's px4_sitl.launch.py. Both run as ordinary launch
processes (no extra terminal windows, so it also works over SSH); their
output goes to the launch log directory (~/.ros/log/latest). Stopping the
launch (Ctrl-C) also stops PX4 and Gazebo.

Runs Gazebo headless by default (HEADLESS=1: the simulation runs, but no
Gazebo GUI window opens).

PX4 is started with the parameters agreed for the Milestone 1 tests (see the
"Decisions to make before Session 1" table in milestone-1-single-vehicle.md),
so every run is flown with the same limits. PX4 saves parameters in SITL, so
these persist between runs.

Open QGroundControl first: with PX4's default NAV_DLL_ACT it refuses to arm
with "No connection to the GCS". (Don't set NAV_DLL_ACT here: it would be
saved too and silently change later runs.)

Usage:
  ros2 launch milestone1_singlevehicle px4_sitl.launch.py
  ros2 launch milestone1_singlevehicle px4_sitl.launch.py headless:=false

Notes:
 - Assumes PX4-Autopilot is cloned at ~/PX4-Autopilot (adjust px4_dir below if not).
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnShutdown
from launch.substitutions import LaunchConfiguration

# Applied through PX4's PX4_PARAM_* start-up mechanism.
SITL_PARAMS = {
    'MPC_XY_VEL_MAX': '2',       # m/s     (PX4 default 12)
    'MPC_Z_VEL_MAX_UP': '1',     # m/s     (PX4 default 3)
    'MPC_Z_VEL_MAX_DN': '1',     # m/s     (PX4 default 1.5)
    'MPC_YAWRAUTO_MAX': '45',    # deg/s   (PX4 default 60)
}


def generate_launch_description():
    headless_arg = DeclareLaunchArgument(
        'headless', default_value='true',
        description='Run Gazebo without its GUI window. Set to false to see the sim.')
    headless = LaunchConfiguration('headless')

    px4_dir = os.path.expanduser('~/PX4-Autopilot')
    env = {'PX4_SYS_AUTOSTART': '4001'}
    env.update({f'PX4_PARAM_{name}': value for name, value in SITL_PARAMS.items()})

    # PX4's interactive shell spins on end-of-input, so give it a stdin that
    # never ends (the launch system doesn't have a terminal to give it).
    px4_cmd = ['bash', '-c', 'tail -f /dev/null | make px4_sitl gz_x500']

    def px4_process(extra_env, condition):
        return ExecuteProcess(
            cmd=px4_cmd, cwd=px4_dir, additional_env={**env, **extra_env},
            output='log', shell=False, condition=condition)

    # The tail/make pipeline, Gazebo and the agent can outlive the launch's own SIGINT.
    cleanup = RegisterEventHandler(OnShutdown(on_shutdown=[
        ExecuteProcess(cmd=['pkill', '-f', 'build/px4_sitl_default/bin/px4']),
        ExecuteProcess(cmd=['pkill', '-f', 'gz sim']),
        ExecuteProcess(cmd=['pkill', '-x', 'MicroXRCEAgent']),
    ]))

    return LaunchDescription([
        headless_arg,
        ExecuteProcess(cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'], output='log'),
        px4_process({'HEADLESS': '1'}, IfCondition(headless)),
        px4_process({}, UnlessCondition(headless)),
        cleanup,
    ])
