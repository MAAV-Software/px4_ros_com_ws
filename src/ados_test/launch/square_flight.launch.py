#!/usr/bin/env python
"""Launch a US-based PX4 SITL drone and fly it in a 50x50 yard square, 5
yards above the ground.

Brings up PX4 SITL (Gazebo) with its home position set to a US location, the
Micro XRCE-DDS Agent, and the ados_test square_flight node, each in its own
terminal - same terminal-launch pattern as week2_px4_sitl/px4_sitl.launch.py
and week3_offboard_control/offboard_solution.launch.py.

The flight path itself lives in resource/waypoints.yaml, not here; this file
only controls where in the world the drone starts.

Usage:
  # SITL/agent not running yet - bring them up too, then the flight node:
  ros2 launch ados_test square_flight.launch.py

  # SITL/agent already running - just the flight node:
  ros2 launch ados_test square_flight.launch.py launch_sitl:=false

  # Fly from a different US location (default is Austin, TX):
  ros2 launch ados_test square_flight.launch.py home_lat:=37.7749 home_lon:=-122.4194 home_alt:=15.0
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# PX4 SITL + Gazebo can take a while to build and boot, especially the first
# time - give it a head start before the flight node begins streaming
# setpoints. If it still isn't armed/flying, PX4 wasn't up in time; just
# rerun this launch file (with launch_sitl:=false, since SITL is up by now).
SITL_STARTUP_DELAY = 20.0

# Default home position: Austin, TX, US.
DEFAULT_HOME_LAT = '30.2672'
DEFAULT_HOME_LON = '-97.7431'
DEFAULT_HOME_ALT = '149.0'


def generate_launch_description():
    launch_sitl_arg = DeclareLaunchArgument(
        'launch_sitl', default_value='true',
        description='Also launch PX4 SITL + the XRCE agent. '
                     'Set to false if they are already running.')
    home_lat_arg = DeclareLaunchArgument('home_lat', default_value=DEFAULT_HOME_LAT,
                                          description='SITL home latitude (deg)')
    home_lon_arg = DeclareLaunchArgument('home_lon', default_value=DEFAULT_HOME_LON,
                                          description='SITL home longitude (deg)')
    home_alt_arg = DeclareLaunchArgument('home_alt', default_value=DEFAULT_HOME_ALT,
                                          description='SITL home altitude (m AMSL)')

    launch_sitl = LaunchConfiguration('launch_sitl')
    home_lat = LaunchConfiguration('home_lat')
    home_lon = LaunchConfiguration('home_lon')
    home_alt = LaunchConfiguration('home_alt')

    px4_dir = os.path.expanduser('~/PX4-Autopilot')

    microxrce_process = ExecuteProcess(
        cmd=['gnome-terminal', '--', 'bash', '-c', 'MicroXRCEAgent udp4 -p 8888; exec bash'],
        shell=False,
        condition=IfCondition(launch_sitl))

    # PX4_HOME_LAT/LON/ALT override the simulated world's origin, which is
    # what puts the drone "in the US" instead of PX4's default (Zurich).
    px4_bash_arg = [
        'cd ', px4_dir,
        ' && PX4_HOME_LAT=', home_lat,
        ' PX4_HOME_LON=', home_lon,
        ' PX4_HOME_ALT=', home_alt,
        ' PX4_SYS_AUTOSTART=4001 make px4_sitl gz_x500; exec bash',
    ]
    px4_process = ExecuteProcess(
        cmd=['gnome-terminal', '--', 'bash', '-c', px4_bash_arg],
        shell=False,
        condition=IfCondition(launch_sitl))

    # We launched SITL ourselves - wait for it to boot before streaming setpoints.
    delayed_flight_node = TimerAction(
        period=SITL_STARTUP_DELAY,
        actions=[
            Node(
                package='ados_test',
                executable='square_flight',
                name='ados_test_square_flight',
                prefix='gnome-terminal --'),
        ],
        condition=IfCondition(launch_sitl))

    # SITL was already running - start immediately, no delay needed.
    immediate_flight_node = Node(
        package='ados_test',
        executable='square_flight',
        name='ados_test_square_flight',
        prefix='gnome-terminal --',
        condition=UnlessCondition(launch_sitl))

    return LaunchDescription([
        launch_sitl_arg,
        home_lat_arg,
        home_lon_arg,
        home_alt_arg,
        microxrce_process,
        px4_process,
        delayed_flight_node,
        immediate_flight_node,
    ])
