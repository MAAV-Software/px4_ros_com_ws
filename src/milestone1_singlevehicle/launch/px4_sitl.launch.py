#!/usr/bin/env python
"""Launch the Micro XRCE-DDS Agent and PX4 SITL (Gazebo, x500), each in its own terminal.

Adapted from week2_px4_sitl's px4_sitl.launch.py. Runs Gazebo headless by
default (HEADLESS=1: the simulation runs, but no Gazebo GUI window opens).

Usage:
  ros2 launch milestone1_singlevehicle px4_sitl.launch.py
  ros2 launch milestone1_singlevehicle px4_sitl.launch.py headless:=false

Notes:
 - Assumes PX4-Autopilot is cloned at ~/PX4-Autopilot (adjust px4_dir below if not).
 - Leaves both the PX4 console and the agent's terminal open so you can interact with them directly.
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    headless_arg = DeclareLaunchArgument(
        'headless', default_value='true',
        description='Run Gazebo without its GUI window. Set to false to see the sim.')
    headless = LaunchConfiguration('headless')

    px4_dir = os.path.expanduser('~/PX4-Autopilot')
    px4_cmd = f'cd {px4_dir} && PX4_SYS_AUTOSTART=4001 make px4_sitl gz_x500'
    microxrce_cmd = 'MicroXRCEAgent udp4 -p 8888'

    return LaunchDescription([
        headless_arg,

        # Start the Micro XRCE-DDS Agent in a new terminal
        ExecuteProcess(
            cmd=['gnome-terminal', '--', 'bash', '-c', f"{microxrce_cmd}; exec bash"],
            shell=False
        ),

        # Start PX4 SITL + Gazebo in a new terminal — headless (PX4's
        # px4-rc.gzsim skips the Gazebo GUI when HEADLESS is set)...
        ExecuteProcess(
            cmd=['gnome-terminal', '--', 'bash', '-c', f"HEADLESS=1 {px4_cmd}; exec bash"],
            shell=False,
            condition=IfCondition(headless)
        ),

        # ...or with the Gazebo GUI
        ExecuteProcess(
            cmd=['gnome-terminal', '--', 'bash', '-c', f"{px4_cmd}; exec bash"],
            shell=False,
            condition=UnlessCondition(headless)
        ),
    ])
