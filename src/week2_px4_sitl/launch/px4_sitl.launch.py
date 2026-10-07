#!/usr/bin/env python
"""Launch the Micro XRCE-DDS Agent and PX4 SITL (Gazebo), each in its own terminal.

Automated version of week 2 tasks 1 and 2. Do the equivalent commands by hand
first (see week-2-px4-sitl-gazebo.md) so you understand what this is doing —
this is the same terminal-launch pattern used in
archive/src/px4_multi_vehicle_offboard/px4_single_plan/launch/px4_sitl_with_model.launch.py.

Usage:
  ros2 launch week2_px4_sitl px4_sitl.launch.py

Notes:
 - Assumes PX4-Autopilot is cloned at ~/PX4-Autopilot (adjust px4_dir below if not).
 - Leaves both the PX4 console and the agent's terminal open so you can interact with them directly.
"""
from launch import LaunchDescription
from launch.actions import ExecuteProcess
import os


def generate_launch_description():
    px4_dir = os.path.expanduser('~/PX4-Autopilot')
    px4_cmd = f'cd {px4_dir} && PX4_SYS_AUTOSTART=4001 make px4_sitl gz_x500'
    microxrce_cmd = 'MicroXRCEAgent udp4 -p 8888'

    return LaunchDescription([
        # Start the Micro XRCE-DDS Agent in a new terminal (task 2)
        ExecuteProcess(
            cmd=['gnome-terminal', '--', 'bash', '-c', f"{microxrce_cmd}; exec bash"],
            shell=False
        ),

        # Start PX4 SITL + Gazebo in a new terminal (task 1)
        ExecuteProcess(
            cmd=['gnome-terminal', '--', 'bash', '-c', f"{px4_cmd}; exec bash"],
            shell=False
        ),
    ])
