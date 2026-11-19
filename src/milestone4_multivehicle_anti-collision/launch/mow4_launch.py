#!/usr/bin/env python
"""Milestone 4 "all four lawnmower" scan for one vehicle.

Like the default scan, but all four drones mow the field instead of three: the nine
north-south rows are split 3/2/2/2 across the drones so the whole 80 by 300 ft field is
covered a little faster. When every drone has reached the end of its share, the checker
(drone 4) stops mowing and re-flies the best path through the ground, found with
probability detect_probability per look. The routes are resource/milestone4_mow4_N.yaml,
made by milestone4_multivehicle/strategies.py. See milestone4_multivehicle/mines.py.

Run it once per vehicle, with topic_prefix and system_id saying which PX4 it flies, and
give every drone the same other arguments.

  ros2 launch milestone4_multivehicle mow4_launch.py system_id:=1
  ros2 launch milestone4_multivehicle mow4_launch.py topic_prefix:=/px4_1 system_id:=2
  ros2 launch milestone4_multivehicle mow4_launch.py topic_prefix:=/px4_2 system_id:=3
  ros2 launch milestone4_multivehicle mow4_launch.py topic_prefix:=/px4_3 system_id:=4
"""
from milestone4_multivehicle.fleet_launch import fleet_launch_description
from milestone4_multivehicle.scan_routes import SPAWNS


def generate_launch_description():
    return fleet_launch_description('milestone4_mow4', SPAWNS, mines=150, search='mow4')
