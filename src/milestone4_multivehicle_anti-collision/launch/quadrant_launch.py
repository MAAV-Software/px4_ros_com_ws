#!/usr/bin/env python
"""Milestone 4 "quadrants" scan for one vehicle: each drone scans one quarter of the field.

The field is quartered west/east and south/north. Drone 1 mows the south-west quadrant,
drone 2 the south-east, drone 3 the north-west and drone 4 the north-east, so no drone
flies the field's full length. The two south drones land south by the spawns and the two
north drones land north. Drone 1 is the checker: once every quadrant is covered it holds,
then re-flies the best path through the ground, found with probability detect_probability
per look. The routes are resource/milestone4_quadrant_N.yaml, made by
milestone4_multivehicle/strategies.py. See milestone4_multivehicle/mines.py.

Run it once per vehicle, with topic_prefix and system_id saying which PX4 it flies, and
give every drone the same other arguments.

  ros2 launch milestone4_multivehicle quadrant_launch.py system_id:=1
  ros2 launch milestone4_multivehicle quadrant_launch.py topic_prefix:=/px4_1 system_id:=2
  ros2 launch milestone4_multivehicle quadrant_launch.py topic_prefix:=/px4_2 system_id:=3
  ros2 launch milestone4_multivehicle quadrant_launch.py topic_prefix:=/px4_3 system_id:=4
"""
from milestone4_multivehicle.fleet_launch import fleet_launch_description
from milestone4_multivehicle.scan_routes import SPAWNS


def generate_launch_description():
    return fleet_launch_description('milestone4_quadrant', SPAWNS, mines=150, search='quadrant')
