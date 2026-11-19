#!/usr/bin/env python
"""Milestone 4 "2x2 swarm" scan for one vehicle: a 2x2 block sweeps the field in bands.

The four drones hold a 2x2 block (two camera columns 3.2 m apart, two ranks 4 m apart)
and sweep it north and south across the field in bands, so every square gets two looks.
When the block has covered the whole field the checker (drone 4) breaks off and re-flies
the best path through the ground, found with probability detect_probability per look. The
routes are resource/milestone4_swarm2x2_N.yaml, made by
milestone4_multivehicle/strategies.py. See milestone4_multivehicle/mines.py.

Run it once per vehicle, with topic_prefix and system_id saying which PX4 it flies, and
give every drone the same other arguments.

  ros2 launch milestone4_multivehicle swarm2x2_launch.py system_id:=1
  ros2 launch milestone4_multivehicle swarm2x2_launch.py topic_prefix:=/px4_1 system_id:=2
  ros2 launch milestone4_multivehicle swarm2x2_launch.py topic_prefix:=/px4_2 system_id:=3
  ros2 launch milestone4_multivehicle swarm2x2_launch.py topic_prefix:=/px4_3 system_id:=4
"""
from milestone4_multivehicle.fleet_launch import fleet_launch_description
from milestone4_multivehicle.scan_routes import SPAWNS


def generate_launch_description():
    return fleet_launch_description('milestone4_swarm2x2', SPAWNS, mines=150, search='swarm2x2')
