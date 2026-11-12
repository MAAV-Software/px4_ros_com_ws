#!/usr/bin/env python
"""Milestone 4's collision test for one vehicle: four diagonal head-on routes that converge at (10, 10).

Run it once per vehicle, with topic_prefix and system_id saying which PX4 it flies.
system_id N is vehicle N-1. avoidance:=orca (the default) has every drone sidestep the
others with ORCA, flying velocity setpoints up to max_speed. avoidance:=priority is the
original rule: the higher the instance the higher the priority, vehicle 3 never yields,
and each lower vehicle brakes, climbs 5 m and waits while a higher one is within 5 m.
Give every drone the same avoidance and max_speed.

Each vehicle takes its own route (path_N) and its own world origin (spawn_x_N,
spawn_y_N), so every drone can be given the same file and the same arguments. The web
launcher does that for every drone ticked on the phone, adding each drone's
topic_prefix and system_id, and sends Go once all have passed their start checks.

It starts no simulator, so it runs unchanged in SITL and on real drones. The
controllers trade positions on /milestone4/global_positions, so their ROS graphs have
to see each other. The default origins are the web launcher's 20 m square SITL layout:
(0,0), (20,0), (20,20), (0,20), world east/north. The origins must be where each
drone really started, or the routes will not converge.

  ros2 launch milestone4_multivehicle collision_launch.py system_id:=1
  ros2 launch milestone4_multivehicle collision_launch.py topic_prefix:=/px4_1 system_id:=2
  ros2 launch milestone4_multivehicle collision_launch.py topic_prefix:=/px4_2 system_id:=3
  ros2 launch milestone4_multivehicle collision_launch.py topic_prefix:=/px4_3 system_id:=4
"""
from milestone4_multivehicle.fleet_launch import (  # noqa: F401 (launch_vehicle is used by tests)
    check_waypoints, fleet_launch_description, launch_vehicle)

# The web launcher's 20 m square SITL layout, world (east, north).
DEFAULT_ORIGINS = ((0.0, 0.0), (20.0, 0.0), (20.0, 20.0), (0.0, 20.0))


def generate_launch_description():
    return fleet_launch_description('milestone4_headon', DEFAULT_ORIGINS)
