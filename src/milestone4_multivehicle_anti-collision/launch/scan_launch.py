#!/usr/bin/env python
"""Milestone 4's scan of the competition field for one vehicle, with ORCA anti-collision by default.

The 80 by 300 ft field (24.4 by 91.4 m) is scanned in rows 2 m apart, the camera
footprint's width. Drones 1-3 (system_id 1-3) each take a third of the width in
north-south rows, then return to their spawn and land. Drone 4 flies east-west rows
over the whole field from the south edge northward, crossing the other three, and lands
at the north end. The routes are resource/milestone4_scan_N.yaml, made by
milestone4_multivehicle/scan_routes.py.

Run it once per vehicle, with topic_prefix and system_id saying which PX4 it flies, and
give every drone the same other arguments. The default origins are the web launcher's
"south edge" SITL layout: (1,-3), (9,-3), (17,-3), (23,-3) world east/north, with the
field's south-west corner at (0, 0). The origins must be where each drone really started.

  ros2 launch milestone4_multivehicle scan_launch.py system_id:=1
  ros2 launch milestone4_multivehicle scan_launch.py topic_prefix:=/px4_1 system_id:=2
  ros2 launch milestone4_multivehicle scan_launch.py topic_prefix:=/px4_2 system_id:=3
  ros2 launch milestone4_multivehicle scan_launch.py topic_prefix:=/px4_3 system_id:=4
"""
from milestone4_multivehicle.fleet_launch import fleet_launch_description
from milestone4_multivehicle.scan_routes import SPAWNS


def generate_launch_description():
    return fleet_launch_description('milestone4_scan', SPAWNS)
