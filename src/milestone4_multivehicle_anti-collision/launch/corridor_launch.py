#!/usr/bin/env python
"""Milestone 4's corridor search for one vehicle: four drones scan one band of the field.

Instead of mowing the whole 80 by 300 ft field, the four drones join up at the south edge
and fly north together over one band at 5.6 m, nose east, and the best path is planned
inside it. If mines block the band they all sidestep and carry on. They land north of
the field. 150 simulated mines are hidden on the field, and each time one passes under a
drone's camera it is found with probability detect_probability. formation sets how many
looks each square gets: abreast 1, 2x2 2, astern 4; auto picks from detect_probability.
See milestone4_multivehicle/corridor.py.

Run it once per vehicle, with topic_prefix and system_id saying which PX4 it flies, and
give every drone the same other arguments. The default origins are the web launcher's
"south edge" SITL layout. The last drone's launch also runs the mine field.

  ros2 launch milestone4_multivehicle corridor_launch.py system_id:=1
  ros2 launch milestone4_multivehicle corridor_launch.py topic_prefix:=/px4_1 system_id:=2
  ros2 launch milestone4_multivehicle corridor_launch.py topic_prefix:=/px4_2 system_id:=3
  ros2 launch milestone4_multivehicle corridor_launch.py topic_prefix:=/px4_3 system_id:=4
"""
from milestone4_multivehicle.fleet_launch import fleet_launch_description
from milestone4_multivehicle.scan_routes import SPAWNS


def generate_launch_description():
    return fleet_launch_description('milestone4_corridor', SPAWNS, mines=150, search='corridor')
