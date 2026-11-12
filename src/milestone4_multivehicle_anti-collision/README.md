# Milestone 4: four drones with priority-based anti-collision

ROS/Python package: `milestone4_multivehicle`. Extract this folder under
`~/px4_ros_com_ws/src/`; milestone 3 can remain installed separately.

```bash
cd ~/px4_ros_com_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to milestone4_multivehicle
source install/setup.bash
colcon test --packages-select milestone4_multivehicle
colcon test-result --verbose
```

The package has one launch file, `collision_launch.py`, which flies the four
head-on routes supplied with this package. It flies one vehicle and starts no
simulator, so run it once per vehicle: `system_id` N is instance N-1. Each picks
its own route and origin from the same arguments.

The easiest way is the web launcher in SITL mode. On its Fleet tab pick the
**20 m square** spawn layout before adding the four SITL drones (the default line
layout won't make the routes converge), then pick this file on the Mission tab,
tick all four and press Start. With SITL (or real drones) already running, by hand,
each in its own sourced terminal:

```bash
ros2 launch milestone4_multivehicle collision_launch.py system_id:=1
ros2 launch milestone4_multivehicle collision_launch.py topic_prefix:=/px4_1 system_id:=2
ros2 launch milestone4_multivehicle collision_launch.py topic_prefix:=/px4_2 system_id:=3
ros2 launch milestone4_multivehicle collision_launch.py topic_prefix:=/px4_3 system_id:=4
```

Defaults put instances 0–3 at world XY `(0,0)`, `(20,0)`, `(20,20)`, `(0,20)`.
The diagonal routes converge at `(10,10)` at 5 m altitude, then continue to the
opposite corners. Equal holds do not synchronize departure; this is a convergence
stress test, not a guaranteed simultaneous or collision-free encounter.

`spawn_x_1/y_1` through `spawn_x_4/y_4` set the fixed controller origins and
`path_1` through `path_4` the routes; pass the same values to every drone.
World X=East, Y=North; YAML positions are local `[North, East, Down]`.
`vehicle_count:=2` or `3` flies a smaller fleet; with `vehicle_count:=2` and
`path_1`/`path_2` set to `milestone4_collision_0.yaml`/`_1.yaml` it flies the
two-drone offset pass.

Priority is **3 > 2 > 1 > 0**. At horizontal distance <=5 m from any higher
instance, the lower instance brakes, climbs once by 5 m from its encounter
altitude, and holds until **all** higher instances are outside the radius and
have fresh messages. The interrupted waypoint resumes with its hold reset.
The shared payload remains only `[instance, global_x, global_y]`.

The controller sets no PX4 parameters. The web launcher's SITL drones use a
1 m/s horizontal cap, 1 m/s climb/descent and 45 degree/s yaw. The cap was 2 m/s
until 2026-11-12; at 2 m/s the first SITL run failed with a mid-air collision.
See the results log.

The original rule is reactive and ignores Z: multiple yielding drones may
climb to the same altitude, and a stationary higher-priority drone can block
a lower drone indefinitely. This implementation extends the policy to four;
it does not add predictive avoidance, altitude lanes or a deadlock resolver.
Use SITL for milestone 4. See [testing and limitations](documentation/milestone-4-four-vehicle.md).
