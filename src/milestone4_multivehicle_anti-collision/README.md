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

Start QGroundControl and stop previous PX4/Gazebo runs first. The four head-on
routes supplied with this package are flown by `collision_launch.py`, which flies
one vehicle and starts no simulator, so run it once per vehicle: `system_id` N is
instance N-1. Each picks its own route and origin from the same arguments. Start
the simulator, then each vehicle in its own sourced terminal:

```bash
ros2 launch milestone4_multivehicle px4_sitl.launch.py headless:=false
ros2 launch milestone4_multivehicle collision_launch.py system_id:=1
ros2 launch milestone4_multivehicle collision_launch.py topic_prefix:=/px4_1 system_id:=2
ros2 launch milestone4_multivehicle collision_launch.py topic_prefix:=/px4_2 system_id:=3
ros2 launch milestone4_multivehicle collision_launch.py topic_prefix:=/px4_3 system_id:=4
```

In the web launcher, pick the **20 m square** spawn layout on the Fleet tab before
adding the four SITL drones; its default line layout won't make the routes converge. Defaults put instances 0–3 at
world XY `(0,0)`, `(20,0)`, `(20,20)`, `(0,20)`. The diagonal routes converge
at `(10,10)` at 5 m altitude, then continue to the opposite corners. Equal
holds do not synchronize departure; this is a convergence stress test, not
a guaranteed simultaneous or collision-free encounter.

For the original, spatially separated baseline routes:

```bash
ros2 launch milestone4_multivehicle offboard_solution.launch.py headless:=false
```

For your own four files:

```bash
ros2 launch milestone4_multivehicle offboard_solution.launch.py \
  headless:=false \
  path_1:="$HOME/m4_paths/drone0_headon.yaml" \
  path_2:="$HOME/m4_paths/drone1_headon.yaml" \
  path_3:="$HOME/m4_paths/drone2_headon.yaml" \
  path_4:="$HOME/m4_paths/drone3_headon.yaml"
```

`spawn_x_1/y_1` through `spawn_x_4/y_4` configure both Gazebo spawns and fixed
controller origins. World X=East, Y=North; YAML positions are local
`[North, East, Down]`. All launchers default to four vehicles. The main,
square and up/down launchers also support `vehicle_count:=2` or `3`.

Priority is **3 > 2 > 1 > 0**. At horizontal distance <=5 m from any higher
instance, the lower instance brakes, climbs once by 5 m from its encounter
altitude, and holds until **all** higher instances are outside the radius and
have fresh messages. The interrupted waypoint resumes with its hold reset.
The shared payload remains only `[instance, global_x, global_y]`.

The supplied archive had a 2 m/s horizontal cap. This version uses **0.5 m/s**
for SITL, retaining 1 m/s climb/descent and 45 degree/s yaw. Verify applied
PX4 parameters in your simulator; external SITL is not configured by the
controller launch. Other flight-control behavior is unchanged.

The original rule is reactive and ignores Z: multiple yielding drones may
climb to the same altitude, and a stationary higher-priority drone can block
a lower drone indefinitely. This implementation extends the policy to four;
it does not add predictive avoidance, altitude lanes or a deadlock resolver.
Use SITL for milestone 4. See [testing and limitations](documentation/milestone-4-four-vehicle.md).
