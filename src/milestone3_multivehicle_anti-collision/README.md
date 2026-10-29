# Milestone 3: two-vehicle anti-collision

ROS package name: **`milestone3_multivehicle`**. Place this folder under
`~/px4_ros_com_ws/src/`. It can coexist with milestone 1 and milestone 2.

Within **5 m horizontally**, instance **1** keeps flying. Instance **0**
holds its current XY, brakes, climbs to **5 m above its encounter altitude**,
and waits until instance 1 is outside the radius. It then resumes the same
waypoint and restarts that waypoint's hold. Z never affects the proximity test.

```bash
cd ~/px4_ros_com_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to milestone3_multivehicle
source install/setup.bash
colcon test --packages-select milestone3_multivehicle
colcon test-result --verbose

# Start QGroundControl and stop any previous PX4/Gazebo run first.
# This launches both drones and automatically starts their missions.
ros2 launch milestone3_multivehicle collision_launch.py headless:=false
```

The demonstration uses a 3 m lateral offset: instance 1 passes through instance
0's horizontal radius, and instance 0 yields. The original full/square/updown
launches and path overrides remain available. Use
`offboard_solution.launch.py` for the original full baseline path.

Both nodes publish only `[instance, global_x, global_y]` on
`/milestone3/global_positions` (`std_msgs/msg/Float64MultiArray`). Shared XY
uses East/North axes: `global_x = origin_x + local.y`,
`global_y = origin_y + local.x`. Origins are fixed parameters; defaults are
`(0,0)` and `(20,0)`. Change `spawn_x_1`, `spawn_y_1`, `spawn_x_2`, `spawn_y_2`
in the launch command to update both Gazebo spawns and the controller origins.
For separately launched SITL or hardware, supply the same actual origins.

See [the test guide](documentation/milestone-3-anti-collision.md) for states,
parameters, recording commands, limitations and the A1–A8 acceptance matrix;
[testing](documentation/testing.md) for unit tests and baseline scoring; and
[the change review](documentation/milestone2-to-milestone3-review.md) for scope.

This is the requested reactive policy, not a guarantee against every collision:
braking and a 5 m climb take time. SITL and hardware acceptance must still be
performed. The retained per-vehicle scorer does not certify anti-collision.
