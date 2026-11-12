# Testing the Milestone 3 package

Assess anti-collision from paired recordings. The [test guide](milestone-3-anti-collision.md) defines M1–M7
and the new A1–A8 acceptance criteria. No SITL or hardware pass is claimed by
these code changes.

## Build and unit tests

```bash
cd ~/px4_ros_com_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to milestone3_multivehicle
source install/setup.bash
colcon test --packages-select milestone3_multivehicle
colcon test-result --verbose
```

Without ROS, run from the extracted package folder:

```bash
python3 -m pip install pytest PyYAML
PYTHONPATH=. python3 -m pytest test -q
```

`test_multivehicle.py` checks vehicle routing, readiness and
`collision_launch.py` with in-memory ROS interfaces. `test_avoidance.py` checks
fixed-origin frame conversion, the exact 5 m boundary, priority, braking, one-time
climb, waiting, resuming, malformed messages, stale data and hold/waypoint
preservation. These tests do not exercise real DDS or flight dynamics.

## Anti-collision SITL run

Start two SITL drones from the web launcher's Fleet tab, in the **Line** spawn
layout. In a sourced terminal, record the shared positions, controller events,
and both actual positions/setpoints:

```bash
ros2 bag record -o m3_avoidance_run01 \
  /milestone3/global_positions /rosout \
  /fmu/out/vehicle_local_position_v1 /fmu/in/trajectory_setpoint \
  /fmu/in/vehicle_command /fmu/out/vehicle_command_ack_v1 \
  /px4_1/fmu/out/vehicle_local_position_v1 /px4_1/fmu/in/trajectory_setpoint \
  /px4_1/fmu/in/vehicle_command /px4_1/fmu/out/vehicle_command_ack_v1
```

Then start the mission from the web launcher's Mission tab, which starts both
controllers together and keeps each running until both have landed. Or, in two
more sourced terminals, one vehicle each:

```bash
ros2 launch milestone3_multivehicle collision_launch.py system_id:=1
ros2 launch milestone3_multivehicle collision_launch.py topic_prefix:=/px4_1 system_id:=2
```

Use the default `(0,0)` and `(20,0)` origins for the demo. Verify instance 1
passes 3 m north of instance 0, enters its 5 m radius, and continues normally.
Instance 0 must brake, climb once from about 5 m to 10 m altitude, hold until
clear, then resume the original waypoint/hold. Keep both controllers running
after their land requests so their XY remains visible to each other. Confirm
actual landing/disarm in PX4; the controller still uses the inherited one-shot
land request.

A quick shared-message inspection (not an acceptance measurement):

```bash
ros2 topic echo /milestone3/global_positions
ros2 node info /offboard_control_0
ros2 node info /offboard_control_1
```

Record A1–A8 results, both ULogs, bag, paths, origins, PX4 version and observed
minimum separation. Review the real trajectory/velocity, not only commanded
setpoints. Repeat three times, then test a second encounter, different peer
altitudes, early clearance during climb, and peer loss in separate SITL runs.
The offset pass demonstrates the policy without aiming both aircraft at the
same point; it does not establish safety for every possible route.

## Peer/controller loss and readiness

Start the SITL drones from the web launcher, so stopping a controller does not
also stop PX4/Gazebo. Start controllers in separate terminals:

```bash
share=$(ros2 pkg prefix --share milestone3_multivehicle)
ros2 run milestone3_multivehicle offboard_solution --ros-args \
  -p vehicle_instance:=0 -p initial_global_x:=0.0 -p initial_global_y:=0.0 \
  -p waypoints_file:="$share/resource/milestone3_collision_0.yaml"
```

In another sourced terminal:

```bash
share=$(ros2 pkg prefix --share milestone3_multivehicle)
ros2 run milestone3_multivehicle offboard_solution --ros-args \
  -p vehicle_instance:=1 -p initial_global_x:=20.0 -p initial_global_y:=0.0 \
  -p waypoints_file:="$share/resource/milestone3_collision_1.yaml"
```

Stop only the selected controller with Ctrl-C. If instance 1 is lost, instance
0 holds rather than interpreting disappearance as clearance; if already in an
encounter it completes the fixed climb and waits. If instance 0 is lost,
instance 1 continues. The stopped node's PX4 must execute its configured
Offboard-loss failsafe; inspect the ULog separately. Restarting an airborne
controller loses its encounter/waypoint state and is not an automatic recovery
procedure. Finish/reset the test before relaunching.

Each `collision_launch.py` starts its controller once its own PX4 is ready;
there is no pair-wide readiness gate. Independently, instance 0 will not arm without fresh shared XY
from instance 1. There is intentionally no bypass in this package for testing
instance 0 alone with the higher-priority peer absent.

## Hardware and milestone sign-off

Only after SITL checks, run the same A1–A8 sequence on the real airframes with
measured common East/North origins, matching PX4/ROS message schemas and
independent takeover. The controller does not set flight-controller speed
parameters. Allow the extra 5 m overhead clearance, and validate the fixed
origin model against actual position estimates. Both stages need recorded
paired evidence; successful unit tests alone are insufficient.
