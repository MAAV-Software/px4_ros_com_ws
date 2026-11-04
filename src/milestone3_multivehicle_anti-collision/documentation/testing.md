# Testing and scoring the Milestone 3 package

Keep the existing per-vehicle baseline scoring, and assess anti-collision from
paired recordings. The [test guide](milestone-3-anti-collision.md) defines M1–M7
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
python3 -m pip install pytest numpy PyYAML
PYTHONPATH=. python3 -m pytest test -q
```

`test_scoring.py` retains the baseline metrics/threshold checks.
`test_multivehicle.py` checks vehicle routing, readiness and launch sequencing
with in-memory ROS interfaces. `test_avoidance.py` checks fixed-origin frame
conversion, the exact 5 m boundary, priority, braking, one-time climb, waiting,
resuming, malformed messages, stale data, hold/waypoint preservation and
consistent launch origins. These tests do not exercise real DDS or flight dynamics.

## Baseline flight and scoring

Start QGroundControl, stop earlier simulations, then run:

```bash
ros2 launch milestone3_multivehicle offboard_solution.launch.py headless:=false
```

The original full/square/updown waypoint values, scoring thresholds, 2 m/s XY,
1 m/s vertical and 45 degree/s yaw settings are unchanged. The default 20 m
spawn spacing separates these baseline paths, so avoidance normally stays idle.
Install `pyulog` for analysis. Score each actual `.ulg` with the matching path:

```bash
ros2 run milestone3_multivehicle m3_analyze /actual/path/vehicle0.ulg \
  --vehicle 0 --stage sitl --out /tmp/m3_baseline
ros2 run milestone3_multivehicle m3_analyze /actual/path/vehicle1.ulg \
  --vehicle 1 --stage sitl --out /tmp/m3_baseline
```

Replace the example log paths. Omit `--path` only for `milestone3_path.yaml`;
otherwise supply the exact scored YAML. `--stage hitl` retains the inherited
real-airframe threshold name. `--vehicle` labels reports, it does not verify
log identity. Verify MAV_SYS_ID/instance mapping yourself. Reports use distinct
`vehicle_0_...` / `vehicle_1_...` prefixes. Exit codes remain 0 for baseline PASS,
1 for FAIL/INCOMPLETE, 2 for a log/config/dependency error.

**Scoring scope:** `m3_analyze` is still a single-vehicle baseline scorer.
It cannot infer priority, shared-frame distance or a peer's exit from one ULog.
Avoidance adds setpoints absent from the original path and can split waypoint
segments, so a correct avoidance flight can yield unmatched-setpoint or metric
failures. Do not weaken thresholds or suppress those segments to force PASS.
Use the baseline paths for baseline reports and the common bag for A1–A8.
The collision-demo files intentionally contain only controller waypoints, with
no baseline thresholds/test IDs; do not pass them to `m3_analyze`.

## Anti-collision SITL run

For a recording that includes startup, launch the simulator first:

```bash
ros2 launch milestone3_multivehicle px4_sitl.launch.py headless:=false
```

In a second sourced terminal, record the shared positions, controller events,
and both actual positions/setpoints:

```bash
ros2 bag record -o m3_avoidance_run01 \
  /milestone3/global_positions /rosout \
  /fmu/out/vehicle_local_position_v1 /fmu/in/trajectory_setpoint \
  /fmu/in/vehicle_command /fmu/out/vehicle_command_ack_v1 \
  /px4_1/fmu/out/vehicle_local_position_v1 /px4_1/fmu/in/trajectory_setpoint \
  /px4_1/fmu/in/vehicle_command /px4_1/fmu/out/vehicle_command_ack_v1
```

In a third sourced terminal:

```bash
ros2 launch milestone3_multivehicle collision_launch.py launch_sitl:=false
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

Use an external simulation for controller-loss tests so stopping a controller
does not also stop PX4/Gazebo. Start controllers in separate terminals:

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

The pair launcher still blocks both controllers when either PX4 readiness
check fails. Independently, instance 0 will not arm without fresh shared XY
from instance 1. There is intentionally no bypass in this package for testing
instance 0 alone with the higher-priority peer absent.

## Hardware and milestone sign-off

Only after SITL checks, run the same A1–A8 sequence on the real airframes with
measured common East/North origins, matching PX4/ROS message schemas and
independent takeover. `launch_sitl:=false` does not set flight-controller speed
parameters. Allow the extra 5 m overhead clearance, and validate the fixed
origin model against actual position estimates. Both stages need recorded
paired evidence; successful unit tests or baseline ULogs alone are insufficient.
