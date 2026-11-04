# Review: Milestone 2 → Milestone 3

This adapts the supplied `milestone3_multivehicle_anti-collision` folder, which
contained milestone 2 code with a partially renamed Python directory and an
unfinished `self.global_position =` assignment. The folder name is retained;
the ROS/Python package is consistently **`milestone3_multivehicle`**.

## Focused changes

| File | Change |
|---|---|
| `milestone3_multivehicle/avoidance.py` | Small pure-Python priority state machine and fixed-origin shared XY conversion |
| `milestone3_multivehicle/offboard_control_solution.py` | Publish/receive three-number XY messages; override waypoint targets while yielding; pause hold/progression; reject stale data |
| `launch/offboard_solution.launch.py`, `launch/px4_sitl.launch.py` | Four shared spawn/origin arguments keep Gazebo placement and controller offsets consistent |
| `launch/collision_launch.py`, `resource/milestone3_collision_0.yaml`, `resource/milestone3_collision_1.yaml` | Optional predetermined offset-pass demonstration |
| Package metadata, imports, resources, entry points | Complete milestone 3 rename; add `std_msgs` runtime dependency; install README |
| Scorer/analyzer | Rename package/commands/report labels; identify the score as baseline-only; no metric/threshold changes |
| Tests and documentation | Retain baseline coverage, add anti-collision tests, explain A1–A8 acceptance and limitations |

No custom message package, central coordinator, flight-controller tuning,
trajectory planner or new PX4 control mode is introduced. The existing 10 Hz
heartbeat/setpoint loop, command destinations, yaw ramp, retry logic, waypoint
format and three baseline paths remain. The avoidance layer chooses a temporary
position target; the original waypoint/landing logic runs only when unpaused.

## Behavior

- Shared `/milestone3/global_positions` uses `Float64MultiArray.data =
  [instance, global_x, global_y]`, with empty layout. No other application data
  is sent. This existing ROS message keeps the package an `ament_python` package.
- Shared XY is East/North. PX4 local North/East is swapped before adding the
  fixed origin. Default origins are `(0,0)` and `(20,0)`.
- At horizontal distance `<= 5 m`, instance 0 yields to instance 1. Equal/lower
  priority never interrupts instance 1. Z is never part of the distance.
- Capture current XYZ/heading. Brake to horizontal speed `<= 0.2 m/s` for 0.5 s,
  then command captured `z - 5`. Hold at that fixed XY/Z; no cumulative climb.
- Complete the climb even if the peer leaves early. Resume only after arrival
  at the raised target and fresh peer distance `> 5 m`. Restart the original
  waypoint's hold; never consume a waypoint during avoidance.
- Missing/stale higher-peer data cannot release a yield. Outside an encounter,
  it pauses the lower vehicle at a fixed position. Instance 0 needs fresh
  instance 1 data before arming. Invalid/stale own data is not rebroadcast.
- Both nodes keep sharing XY after requesting Land; avoidance itself ends at
  the handoff to PX4 Land. An XY-only protocol cannot distinguish landed peers.

## Parameters and scope

New node parameters: `initial_global_x`, `initial_global_y`, read once on
startup. Pair launch `spawn_x_1/y_1/x_2/y_2` sets those and the simulator spawns.
Existing routing and `waypoints_file` parameters remain. The package supports
instances 0 and 1; extending to four aircraft is milestone 4.

The fixed origins assume aligned NED estimates without origin resets. This is
reactive avoidance without prediction: latency, stopping distance, climb time,
vertical obstacles and a peer already above the lower drone are not solved by
this policy. Preserve the 5 m rule and validate the chosen routes in SITL.

## Validation status

All 84 Python regression tests passed, covering original scorer/routing/launch
behavior, shared-frame/priority/yield behavior, and a two-controller demonstration
rollout with a simple rate-limited kinematic plant. That rollout does not model
PX4 flight dynamics. See `testing.md` for exact commands.
No ROS installation, Gazebo, PX4 or hardware flights were available during this
edit. The A1–A8 SITL and real-airframe acceptance rows remain pending. The
baseline ULog scorer is not a pair-level anti-collision evaluator.
