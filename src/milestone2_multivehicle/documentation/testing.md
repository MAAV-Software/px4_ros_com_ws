# Testing and scoring the Milestone 2 package

The workflow matches milestone 1: unit tests, replay a log, fly and score SITL,
then check failsafe behavior. Repeat flight scoring for each vehicle and add
pair-level evidence from the [test guide](milestone-2-multi-vehicle.md).

## Overview

| Level | Checks | Needs |
|---|---|---|
| 1. Unit tests | Scoring, topic/command isolation, readiness and launch behavior | Python, pytest, numpy, PyYAML |
| 2. Recorded logs | Each flight against its own path and stage limits | Two identified `.ulg` files, pyulog |
| 3. SITL | Both full missions, concurrent and independent | Built ROS workspace, PX4, Gazebo, XRCE agent, QGroundControl |
| 4. Failure/isolation tests | One mission's loss does not control the other; readiness blocks startup | External SITL session and separate controller terminals |
| 5. Real flights | Same per-vehicle and pair criteria on two airframes | Completed SITL checks and the guide's hardware preparation |

## One-time setup

```bash
python3 -m pip install pyulog
cd ~/px4_ros_com_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to milestone2_multivehicle
source install/setup.bash
```

Install numpy, PyYAML and pytest through your ROS/Python environment if needed.
`pyulog` is an optional analysis dependency, not needed to run controllers.
Source `install/setup.bash` in each new terminal.

## 1. Unit tests

```bash
colcon test --packages-select milestone2_multivehicle
colcon test-result --verbose
```

Or without ROS/colcon, from the workspace root:

```bash
PYTHONPATH=src/milestone2_multivehicle python3 -m pytest src/milestone2_multivehicle/test -q
```

The copied `test_scoring.py` exercises metric calculations, thresholds,
path validation, failsafe/takeover classification and report output.
`test_multivehicle.py` checks both controller instances using simulated ROS
interfaces: every publisher/subscriber's routing, command target IDs, independent
state, readiness, and launch gating. These checks do not replace DDS/SITL tests.

## 2. Score two recorded logs

Use logs from the **same run**, with their actual waypoint files. Example
filenames below are placeholders; replace them with real logs and paths:

```bash
ros2 run milestone2_multivehicle m2_analyze /path/to/vehicle0.ulg \
  --vehicle 0 --stage sitl --path /path/to/vehicle0_path.yaml --out /tmp/m2_run01
ros2 run milestone2_multivehicle m2_analyze /path/to/vehicle1.ulg \
  --vehicle 1 --stage sitl --path /path/to/vehicle1_path.yaml --out /tmp/m2_run01
```

Omit `--path` only if that vehicle flew the installed `milestone2_path.yaml`.
Use `--stage hitl` for real-airframe thresholds, retaining milestone 1's CLI
name. `--vehicle` is a report label, not a check of the log's hardware identity;
verify that mapping before analyzing. Reports include the label and use names
such as `vehicle_0_<log>_sitl.md` / `.json`, so equal log basenames from two
PX4s do not overwrite one another. Use a separate output folder per run.
Default output folder: `~/px4_ros_com_ws/log/m2_reports/`.

| Exit code | Meaning for this one vehicle |
|---|---|
| 0 | PASS |
| 1 | FAIL or INCOMPLETE |
| 2 | Log/config/dependency error |

The pair is not considered passed unless **both** required flight reports and
the manual M1–M7 evidence pass. Scoring two unrelated successful flights does
not establish concurrent control. Keep a common bag or timestamped observation
showing both airborne/Offboard during overlapping time.

## 3. SITL flight, scored end to end

1. Open QGroundControl; ensure no older PX4/agent/Gazebo run remains.
2. Start `ros2 launch milestone2_multivehicle px4_sitl.launch.py headless:=false`.
3. Check both topic sets, record a bag (commands in the test guide).
4. Start `ros2 launch milestone2_multivehicle offboard_solution.launch.py launch_sitl:=false`.
5. Watch both nodes log waypoint/hold progression independently and request land.
6. Confirm both aircraft land and disarm; stop recording, then stop the launch.
7. Retrieve each `.ulg` from `rootfs/0/fs/log/` and `rootfs/1/fs/log/` under
   `~/PX4-Autopilot/build/px4_sitl_default/`; score both as above.
8. Record pair-level observations and inspect oscillation/route separation manually.

Use up/down and square launches before the full path. For different mission
lengths, pass `path_1` as the up/down YAML and `path_2` as the full-path YAML.
The second mission must keep progressing after the first aircraft lands.

## 4. Failure and isolation tests

Keep SITL/agent running in their **own** launch terminal. Stop any previous
controllers before starting another for the same vehicle.

### One vehicle at a time (M2/M3)

In a sourced terminal, explicitly wait for vehicle 0, then start its controller:

```bash
ros2 run milestone2_multivehicle m2_wait_ready --timeout 300 --ros-args -p vehicle_instance:=0
ros2 run milestone2_multivehicle offboard_solution --ros-args \
  -r __node:=offboard_control_0 -p vehicle_instance:=0 \
  -p waypoints_file:=$HOME/px4_ros_com_ws/src/milestone2_multivehicle/resource/milestone2_path.yaml
```

Only run the controller if readiness exited successfully. Vehicle 1 must stay
disarmed. After landing, stop this controller and repeat using `vehicle_instance:=1`
and `__node:=offboard_control_1`; vehicle 0 must remain disarmed.
Individual controller CLI runs have no launch readiness barrier; the separate
wait command is intentional.

### One-controller loss (M6/F1)

Start the simulator separately and start a controller in each of two terminals,
using the commands above with the appropriate instance/name. With both flying,
Ctrl-C **one controller terminal only**. Keep the simulator, agent and other
controller running. Check that the selected PX4 performs its configured
Offboard-loss action while the other aircraft continues its own route.
Repeat for the other vehicle in a fresh run. Record actual PX4 failsafe mode,
trajectory and the other controller's continuous setpoint stream.

Do not use Ctrl-C on the all-in-one launch for this test: that stops the entire
simulation, so it cannot demonstrate the selected PX4's failsafe response.
The inherited scorer accepts Hold/Land for F1 and flags other actions; record
and review the configured action rather than silently changing thresholds.
A deliberately interrupted mission may be INCOMPLETE overall while its F1 row
passes. It does not replace a clean full-path acceptance run.

### Readiness failure (M7)

With only one external PX4 running, start the pair with `launch_sitl:=false`.
After the 300 s timeout, the launch must stop without starting either mission
controller or arming either vehicle. Repeat with the other vehicle missing.
Unit tests cover the same gate without waiting five minutes.

### Routing / graph checks (M1)

```bash
ros2 node info /offboard_control_0
ros2 node info /offboard_control_1
ros2 topic echo /fmu/in/vehicle_command --once
ros2 topic echo /px4_1/fmu/in/vehicle_command --once
```

Read command topics while commands are being sent. Check target_system 1 vs 2
(or your explicit IDs) and disjoint subscriptions/publishers. Topic schema
suffixes must match the firmware/px4_msgs combination, as in milestone 1.

## Scoring design

`scoring.py` is copied from milestone 1. Only report labels/metadata change;
metric calculations and pass thresholds are retained. It is pure Python with
numpy, independent of ROS. `analyze_flight.py` loads ULog topics with pyulog:
`trajectory_setpoint`, `vehicle_local_position`, `vehicle_attitude`, and
`vehicle_status`. It uses logged setpoint changes to split legs and match them
to YAML waypoints, then computes position/heading error, overshoot, settling,
cross-track, hover drift and tilt. S1/F1/L1 use logged vehicle status.

Yaw values in YAML are radians; the scorer converts them to degrees. The
controller requires full `{pos, yaw, hold_s}` waypoints; the scorer can also
read legacy position-only paths for old log analysis. The pair launch validates
both mission files before starting controllers.

| Report status | Meaning |
|---|---|
| PASS / FAIL | Available measures meet / violate thresholds |
| NO DATA | A required measure could not be computed |
| NOT REACHED / NOT FLOWN | A waypoint was missed / never visited |
| INCOMPLETE | Flight or leg ended before completion |
| NOT TESTED | Event such as F1 did not occur in this normal flight |
| NOT SCORED | Unsupported test row (e.g. V1 or R2) |

There is no live scoring node or automatic pair/collision scorer. ROS node
logs show progression during flight. Use both ULog reports and the manual
pair matrix; inspect oscillation and actual world-frame separation separately.

## Troubleshooting

| Problem | Fix |
|---|---|
| `pyulog` missing | Install it in the Python environment used by ROS |
| Package not installed | Build/source this package, or pass `--path` for standalone analysis |
| Setpoints not in path | Use exactly the file that particular vehicle flew |
| No ULog | Check that the selected aircraft actually armed, and inspect its own instance folder |
| Reports mixed up | Use `--vehicle`, separate run output folders and verified airframe/log mappings |
| Source edits not reflected | Rebuild/source the intended workspace, especially for installed launch/YAML files |
