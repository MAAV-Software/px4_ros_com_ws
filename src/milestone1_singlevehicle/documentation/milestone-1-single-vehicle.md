Re# Milestone 1: 1 Drone Autonomous

**Goal (from [PLAN.md](../../../PLAN.md)):** baseline offboard control of a single drone on a single track, validated in sim **and** on one of the 2 old-design drones.

This package is the test for that milestone. It checks freedom of flight and control: can our ROS 2 stack take one drone from the ground, move it in every direction it can be controlled in (see the [test matrix](#freedom-of-flight-test-matrix)) under offboard control, and hand it back to PX4 safely? It runs in two stages:

1. **SITL:** PX4 SITL + Gazebo, headless by default.
2. **HITL:** the same node, unchanged, on a real airframe.

Pass SITL before you try HITL. Milestone 1 is complete when both stages pass, and Milestone 2 (2 drones) builds on this code.

## What the test flies

[`offboard_control_solution.py`](../milestone1_singlevehicle/offboard_control_solution.py) is the week 3 onboarding controller, promoted to the milestone baseline. In order, it:

1. Streams position setpoints at 10 Hz for 1 s (`ARM_AFTER_TICKS = 10`).
2. Sends **arm**, then switches PX4 to **offboard** mode.
3. Takes off to the first waypoint and flies the path in [`resource/milestone1_path.yaml`](../resource/milestone1_path.yaml). A waypoint counts as reached within 0.5 m (`ACCEPTANCE_RADIUS`).
4. Sends **land** after the last waypoint and stops streaming setpoints.

Default path, in the local NED frame (metres; x = north, y = east, z = down, so negative z is up):

| # | x | y | z | Exercises |
|---|---|---|---|---|
| 0 | 0 | 0 | -5 | Takeoff / climb |
| 1 | 5 | 0 | -5 | North |
| 2 | 5 | 5 | -5 | East |
| 3 | 0 | 5 | -5 | South (return leg) |
| — | — | — | — | Land in place at #3 |

The origin is wherever PX4's local position estimate initialised, which is normally where the drone was when it booted.

To change the path, edit `milestone1_path.yaml` and rebuild (`colcon build --packages-select milestone1_singlevehicle`). The node reads the installed copy, so edits don't take effect until you rebuild. On startup the node logs `loaded N waypoints from milestone1_path.yaml`. If you change the path, update the table above and the HITL area size to match.

**Known limits:** single vehicle, no namespacing, no failure recovery, and yaw fixed at 0 (facing north). It doesn't check whether the arm or mode switch succeeded; it just keeps streaming. It also moves on as soon as a waypoint is reached, with no hover time. Several matrix rows below need node changes because of these limits. The HITL safety rules below assume them.

## Freedom-of-flight test matrix

A quadrotor moves in 6 ways: x, y and z position, plus roll, pitch and yaw rotation. It has only 4 independent controls (total thrust, plus roll, pitch and yaw torque), so roll and pitch can't be held separately from horizontal movement: the drone tilts in order to move. In practice, "every degree of freedom" means:

- **x, y, z and yaw:** commanded directly, in both directions, alone and combined.
- **Roll and pitch:** tested through the tilt that horizontal moves cause, and optionally commanded directly in attitude mode.

Every row below must pass in SITL. HITL runs the same rows in the [order below](#test-order), and rows marked *SITL only* are optional on hardware.

**Status key:**
- **Default path:** the current `milestone1_path.yaml` already flies it.
- **YAML:** needs extra waypoints in `milestone1_path.yaml`, with no code change.
- **Node change:** needs a change to `offboard_control_solution.py` that isn't implemented yet.

### Translation (x, y, z)

| ID | Movement | How it's tested | Status |
|---|---|---|---|
| T1 | Climb (−z) | Takeoff to waypoint 0 | Default path |
| T2 | Descend under offboard control (+z) | Waypoint at a lower altitude than the one before, e.g. 5 m → 3 m | YAML |
| T3 | North (+x) | Waypoint 0 → 1 | Default path |
| T4 | South (−x) | Waypoint 2 → 3 | Default path |
| T5 | East (+y) | Waypoint 1 → 2 | Default path |
| T6 | West (−y) | Waypoint with a smaller y than the one before | YAML |
| T7 | Diagonal in the horizontal plane | x and y both change in one leg, in both diagonal directions | YAML |
| T8 | Combined 3D move | x, y and z all change in one leg, climbing and descending | YAML |
| T9 | Small corrections | Legs of ~1 m, to check fine positioning rather than large moves | YAML |

### Rotation (yaw, roll, pitch)

| ID | Movement | How it's tested | Status |
|---|---|---|---|
| Y1 | Yaw in place, both directions | Hover and rotate 0 → 90 → 180 → −90 → 0°, with no translation | Node change: yaw per waypoint in the YAML, sent in `TrajectorySetpoint.yaw`, and heading included in the "reached" check |
| Y2 | Yaw while moving | Heading changes during a translation leg | Node change (same as Y1) |
| R1 | Roll and pitch, indirect | Tilt measured from `vehicle_attitude` during T3–T8 | Default path (recording needed, see [Recording and measuring](#recording-and-measuring)) |
| R2 | Roll and pitch, direct | Small attitude setpoints (≤10°) through `VehicleAttitudeSetpoint` in attitude offboard mode | Node change. *SITL only* unless needed on hardware |

### Holding, control modes and safety

| ID | Behaviour | How it's tested | Status |
|---|---|---|---|
| H1 | Hover hold | Hold at a waypoint for a set time (e.g. 10 s) before moving on, and measure drift | Node change: hold time per waypoint |
| V1 | Velocity control | Legs flown at a constant commanded velocity instead of to a position | Node change: velocity offboard mode |
| S1 | Arm and mode confirmation | Node confirms it's armed and in Offboard mode from `vehicle_status`, and logs it if not | Node change: `vehicle_status` subscription |
| F1 | Offboard-loss failsafe | Stop the node mid-path; PX4 runs its failsafe action | Default path (manual, `Ctrl-C`) |
| L1 | Landing | PX4 land mode after the last waypoint; lands and disarms | Default path |

## Pass thresholds

**Proposed: agree these as a team before the first recorded run**, and update this table if they change. HITL limits are looser because real positioning (GPS) is noisier than the sim.

| Measure | Applies to | SITL | HITL |
|---|---|---|---|
| Position error once the waypoint is reached | T1–T9 | ≤ 0.5 m | ≤ 0.5 m |
| Overshoot past a waypoint | T1–T9 | ≤ 0.5 m | ≤ 1.0 m |
| Time to settle at a waypoint after arriving | T1–T9 | ≤ 3 s | ≤ 5 s |
| Heading error once reached | Y1, Y2 | ≤ 5° | ≤ 10° |
| Drift during a 10 s hover | H1 | ≤ 0.2 m | ≤ 1.0 m |
| Tilt angle | R1, R2 | Stays within PX4's limit (`MPC_TILTMAX_AIR`), with no oscillation | Same |
| Visible oscillation in position or attitude | All | None | None |

## Recording and measuring

Reaching each waypoint isn't enough to pass the matrix: most thresholds need recorded data.

**SITL: record a ros2 bag for every run.** Check the exact topic names first, because this PX4 build publishes some of them under versioned names (for example `vehicle_local_position_v1`):

```
ros2 topic list | grep -E 'vehicle_local_position|vehicle_attitude|vehicle_status|trajectory_setpoint'
ros2 bag record -o m1_sitl_<date>_<run> \
  /fmu/out/vehicle_local_position_v1 \
  /fmu/out/vehicle_attitude \
  /fmu/out/vehicle_status_v1 \
  /fmu/in/trajectory_setpoint
```

Replace the topic names with the ones `ros2 topic list` shows.

**SITL and HITL: keep the PX4 flight log (`.ulg`).** SITL writes it under `~/PX4-Autopilot/build/px4_sitl_default/rootfs/fs/log/`. On hardware, download it from the flight controller with QGroundControl (Analyze Tools → Log Download).

**Analyse** with PlotJuggler (reads both bags and `.ulg` files) or PX4 Flight Review (upload the `.ulg`). For each matrix row, compare the setpoint against the actual position or attitude, and note:

- Position error once reached, overshoot, and time to settle (T rows)
- Heading error (Y rows)
- Drift while hovering (H1)
- Maximum tilt and any oscillation (R rows)

Put the worst-case numbers in the [results log](#results-log), and save the bag or `.ulg` file name with them.

## Test order

Each step must pass before moving on, in SITL first and then in HITL.

1. **Default path:** T1, T3, T4, T5, F1, L1, plus R1 from the recording.
2. **Extended path (YAML only):** T2, T6, T7, T8, T9.
3. **After node changes:** S1, H1, then Y1, then Y2.
4. **Control modes:** V1, then R2 (R2 in SITL only unless hardware testing is needed).

On hardware, start each step with shorter distances and lower altitude than in SITL, then scale up to the full path.

## Stage 1: SITL

### Run

```
cd ~/px4_ros_com_ws
colcon build --packages-select milestone1_singlevehicle
source install/setup.bash
ros2 launch milestone1_singlevehicle offboard_solution.launch.py
```

This launch file:

- Opens the Micro XRCE-DDS Agent and PX4 SITL (`gz_x500`) in their own terminals, using [`px4_sitl.launch.py`](../launch/px4_sitl.launch.py).
- Runs Gazebo **headless** (`HEADLESS=1`). The sim runs normally, but no Gazebo window opens. Watch the flight in QGroundControl or the PX4 console.
- Waits 20 s for SITL to boot, then starts the node in its own terminal.

Options:

| Argument | Default | Use |
|---|---|---|
| `headless:=false` | `true` | Open the Gazebo GUI to watch the drone |
| `launch_sitl:=false` | `true` | SITL + agent already running; start only the node, immediately |

If the drone never arms on the first run, PX4 was probably still building or booting. Leave SITL up and rerun with `launch_sitl:=false`.

SITL alone (no node) is also available: `ros2 launch milestone1_singlevehicle px4_sitl.launch.py [headless:=false]`.

To record a run, start PX4 SITL alone, start `ros2 bag record` (see [Recording and measuring](#recording-and-measuring)), then start the node with `launch_sitl:=false`.

### SITL pass criteria

Record each run in the [results log](#results-log).

- [ ] Node logs `arm command sent` and `offboard mode command sent`; PX4 console shows armed and in Offboard mode
- [ ] Climbs to ~5 m and holds steady (no oscillation or drift)
- [ ] Logs `reached waypoint 0, advancing to 1` through to `reached final waypoint, landing`, for every waypoint in the path
- [ ] Lands and disarms on its own
- [ ] No manual intervention from QGroundControl at any point
- [ ] **Failsafe check (F1):** on a separate run, `Ctrl-C` the node mid-path. PX4 leaves Offboard mode and runs its offboard-loss failsafe (hold / land / RTL, depending on the `COM_OBL_RC_ACT` parameter). Note which one happened.
- [ ] Every [test matrix](#freedom-of-flight-test-matrix) row in the current [test order](#test-order) step is within the SITL [pass thresholds](#pass-thresholds), measured from a recording
- [ ] 3 consecutive clean runs

### Troubleshooting

- **Climbs to the first waypoint but never advances, or you see `RTPS_READER_HISTORY Error ... cannot be resized`:** the message type doesn't match the topic. The node subscribes to `/fmu/out/vehicle_local_position_v1`, the versioned name this PX4 build publishes. Check with `ros2 topic list | grep vehicle_local_position`.
- **No `/fmu/...` topics at all:** the XRCE agent isn't running, or PX4 didn't connect to it. Check the agent's terminal.
- **Mode switch rejected:** PX4 needs a steady setpoint stream *before* it accepts Offboard mode. Check that the node is publishing (`ros2 topic hz /fmu/in/trajectory_setpoint`, which should show ~10 Hz).

## Stage 2: HITL (real airframe)

Fly the same node, unmodified, on one of the 2 old-design drones. The goal is to show that what passed in SITL behaves the same on hardware.

> **Safety: read before every flight.** The node **arms the drone ~1 s after it starts**, without checking anything first. Only start it when the drone is in position, the area is clear, and the safety pilot is ready. There's no failure recovery in the code. The RC safety pilot and PX4's own failsafes are the only protection.

### Before the flight

**Setup to confirm (fill in for our airframes):**

- [ ] Companion computer / ground station running ROS 2 and connected to the flight controller: _TODO: connection type (serial / UDP) and the matching `MicroXRCEAgent` command_
- [ ] PX4 firmware version on the airframe: _TODO_. If it's older or newer than the SITL build, check the versioned topic names again (see troubleshooting above).
- [ ] Airframe-specific notes: _TODO_

**Every flight:**

- [ ] Clear, open area sized to the path being flown, with at least 5 m of margin on every side. The default path (a 5 m square at 5 m altitude) needs at least **15 m × 15 m**. Work out the area again whenever `milestone1_path.yaml` changes.
- [ ] Drone placed so that the path's directions from its start point are clear. The default path goes north (+x) first, then east (+y).
- [ ] Good position lock (GPS or other positioning) shown in QGroundControl before starting the node. PX4 rejects arming and offboard mode without a valid local position.
- [ ] RC transmitter on, with a safety pilot holding it. Kill switch and manual-mode switch tested.
- [ ] Offboard-loss and RC-loss failsafe parameters checked (`COM_OBL_RC_ACT`, `COM_RCL_EXCEPT`, `NAV_RCL_ACT`).
- [ ] Battery charged; props checked.
- [ ] Recommended first step: a **props-off bench run**. Start the node and confirm it arms, enters Offboard, and streams setpoints, then disarm.

### Run

On the machine connected to the drone, start the XRCE agent for the hardware link, then start only the node (no SITL):

```
source install/setup.bash
ros2 launch milestone1_singlevehicle offboard_solution.launch.py launch_sitl:=false
```

The safety pilot takes over (switches out of Offboard) at the first sign of unexpected motion.

### HITL pass criteria

- [ ] Arms and enters Offboard mode
- [ ] Stable takeoff to ~5 m and hover
- [ ] Flies every waypoint in the path; path shape matches SITL
- [ ] Lands and disarms on its own
- [ ] No safety-pilot takeover needed
- [ ] **Failsafe check (F1):** stop the node mid-path, and confirm PX4 runs the same failsafe action as in SITL. The safety pilot must be ready.
- [ ] Every [test matrix](#freedom-of-flight-test-matrix) row in the current [test order](#test-order) step (except *SITL only* rows) is within the HITL [pass thresholds](#pass-thresholds), measured from the flight log
- [ ] Flight log (`.ulg`) pulled from the flight controller and saved with the results

## Results log

Add a row per run, and list the matrix rows each run covered. A stage passes when its checklist is fully ticked and every matrix row for the current test order step has a passing run. Write "—" for measures that don't apply to that run.

| Date | Stage | Airframe / PX4 version | Path file / version | Matrix rows covered | Max position error (m) | Max overshoot (m) | Max settle time (s) | Max heading error (°) | Hover drift (m) | Failsafe action seen | Result | Recording / log file | Notes |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| | SITL | x500 (sim) | | | | | | | | | | | |
| | HITL | | | | | | | | | | | | |
