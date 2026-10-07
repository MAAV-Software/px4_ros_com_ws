# Milestone 1: 1 Drone Autonomous

This guide has two parts: the [test guide](#test-guide) (what is tested, how, and the pass criteria) and the [session plan](#session-plan) (how to finish the tests in three working sessions). Scoring and test commands are in [`testing.md`](testing.md).

## Test guide

**Goal (from [PLAN.md](../../../PLAN.md)):** baseline offboard control of a single drone on a single track, validated in sim **and** on one of the 2 old-design drones.

This package is the test for that milestone. It checks freedom of flight and control: can our ROS 2 stack take one drone from the ground, move it in every direction it can be controlled in (see the [test matrix](#freedom-of-flight-test-matrix)) under offboard control, and hand it back to PX4 safely? It runs in two stages:

1. **SITL:** PX4 SITL + Gazebo, headless by default.
2. **HITL:** the same node, unchanged, on a real airframe.

Pass SITL before you try HITL. Milestone 1 is complete when both stages pass, and Milestone 2 (2 drones) builds on this code.

### What the test flies

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

### Freedom-of-flight test matrix

A quadrotor moves in 6 ways: x, y and z position, plus roll, pitch and yaw rotation. It has only 4 independent controls (total thrust, plus roll, pitch and yaw torque), so roll and pitch can't be held separately from horizontal movement: the drone tilts in order to move. In practice, "every degree of freedom" means:

- **x, y, z and yaw:** commanded directly, in both directions, alone and combined.
- **Roll and pitch:** tested through the tilt that horizontal moves cause, and optionally commanded directly in attitude mode.

Every row below must pass in SITL. HITL runs the same rows in the [order below](#test-order), and rows marked *SITL only* are optional on hardware.

**Status key:**
- **Default path:** the current `milestone1_path.yaml` already flies it.
- **YAML:** needs extra waypoints in `milestone1_path.yaml`, with no code change.
- **Node change:** needs a change to `offboard_control_solution.py` that isn't implemented yet.

#### Translation (x, y, z)

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

#### Rotation (yaw, roll, pitch)

| ID | Movement | How it's tested | Status |
|---|---|---|---|
| Y1 | Yaw in place, both directions | Hover and rotate 0 → 90 → 180 → −90 → 0°, with no translation | Node change: yaw per waypoint in the YAML, sent in `TrajectorySetpoint.yaw`, and heading included in the "reached" check |
| Y2 | Yaw while moving | Heading changes during a translation leg | Node change (same as Y1) |
| R1 | Roll and pitch, indirect | Tilt measured from `vehicle_attitude` during T3–T8 | Default path (recording needed, see [Recording and measuring](#recording-and-measuring)) |
| R2 | Roll and pitch, direct | Small attitude setpoints (≤10°) through `VehicleAttitudeSetpoint` in attitude offboard mode | Node change. *SITL only* unless needed on hardware |

#### Holding, control modes and safety

| ID | Behaviour | How it's tested | Status |
|---|---|---|---|
| H1 | Hover hold | Hold at a waypoint for a set time (e.g. 10 s) before moving on, and measure drift | Node change: hold time per waypoint |
| V1 | Velocity control | Legs flown at a constant commanded velocity instead of to a position | Node change: velocity offboard mode |
| S1 | Arm and mode confirmation | Node confirms it's armed and in Offboard mode from `vehicle_status`, and logs it if not | Node change: `vehicle_status` subscription |
| F1 | Offboard-loss failsafe | Stop the node mid-path; PX4 runs its failsafe action | Default path (manual, `Ctrl-C`) |
| L1 | Landing | PX4 land mode after the last waypoint; lands and disarms | Default path |

### Pass thresholds

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

### Recording and measuring

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

### Test order

Each step must pass before moving on, in SITL first and then in HITL.

1. **Default path:** T1, T3, T4, T5, F1, L1, plus R1 from the recording.
2. **Extended path (YAML only):** T2, T6, T7, T8, T9.
3. **After node changes:** S1, H1, then Y1, then Y2.
4. **Control modes:** V1, then R2 (R2 in SITL only unless hardware testing is needed).

On hardware, start each step with shorter distances and lower altitude than in SITL, then scale up to the full path.

### Stage 1: SITL

#### Run

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

#### SITL pass criteria

Record each run in the [results log](#results-log).

- [ ] Node logs `arm command sent` and `offboard mode command sent`; PX4 console shows armed and in Offboard mode
- [ ] Climbs to ~5 m and holds steady (no oscillation or drift)
- [ ] Logs `reached waypoint 0, advancing to 1` through to `reached final waypoint, landing`, for every waypoint in the path
- [ ] Lands and disarms on its own
- [ ] No manual intervention from QGroundControl at any point
- [ ] **Failsafe check (F1):** on a separate run, `Ctrl-C` the node mid-path. PX4 leaves Offboard mode and runs its offboard-loss failsafe (hold / land / RTL, depending on the `COM_OBL_RC_ACT` parameter). Note which one happened.
- [ ] Every [test matrix](#freedom-of-flight-test-matrix) row in the current [test order](#test-order) step is within the SITL [pass thresholds](#pass-thresholds), measured from a recording
- [ ] 3 consecutive clean runs

#### Troubleshooting

- **Climbs to the first waypoint but never advances, or you see `RTPS_READER_HISTORY Error ... cannot be resized`:** the message type doesn't match the topic. The node subscribes to `/fmu/out/vehicle_local_position_v1`, the versioned name this PX4 build publishes. Check with `ros2 topic list | grep vehicle_local_position`.
- **No `/fmu/...` topics at all:** the XRCE agent isn't running, or PX4 didn't connect to it. Check the agent's terminal.
- **Mode switch rejected:** PX4 needs a steady setpoint stream *before* it accepts Offboard mode. Check that the node is publishing (`ros2 topic hz /fmu/in/trajectory_setpoint`, which should show ~10 Hz).

### Stage 2: HITL (real airframe)

Fly the same node, unmodified, on one of the 2 old-design drones. The goal is to show that what passed in SITL behaves the same on hardware.

> **Safety: read before every flight.** The node **arms the drone ~1 s after it starts**, without checking anything first. Only start it when the drone is in position, the area is clear, and the safety pilot is ready. There's no failure recovery in the code. The RC safety pilot and PX4's own failsafes are the only protection.

#### Before the flight

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

#### Run

On the machine connected to the drone, start the XRCE agent for the hardware link, then start only the node (no SITL):

```
source install/setup.bash
ros2 launch milestone1_singlevehicle offboard_solution.launch.py launch_sitl:=false
```

The safety pilot takes over (switches out of Offboard) at the first sign of unexpected motion.

#### HITL pass criteria

- [ ] Arms and enters Offboard mode
- [ ] Stable takeoff to ~5 m and hover
- [ ] Flies every waypoint in the path; path shape matches SITL
- [ ] Lands and disarms on its own
- [ ] No safety-pilot takeover needed
- [ ] **Failsafe check (F1):** stop the node mid-path, and confirm PX4 runs the same failsafe action as in SITL. The safety pilot must be ready.
- [ ] Every [test matrix](#freedom-of-flight-test-matrix) row in the current [test order](#test-order) step (except *SITL only* rows) is within the HITL [pass thresholds](#pass-thresholds), measured from the flight log
- [ ] Flight log (`.ulg`) pulled from the flight controller and saved with the results

### Results log

Add a row per run, and list the matrix rows each run covered. A stage passes when its checklist is fully ticked and every matrix row for the current test order step has a passing run. Write "—" for measures that don't apply to that run.

| Date | Stage | Airframe / PX4 version | Path file / version | Matrix rows covered | Max position error (m) | Max overshoot (m) | Max settle time (s) | Max heading error (°) | Hover drift (m) | Failsafe action seen | Result | Recording / log file | Notes |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| | SITL | x500 (sim) | | | | | | | | | | | |
| | HITL | | | | | | | | | | | | |

## Session plan

**Goal:** finish the Milestone 1 tests in three working sessions:

| Session | Length | Stage | Outcome |
|---|---|---|---|
| 1 | 1.5 h | SITL | Node changes written, reviewed and smoke-tested; first full-path run scored |
| 2 | 1.5 h | SITL | Every in-scope test passes in 3 consecutive runs; failsafe (F1) recorded; go/no-go for HITL |
| 3 | 3 h | HITL (real airframe) | The same tests pass on one of the old-design drones |

Related docs:
- The test matrix and thresholds: the [test guide](#test-guide) above
- Test commands and the full scoring design: [`testing.md`](testing.md#scoring-design)

### Scope

| In scope | Deferred (Milestone 1 follow-up) | Why deferred |
|---|---|---|
| T1–T9 (translation), Y1–Y2 (yaw), R1 (indirect roll/pitch), H1 (hover hold), S1, F1, L1 | V1 (velocity mode), R2 (direct attitude mode) | PLAN.md defines Milestone 1 as baseline offboard control on a single track. Later milestones use position control. V1 and R2 each add a new control mode with its own safety work. V1 can be tried in SITL in Session 2's [stretch slot](#session-2-sitl-runs-90-min). |
| Post-flight scoring from the `.ulg` log (`m1_analyze`) | Live monitor node from `testing.md` | The `.ulg` logs contain everything needed, so one tool covers both SITL and HITL. During a flight, the node's own logs show progress. |

### The test path

One path covers every in-scope row. It stays inside a 5 m × 5 m square at 3–6 m altitude, so the guide's 15 m × 15 m HITL area still applies. Yaw is in degrees in the NED frame: 0 = north, 90 = east (clockwise seen from above).

| # | x | y | z | Yaw | Hold (s) | Tests |
|---|---|---|---|---|---|---|
| 0 | 0 | 0 | −5 | 0 | 5 | T1 climb |
| 1 | 5 | 0 | −5 | 0 | 5 | T3 north |
| 2 | 5 | 5 | −5 | 0 | 5 | T5 east |
| 3 | 0 | 5 | −5 | 0 | 5 | T4 south |
| 4 | 0 | 0 | −5 | 0 | 5 | T6 west |
| 5 | 0 | 0 | −3 | 0 | 5 | T2 descend |
| 6 | 4 | 4 | −3 | 0 | 5 | T7 diagonal (NE) |
| 7 | 4 | 0 | −3 | 0 | 3 | — (reposition) |
| 8 | 0 | 4 | −3 | 0 | 5 | T7 diagonal (NW) |
| 9 | 4 | 0 | −6 | 0 | 5 | T8 3D (SE + climb) |
| 10 | 0 | 4 | −4 | 0 | 5 | T8 3D (NW + descend) |
| 11 | 1 | 4 | −4 | 0 | 3 | T9 small (1 m north) |
| 12 | 1 | 5 | −4 | 0 | 3 | T9 small (1 m east) |
| 13 | 2 | 2 | −5 | 0 | 10 | H1 hover hold |
| 14 | 2 | 2 | −5 | 90 | 3 | Y1 yaw clockwise |
| 15 | 2 | 2 | −5 | 180 | 3 | Y1 yaw clockwise |
| 16 | 2 | 2 | −5 | 90 | 3 | Y1 yaw counter-clockwise |
| 17 | 2 | 2 | −5 | 0 | 3 | Y1 yaw counter-clockwise |
| 18 | 2 | 2 | −5 | −90 | 3 | Y1 yaw counter-clockwise |
| 19 | 2 | 2 | −5 | 0 | 3 | Y1 yaw clockwise |
| 20 | 4 | 2 | −5 | 90 | 5 | Y2 yaw while moving |
| 21 | 2 | 2 | −5 | 0 | 5 | Y2 yaw while moving, then land |

- Yaw changes never exceed 90° in one step, so the turn direction is never ambiguous.
- R1 (tilt) is scored on every leg, and S1, F1 and L1 from `vehicle_status`.
- Expected flight time is about 3–4 minutes with the speed limits below: about 100 s of hold time, plus travel, takeoff and landing.
- A second file, `milestone1_path_square.yaml`, holds the original 4-waypoint square at 3 m altitude. It is used for the first flight of each session.

### Code changes

#### Prepared before Session 1: the scorer

The lead prepares this, with Claude's help if wanted (about 45 min). It's test tooling rather than flight code, and Session 1 needs it working to score the first run.

1. **`m1_analyze`:** a post-flight scorer.
   ```
   ros2 run milestone1_singlevehicle m1_analyze <flight.ulg> --stage sitl|hitl [--path <yaml>]
   ```
   - It splits the flight into legs wherever the logged `trajectory_setpoint` changes. This is the setpoint the node sent. It corrects `testing.md`, which used `vehicle_local_position_setpoint`. That topic can change continuously, which would make leg splitting unreliable.
   - It scores each leg's measures against the thresholds in the path YAML and writes a PASS/FAIL report to `log/m1_reports/`.
   - It needs `pip install pyulog`.
2. **Unit tests** for the scoring maths, using made-up flight data.
3. **The path files:**
   - `milestone1_path.yaml`: the full path above, with per-waypoint `yaw`, `hold_s` and `tests` IDs, plus the SITL and HITL `thresholds`.
   - `milestone1_path_square.yaml`.

#### Written in Session 1: node changes

In pairs, in `offboard_control_solution.py` and the launch file:

| Pair | Change |
|---|---|
| A | **Yaw and hold.** Read `yaw` (degrees) and `hold_s` per waypoint. Send yaw in `TrajectorySetpoint.yaw`, converted to radians. A waypoint is reached when the drone is within 0.5 m **and** its heading (`VehicleLocalPosition.heading`) is within 5°. The node then holds there for `hold_s` before moving on. |
| B | **S1 and path selection.** Subscribe to `vehicle_status` again. Resend arm and offboard once per second until the drone reports armed and in Offboard mode. Log the confirmation. If it isn't confirmed after 10 s, log an error and stop. Add a `path_file` parameter (default `milestone1_path.yaml`) and a matching `path` launch argument. |

Both changes are in the same file, so pair B rebases on pair A's change before the review.

#### Decisions to make before Session 1

- **Agree the pass thresholds** in the guide. They're still marked "proposed".
- **Agree the speed limits.** Use the same values in SITL and HITL so the results are comparable:

  | Parameter | PX4 default | For these tests |
  |---|---|---|
  | `MPC_XY_VEL_MAX` | 12 m/s | 2 m/s |
  | `MPC_Z_VEL_MAX_UP` | 3 m/s | 1 m/s |
  | `MPC_Z_VEL_MAX_DN` | 1.5 m/s | 1 m/s |
  | `MPC_YAWRAUTO_MAX` | 60°/s | 45°/s |

### Session 1: SITL, build (90 min)

**Roles:**
- **Pairs A and B:** write the node changes.
- **Operator:** sets up SITL and validates the scorer while the pairs code.
- **Recorder:** notes decisions and follow-ups, and commits.

| Time | Activity |
|---|---|
| 0:00–0:10 | Pull the branch, `colcon build`, `colcon test`. Start SITL headless, and set the speed limits in the PX4 console with `param set`. Run `param show` after the next SITL restart to confirm they were kept. |
| 0:10–0:50 | **Pairs A and B write the node changes.** Meanwhile the operator flies the *current* node on `milestone1_path_square.yaml` (pass it with `path:=` once pair B's change is in, or temporarily swap the file) and runs `m1_analyze` on the `.ulg`. This checks the scorer on a real flight before the new node is ready. |
| 0:50–1:05 | Code review of both changes as a group. Fix anything that blocks flying; log everything else as follow-ups. |
| 1:05–1:15 | **Smoke test:** fly `milestone1_path_square.yaml` with the new node. Confirms S1, T1, T3–T5, L1 and the hold behaviour. |
| 1:15–1:25 | **First full-path run**, scored with `m1_analyze`. |
| 1:25–1:30 | Commit. List anything that failed, and who fixes it before Session 2. |

**If running behind:** skip the first full-path run. It becomes the first run of Session 2.

### Between Sessions 1 and 2

- Fix anything from Session 1's failure list.
- If any measure failed because the threshold looks unrealistic rather than because of a bug, raise it with the team before Session 2. Don't change thresholds silently.

### Session 2: SITL, runs (90 min)

**Roles:**
- **Operator:** runs SITL and the node.
- **Analyst:** runs `m1_analyze` and checks the numbers.
- **Recorder:** fills in the results log and commits.
- **Everyone else:** watch QGroundControl and the node logs.

| Time | Activity |
|---|---|
| 0:00–0:10 | Pull, build, start SITL. Check the speed limits are still set. |
| 0:10–0:40 | **Full path × 3 consecutive runs.** Restart SITL between runs, so each run takes off from the origin and T1 is a pure climb. Score each run as soon as it lands. Any failure restarts the count. |
| 0:40–0:50 | **F1 run:** `Ctrl-C` the node mid-path. Record the failsafe action PX4 took. |
| 0:50–1:10 | **Buffer / stretch.** Fix and re-run anything that failed. If all passed, optionally try V1 (velocity mode) in SITL. It isn't required for Milestone 1. |
| 1:10–1:30 | Fill in the results log, make the go/no-go call for HITL, commit, and assign the HITL prerequisites. |

**Go to HITL only if:**
- 3 consecutive full-path runs pass every in-scope row within the SITL thresholds
- The F1 action is recorded and is acceptable for hardware (hold or land, not something unexpected)
- No open bugs in the node

If these aren't met, use the first part of Session 3 indoors for the fixes and the bench test, and only fly if there's time for the full Session 3 flight plan below.

### Before Session 3: HITL prerequisites

These should be done **before** Session 3, so the session is spent flying.

| Task | Notes |
|---|---|
| Fill in the guide's 3 HITL `TODO`s | Flight controller link (serial or UDP), the matching `MicroXRCEAgent` command, the drone's PX4 version, airframe notes |
| **Indoor bench test, props off** (~30 min) | Connect, check that the `/fmu/out/...` topic names match what the node uses (they may differ on the drone's PX4 version), and run the node to confirm S1: it arms and enters Offboard mode. Disarm. If this can't be done beforehand, it's the first block of Session 3 and Flight 5 is cut. |
| Set parameters on the drone | The speed limits above. Check `MPC_TILTMAX_AIR`, `COM_OBL_RC_ACT` and `COM_OF_LOSS_T` (default 1 s), and set a geofence of about 20 m horizontal and 10 m vertical. |
| Logistics | Test site booked, weather checked, at least 6 charged batteries, a safety pilot confirmed, and the scoring laptop with `pyulog` installed |

### Session 3: HITL (3 h)

**Roles:**
- **Safety pilot:** holds the RC transmitter. This is their only job.
- **Operator:** runs the ground station and the node.
- **Spotter:** watches the area and calls abort.
- **Analyst/recorder:** pulls logs, runs `m1_analyze`, fills in the results log.

Rotate the operator, spotter and analyst roles between flights so more of the team gets hands-on time. The safety pilot doesn't rotate.

| Time | Activity |
|---|---|
| 0:00–0:30 | Set up at the site, then run the guide's every-flight checklist: area clear, position lock, RC and kill switch tested, agent connected, topics present. Confirm the drone's parameters (speed limits, failsafes, geofence). |
| 0:30–0:45 | **Flight 1:** `milestone1_path_square.yaml` at 3 m. Pull the log, run a quick `m1_analyze`, and decide whether to fly the full path. |
| 0:45–1:00 | **Flight 2:** full path. Score it before the next flight. |
| 1:00–1:15 | **Flight 3:** F1. Stop the node mid-path, with the safety pilot ready to take over. |
| 1:15–1:30 | Break. Swap batteries, check the airframe (props, motor temperatures, mounts), and review scores so far. |
| 1:30–1:45 | **Flight 4:** full path repeated |
| 1:45–2:00 | **Flight 5:** full path repeated again. This gives 3 full-path flights to compare with SITL. |
| 2:00–2:30 | **Buffer:** re-fly anything that failed or was aborted (weather, position lock, takeover). |
| 2:30–3:00 | Pull all `.ulg` logs, score with `--stage hitl`, fill in the results log, debrief, pack up, commit |

**If running behind:** cut Flight 5 first, then Flight 4. The order of priority is the square, then the full path, then F1.

**Abort the session** if wind is strong enough to affect the hover, position lock is poor, or the safety pilot has to take over twice.

**HITL passes if** every in-scope row is within the HITL thresholds in at least one full-path flight (preferably in all of them), and F1 matches the SITL result.

### Risks

| Risk | Mitigation |
|---|---|
| Scorer not ready before Session 1 | The pairs still write the node changes; scoring moves into Session 2's buffer, and the 3 runs are scored afterwards |
| Node changes take longer than 40 min in Session 1 | Cut the first full-path run. Session 2's buffer absorbs the rest. |
| Topic names or the link differ on the drone | The indoor bench test before Session 3 |
| PX4's default speed (12 m/s) on old airframes | The speed limits above, the same in all sessions |
| SITL fails thresholds | Tune the node, or relax a threshold with the team's agreement and record why. Don't change thresholds silently. |
| Weather or site problems | Book a backup date for Session 3. Run Flight 1 as early as possible. |

### Done when

- SITL and HITL checklists in the guide are ticked for all in-scope rows
- The results log contains the `m1_analyze` reports and flight logs for each run
- V1, R2 and the live monitor are logged as Milestone 1 follow-ups
