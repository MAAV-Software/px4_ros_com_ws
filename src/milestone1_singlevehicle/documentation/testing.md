# Testing and scoring the Milestone 1 package

How to test `milestone1_singlevehicle` and score its flights: the flight scorer's unit tests, scoring a recorded flight log, and scoring a fresh SITL flight end to end. The last part, [Scoring design](#scoring-design), explains what the scorer measures and how each matrix row is decided.

The test matrix and thresholds are in [`milestone-1-single-vehicle.md`](milestone-1-single-vehicle.md).

## Overview

| Level | What it checks | Needs | Time |
|---|---|---|---|
| [1. Unit tests](#1-unit-tests) | Scoring maths and PASS/FAIL logic, on made-up flights | Built workspace | < 1 min |
| [2. Replay a recorded log](#2-replay-a-recorded-log) | Reading a real `.ulg` and scoring it | A SITL log from an earlier flight | 2 min |
| [3. SITL flight, scored end to end](#3-sitl-flight-scored-end-to-end) | Fly the node in SITL, then score the log it produced | PX4 SITL | 10 min |
| [4. Failsafe and takeover runs](#4-failsafe-and-takeover-runs) | F1 scoring and takeover detection | PX4 SITL, QGroundControl for the takeover run | 10 min |

Run them in order. Each level assumes the ones before it pass.

## One-time setup

```
pip install pyulog
cd ~/px4_ros_com_ws
colcon build --packages-select milestone1_singlevehicle
source install/setup.bash
```

`pyulog` reads PX4 `.ulg` logs. It has no rosdep key, so `colcon build` doesn't install it.

Run `source install/setup.bash` in every new terminal before the commands below.

> **Copying commands:** every command below is either on one line or uses `\` at the very end of a line. Paste them as they are. Don't join lines that end in `\`: on a single line, `\ ` escapes the space and breaks the next argument.

## 1. Unit tests

```
cd ~/px4_ros_com_ws
colcon test --packages-select milestone1_singlevehicle
colcon test-result --verbose
```

**Expected:** `22 tests, 0 errors, 0 failures, 0 skipped`.

The tests are in [`test/test_scoring.py`](../test/test_scoring.py). They feed made-up flights with known answers into the scorer, covering:

- each measure (cross-track, overshoot, settle time, position error, hover drift, heading error, tilt)
- limits being inclusive (a value equal to its threshold passes)
- a clean flight, a failsafe into Hold (F1 passes) or RTL (F1 fails), and a pilot takeover
- a setpoint that isn't in the path file
- path files the scorer must reject

To run them faster while editing, without colcon:

```
cd ~/px4_ros_com_ws/src/milestone1_singlevehicle
python3 -m pytest test/ -q -p no:cacheprovider
```

## 2. Replay a recorded log

Scores a real SITL log from 20 September 2026. That flight used an older path: a 45.72 m square at 4.57 m altitude, with no hover time at the waypoints.

**Step 1.** Create a path file that matches that flight:

```
cat > /tmp/m1_replay_square.yaml <<'EOF'
waypoints:
  - {pos: [0.0, 0.0, -4.57], tests: [T1]}
  - {pos: [45.72, 0.0, -4.57], tests: [T3]}
  - {pos: [45.72, 45.72, -4.57], tests: [T5]}
  - {pos: [0.0, 45.72, -4.57], tests: [T4]}
  - {pos: [0.0, 0.0, -4.57], tests: [T6]}
thresholds:
  sitl: {position_error_m: 0.5, overshoot_m: 0.5, settle_time_s: 3, cross_track_m: 1.0, heading_error_deg: 5, hover_drift_m: 0.2, tilt_max_deg: 45}
  hitl: {position_error_m: 0.5, overshoot_m: 1.0, settle_time_s: 5, cross_track_m: 2.0, heading_error_deg: 10, hover_drift_m: 1.0, tilt_max_deg: 45}
EOF
```

**Step 2.** Score the log:

```
ros2 run milestone1_singlevehicle m1_analyze \
  ~/PX4-Autopilot/build/px4_sitl_default/rootfs/fs/log/2026-09-20/21_42_20.ulg \
  --path /tmp/m1_replay_square.yaml
```

**Expected:**

| Row | Result | Why |
|---|---|---|
| S1, L1, R1 | PASS | Armed into Offboard at once; landed and disarmed; max tilt 44.0° (limit 45°) |
| T1, T3 | NO DATA | The node moved on as soon as it arrived, so overshoot and settle time can't be measured |
| T4, T5, T6 | FAIL | Cross-track error 1.4–2.0 m (limit 1.0 m), from PX4's default 12 m/s top speed |
| F1 | NOT TESTED | No failsafe in this flight |
| **Overall** | **FAIL** | |

The command ends with `[ros2run]: Process exited with failure 1`. That's expected: exit code 1 means the flight didn't pass, not that the tool broke.

**Step 3.** Check the other cases:

| Try | Expected |
|---|---|
| Same command with `--stage hitl` | Looser HITL limits (cross-track 2.0 m): T4, T5 and T6 no longer fail and show NO DATA like T1 and T3, so the overall result is INCOMPLETE |
| Same command without `--path` | Every setpoint reported as "not in the path file", and the overall result is INCOMPLETE. Without `--path`, the scorer uses the installed `milestone1_path.yaml`, which doesn't match this flight. |
| A log path that doesn't exist | `error: [Errno 2] No such file or directory`, exit code 2 |

**Step 4 (optional).** Check the numbers independently. Open the same `.ulg` in [PlotJuggler](https://plotjuggler.io) or [PX4 Flight Review](https://review.px4.io). For one leg, compare the sideways deviation and the maximum tilt by eye with the report. This catches mistakes that the unit tests would share with the scorer.

## 3. SITL flight, scored end to end

Flies the full 22-waypoint path in SITL and scores the log it produces. The node loads the installed `milestone1_path.yaml` (the `{pos, yaw, hold_s, tests}` format), and the scorer reads the same file, so no temporary path file is needed. If you edit the path YAML, rebuild: the node reads the installed copy.

> **Open QGroundControl first.** PX4 refuses to arm with "Preflight Fail: No connection to the GCS" (`Arming denied`), and the node sends its arm command only once, so the flight never starts. Don't work around this by changing `NAV_DLL_ACT`: PX4 saves parameter changes in SITL, so the change persists into later runs.

**Step 1.** Build and source the workspace:

```
cd ~/px4_ros_com_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select milestone1_singlevehicle
source install/setup.bash
```

**Step 2.** Fly. Pick one:

*Launch file* (needs a desktop session). It opens the XRCE agent, PX4 SITL (headless) and the node in their own terminals:

```
ros2 launch milestone1_singlevehicle offboard_solution.launch.py
```

The node waits 20 s for PX4 to boot. If it never arms, PX4 wasn't ready in time: rerun with `launch_sitl:=false`. Add `headless:=false` to show the Gazebo window.

*Manual, three terminals* (also works over SSH or without gnome-terminal):

```
# Terminal 1
MicroXRCEAgent udp4 -p 8888

# Terminal 2: wait for "Ready for takeoff!"
cd ~/PX4-Autopilot && PX4_SYS_AUTOSTART=4001 HEADLESS=1 make px4_sitl gz_x500

# Terminal 3 (workspace sourced as in step 1)
ros2 launch milestone1_singlevehicle offboard_solution.launch.py launch_sitl:=false
```

The full path takes about 3 minutes. Wait until the node logs `Final hold complete; landing` and PX4 logs `Disarmed by landing`.

**Step 3.** Score the newest log. PX4 starts a new log at each arming, and the scorer uses the installed path file by default:

```
ros2 run milestone1_singlevehicle m1_analyze \
  "$(ls -t ~/PX4-Autopilot/build/px4_sitl_default/rootfs/fs/log/*/*.ulg | head -1)"
```

**Expected:**
- F1 is NOT TESTED, because the flight has no offboard-loss failsafe (see level 4).
- S1, L1, R1, H1 and T1 to T9 PASS, and all 22 legs show `Reached: yes`.
- Y1 and Y2 currently FAIL: measured heading error is about 14° (Y1) and 8° (Y2) against the 5° SITL limit. Seen on 2026-10-07, cause not yet investigated.
- T8 sits on its limit: cross-track at waypoint 9 measured 1.00 m and 1.03 m in two runs against a 1.0 m limit, so it can pass or fail between runs.
- The overall result is therefore FAIL until the yaw issue is fixed.

## 4. Failsafe and takeover runs

Use the same setup as level 3 (QGroundControl open). Restart SITL between runs, so each run takes off from the origin.

**Failsafe (F1):**
1. Start the flight as in level 3, step 2.
2. While it's flying between waypoints, press `Ctrl-C` in the node's terminal.
3. PX4 waits `COM_OF_LOSS_T` (1 s by default), then switches to its offboard-loss failsafe action. Wait for the drone to land or hold, then stop SITL.
4. Score the newest log as in level 3, step 3.

**Expected:**
- F1 shows the mode PX4 switched to. Hold or Land is PASS; anything else (e.g. RTL) is FAIL.
- The leg that was cut short shows `INCOMPLETE`, and later waypoints are not flown.
- The overall result counts only F1 and the legs actually flown.

**Takeover:**
1. Start the flight again. Mid-path, switch the drone to Position mode in QGroundControl.
2. Score the newest log.

**Expected:** a note that the drone left Offboard mode for Position without a failsafe ("a safety-pilot takeover or a mode switch from the ground station?"), F1 `NOT TESTED`, and an overall result of INCOMPLETE.

## Reading the results

Every `m1_analyze` run prints a report and saves it as two files in `~/px4_ros_com_ws/log/m1_reports/`. Use `--out <folder>` to save somewhere else.

| File | Contents |
|---|---|
| `<log>_<stage>.md` | The printed report: summary, matrix rows, per-leg measures, notes, and a row to paste into the guide's results log |
| `<log>_<stage>.json` | The same results, machine-readable |

Scoring the same log with the same stage again overwrites both files. `log/` is gitignored.

**Results:**

| Result | Meaning |
|---|---|
| PASS | Every measure for the row is within its limit (equal counts as within) |
| FAIL | A measure is over its limit, or the waypoint was never reached |
| NO DATA | A measure couldn't be computed, e.g. no hover time after arriving |
| INCOMPLETE | The leg was cut short (failsafe, takeover or end of log) |
| NOT FLOWN | The flight never reached this waypoint |
| NOT TESTED | Flight-level row that didn't happen in this flight (e.g. F1 in a normal run) |
| NOT SCORED | Test ID the scorer has no measures for (V1, R2) |

**Exit codes:**

| Code | Meaning |
|---|---|
| 0 | Overall PASS |
| 1 | Overall FAIL or INCOMPLETE |
| 2 | The log or path file couldn't be read |

## Troubleshooting

| Problem | Fix |
|---|---|
| `bash: syntax error near unexpected token 'newline'` | A `<placeholder>` was pasted literally. Replace it with a real file path. |
| `error: Python module "pyulog" is not installed` | `pip install pyulog` |
| `milestone1_singlevehicle is not installed` | `source install/setup.bash`, or pass `--path` |
| Every setpoint "not in the path file" | `--path` doesn't match the path the flight used. Pass the same file the node flew. |
| No log in `rootfs/fs/log/` | The drone never armed, so PX4 didn't log. Check the node and PX4 terminals. |
| Node stops with `ValueError: could not convert string to float: 'pos'` | The node was given the new `{pos: ...}` format. Use the level 3 workaround until Session 1. |
| `colcon test` runs 0 tests | The package wasn't rebuilt after pulling. Run `colcon build --packages-select milestone1_singlevehicle` first. |

## After Session 1

Once the node reads the new path format and holds at waypoints (Session 1, pair A), and has a `path` launch argument (pair B):

- Drop the level 3 workaround. Fly with the real path file, for example `ros2 launch milestone1_singlevehicle offboard_solution.launch.py path:=milestone1_path_square.yaml`.
- Score without `--path`, or with the same file the node flew.
- Overshoot, settle time, position error and hover drift (H1) get real values instead of `—`, and Y1 and Y2 get scored once waypoints have a yaw.

## Scoring design

> **Status:** only the post-flight scorer (`m1_analyze`) and its unit tests are built. The live monitor node and the node's hold-at-waypoint change are still planned. Sections below describe the full design, so where they differ from what's built, the code and the test levels above are correct. One known correction: `m1_analyze` splits legs on the logged `trajectory_setpoint`, not on `vehicle_local_position_setpoint` as written below, because that topic can change continuously and makes leg splitting unreliable.

### Context

`milestone-1-single-vehicle.md` now defines a freedom-of-flight test matrix (T1–T9, Y1–Y2, R1–R2, H1, V1, S1, F1, L1) and SITL/HITL pass thresholds. At the moment, the only way to tell whether a flight met them is to check by hand in PlotJuggler or Flight Review. The goal is for every test flight to produce a pass/fail result per matrix row automatically, in two ways:

- **Live:** a monitor node scores each leg during the flight, prints PASS/FAIL as it goes, and writes a report when the flight ends.
- **Post-flight:** a script re-scores the flight from PX4's `.ulg` log. This is the reference result for HITL, where the ROS link can drop messages.

Both use the same metric code and the same configuration, so they should agree.

**Prerequisite, found while designing this:** the node moves to the next waypoint as soon as it is within 0.5 m. That makes overshoot, settling time and hover drift impossible to measure at any waypoint. A per-waypoint hold time is the one controller change this plan needs.

### Configuration: `resource/milestone1_path.yaml`

As agreed, each leg's test IDs and the thresholds live in the path file. New format:

```yaml
waypoints:
  - {pos: [0.0, 0.0, -5.0], hold_s: 5, tests: [T1]}
  - {pos: [5.0, 0.0, -5.0], hold_s: 5, tests: [T3]}
  - {pos: [5.0, 5.0, -5.0], hold_s: 5, tests: [T5]}
  - {pos: [0.0, 5.0, -5.0], hold_s: 10, tests: [T4, H1]}
thresholds:            # proposed values, copied from the guide's table
  sitl: {position_error_m: 0.5, overshoot_m: 0.5, settle_time_s: 3,
         cross_track_m: 1.0, heading_error_deg: 5, hover_drift_m: 0.2, tilt_max_deg: 45}
  hitl: {position_error_m: 0.5, overshoot_m: 1.0, settle_time_s: 5,
         cross_track_m: 2.0, heading_error_deg: 10, hover_drift_m: 1.0, tilt_max_deg: 45}
```

- Plain `[x, y, z]` entries are still accepted, with `hold_s: 0` and no tests.
- `cross_track_m` (largest sideways deviation from the straight line between waypoints) is a new measure. It is the only way to score movement *during* a leg, so it is added to the guide's threshold table.
- `tilt_max_deg: 45` matches PX4's default `MPC_TILTMAX_AIR`.

### Shared code: `milestone1_singlevehicle/flight_test.py` (new)

Pure Python with numpy and no ROS imports, so both tools and the unit tests can use it.

- **`load_config(path)`:** reads and validates the YAML above. It replaces `load_waypoints()` in the node, so the YAML is read in only one place.
- **`Leg`:** a waypoint index, its target, the previous target, and sample arrays (time, position, heading, tilt).
- **`score_leg(leg, thresholds)`** computes, for each leg:
  - `cross_track_m`: largest perpendicular distance from the line between the previous and current target, before arriving.
  - Arrival time: the first time the drone is within 0.5 m (`ACCEPTANCE_RADIUS`).
  - `overshoot_m`: largest distance from the target after arriving.
  - `settle_time_s`: time from arriving until the error stays within 0.5 m for the rest of the hold.
  - `position_error_m`: mean error over the last 1 s of the hold.
  - `hover_drift_m`: largest distance from the mean hover position after settling.
  - `heading_error_deg`: largest wrapped yaw error after settling.
  - `tilt_max_deg`: from the attitude quaternion, `acos(1 - 2(qx² + qy²))`, over the whole leg.
- **`TEST_MEASURES`:** which measures decide each matrix row. T* rows use position error, overshoot, settle time and cross-track. Y* rows use heading error. H1 uses hover drift. R1 uses tilt on every leg.
- **`score_events(status_samples)`:** checks S1, L1 and F1 from `vehicle_status`:
  - S1: armed and in Offboard mode (`nav_state` 14) within 5 s of the first setpoint.
  - L1: in land mode, then disarmed.
  - F1: the failsafe `nav_state` reached after the setpoint stream stops, reported by name, e.g. "Hold" or "Land".
- **`write_report(results, out_dir, source)`:** writes a JSON file and a Markdown file. The Markdown has a PASS/FAIL table per matrix row and a ready-to-paste row for the guide's results log.

#### Splitting a flight into legs

Both tools split the flight the same way. A new leg starts whenever the active setpoint changes. The setpoint is matched to the nearest waypoint in the YAML (within 0.1 m), which gives that leg's test IDs. A leg ends at the next setpoint change or when land mode starts. This means neither tool needs changes in the flight node to know which leg is which.

### Live tool: `milestone1_singlevehicle/flight_monitor.py` (new node)

- **Subscribes to:**
  - `/fmu/in/trajectory_setpoint`: what the flight node commands.
  - `/fmu/out/vehicle_local_position_v1`
  - `/fmu/out/vehicle_attitude`
  - `/fmu/out/vehicle_status_v1`
  - It uses the same best-effort QoS as the flight node. Topic names are ROS parameters, so they can be changed for a PX4 version with different names.
- **Timing:** each sample is stamped with the time it arrives at the node, which avoids clock mismatches between PX4 and ROS.
- **During the flight:** when a leg ends, it scores the leg and logs, for example, `[T3] PASS  err 0.21 m  overshoot 0.34 m  settle 1.8 s  xtrack 0.40 m`.
- **When the flight ends** (disarmed after landing, or shut down), it scores S1, L1 and F1 and writes the report.
- **Parameters:**
  - `stage`: `sitl` or `hitl`, which picks the threshold set.
  - `path_file`: defaults to the installed `milestone1_path.yaml`.
  - `report_dir`: defaults to `~/px4_ros_com_ws/log/m1_reports`. `log/*` is already in `.gitignore`.

### Post-flight tool: `milestone1_singlevehicle/analyze_flight.py` (new command, `m1_analyze`)

```
ros2 run milestone1_singlevehicle m1_analyze <flight.ulg> --stage hitl [--path milestone1_path.yaml]
```

- **Reads the `.ulg` with pyulog.** It is already installed at `~/.local/bin/ulog_info`. There's no rosdep key for it, so the guide will say `pip install pyulog`.
- **Uses these PX4 topics**, all of which PX4 logs by default (`logged_topics.cpp`):
  - `vehicle_local_position_setpoint` (logged every 100 ms): PX4's own record of the active setpoint, with PX4 timestamps. This avoids the ROS-stamped `trajectory_setpoint`.
  - `vehicle_local_position` (100 ms) and `vehicle_attitude` (50 ms).
  - `vehicle_status`.
- **Output:** the same splitting into legs and the same `score_leg` and `score_events` as the live tool, and the same report format, so the two reports can be compared directly.

### Changes to existing files

- **`offboard_control_solution.py`:**
  - Load the path with `flight_test.load_config`, and delete `load_waypoints()`.
  - Add the hold: when a waypoint is reached, note the time, and only move to the next one after `hold_s` has passed. This is a small change to `advance_waypoint_if_reached()`. Nothing else in the control flow changes.
- **`launch/offboard_solution.launch.py`:** add `monitor` (default `true`) and `stage` (default `sitl`) arguments. When `monitor` is on, start `flight_monitor` with `stage`. It doesn't need a terminal window, so its output goes to the launch console.
- **`setup.py`:** add the `flight_monitor` and `m1_analyze` console scripts.
- **`package.xml`:** add `python3-numpy`, and test dependencies for pytest.
- **`milestone-1-single-vehicle.md`:**
  - New section "Scoring a flight": how to run each tool and how to read the report.
  - Add `cross_track_m` to the thresholds table.
  - Mark H1 as available.
  - HITL run: add `stage:=hitl`, and run `m1_analyze` on the `.ulg` as the reference result.
  - Note that oscillation is still judged by eye.

### Out of scope (unchanged)

Yaw setpoints (Y1, Y2), velocity mode (V1), attitude mode (R2) and extending the path to T2/T6–T9. The scoring code already handles heading, so Y rows will score once yaw is added to the node, with no further changes.

### Verification plan

1. **Unit tests** (`test/test_flight_test.py`, run with `colcon test --packages-select milestone1_singlevehicle`): feed made-up flight data with known overshoot, drift, sideways deviation and tilt into `score_leg` and `score_events`. Check that each measure comes out as expected and the PASS/FAIL boundaries are right. Also check that `load_config` rejects bad YAML, and still accepts plain `[x, y, z]` entries.
2. **SITL with the live monitor:** `ros2 launch milestone1_singlevehicle offboard_solution.launch.py`. Check that the monitor logs one line per leg, that the drone holds at each waypoint for `hold_s`, and that a report appears in `log/m1_reports/` with T1, T3, T4, T5, H1, R1, S1 and L1 scored.
3. **Post-flight check on the same flight:** run `m1_analyze` on the newest `.ulg` in `~/PX4-Autopilot/build/px4_sitl_default/rootfs/fs/log/`. Its PASS/FAIL results should match the live report, and its numbers should be within about 0.1 m and 0.5 s.
4. **F1:** rerun, `Ctrl-C` the flight node mid-path, and check that both reports name the failsafe mode PX4 switched to.
5. **Failure case:** temporarily set `settle_time_s: 0.1` in the YAML and rebuild. Both tools should report FAIL for T rows.
