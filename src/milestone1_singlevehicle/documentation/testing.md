# Testing the Milestone 1 package

How to run the tests for `milestone1_singlevehicle`: the flight scorer's unit tests, scoring a recorded flight log, and scoring a fresh SITL flight end to end.

For what the scorer measures and how each matrix row is decided, see [`flight-scoring-plan.md`](flight-scoring-plan.md) and the test matrix in [`milestone-1-single-vehicle.md`](milestone-1-single-vehicle.md).

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

Flies the node in SITL and scores the log it produces.

> **Temporary workaround, until Session 1.** The node can only read plain `[x, y, z]` waypoints. The current `milestone1_path.yaml` uses the new `{pos: ...}` format, so the node can't load it until the node changes from Session 1 are in. Until then, this test replaces the *installed* copy of the path file with a plain version. Your source file in `src/` is not touched, and the next `colcon build` puts the installed copy back.
>
> This only works if the workspace was built **without** `--symlink-install`. With symlinks, writing to the installed copy would overwrite your source file. Check with `ls -l install/milestone1_singlevehicle/share/milestone1_singlevehicle/resource/`: the file must not show `->`.

**Step 1.** Write a plain path file and put it in place of the installed copy:

```
cat > /tmp/m1_sitl_square.yaml <<'EOF'
waypoints:
  - [0.0, 0.0, -5.0]
  - [5.0, 0.0, -5.0]
  - [5.0, 5.0, -5.0]
  - [0.0, 5.0, -5.0]
thresholds:
  sitl: {position_error_m: 0.5, overshoot_m: 0.5, settle_time_s: 3, cross_track_m: 1.0, heading_error_deg: 5, hover_drift_m: 0.2, tilt_max_deg: 45}
  hitl: {position_error_m: 0.5, overshoot_m: 1.0, settle_time_s: 5, cross_track_m: 2.0, heading_error_deg: 10, hover_drift_m: 1.0, tilt_max_deg: 45}
EOF
cp /tmp/m1_sitl_square.yaml ~/px4_ros_com_ws/install/milestone1_singlevehicle/share/milestone1_singlevehicle/resource/milestone1_path.yaml
```

The node ignores the `thresholds` section, and the scorer accepts plain waypoints, so both can use this file.

**Step 2.** Fly:

```
ros2 launch milestone1_singlevehicle offboard_solution.launch.py
```

This opens the XRCE agent, PX4 SITL (headless) and the node in their own terminals, so it needs a desktop session. Wait until the node logs `reached final waypoint, landing` and the drone has landed and disarmed. If it never arms, PX4 was still starting: rerun with `launch_sitl:=false`.

**Step 3.** Score the newest log. PX4 starts a new log at each arming:

```
ros2 run milestone1_singlevehicle m1_analyze \
  "$(ls -t ~/PX4-Autopilot/build/px4_sitl_default/rootfs/fs/log/*/*.ulg | head -1)" \
  --path /tmp/m1_sitl_square.yaml
```

**Expected:**
- S1, L1 and R1 PASS.
- 4 legs in the legs table, all `Reached: yes`, with cross-track and tilt values. Overshoot, settle time and position error show `—`, because the node doesn't hold at waypoints yet.
- No notes about setpoints missing from the path file.
- These waypoints have no test IDs, so the T rows don't appear in the matrix table. Only S1, L1, R1 and F1 do. That's expected for this temporary file.

**Step 4.** Put the real path file back once you're done with level 4:

```
cd ~/px4_ros_com_ws && colcon build --packages-select milestone1_singlevehicle
```

## 4. Failsafe and takeover runs

Use the same setup as level 3, with the temporary path file still installed. Restart SITL between runs, so each run takes off from the origin.

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
