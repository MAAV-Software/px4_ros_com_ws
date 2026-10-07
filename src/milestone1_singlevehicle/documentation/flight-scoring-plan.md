# Plan: automatic pass/fail scoring of Milestone 1 test flights

## Context

`milestone-1-single-vehicle.md` now defines a freedom-of-flight test matrix (T1–T9, Y1–Y2, R1–R2, H1, V1, S1, F1, L1) and SITL/HITL pass thresholds. At the moment, the only way to tell whether a flight met them is to check by hand in PlotJuggler or Flight Review. The goal is for every test flight to produce a pass/fail result per matrix row automatically, in two ways:

- **Live:** a monitor node scores each leg during the flight, prints PASS/FAIL as it goes, and writes a report when the flight ends.
- **Post-flight:** a script re-scores the flight from PX4's `.ulg` log. This is the reference result for HITL, where the ROS link can drop messages.

Both use the same metric code and the same configuration, so they should agree.

**Prerequisite, found while designing this:** the node moves to the next waypoint as soon as it is within 0.5 m. That makes overshoot, settling time and hover drift impossible to measure at any waypoint. A per-waypoint hold time is the one controller change this plan needs.

## Step 0: save this plan in the package

Copy this plan to `src/milestone1_singlevehicle/documentation/flight-scoring-plan.md` as the first step, so it stays in the package's documentation folder with the Milestone 1 guide.

## Configuration: `resource/milestone1_path.yaml`

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

## Shared code: `milestone1_singlevehicle/flight_test.py` (new)

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

### Splitting a flight into legs

Both tools split the flight the same way. A new leg starts whenever the active setpoint changes. The setpoint is matched to the nearest waypoint in the YAML (within 0.1 m), which gives that leg's test IDs. A leg ends at the next setpoint change or when land mode starts. This means neither tool needs changes in the flight node to know which leg is which.

## Live tool: `milestone1_singlevehicle/flight_monitor.py` (new node)

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

## Post-flight tool: `milestone1_singlevehicle/analyze_flight.py` (new command, `m1_analyze`)

```
ros2 run milestone1_singlevehicle m1_analyze <flight.ulg> --stage hitl [--path milestone1_path.yaml]
```

- **Reads the `.ulg` with pyulog.** It is already installed at `~/.local/bin/ulog_info`. There's no rosdep key for it, so the guide will say `pip install pyulog`.
- **Uses these PX4 topics**, all of which PX4 logs by default (`logged_topics.cpp`):
  - `vehicle_local_position_setpoint` (logged every 100 ms): PX4's own record of the active setpoint, with PX4 timestamps. This avoids the ROS-stamped `trajectory_setpoint`.
  - `vehicle_local_position` (100 ms) and `vehicle_attitude` (50 ms).
  - `vehicle_status`.
- **Output:** the same splitting into legs and the same `score_leg` and `score_events` as the live tool, and the same report format, so the two reports can be compared directly.

## Changes to existing files

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

## Out of scope (unchanged)

Yaw setpoints (Y1, Y2), velocity mode (V1), attitude mode (R2) and extending the path to T2/T6–T9. The scoring code already handles heading, so Y rows will score once yaw is added to the node, with no further changes.

## Verification

1. **Unit tests** (`test/test_flight_test.py`, run with `colcon test --packages-select milestone1_singlevehicle`): feed made-up flight data with known overshoot, drift, sideways deviation and tilt into `score_leg` and `score_events`. Check that each measure comes out as expected and the PASS/FAIL boundaries are right. Also check that `load_config` rejects bad YAML, and still accepts plain `[x, y, z]` entries.
2. **SITL with the live monitor:** `ros2 launch milestone1_singlevehicle offboard_solution.launch.py`. Check that the monitor logs one line per leg, that the drone holds at each waypoint for `hold_s`, and that a report appears in `log/m1_reports/` with T1, T3, T4, T5, H1, R1, S1 and L1 scored.
3. **Post-flight check on the same flight:** run `m1_analyze` on the newest `.ulg` in `~/PX4-Autopilot/build/px4_sitl_default/rootfs/fs/log/`. Its PASS/FAIL results should match the live report, and its numbers should be within about 0.1 m and 0.5 s.
4. **F1:** rerun, `Ctrl-C` the flight node mid-path, and check that both reports name the failsafe mode PX4 switched to.
5. **Failure case:** temporarily set `settle_time_s: 0.1` in the YAML and rebuild. Both tools should report FAIL for T rows.
