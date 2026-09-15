# Week 3: Offboard Control Basics

**Dates:** Sep 29-Oct 5, 2026
**Outcome:** Each new member flies a single simulated drone through a scripted waypoint path.

## Topics

- Offboard control mode: what it is, why PX4 requires a steady setpoint stream to enter/stay in it
- Arming, takeoff, and landing via ROS2 (not manual QGroundControl control)
- Sending position/velocity setpoints via `px4_msgs`
- Waypoint navigation: sequencing multiple setpoints into a path
- Common failure modes: setpoint stream dropouts, mode rejections, frame/coordinate mistakes (NED vs ENU)

## Hands-on tasks

1. Write a ROS2 node that arms the simulated drone and commands takeoff via offboard setpoints.
2. Extend it to fly to a single target waypoint and hold.
3. Extend further to fly a short sequence of waypoints (a simple scripted path) and land.
4. Deliberately break the setpoint stream (e.g. stop publishing) and observe/explain PX4's failsafe behavior.

## Instructions

A `week3_offboard_control` package is provided at [`src/week3_offboard_control`](.) with two node files:

- [`offboard_control_stub.py`](week3_offboard_control/offboard_control_stub.py) — all the publisher/subscriber/message plumbing is wired up for you, but the actual control logic (`on_timer()`) is left as `TODO`s mapped to tasks 1-3. **This is the file you write.**
- [`offboard_control_solution.py`](week3_offboard_control/offboard_control_solution.py) — a complete reference implementation. Don't open this until you've made a real attempt at the stub — it exists to check your work against, not to copy from.

Unlike weeks 1-2, there's no manual-CLI warm-up this week: offboard control requires a steady ≥2 Hz setpoint stream just to be accepted, which isn't practical to fake with one-off `ros2 topic pub` commands, so you go straight to writing a node.

Before starting, bring up PX4 SITL + the XRCE agent from week 2:
```
ros2 launch week2_px4_sitl px4_sitl.launch.py
```

### Task 1 — Arm and take off

Open `offboard_control_stub.py`. Note the constants at the top (`WAYPOINTS`, `ACCEPTANCE_RADIUS`, `ARM_AFTER_TICKS`) and the already-written helpers (`publish_offboard_control_mode`, `publish_trajectory_setpoint`, `arm`, `engage_offboard_mode`, `land`) — you're filling in `on_timer()`, not writing message plumbing from scratch.

In `on_timer()`:
1. Every tick, call `self.publish_offboard_control_mode()` and `self.publish_trajectory_setpoint(target)` with some target position (straight up is fine for now, e.g. `(0.0, 0.0, -5.0)` in NED — negative z is up).
2. Once you've been streaming for `ARM_AFTER_TICKS` ticks, call `self.arm()` then `self.engage_offboard_mode()`. PX4 rejects the offboard mode switch if it hasn't already seen a steady setpoint stream — that ordering matters.

Build and run:
```
colcon build --packages-select week3_offboard_control
source install/setup.bash
ros2 launch week3_offboard_control offboard_stub.launch.py
```
Watch the PX4 console (from the SITL terminal) and/or QGroundControl to confirm it arms and climbs.

### Task 2 — Fly to a waypoint and hold

Set `WAYPOINTS = [(0.0, 0.0, -5.0)]` (or wherever you want it to hover) and make `on_timer()` publish that as the target. Rebuild and rerun — it should take off, fly to that point, and hold there indefinitely.

### Task 3 — Multi-waypoint path and landing

Extend `WAYPOINTS` to a short hardcoded list, e.g.:
```python
WAYPOINTS = [
    (0.0, 0.0, -5.0),
    (5.0, 0.0, -5.0),
    (5.0, 5.0, -5.0),
    (0.0, 5.0, -5.0),
]
```
In `on_timer()`, once `self.current_position` is populated (from the `vehicle_local_position` subscription), compute the distance to the current target and advance `self.waypoint_index` when it's within `ACCEPTANCE_RADIUS`. After the last waypoint, call `self.land()`.

Rebuild and run the same way as task 1. Once it completes a full lap and lands on its own, compare your `on_timer()` against `offboard_control_solution.py`'s — same idea, but you can run the solution directly with `ros2 launch week3_offboard_control offboard_solution.launch.py` if you want to see a known-working version fly first.

### Task 4 — Break the setpoint stream

No code changes needed. With your (or the solution's) node mid-flight, `Ctrl-C` the node's terminal to kill the setpoint stream outright. Watch the PX4 console / QGroundControl: PX4 should drop out of offboard mode and trigger its failsafe behavior (typically hold or RTL, depending on parameters). Be ready to explain, in your own words, why PX4 does this rather than just continuing on the last setpoint forever.

### Extension (optional) — load waypoints from YAML

A [`resource/waypoints.yaml`](resource/waypoints.yaml) file is provided with the same path as task 3's hardcoded list. As a stretch goal, modify your node to load `WAYPOINTS` from this file at runtime (via `ament_index_python.packages.get_package_share_directory('week3_offboard_control')` + `yaml.safe_load`) instead of hardcoding it in Python — this is the more realistic shape for how a real mission's waypoints would be configured, separate from the control code itself.

## Checkpoint

Each new member demos a single simulated drone autonomously arming, flying a multi-waypoint path, and landing — no manual QGroundControl intervention. This is effectively a first pass at Milestone 1 from PLAN.md.

## Notes

Curriculum is a draft. This week is the most technically dense one — expect it to run long for some members and plan slack accordingly.
