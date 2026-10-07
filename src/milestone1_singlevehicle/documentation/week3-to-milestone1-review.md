# Review: week 3 onboarding → Milestone 1 package

This document covers every change from the week 3 onboarding package to the current `milestone1_singlevehicle` package. It's for reviewing the changes before Milestone 1 testing starts.

| | Ref |
|---|---|
| **Before** | `src/week3_offboard_control` on `dev-test` (`7886bd9`) |
| **After** | `src/milestone1_singlevehicle` on `milestone2-multivehicle` (`ae47e50`), plus the uncommitted changes listed in [Not yet committed](#not-yet-committed) |

To see the full code diff yourself:

```
git diff -M origin/dev-test:src/week3_offboard_control HEAD:src/milestone1_singlevehicle
```

## Summary

- **Flight behaviour is unchanged.** The control loop, arm and offboard timing, topics, QoS, acceptance radius and the flight path itself are the same as in the week 3 reference solution.
- **The package is now standalone.** It no longer needs `week2_px4_sitl` to start SITL, and Gazebo runs headless by default.
- **The path comes from a YAML file.** The hardcoded `WAYPOINTS` list is gone, and the node reads `resource/milestone1_path.yaml` at startup.
- **The onboarding exercise is gone.** The stub node and its launch file were removed, and the week 3 lesson guide was replaced by the Milestone 1 test guide.

## Commits

| Commit | Message |
|---|---|
| `d34e01c` | Add milestone1_singlevehicle package from week3 onboarding |
| `000fb63` | Set up milestone1_singlevehicle as the Milestone 1 SITL/HITL test |
| `ae47e50` | Added waypoint loading from .yaml file |

## Changes by file

### Renamed (no functional change)

| Before | After |
|---|---|
| Package `week3_offboard_control` | `milestone1_singlevehicle` (in `package.xml`, `setup.py`, `setup.cfg` and both launch files) |
| `week3_offboard_control/` (Python package) | `milestone1_singlevehicle/` |
| `resource/week3_offboard_control` (ament marker) | `resource/milestone1_singlevehicle` |
| `resource/waypoints.yaml` | `resource/milestone1_path.yaml` |

`milestone1_singlevehicle` uses an underscore because ROS 2 package names can't contain hyphens.

### Removed

| File | Why |
|---|---|
| `week3_offboard_control/offboard_control_stub.py` | Onboarding exercise with `TODO`s; not needed for the milestone |
| `launch/offboard_stub.launch.py` | Launched the stub |
| `offboard_stub` entry point in `setup.py` | Pointed at the stub |
| `week-3-offboard-control.md` | Week 3 lesson guide; replaced by `milestone-1-single-vehicle.md`. The original is still on `dev-test`. |
| `__pycache__/*.pyc` (5 files) | Compiled Python files that had been committed on `dev-test` |

### Added

| File | What it does |
|---|---|
| `launch/px4_sitl.launch.py` | Starts the Micro XRCE-DDS Agent and PX4 SITL (`gz_x500`), each in its own `gnome-terminal`. Adapted from `week2_px4_sitl`. New `headless` argument, default `true`: PX4 is started with `HEADLESS=1`, so no Gazebo window opens. |
| `milestone-1-single-vehicle.md` | Milestone 1 test guide: what the test flies, SITL and HITL procedures, safety checklist, pass criteria and results log. |

### `milestone1_singlevehicle/offboard_control_solution.py`

| Change | Effect on flight |
|---|---|
| Hardcoded `WAYPOINTS` list and the commented-out YAML loader replaced by `load_waypoints()`, which reads `milestone1_path.yaml` from the installed package | None: same 4 waypoints. If the file is empty or an entry isn't `[x, y, z]`, the node now stops with a `ValueError` at startup, before it arms. |
| Logs `loaded N waypoints from milestone1_path.yaml` at startup | Log only |
| Removed the `vehicle_status` subscription, its `on_status` handler and the `VehicleStatus` import | None: the value was stored but never read |
| Docstring and comments updated | None |

**Unchanged:**
- `on_timer()`, `advance_waypoint_if_reached()` and all publish helpers
- QoS profile and topic names
- `ACCEPTANCE_RADIUS` (0.5 m) and `ARM_AFTER_TICKS` (10)
- The 10 Hz timer

### `launch/offboard_solution.launch.py`

- Includes this package's own `px4_sitl.launch.py` instead of `week2_px4_sitl`'s. Before this change, the default launch failed in this workspace, because `week2_px4_sitl` isn't here.
- New `headless` argument (default `true`), passed through to the SITL launch.
- Package names and the docstring updated.
- **Unchanged:** the 20 s SITL start-up delay, the `launch_sitl` argument, and the node running in its own `gnome-terminal`.

### `package.xml`, `setup.py`, `setup.cfg`

- **Dependencies:** removed `week2_px4_sitl` and added `ament_index_python`, which `load_waypoints()` needs. `python3-yaml` was already listed.
- **Names:** package name, description and install paths updated.
- **Entry points:** only `offboard_solution` is installed now.

### `resource/milestone1_path.yaml`

Only the header comment changed. The waypoints are the same: `(0,0,-5) → (5,0,-5) → (5,5,-5) → (0,5,-5)`.

## Not yet committed

| File | State |
|---|---|
| `milestone-1-single-vehicle.md` | Edited but not committed (117 lines added). Adds the freedom-of-flight test matrix, the proposed pass thresholds, the recording and measuring steps, and the test order. Updates the pass criteria and the results log to match. |
| `flight-scoring-plan.md` | New, not committed. A plan for automatic pass/fail scoring of test flights. Only the plan document exists; none of the plan has been built. |
| `week3-to-milestone1-review.md` | This document. |

## What's been verified

- `colcon build --packages-select milestone1_singlevehicle` succeeds.
- `ros2 pkg executables` shows only `offboard_solution`.
- Both launch files show their arguments (`launch_sitl`, `headless`) with `--show-args`.
- `load_waypoints()` returns the same 4 waypoints as the old hardcoded list.
- The node starts, logs `loaded 4 waypoints from milestone1_path.yaml`, and sends its arm and offboard commands. This was checked with no SITL running.

**Not yet verified:**
- A full SITL flight with the renamed package and headless launch
- Any hardware flight

## Points for reviewers

**Inherited from week 3, still true:**

1. **The node arms ~1 s after starting, with no checks.** It doesn't confirm that it actually armed or entered Offboard mode. This is acceptable in SITL, but on hardware the RC safety pilot is the only protection. The guide's HITL section says this.
2. **No hover time at waypoints.** The node moves on as soon as it's within 0.5 m, so overshoot, settling and hover drift can't be measured. The test matrix lists this as a needed node change (H1).
3. **Yaw is fixed at 0, and descent is done by PX4's land mode,** not under offboard control. These limit which matrix rows the current path can test.
4. **Topic names are hardcoded**, e.g. `/fmu/out/vehicle_local_position_v1`. They may differ on the drones' PX4 version.
5. **The SITL launch assumes PX4 is at `~/PX4-Autopilot`,** and waits a fixed 20 s for it to start.
6. **`package.xml` lists lint test dependencies** (`ament_flake8`, `ament_pep257`, `ament_copyright`), but the package has no `test/` folder, so `colcon test` runs nothing.

**New with these changes:**

7. **"Headless" only means no Gazebo window.** PX4, the agent and the node still open `gnome-terminal` windows, so a desktop session is required. It won't run over plain SSH.
8. **The path is read from the installed copy** of `milestone1_path.yaml`. Edits need a `colcon build` before they take effect.
9. **The week 3 lesson material is no longer in this package.** Anyone onboarding needs `src/week3_offboard_control` on `dev-test`.
10. **The guide's HITL section has three `TODO`s:** the flight controller link and agent command, the PX4 version, and airframe notes. They need someone who knows the drones.

## Reviewer checklist

- [ ] Rename is complete (no `week3_offboard_control` references left: `grep -rn week3_offboard_control src/milestone1_singlevehicle`)
- [x] Removing the stub and the onboarding guide from this package is intended
- [x] Headless as the default is the right choice
- [ ] Startup error on a bad path file is acceptable behaviour
- [ ] Reviewer points 1–10 are accepted or turned into follow-up tasks
- [ ] Uncommitted guide changes reviewed before they're committed
- [ ] SITL flight run with the current package before HITL
