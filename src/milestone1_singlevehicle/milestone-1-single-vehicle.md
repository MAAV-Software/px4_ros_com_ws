# Milestone 1: 1 Drone Autonomous

**Goal (from [PLAN.md](../../PLAN.md)):** baseline offboard control of a single drone on a single track, validated in sim **and** on one of the 2 old-design drones.

This package is the test for that milestone. It checks freedom of flight and control: can our ROS 2 stack take one drone from the ground, move it in every axis (climb, north, east, back, descend) under offboard control, and hand it back to PX4 safely? It runs in two stages:

1. **SITL:** PX4 SITL + Gazebo, headless by default.
2. **HITL:** the same node, unchanged, on a real airframe.

Pass SITL before you try HITL. Milestone 1 is complete when both stages pass, and Milestone 2 (2 drones) builds on this code.

## What the test flies

[`offboard_control_solution.py`](milestone1_singlevehicle/offboard_control_solution.py) is the week 3 onboarding controller, promoted to the milestone baseline. In order, it:

1. Streams position setpoints at 10 Hz for 1 s (`ARM_AFTER_TICKS = 10`).
2. Sends **arm**, then switches PX4 to **offboard** mode.
3. Takes off to the first waypoint and flies the path in [`resource/milestone1_path.yaml`](resource/milestone1_path.yaml). A waypoint counts as reached within 0.5 m (`ACCEPTANCE_RADIUS`).
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

**Known limits:** single vehicle, no namespacing, no failure recovery, and yaw fixed at 0 (facing north). It doesn't check whether the arm or mode switch succeeded; it just keeps streaming. These limits are acceptable for Milestone 1, and the HITL safety rules below assume them.

## Stage 1: SITL

### Run

```
cd ~/px4_ros_com_ws
colcon build --packages-select milestone1_singlevehicle
source install/setup.bash
ros2 launch milestone1_singlevehicle offboard_solution.launch.py
```

This launch file:

- Opens the Micro XRCE-DDS Agent and PX4 SITL (`gz_x500`) in their own terminals, using [`px4_sitl.launch.py`](launch/px4_sitl.launch.py).
- Runs Gazebo **headless** (`HEADLESS=1`). The sim runs normally, but no Gazebo window opens. Watch the flight in QGroundControl or the PX4 console.
- Waits 20 s for SITL to boot, then starts the node in its own terminal.

Options:

| Argument | Default | Use |
|---|---|---|
| `headless:=false` | `true` | Open the Gazebo GUI to watch the drone |
| `launch_sitl:=false` | `true` | SITL + agent already running; start only the node, immediately |

If the drone never arms on the first run, PX4 was probably still building or booting. Leave SITL up and rerun with `launch_sitl:=false`.

SITL alone (no node) is also available: `ros2 launch milestone1_singlevehicle px4_sitl.launch.py [headless:=false]`.

### SITL pass criteria

Record each run in the [results log](#results-log).

- [ ] Node logs `arm command sent` and `offboard mode command sent`; PX4 console shows armed and in Offboard mode
- [ ] Climbs to ~5 m and holds steady (no oscillation or drift)
- [ ] Logs `reached waypoint 0, advancing to 1` through `reached waypoint 2, advancing to 3`, then `reached final waypoint, landing`
- [ ] Lands and disarms on its own
- [ ] No manual intervention from QGroundControl at any point
- [ ] **Failsafe check:** on a second run, `Ctrl-C` the node mid-path. PX4 leaves Offboard mode and runs its offboard-loss failsafe (hold / land / RTL, depending on the `COM_OBL_RC_ACT` parameter). Note which one happened.
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

- [ ] Clear, open area of at least **15 m × 15 m**. The path is a 5 m square at 5 m altitude, plus margin.
- [ ] Drone placed so that **north (+x)** and **east (+y)** from its start point are clear. The path goes north first.
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
- [ ] Flies all 4 waypoints; path shape matches SITL (north, then east, then south)
- [ ] Lands and disarms on its own
- [ ] No safety-pilot takeover needed
- [ ] **Failsafe check:** stop the node mid-path, and confirm PX4 runs the same failsafe action as in SITL. The safety pilot must be ready.
- [ ] Flight log (`.ulg`) pulled from the flight controller and saved with the results

## Results log

Add a row per run. A milestone stage passes when its checklist is fully ticked.

| Date | Stage | Airframe / PX4 version | Result | Failsafe action seen | Notes / log file |
|---|---|---|---|---|---|
| | SITL | x500 (sim) | | | |
| | HITL | | | | |
