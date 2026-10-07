# Week 2: PX4 SITL + Gazebo Setup

**Dates:** Sep 22-28, 2026
**Outcome:** Everyone can launch SITL + Gazebo and see PX4 topics in ROS2.

## Topics

- What SITL (Software-In-The-Loop) is and why we simulate before flying real hardware
- PX4 + Gazebo setup and basic simulated flight (manual takeoff/land via QGroundControl or CLI)
- The PX4 <-> ROS2 bridge (uXRCE-DDS) — how PX4 topics show up as ROS2 topics
- MAVLink basics — enough to understand what's flowing under the hood, not a deep dive
- Tour of this repo's structure: `px4_msgs`, workspace layout, how the build/install/src dirs relate

## Hands-on tasks

1. Build and launch PX4 SITL with Gazebo; get a drone airborne manually via QGroundControl.
2. Start the uXRCE-DDS agent and confirm PX4 topics (e.g. vehicle status, position) appear under `ros2 topic list`.
3. Subscribe to a PX4 topic (e.g. `/fmu/out/vehicle_local_position_v1`) from a ROS2 node and print incoming data.
4. Walk through this workspace's directory layout with a returning member; identify where `px4_msgs` message definitions live.

## Instructions

A `week2_px4_sitl` package is provided at [`src/week2_px4_sitl`](.) with the automated versions of tasks 1-3. As in week 1, do each task by hand first so you understand what's actually happening, then use the provided launch file as the "now do this in one step" shortcut.

### Task 1 — Build and launch PX4 SITL with Gazebo

**By hand**, in its own terminal:
```
cd ~/PX4-Autopilot
PX4_SYS_AUTOSTART=4001 make px4_sitl gz_x500
```
This builds and starts PX4 SITL with a Gazebo `gz_x500` quadrotor model (magnetometer + battery included via that autostart ID). Leave this terminal open — it's your PX4 console (`pxh>` prompt).

In a second terminal, start QGroundControl:
```
flatpak run org.mavlink.qgroundcontrol
```
QGC should auto-connect to the SITL instance. Once connected, arm and take off manually (QGC's takeoff/arm buttons, or fly it with a virtual joystick), then land.

**Automated** (does this step and task 2 together): see below.

### Task 2 — Start the uXRCE-DDS agent, confirm PX4 topics in ROS2

**By hand**, in a third terminal (alongside the PX4 console from task 1):
```
MicroXRCEAgent udp4 -p 8888
```
This bridges PX4's internal uORB topics to ROS2 over DDS. With it running, confirm in a fourth terminal:
```
ros2 topic list | grep fmu
ros2 topic echo /fmu/out/vehicle_local_position_v1
```
You should see a stream of `/fmu/...` topics, and `vehicle_local_position_v1` messages arriving as the simulated drone sits/flies. (The `_v1` suffix is PX4 republishing this message under a versioned topic name via its `translation_node`, rather than the bare `/fmu/out/vehicle_local_position` — check `ros2 topic list | grep vehicle_local_position` if this ever stops matching after a PX4 update.)

**Automated**: once you've done tasks 1 and 2 by hand and understand what each process is doing, use the provided launch file to start both PX4 SITL and the agent together, each in its own terminal (same terminal-launch pattern as week 1 and the archive's `px4_sitl_with_model.launch.py`):
```
colcon build --packages-select week2_px4_sitl
source install/setup.bash
ros2 launch week2_px4_sitl px4_sitl.launch.py
```

### Task 3 — Subscribe to a PX4 topic from a ROS2 node

**By hand** (recap from task 2): `ros2 topic echo /fmu/out/vehicle_local_position_v1` already shows you the raw data from the CLI.

Now do the same thing from code. Read [`listener.py`](week2_px4_sitl/listener.py) — note the QoS profile (`BEST_EFFORT` reliability, `TRANSIENT_LOCAL` durability): PX4 publishes with this QoS over the DDS bridge, and a subscriber using ROS2's default QoS will silently receive nothing, which is a common first stumbling block. Then run it:
```
ros2 launch week2_px4_sitl listener.launch.py
```
(requires PX4 SITL + the agent already running from task 1/2). Compare its logged output to what `ros2 topic echo` showed you by hand.

### Task 4 — Workspace directory tour

Pair with a returning member and walk through, out loud:
- `src/px4_msgs/msg/` — where PX4's ROS2 message definitions (like `VehicleLocalPosition.msg`) live, and how a `.msg` file becomes a Python-importable type (`from px4_msgs.msg import VehicleLocalPosition`)
- `src/week1_talker_listener/` and `src/week2_px4_sitl/` — the shape of an `ament_python` package: `package.xml`, `setup.py`, `launch/`, and the actual node code
- `build/`, `install/`, `log/` — what `colcon build` generates in each, and why only `src/` (plus `PLAN.md`, `onboarding/`) is meant to be hand-edited
- `PLAN.md` and `onboarding/` — how this week's work fits into the rest of the year's milestones

## Checkpoint

Each new member independently launches SITL + Gazebo, gets a simulated drone flying manually, and shows PX4 data arriving in ROS2 via a subscriber node they wrote.

## Notes

Curriculum is a draft. If setup friction eats the whole session, that's useful signal to simplify week 2 next cycle — flag it in PLAN.md's open items rather than pushing through.
