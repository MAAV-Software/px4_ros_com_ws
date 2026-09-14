# px4_ros_com_ws — Starter Environment

Minimal PX4 + ROS 2 dev environment. This workspace intentionally contains
just one package, `px4_msgs` (the standard PX4 ROS 2 message definitions) —
no custom control code, no missions, no multi-vehicle logic. The goal is to
get a single default drone flying in simulation and to see it talking to
ROS 2, using nothing but manual terminal commands.

Prior project work (multi-vehicle offboard control packages, the web ground
station, old build artifacts) is not part of this starter state.

## Prerequisites

Already set up on this machine:
- ROS 2 Humble (`/opt/ros/humble`)
- PX4-Autopilot, built, at `~/PX4-Autopilot` (`make px4_sitl` already run once)
- Gazebo (`gz sim`)
- Micro XRCE-DDS Agent (`MicroXRCEAgent`)

## Build

```bash
cd ~/px4_ros_com_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select px4_msgs
```

## Manually launch one default drone

Open three terminals.

**Terminal 1 — Micro XRCE-DDS Agent** (bridges PX4's uORB topics to ROS 2):
```bash
MicroXRCEAgent udp4 -p 8888
```

**Terminal 2 — PX4 SITL + Gazebo** (spawns one default `x500` quadrotor):
```bash
cd ~/PX4-Autopilot
make px4_sitl gz_x500
```
This opens the Gazebo GUI with the drone on the ground and drops you into
the PX4 shell (`pxh>`). From that shell you can fly it manually with no
other tooling at all:
```
pxh> commander takeoff
pxh> commander land
```

**Terminal 3 — confirm the ROS 2 bridge is alive:**
```bash
cd ~/px4_ros_com_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 topic list | grep fmu
ros2 topic echo /fmu/out/vehicle_status_v4
```
You should see PX4's `/fmu/out/...` and `/fmu/in/...` topics, backed by the
message types in `src/px4_msgs`. (Topic names carry a version suffix, e.g.
`_v4` — if a name doesn't match after a PX4/px4_msgs update, `ros2 topic
list | grep fmu` will show the current one.)

That's the whole starter loop: one drone, one manual takeoff, ROS 2 topics
visible. Everything else (offboard control nodes, waypoint missions, the
web ground station, multi-drone swarms) is deliberately not here.
