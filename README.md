# px4_ros_com_ws

A PX4 + ROS 2 workspace for flying a simulated drone and talking to it over
ROS 2.

## The stack

Four pieces, each running as its own process, connected in a chain:

```
PX4 SITL  <-- shared memory -->  Gazebo
   |
   | uORB topics
   v
Micro XRCE-DDS Agent  <-- UDP -->  PX4's embedded XRCE-DDS client
   |
   | DDS
   v
ROS 2 (rclpy / rclcpp nodes), using message types from px4_msgs
```

- **PX4 SITL** — the actual PX4 flight-control firmware, compiled to run as
  a regular process on this machine instead of on flight hardware
  (`~/PX4-Autopilot`, built with `make px4_sitl gz_x500`). It runs the full
  estimation, control, and commander state machine exactly as it would on a
  real vehicle.
- **Gazebo** — the physics simulator. PX4 SITL drives a simulated `x500`
  quadrotor model in Gazebo and reads its simulated sensors back, standing
  in for real motors/IMU/GPS.
- **Micro XRCE-DDS Agent** — a bridge process (`MicroXRCEAgent`) that PX4
  connects to over UDP. PX4 itself speaks XRCE-DDS (a lightweight DDS
  variant meant for constrained systems); the Agent translates that into
  full DDS, which is what ROS 2 speaks natively. This is what makes PX4's
  internal `uORB` topics (vehicle status, position, sensor data, etc.) show
  up as ordinary ROS 2 topics under `/fmu/out/...` (PX4 → ROS 2) and
  `/fmu/in/...` (ROS 2 → PX4).
- **px4_msgs** (`src/px4_msgs`) — the ROS 2 message/service definitions
  matching PX4's uORB topics. Any ROS 2 node that wants to read or command
  the drone imports message types from here; without it, the topics carry
  data ROS 2 has no schema for.

Nothing in this workspace runs a mission or offboard-control node — it's
just the four pieces above, wired together, so you can see the drone fly
and see its telemetry on ROS 2.

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

## Run it

Open three terminals.

**Terminal 1 — Micro XRCE-DDS Agent:**
```bash
MicroXRCEAgent udp4 -p 8888
```

**Terminal 2 — PX4 SITL + Gazebo** (spawns one default `x500` quadrotor):
```bash
cd ~/PX4-Autopilot
make px4_sitl gz_x500
```
This opens the Gazebo GUI with the drone on the ground and drops you into
the PX4 shell (`pxh>`), where you can fly it directly:
```
pxh> commander takeoff
pxh> commander land
```

**Terminal 3 — the ROS 2 side of the bridge:**
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
