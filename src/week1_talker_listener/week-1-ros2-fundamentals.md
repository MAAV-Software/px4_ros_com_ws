# Week 1: Linux / Dev Env + ROS2 Fundamentals

**Dates:** Sep 15-21, 2026
**Outcome:** Everyone can build and run a basic ROS2 pub-sub node.

## Topics

- Linux command line basics (if needed) — filesystem navigation, permissions, package management
- ROS2 core concepts: nodes, topics, publishers/subscribers, services, actions
- `colcon` build system, workspaces, packages
- Launch files: what they are, why we use them instead of running nodes by hand
- ROS2 CLI tools: `ros2 topic`, `ros2 node`, `ros2 run`, `ros2 launch`

## Setup

Before anything else, get a working copy of the workspace using MAAV's `starter-code` branch:

```
mkdir -p ~/px4_ros_com_ws
cd ~/px4_ros_com_ws
git clone -b starter-code git@github.com:MAAV-Software/px4_ros_com_ws.git .
source /opt/ros/humble/setup.bash
colcon build --symlink-install
echo "source ~/px4_ros_com_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

This replaces a generic "install ROS2" step with the actual recipe MAAV uses — clone the `starter-code` branch of this repo (not raw upstream PX4 repos), build it, and have it auto-sourced in every new terminal. Everything in Part A and Part B below assumes you're working inside this workspace.

## Hands-on tasks

### Part A: By hand — the `ros2 topic` CLI

No code yet. Everything below uses plain `ros2 topic` commands across a few terminals, all against the same topic name (`/chatter`) that the automated package in Part B uses.

1. With nothing else running, run `ros2 topic list` to see the baseline set of topics on a bare ROS2 system (should be empty, or close to it).
2. **Terminal 1 — publish manually:**
   ```
   ros2 topic pub /chatter std_msgs/msg/String "data: 'hello from the CLI'"
   ```
   By default this publishes repeatedly (1 Hz). Leave it running. (Add `--once` instead if you just want to send a single message.)
3. **Terminal 2 — echo:** while terminal 1 is still publishing, run `ros2 topic echo /chatter` and watch the messages arrive in real time.
4. **Terminal 3 — inspect:**
   - `ros2 topic list` — `/chatter` now shows up, since terminal 1 is publishing on it
   - `ros2 topic info /chatter` — shows the message type and the current publisher/subscriber counts
5. Briefly try (don't worry about mastering these, just see what they do): `ros2 topic hz /chatter` (publish rate), `ros2 topic bw /chatter` (bandwidth), `ros2 topic type /chatter` (message type), `ros2 topic find std_msgs/msg/String` (all topics of a given type).
6. Stop the manual publisher (Ctrl-C in terminal 1). Confirm `ros2 topic echo` in terminal 2 stops receiving new messages, and `ros2 topic info /chatter` in terminal 3 now shows 0 publishers.

### Part B: Automated — the `week1_talker_listener` package

7. Build the `week1_talker_listener` package (lives in [`src/week1_talker_listener`](../src/week1_talker_listener), alongside the rest of the workspace's code) and source the install:
   ```
   colcon build --packages-select week1_talker_listener
   source install/setup.bash
   ```
8. Read [`talker.py`](../src/week1_talker_listener/week1_talker_listener/talker.py) and [`listener.py`](../src/week1_talker_listener/week1_talker_listener/listener.py). Compare them to what you just did by hand in Part A: `talker.py` is basically `ros2 topic pub` wrapped in a program that runs on a timer instead of a one-off terminal command, and `listener.py` is doing what `ros2 topic echo` does, but inside your own code where you can react to each message.
9. Run them together:
   ```
   ros2 launch week1_talker_listener talker_listener.launch.py
   ```
   This mirrors the **terminal-launch** pattern used in `archive/src/px4_multi_vehicle_offboard/px4_single_plan/launch/offboard_waypoint.launch.py`: each node gets its own `gnome-terminal` window via the `prefix='gnome-terminal --'` argument, so you can watch it directly instead of both nodes' logs interleaving in one process. We'll rely on this same pattern for real PX4/SITL launches later.
10. Confirm messages flowing independently of the launch file: run `ros2 topic echo /chatter` in a third terminal while talker/listener are up — same command as Part A step 3, but now the messages are coming from `talker.py` instead of your own `ros2 topic pub`.
11. Modify `talker.py` to publish a different message or rate, and `listener.py` to log something extra — small edits to build comfort with the pub/sub loop. Rebuild (`colcon build --packages-select week1_talker_listener`) to pick up changes.
12. Explore this workspace's existing `px4_msgs` package with `ros2 interface show` to see message definitions before we start using them next week.

## Checkpoint

**After Part A:** each new member can explain what a topic is purely from CLI observation — no code needed — and has used `pub`, `echo`, `list`, and `info` at minimum.

**After Part B:** each new member builds and runs the talker/listener pair via `ros2 launch`, sees both terminals log independently, and can connect the CLI mental model from Part A to real code — explaining why we'd write a node instead of just running `ros2 topic pub` forever (persistent behavior, timers, reacting to messages, combining with other logic) — plus why the terminal-launch pattern is useful for interactive processes like a PX4 SITL console.

## Notes

Curriculum is a draft — adjust pacing/depth based on how much ROS2 experience actually shows up in week 1.
