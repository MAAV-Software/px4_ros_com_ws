# Milestone 2: 2 Drones Autonomous

This guide has two parts: the [test guide](#test-guide) (what is tested, how,
and the pass criteria) and the [session plan](#session-plan). Scoring and test
commands are in [`testing.md`](testing.md). Changes from milestone 1 are in
[`milestone1-to-milestone2-review.md`](milestone1-to-milestone2-review.md).

# ChatGPT Convo Link
The link below shows how this folder was created and includes the troubleshooting as well as simply useful information on how to run multivehicle control, testing, and proposed changes

https://chatgpt.com/share/6aced206-28c4-83e9-87f0-b1d230a309df


## Test guide

**Goal (from [PLAN.md](../../../PLAN.md)):** independent multi-vehicle SITL
with two drones, followed by validation on both old-design real airframes.

This is a copy of `milestone1_singlevehicle`, extended to run two independent
instances of its controller. Each drone arms, takes off, follows its own
waypoint path, and lands. Milestone 2 is complete only after both the SITL
and real-flight stages pass. Anti-collision and route coordination begin in
milestone 3; four-drone operation and integrations belong to later milestones.

### What the test flies

[`offboard_control_solution.py`](../milestone2_multivehicle/offboard_control_solution.py)
retains milestone 1's per-vehicle behavior:

1. Load `{pos, yaw, hold_s}` waypoints; yaw is in radians.
2. Stream position setpoints and OffboardControlMode at 10 Hz for about 1 s.
3. Request arm and Offboard; retry until both command ACKs are accepted
   (timeout after 30 s). ACK acceptance is not continuous flight-state monitoring.
4. Reach each target within 0.5 m and 5° heading error; remain there for the
   continuous `hold_s`. Leaving either tolerance resets the hold.
5. Ramp commanded yaw at 45°/s, as in milestone 1.
6. Request land after the final hold and stop streaming. Confirm actual
   landing and disarm from PX4/flight logs; the controller does not verify them.

Both readiness checks must succeed before either controller starts. Once
started, each controller has its own position, ACK state, target index, yaw
ramp and hold timer. There is no barrier at individual waypoints and no
formation or shared-state control.

### Vehicle routing

| Vehicle label | PX4 instance | Default DDS topics | Command target (`MAV_SYS_ID`) | ROS controller name |
|---|---|---|---|---|
| 0 | 0 | `/fmu/...` | 1 | `/offboard_control_0` |
| 1 | 1 | `/px4_1/fmu/...` | 2 | `/offboard_control_1` |

All three publishers (heartbeat, setpoint, command) and both subscriptions
(local position, command ACK) use the selected vehicle's topic prefix.
`VehicleCommand.target_system` addresses that vehicle explicitly, never
broadcast ID 0. ROS node names alone do not route PX4 messages.

The built-in SITL launch explicitly sets `PX4_UXRCE_DDS_NS` for both vehicles,
so changes to PX4's default namespace convention cannot silently reroute them.
For external SITL or hardware, inspect the actual topics and `MAV_SYS_ID`
values and pass the matching namespace and system-ID arguments.

### Paths and separation

The full 22-waypoint path, square and up/down files are copied from milestone 1
with milestone 2 names. Both drones use the same full path by default; `path_1`
and `path_2` let them fly different files. Each file is interpreted in that
vehicle's **own local NED frame**, not a shared global frame: x = north,
y = east, z = down, negative z = altitude. The origin is where that PX4 local
estimate initialized, normally close to the boot location.

The built-in simulator spawns vehicle 0 at Gazebo `0,0,0` and vehicle 1 at
`20,0,0` (20 m east in Gazebo's world frame). The default route spans 0–5 m
east relative to each origin, giving a nominal 15 m gap between route boxes.
Confirm the estimated local origins and the world trajectories in SITL.
This is planned spatial separation, not an anti-collision algorithm or a
guarantee of clearance. Review both paths together whenever either changes.
Do not copy Gazebo world coordinates into local-NED waypoint files.

| File | Motion | Use |
|---|---|---|
| `resource/milestone2_updown.yaml` | Climb to 5 m, descend, request land | First smoke test |
| `resource/milestone2_square.yaml` | Climb, 5 m square, descend, request land | Basic two-drone flight |
| `resource/milestone2_path.yaml` | Translation, yaw and hover, then land | Full acceptance run |

Default full path (yaw shown in degrees here; YAML stores radians):

| # | x | y | z | Yaw | Hold (s) | Tests |
|---|---|---|---|---|---|---|
| 0 | 0 | 0 | −5 | 0 | 5 | T1 climb |
| 1 | 5 | 0 | −5 | 0 | 5 | T3 north |
| 2 | 5 | 5 | −5 | 0 | 5 | T5 east |
| 3 | 0 | 5 | −5 | 0 | 5 | T4 south |
| 4 | 0 | 0 | −5 | 0 | 5 | T6 west |
| 5 | 0 | 0 | −3 | 0 | 5 | T2 descend |
| 6 | 4 | 4 | −3 | 0 | 5 | T7 diagonal (NE) |
| 7 | 4 | 0 | −3 | 0 | 3 | — (reposition) |
| 8 | 0 | 4 | −3 | 0 | 5 | T7 diagonal (NW) |
| 9 | 4 | 0 | −6 | 0 | 5 | T8 3D (SE + climb) |
| 10 | 0 | 4 | −4 | 0 | 5 | T8 3D (NW + descend) |
| 11 | 1 | 4 | −4 | 0 | 3 | T9 small (1 m north) |
| 12 | 1 | 5 | −4 | 0 | 3 | T9 small (1 m east) |
| 13 | 2 | 2 | −5 | 0 | 10 | H1 hover hold |
| 14 | 2 | 2 | −5 | 90 | 3 | Y1 yaw clockwise |
| 15 | 2 | 2 | −5 | 180 | 3 | Y1 yaw clockwise |
| 16 | 2 | 2 | −5 | 90 | 3 | Y1 yaw counter-clockwise |
| 17 | 2 | 2 | −5 | 0 | 3 | Y1 yaw counter-clockwise |
| 18 | 2 | 2 | −5 | −90 | 3 | Y1 yaw counter-clockwise |
| 19 | 2 | 2 | −5 | 0 | 3 | Y1 yaw clockwise |
| 20 | 4 | 2 | −5 | 90 | 5 | Y2 yaw while moving |
| 21 | 2 | 2 | −5 | 0 | 5 | Y2 yaw while moving, then land |


The path stays within a 5 m × 5 m horizontal box at 3–6 m altitude. Both drones
land at their own final waypoint, not necessarily their takeoff point. Allow
about 3–4 minutes per full mission at the speed limits below. Edit/rebuild the
installed YAML or pass an absolute path to your edited file.

**Known limits inherited from milestone 1:** no telemetry freshness watchdog,
no ongoing armed/offboard-state monitoring, no automatic failure recovery,
and one landing request without retry/confirmation. A 30 s arm/offboard
confirmation timeout stops that controller's stream; it does not prove the
airframe is disarmed. PX4 failsafes and the safety pilots remain essential.
The launch readiness check only confirms a finite, valid position estimate.

### Freedom-of-flight test matrix (each vehicle)

| ID | Movement / behavior | Current coverage |
|---|---|---|
| T1–T6 | Climb, descend, north, south, east, west | Full path; square covers these too |
| T7–T9 | Horizontal diagonals, combined 3D motion, small corrections | Full path |
| Y1–Y2 | Yaw both directions in hover and while translating | Full path |
| R1 | Indirect roll/pitch during translation | Scored from attitude log |
| H1 | Continuous hover hold | Full path |
| S1 | Arm and enter Offboard | Confirm actual state in flight log; controller uses ACKs |
| F1 | Loss of one controller's setpoint stream | Separate controlled test, one vehicle at a time |
| L1 | Land and disarm | Confirm actual state in flight log |
| V1, R2 | Velocity control, direct attitude control | Deferred, as in milestone 1 |

### Two-vehicle test matrix

These rows are checked from ROS graphs, paired logs and observations; the
per-flight scorer does not automatically score them.

| ID | Test | Pass evidence |
|---|---|---|
| M1 | Distinct routing and identities | Both topic sets present; each controller publishes only to its vehicle, with target IDs 1/2 (or configured hardware IDs) |
| M2 | Vehicle 0 isolation | Vehicle 0 completes a path while vehicle 1 stays disarmed |
| M3 | Vehicle 1 isolation | Vehicle 1 completes a path while vehicle 0 stays disarmed |
| M4 | Concurrent independent missions | Both fly during overlapping time, each completes its assigned path and lands/disarms |
| M5 | Different paths / completion times | Fly one up/down and one full path; the longer mission continues after the shorter one lands |
| M6 | One-controller loss | Selected PX4 executes its configured failsafe; other controller keeps its own mission and stream |
| M7 | Readiness failure | With only one PX4 available, pair launch times out and neither controller starts |

### Pass thresholds

Retained from milestone 1; agree them as a team before recorded flights.
`hitl` remains the scorer's name for real-airframe thresholds for compatibility
with milestone 1. These are real flights, not a hardware-in-the-loop simulator.

| Measure | Applies to | SITL | Real flight (`hitl`) |
|---|---|---|---|
| Position error after reaching target | T1–T9 | ≤ 0.5 m | ≤ 0.5 m |
| Overshoot | T1–T9 | ≤ 0.5 m | ≤ 1.0 m |
| Settling time after arrival | T1–T9 | ≤ 3 s | ≤ 5 s |
| Cross-track error | T1–T9 | ≤ 1.0 m | ≤ 2.0 m |
| Heading error after arrival | Y1–Y2 | ≤ 5° | ≤ 10° |
| Drift during hover | H1 | ≤ 0.2 m | ≤ 1.0 m |
| Tilt angle | R1 | ≤ 45° | ≤ 45° |
| Visible oscillation | All | None | None |

Controller advancement still uses 0.5 m / 5° on both stages; the looser real
flight scoring limits do not change the live controller's acceptance rule.

### Stage 1: SITL

#### Build and run

Prerequisites match milestone 1: ROS 2 Humble, Gazebo, MicroXRCEAgent,
matching `px4_msgs`/PX4 topic schemas, and QGroundControl. Build PX4 once:

```bash
cd ~/PX4-Autopilot
make px4_sitl
cd ~/px4_ros_com_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to milestone2_multivehicle
source install/setup.bash
ros2 launch milestone2_multivehicle updown_launch.py headless:=false
```

Open QGroundControl first so both PX4 instances can satisfy their configured
GCS checks. Do not run another PX4, agent or Gazebo session at the same time.
The launcher refuses to attach a new simulation to one already running.

The launch starts one UDP agent on port 8888, starts PX4 instance 0, waits
for its local position, then starts instance 1 in the same world. It runs two
readiness helpers (300 s timeout each) and starts both mission controllers only
when both succeed. Processes run without extra terminal windows. PX4 and
agent output is in `~/.ros/log/latest/`; controller progress is on screen.
Ctrl-C stops processes belonging to this launch. With `launch_sitl:=false`,
the external simulator/agents keep running when controllers stop.

Use `square_launch.py` next, then `offboard_solution.launch.py` for the full path.

| Argument | Default | Purpose |
|---|---|---|
| `launch_sitl` | `true` | Start sim and agent; `false` for external SITL or hardware |
| `headless` | `true` | `false` opens Gazebo GUI |
| `px4_dir` | `~/PX4-Autopilot` | Built PX4 source checkout |
| `path` | Installed full path (up/down or square in wrappers) | Common default for both vehicles |
| `path_1`, `path_2` | `path` | Independent path overrides |
| `namespace_1`, `namespace_2` | `/`, `/px4_1` | Actual DDS namespace prefixes |
| `system_id_1`, `system_id_2` | `1`, `2` | Actual MAV_SYS_ID; overrides for external SITL/hardware |

All mission launches accept these arguments. Built-in SITL requires target
IDs 1 and 2. Equal namespaces or equal system IDs are rejected before startup.
Both files are validated before any mission controller starts.

SITL only, for recording and isolated-controller tests:

```bash
ros2 launch milestone2_multivehicle px4_sitl.launch.py headless:=false
```

Then launch the pair in another sourced terminal:

```bash
ros2 launch milestone2_multivehicle offboard_solution.launch.py launch_sitl:=false
```

The simulator applies the same limits to both PX4 instances:
`MPC_XY_VEL_MAX=2`, `MPC_Z_VEL_MAX_UP=1`, `MPC_Z_VEL_MAX_DN=1` (m/s) and
`MPC_YAWRAUTO_MAX=45` (deg/s). It leaves GCS/RC/failsafe policy unchanged.
SITL saves these parameters separately for each instance.

#### Recording and measuring

Record both sets of topics in one bag to preserve their overlap in time.
Discover exact version suffixes with `ros2 topic list` before recording:

```bash
ros2 bag record -o m2_sitl_run01 \
  /fmu/in/trajectory_setpoint /fmu/in/vehicle_command \
  /fmu/out/vehicle_local_position_v1 /fmu/out/vehicle_command_ack_v1 \
  /fmu/out/vehicle_attitude /fmu/out/vehicle_status_v1 \
  /px4_1/fmu/in/trajectory_setpoint /px4_1/fmu/in/vehicle_command \
  /px4_1/fmu/out/vehicle_local_position_v1 /px4_1/fmu/out/vehicle_command_ack_v1 \
  /px4_1/fmu/out/vehicle_attitude /px4_1/fmu/out/vehicle_status_v1
```

Save one `.ulg` per vehicle. The built-in launch uses separate working folders:
`~/PX4-Autopilot/build/px4_sitl_default/rootfs/0/fs/log/` and `.../1/fs/log/`.
On real airframes, download each controller's log in QGroundControl and label
it with airframe identity. Run `m2_analyze` twice with the matching path and
`--vehicle 0` / `--vehicle 1`; see [`testing.md`](testing.md).

#### Test order and SITL pass criteria

1. Inspect routing, then fly vehicle 0 alone and vehicle 1 alone (M1–M3).
2. Pair up/down, pair square, then pair full paths (M4).
3. Use different paths to demonstrate independent progression and landing (M5).
4. Test readiness failure and loss of one controller at a time (M6–M7).

- [ ] Both vehicles complete every waypoint, hold and yaw in their own file
- [ ] Both actually enter Offboard, land and disarm without manual intervention
- [ ] All in-scope baseline rows pass for each vehicle; F1 is recorded separately
- [ ] M1–M7 pass with paired evidence; three consecutive clean paired full runs
- [ ] No visible oscillation; local-frame/world-route separation checked
- [ ] Both flight logs, path files, reports and bag saved with the same run label

#### Troubleshooting

| Problem | Check |
|---|---|
| Only one drone moves | Target system IDs and namespace overrides; inspect commands on both topics |
| PX4/Gazebo already running | Use `launch_sitl:=false` for a correctly configured existing pair; otherwise stop the previous session |
| No topics for vehicle 1 | Instance 0 readiness, second PX4 log, agent connection, explicit DDS namespace |
| No arm / mode accepted | QGroundControl connection, PX4 preflight checks, 10 Hz streams and actual MAV_SYS_ID |
| Local position never received / RTPS size error | Firmware and px4_msgs schema/versioned topic compatibility |
| Trajectories overlap | Local-frame origins and both path files; namespace isolation does not provide spatial separation |
| Readiness passes but arming fails | Valid position alone is not the full PX4 preflight check |

### Stage 2: real airframes

Run the same controller code on both old-design airframes after SITL passes.
The launch can arm both shortly after readiness succeeds; only start missions
when both aircraft and safety pilots are ready. Physical route separation is
required because this milestone has no anti-collision controller.

#### Before the flight

- [ ] Record both PX4 versions, airframe names, hardware link types and agent commands
- [ ] Configure distinct nonzero `MAV_SYS_ID` and distinct `UXRCE_DDS_KEY` values
- [ ] Configure distinct DDS namespaces on the PX4 clients (client `-n` / supported
  firmware namespace setting); ROS launch namespaces alone do not change the clients
- [ ] Verify the actual local-position/ACK topic schemas on both links
- [ ] Review both physical flight boxes, landing locations and return/failsafe routes;
  use at least the milestone 1 margin (5 m on each side), increased for positioning error
- [ ] Position lock, charged batteries, props/airframe checks, pilot and independent
  takeover capability for each vehicle; check Offboard-loss and RC-loss policies
- [ ] Bench-check identity/routing with props removed, then fly each airframe alone
- [ ] Apply/review the agreed speed and yaw limits on both real flight controllers;
  `launch_sitl:=false` does not set hardware parameters

Hardware transport settings and the required physical separation depend on the
airframes/site and must be recorded by the team; the simulator's 20 m spawn
offset does not configure hardware origins or ensure real-flight clearance.
One UDP agent can serve both clients; serial links need the agent setup
appropriate to each link. Start those hardware agents externally.

Example, **after** configuring clients with namespaces `drone_a` / `drone_b`
and system IDs 11 / 12:

```bash
ros2 launch milestone2_multivehicle offboard_solution.launch.py \
  launch_sitl:=false namespace_1:=/drone_a namespace_2:=/drone_b \
  system_id_1:=11 system_id_2:=12 \
  path_1:=/absolute/path/airframe_a.yaml path_2:=/absolute/path/airframe_b.yaml
```

Each safety pilot takes over at the first unexpected motion. A stopped mission
controller relies on PX4's configured failsafe. Stopping the pair launch stops
both controllers; for the one-controller-loss test use separate controller
terminals as described in `testing.md`.

#### Real-flight pass criteria

- [ ] Each vehicle passes the baseline in-scope rows on its own
- [ ] Both complete overlapping independent missions, then land/disarm
- [ ] No takeover needed in normal acceptance runs
- [ ] Routing, different-path and one-controller-loss tests verified with pilots ready
- [ ] Each vehicle passes real-flight thresholds; paired observations confirm isolation
- [ ] Both logs downloaded and scored, with airframe/namespace/system-ID mappings saved

### Results log

Add a row per **vehicle** per run; put the common run label in Notes. These
columns match each scorer's ready-to-paste row. No flights are claimed here.

| Date | Stage | Vehicle | Airframe / PX4 version | Path file / version | Matrix rows | Max position error (m) | Max overshoot (m) | Max settle time (s) | Max heading error (°) | Hover drift (m) | Failsafe action | Result | Recording / log | Notes |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| | SITL | 0 | x500 | | | | | | | | | | | |
| | SITL | 1 | x500 | | | | | | | | | | | |
| | Real flight | 0 | | | | | | | | | | | | |
| | Real flight | 1 | | | | | | | | | | | | |

Pair-level evidence (manual; a pair is incomplete if either required report or
observation is missing, even if the other drone passed):

| Run | Stage | M1 | M2 | M3 | M4 | M5 | M6 | M7 | Vehicle 0 report | Vehicle 1 report | Bag / observations | Result |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| | | | | | | | | | | | | |

## Session plan

Milestones remain rolling/reactive as in PLAN.md; these sessions are a working
sequence, not fixed dates or a guarantee of completion in one meeting.

| Session | Stage | Outcome |
|---|---|---|
| 1 (~1.5 h) | SITL | Build, inspect routing, isolated flights and paired up/down/square |
| 2 (~1.5 h) | SITL | Full paired runs, different paths, one-controller-loss/readiness tests, both logs scored |
| 3 (~3 h) | Real airframes | Bench identities, isolated flights, then paired acceptance with two safety pilots |

### Decisions before Session 1

| Decision | Starting point |
|---|---|
| Firmware / message definitions | Same tested combination as milestone 1; record commit/version |
| Speed/yaw limits | 2 m/s horizontal, 1 m/s vertical, 45°/s yaw |
| Routes | Copied milestone 1 files; physically separate local origins |
| Thresholds | Table above and matching YAML values |
| Failsafe action | Inspect/agree per vehicle; test intentionally, not by stopping the simulator |
| Hardware mapping | Link, namespace, MAV_SYS_ID, UXRCE_DDS_KEY, pilot, physical flight box per airframe |

### Scope and done criteria

The package retains milestone 1's baseline control and post-flight scoring.
Velocity/direct-attitude modes, live scoring, anti-collision, generated routes,
webapp, object detection and path-finding integration are outside milestone 2.

Done when SITL and real-flight checklists pass for **both** vehicles, paired
independence evidence is saved, and both logs/reports are attached to each
recorded acceptance run. Then proceed to milestone 3's anti-collision work.
