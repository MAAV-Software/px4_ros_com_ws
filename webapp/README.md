# swarm_launcher

The ROS 2 package `swarm_launcher`, kept in `webapp/` beside `src/`. `colcon build` from the workspace root still finds it.

The web launcher for the drone swarm: pick a mission on a phone, start it, watch it, stop it. This is Phase 1 of the design spec, the skeleton for one SITL drone.

```
Phone browser  --HTTP + WebSocket-->  hub  <--WebSocket-->  agent  --ros2 launch + ROS 2-->  PX4
```

- **Hub** (`hub.py`, `hub_core.py`): serves the site, owns the mission state machine. It starts a mission and mirrors what comes back, but does not run it.
- **Agent** (`agent.py`, `agent_core.py`): one per drone. Starts and stops the launch, runs the start checks and handles abort. After Go it needs nothing from the hub.
- **Flight backend** (`flight_ros.py`, or `flight_fake.py` with no ROS): the only code that touches ROS 2 and PX4.
- **Missions** (`missions/*.yaml`): one file per mission. Adding one needs no web code.
- **Web client** (`web/`): React and TypeScript, phone first.

## Try it with no ROS or PX4

This runs on any machine, using a pretend drone. It shows the app working and proves nothing about PX4.

```bash
cd webapp
pip3 install -r requirements.txt
(cd web && npm ci && npm run build)
python3 -m swarm_launcher.hub --fake-drone drone1
```

Open <http://localhost:8080>.

## Run it against one SITL drone

You need the setup from the top-level README: ROS 2 Humble and a built PX4 at `~/PX4-Autopilot`. The web client needs Node to build, but only once, and `web/dist` can be copied from another machine.

Build, from the workspace root. `px4_msgs` takes about 6 minutes on a 4-core VM:

```bash
pip3 install -r webapp/requirements.txt
(cd webapp/web && npm ci && npm run build)
source /opt/ros/humble/setup.bash
colcon build --packages-select px4_msgs milestone1_singlevehicle swarm_launcher
```

Then run each of these in its own terminal, after `source install/setup.bash`:

```bash
ros2 launch milestone1_singlevehicle px4_sitl.launch.py
```

```bash
ros2 run swarm_launcher hub
```

```bash
ros2 run swarm_launcher agent --drone-id drone1
```

Open QGroundControl too, or PX4 may refuse to arm with "No connection to the GCS", depending on `NAV_DLL_ACT`. On a machine with no QGroundControl, run `python3 webapp/tools/fake_gcs.py` instead, which leaves that saved parameter alone. Add `--sticks` to it to give PX4 centred RC sticks, which is how to test a pilot takeover in SITL.

Open `http://<this machine's IP>:8080` on a phone on the same network.

SITL has to be started separately, and missions must keep `launch_sitl: "false"`. Stopping a launch that owns SITL also stops PX4 and Gazebo, so an abort would take the simulator down with it.

To check the launcher against PX4's own view of a flight, run `python3 webapp/tools/watch_sitl.py complete,aborted 120` alongside it.

## px4_msgs must match the firmware

The workspace's `px4_msgs` is generated from PX4 `v1.18.0-beta1-1182-ga99287097a`, the same files PX4's `Tools/copy_to_ros_ws.sh` copies: `msg/*.msg`, `msg/versioned/*.msg` and `srv/*.srv`. A drone on other firmware needs its own copy. To regenerate one, from the workspace root:

```bash
rm src/px4_msgs/msg/*.msg src/px4_msgs/srv/*.srv
cp ~/PX4-Autopilot/msg/*.msg ~/PX4-Autopilot/msg/versioned/*.msg src/px4_msgs/msg/
cp ~/PX4-Autopilot/srv/*.srv src/px4_msgs/srv/
```

On a mismatch, `vehicle_status`, `failsafe_flags` and `vehicle_gnss` stop arriving. The agent then falls back. It infers armed from command acks and being in the air, confirms Hold from the ack instead of PX4's mode, and can't see a pilot's takeover. It also skips the already-armed and PX4 preflight start checks.

## What has been checked in SITL

Checked on 2026-11-05 with PX4 v1.18.0-beta1 on an Ubuntu 22.04 ARM VM, Python 3.10, with the matching `px4_msgs`. PX4's own `listener` output was logged beside the hub's view throughout.

| Scenario | What happened |
| --- | --- |
| Full mission, started from the phone client | Armed, flew, landed. Complete only once PX4 disarmed, about 2 s after touchdown |
| Two missions back to back | Second one passed its start checks and flew |
| Abort in flight | All drones answered in about 0.1 s. PX4 in HOLD at altitude, launch stopped |
| Then Land or Return | PX4 in LAND or RTL, landed, disarmed |
| Abort at seven moments from 12 s to 15 s, either side of arming | Launch stopped in 0.3 s every time. Never above 3 cm, disarmed |
| Abort just after liftoff | PX4 in HOLD, then returned and landed |
| Hold with no choice | PX4 in RTL at about 61 s, landed |
| Pilot takes over (Position mode) | Launch stopped in under 1.5 s. Abort then left the pilot alone: no Hold, no Return |
| Outside switch to Hold | Launch stopped. Abort then offered Return or Land |
| Offboard node killed in flight, no RC | Seen in about 0.6 s. PX4 in RTL, landed |
| Hub killed mid-flight, back while flying | Drone flew on. Hub rebuilt the run, and abort worked |
| Hub killed, back after landing | Hub reported Complete with the right clock |
| No ground station | Start refused straight away: "no ground station connected" |
| PX4 already armed | Start refused |
| Bags | Recorded and closed cleanly, with the one-flag fix below |

**Not checked yet:** real hardware, an iPhone (the client ran in a phone-sized desktop browser), and more than one drone. The last fix, which keeps a failed run from showing Complete until PX4 disarms, passed its tests but hasn't flown.

### Bugs SITL found that the tests had missed

- Stopping a launch signalled only `ros2 launch`, which doesn't forward SIGINT, so the flight node ran for 5 more seconds. An abort during the countdown let it arm the drone, which then flew a failsafe Return. A test from before this was found passed only because the abort came early. Fixed by signalling the whole process group.
- PX4 goes back to Offboard after disarming, where its preflight result always fails. The new start check would have refused every mission after the first. Fixed by putting PX4 in Hold after a run.
- An abort just after arming left the drone armed in Hold on the ground. It now disarms.

## Limits to know about

- **Bags aren't being recorded on Humble.** `offboard_solution.launch.py` passes `--topics` to `ros2 bag record`, which Humble doesn't accept, so the recorder exits straight away. The flight is unaffected, and removing that flag fixes it (checked in SITL). Its topic list also names `vehicle_gps_position` and `estimator_gps_status`, which this firmware doesn't publish, so GPS data isn't recorded.
- **With RC present, a crashed node leaves the drone hovering.** That's `COM_OBL_RC_ACT=0` handing control to the pilot in Position mode, as the runbook intends. With no RC, PX4 flies a Return.
- **Waypoint progress comes from the offboard node's log lines**, until the node publishes it properly. `test_flight_state.py` fails if someone rewords one of those lines.
- **Not in Phase 1:** PIN, HTTPS, voice, the map, `path.txt`, multiple drones, and the RC link and safety pilot start checks. The RC check can now be built, because `failsafe_flags` reads. Plain HTTP also means the phone can't use its microphone or wake lock yet.

## Adding a mission

Add a YAML file to `missions/`, then rebuild the package. Launch arguments must be quoted strings.

```yaml
id: m1_square
name: Square, one drone
spoken_name: square
inputs: []
drones:
  drone1: {role: search}
launch:
  package: milestone1_singlevehicle
  file: offboard_solution.launch.py
  waypoints: {package: milestone1_singlevehicle, file: resource/milestone1_square.yaml}
  args: {launch_sitl: "false", record: "true", arm_delay_s: "10"}
```

The launch file must accept `topic_prefix`, and `path` if the mission names waypoints.

## Tests

```bash
cd webapp
pip3 install pytest requests
python3 -m pytest test
```

On Ubuntu 22.04, set `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1` first: the `anyio` package's pytest plugin doesn't work with that release's older pytest.

They need no ROS install. `test_flight_ros.py` runs against stand-ins for `rclpy` and `px4_msgs`, so it catches wiring mistakes but says nothing about real PX4 behaviour. The SITL table above is the evidence for that.

Run files go to `~/launcher_runs/<run id>/`: `run.json`, `events.jsonl` and `log.jsonl` from the hub, and the mission package from each agent.
