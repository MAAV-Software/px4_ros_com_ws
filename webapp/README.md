# swarm_launcher

The ROS 2 package `swarm_launcher`, kept in `webapp/` beside `src/`. `colcon build` from the workspace root still finds it.

The web launcher for the drone swarm: pick a launch file on a phone, start it, watch it, stop it. It flies SITL drones it starts itself, or real drones in HITL mode, with the same launch files either way. It takes voice commands offline, and draws every drone on the competition grid. Phases 1 and 2 of the design spec are done, and Phase 4 is under way.

```
Phone browser  --HTTP + WebSocket-->  hub  <--WebSocket-->  agent  --ros2 launch + ROS 2-->  PX4
```

- **Hub** (`hub.py`, `hub_core.py`): serves the site, owns the mission state machine. It starts a mission and mirrors what comes back, but does not run it.
- **Agent** (`agent.py`, `agent_core.py`): one per drone. Starts and stops the launch, runs the start checks and handles abort. After Go it needs nothing from the hub.
- **Flight backend** (`flight_ros.py`, or `flight_fake.py` with no ROS): the only code that touches ROS 2 and PX4.
- **Launch library** (`launch_library.py`, `launch_args.py`): each agent lists the launch files in its own workspace's `src` and reads the arguments each one declares. A new launch file shows up on the phone with no launcher change.
- **SITL manager** (`sitl_manager.py`): starts and stops SITL drones from the Fleet tab, each a PX4 instance plus its own agent, on the hub's machine.
- **Grid** (`grid.py`): the search area's four corners to the 40 by 150 competition grid, and each drone's PX4 position to latitude, longitude and grid square.
- **Voice** (`voice.py`): Vosk on the hub, limited to the launcher's commands. **TLS** (`tls.py`): the hub's own certificate authority, so phones get HTTPS and with it the microphone.
- **Web client** (`web/`): React and TypeScript, phone first.

## Try it with no ROS or PX4

This runs on any machine, using pretend drones. It shows the app working and proves nothing about PX4. Pretend drones offer a fixed list of launch files, not your `src`.

```bash
cd webapp
pip3 install -r requirements.txt
(cd web && npm ci && npm run build)
python3 -m swarm_launcher.hub --mode sitl --fake-sitl
```

Open <http://localhost:8080>, then add SITL drones on the Fleet tab. With `--fake-sitl` they are pretend drones, not PX4 instances.

## SITL or HITL

The toggle at the top of the Mission and Fleet tabs says what the launch is for, and the top bar always shows it. Launch files don't change between the two; what changes is which drones can fly and what has to be up first.

| | SITL | HITL |
| --- | --- | --- |
| Drones | Started by the hub from the Fleet tab, on the hub's machine | Real drones, each running its own agent |
| Pickable | Only SITL drones (agents started with `--sim`) | Only real drones |
| Extra start checks | None | PX4's `vehicle_status` arriving, and the RC link up |

A drone refuses to fly in the other mode, as well as the phone not offering it, so a real drone can never skip the HITL checks. The mode can't change during a run, and changing it clears the pick. The hub keeps the last mode in `~/launcher_runs/settings.json`. `--mode` overrides it, and HITL is the default.

## Build

You need the setup from the top-level README: ROS 2 Humble and a built PX4 at `~/PX4-Autopilot`. The web client needs Node to build, but only once, and `web/dist` can be copied from another machine.

Build, from the workspace root. `px4_msgs` takes about 6 minutes on a 4-core VM:

```bash
pip3 install -r webapp/requirements.txt
(cd webapp/web && npm ci && npm run build)
source /opt/ros/humble/setup.bash
colcon build --packages-select px4_msgs milestone1_singlevehicle swarm_launcher
```

Build any other package you want to fly the same way. The phone lists every package in `src`, but only built ones can run.

Agents list the launch files in `~/px4_ros_com_ws/src`. Pass `--workspace` for a workspace somewhere else. A package built after an agent started stays "not built" until the agent is restarted from a shell that has sourced it again.

## Run it in SITL

After `source install/setup.bash`:

```bash
ros2 run swarm_launcher hub --mode sitl
```

Open `http://<this machine's IP>:8080`, and add SITL drones on the Fleet tab. Each one is a PX4 instance (`gz_x500`, headless) plus an agent, laid out as milestone 2's `px4_sitl.launch.py` does: one Micro XRCE-DDS agent on UDP 8888, one Gazebo world started by the first drone, and drone N 20 m east of the first, with topics under `/px4_N` and `MAV_SYS_ID` N+1. A drone is ready once its PX4 has a position, about 15 s on the VM. The next one can only be added then, since it has to join the first one's world. For the same reason they are removed newest first.

SITL drones get `NAV_DLL_ACT=0`, so they arm with no QGroundControl connected, and the same speed limits milestone 2 uses. Their PX4 errors and warnings show on the Log tab. They outlive a hub restart, and their agents reconnect. A restarted hub has lost track of them, though, so Stop all on the Fleet tab stops them, using the record the hub kept in `~/launcher_runs/sitl_processes.json`.

`--px4-dir` points at a PX4 checkout other than `~/PX4-Autopilot`. Adding the first SITL drone is refused while PX4, Gazebo or a Micro XRCE-DDS agent is already running from something else, such as `px4_sitl.launch.py`.

`python3 webapp/tools/fake_gcs.py --sticks` gives PX4 centred RC sticks, which is how to test a pilot takeover in SITL. To check the launcher against PX4's own view of a flight, run `python3 webapp/tools/watch_sitl.py complete,aborted 120` alongside it.

## Run it on real drones (HITL)

Start the hub in HITL mode on the machine the phone connects to:

```bash
ros2 run swarm_launcher hub --mode hitl
```

On each drone, first bring up its link to PX4 as the HITL runbook describes, with the Micro XRCE-DDS agent and QGroundControl. Then start its agent:

```bash
ros2 run swarm_launcher agent --hub ws://<hub IP>:8080/agent --drone-id drone1
ros2 run swarm_launcher agent --hub ws://<hub IP>:8080/agent --drone-id drone2 --topic-prefix /px4_1 --system-id 2
```

`--topic-prefix` and `--system-id` say which PX4 the drone is, and the launcher passes them to every launch file that declares them. A drone passes its start checks only once its PX4 status is arriving, its RC link is up, it has a position, and PX4's own preflight checks pass.

## Voice

Voice start is a competition rule. Hold the talk button on the Mission tab and say a command; letting go sends the clip to the hub, which decodes it offline with Vosk.

| Say | Does |
| --- | --- |
| "start mission" | Opens the confirm for what is picked on screen, exactly as tapping Start does. The clock counts from when the button went down |
| "confirm" | Confirms a start within its 10 s, as the button does. Hold to talk works inside the confirm too |
| "cancel" | Closes the confirm |
| "abort" or "stop search" | Aborts at once, on the hub |

Vosk only listens for those phrases, so anything else is heard as nothing. "start" or "mission" alone do nothing. Voice works until drones are airborne. The hub then unloads the recognizer, freeing about 300 MB, and abort is the button only. There's also a box to type a command, which phone keyboard dictation can fill.

The hub needs Vosk and its small English model, about 40 MB:

```bash
pip3 install --user vosk
curl -L -o /tmp/vosk-model.zip https://alphacephei.com/vosk/models/vosk-model-small-en-us-0.15.zip
python3 -c "import zipfile; zipfile.ZipFile('/tmp/vosk-model.zip').extractall('$HOME')"
```

`--vosk-model` points at the model folder if it's not `~/vosk-model-small-en-us-0.15`. Without Vosk or a model, the talk button says why, and typed commands still work. `--fake-voice "start mission"` makes every clip heard as that phrase, for trying the phone side without Vosk.

**The prop-noise test** is Phase 4's gate. Record each command several times in the team's own voices, and the props at flying power from where the operator stands, as 16-bit mono WAV. Name each clip for what it should mean (`start_1.wav`, `confirm_2.wav`, `abort_3.wav`, `stop_search_1.wav`, and `other_*.wav` for things that must do nothing), then:

```bash
python3 webapp/tools/voice_noise_test.py --clips my_clips --noise props.wav --snr 20 10 5 0
```

It prints what was heard at each speech-to-noise ratio and fails on any false start. The clips in `test/voice_clips` are synthetic (macOS `say`), so they only show the tool works.

## Reaching the hub by name

Phones and laptops on the same network can use **maavnetwork.local** instead of an address. The hub announces that name itself over mDNS, through avahi, which Ubuntu runs anyway. iPhones and Android 12+ find it with no DNS server, so it works on Drone 1's Wi-Fi in the field.

- **Per network:** each of the hub's networks is announced with its own address, so a phone on Drone 1's Wi-Fi isn't handed the mesh radio's. A hotspot that comes up after the hub is picked up within 30 s.
- **Why .local:** that ending is reserved for mDNS and never goes to the internet, so a phone can't be sent to someone else's site under the same name. Endings like `.mobile` are real public domains.
- **Options:** `--name` changes the name, and `--name ''` turns it off. The hub's HTTPS certificate includes the name, so phones that trust the hub's CA trust it too.
- **No port:** to have `http://maavnetwork.local` and `https://maavnetwork.local` work without `:8080` or `:8443`, run this once on the hub machine. It redirects ports 80 and 443 to the hub, which keeps running as a normal user:

```bash
sudo webapp/tools/serve_on_80_and_443.sh --persist
```

`--persist` keeps the redirects across reboots (it needs `iptables-persistent`), and `--remove` takes them away.

A page opened over plain HTTP moves itself to HTTPS once the phone trusts the hub's CA, so that voice works. A phone that doesn't trust it yet stays on HTTP, where the talk button shows how to install the CA.

The SITL VM is on a private network inside the Mac it runs on, so only that Mac can reach it, by name or by address. Trying the site from a phone needs the hub on a machine on the phone's network, as Drone 1 will be.

## HTTPS for phones

Phones give a page the microphone and the screen wake lock only over HTTPS. The hub makes its own certificate authority the first time it runs, in `~/launcher_runs/tls`, and serves the site on both `http://<hub>:8080` and `https://<hub>:8443`. Agents keep using plain HTTP.

Once per phone: open `http://<hub>:8080/ca.crt` and install it. On an iPhone that's Settings, General, VPN & Device Management, then Settings, General, About, Certificate Trust Settings, where it has to be turned on. Then use the HTTPS address. On plain HTTP the talk button shows these steps with the links.

The hub's certificate names the addresses it found at startup (it prints them). It's reissued whenever they change, without touching the CA, so phones keep trusting it. `--https-host` adds a name or address, and `--https-port 0` turns HTTPS off.

## Search area and map

The Map tab draws the 80 by 300 ft field as the competition's 40 columns by 150 rows, north up, with each connected drone on it.

Type the four corners as WGS84 degrees (latitude, longitude), ideally from an RTK fix, since a square is only 0.61 m. Corners more than one square away from 80 by 300 ft, out of square, or in the wrong order are flagged, but still used. The hub converts every drone's position with the same transform, so all drones agree. It works each position out from PX4's own `ref_lat`/`ref_lon` and local position, with PX4's projection. A drone off the field is drawn hollow on its edge, with its distance off. The area is kept across restarts, and can't change during a run.

In SITL there are no measured corners, so "Place a test field around SITL 1" puts a true north-south field with that drone 2 m in from the south-west corner. A second SITL drone 20 m east is then still on the field.

Each drone has its own colour, kept while the hub knows it, on its map marker, its trail and its Fleet card. From Start until New mission, the map draws where each picked drone has been as a line in its colour. The hub records a point whenever a drone moves 0.3 m, so every phone draws the same trail. A phone that reconnects gets the whole trail again, and after that only new points. A trail that leaves the field is cut off at its edge.

Routes, mines and the path come later (Phase 5), so the map keeps blue, green, red, orange and yellow free for them. The six drone colours (violet, maroon, mustard, lavender, pink, brown, in that order) avoid those. They were checked with a colour-vision validator against the map's background: the first four stay apart even for colour-blind viewers. A seventh drone is drawn in black, and the number on each marker always says which drone it is.

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
| Launch file picked on the phone (2026-11-10): `updown_launch.py` from `src`, `arm_delay_s` changed to 10 | Ran as `ros2 launch milestone1_singlevehicle updown_launch.py arm_delay_s:=10 launch_sitl:=false`. Armed after the delay, flew, Complete after disarm. The pick was still there after New mission |
| Voice and the map (2026-11-11), Vosk small English model, HTTPS from this Mac with the hub's CA | A "start mission" clip over HTTPS heard correctly in 0.27 s. Without the CA, the certificate was refused. Two SITL drones with a test field: milestone 2's square traced 3 to 12 in both rows and columns (5 m), SITL 2 showed off the field's east edge and back. Voice turned off at takeoff, the recognizer unloaded, and voice came back after landing |
| Milestone 3's collision test (2026-11-12): `collision_launch.py` on two SITL drones | SITL 2 passed 3 m from SITL 1, which braked, climbed to 10 m, waited, and came back down (log: BRAKE, CLIMB, WAIT, NORMAL). SITL 1 then finished its hold, moved 5 m north and landed. Complete in about 1.5 min, with no position dropouts and no launches left running. The first try found two bugs, below |
| Two SITL drones added on the Fleet tab, then milestone 2's `offboard_solution.launch.py` on both (2026-11-11) | Each ready about 15 s after Add, in one world. Ran as `... offboard_solution.launch.py system_id:=1` and `... topic_prefix:=/px4_1 system_id:=2`. Both flew all 22 waypoints together, landed, Complete after both disarmed |

**Not checked yet:** HITL on real hardware, an iPhone (the client ran in a phone-sized desktop browser), and removing a SITL drone or Stop all against real PX4 (only against pretend drones). The fix that keeps a failed run from showing Complete until PX4 disarms passed its tests but hasn't flown.

### Bugs SITL found that the tests had missed

- Stopping a launch signalled only `ros2 launch`, which doesn't forward SIGINT, so the flight node ran for 5 more seconds. An abort during the countdown let it arm the drone, which then flew a failsafe Return. A test from before this was found passed only because the abort came early. Fixed by signalling the whole process group.
- PX4 goes back to Offboard after disarming, where its preflight result always fails. The new start check would have refused every mission after the first. Fixed by putting PX4 in Hold after a run.
- An abort just after arming left the drone armed in Hold on the ground. It now disarms.
- A drone's launch was stopped as soon as it landed. In milestone 3's collision test that cut off SITL 2's position, and SITL 1 held in the air waiting for it. Landed drones now keep their launches until the run is complete.
- Stopped agents never exited. rclpy's own signal handlers shut ROS down but kept the process alive, so agents from an earlier hub reconnected under the same names, and the hub mixed their reports with the new drones'. Agents now start ROS without those handlers, and the hub listens only to the newest agent of a name and warns about the other.

## Limits to know about

- **Bags aren't being recorded on Humble.** `offboard_solution.launch.py` passes `--topics` to `ros2 bag record`, which Humble doesn't accept, so the recorder exits straight away. The flight is unaffected, and removing that flag fixes it (checked in SITL). Its topic list also names `vehicle_gps_position` and `estimator_gps_status`, which this firmware doesn't publish, so GPS data isn't recorded.
- **With RC present, a crashed node leaves the drone hovering.** That's `COM_OBL_RC_ACT=0` handing control to the pilot in Position mode, as the runbook intends. With no RC, PX4 flies a Return.
- **Waypoint progress comes from the offboard node's log lines**, until the node publishes it properly. `test_flight_state.py` fails if someone rewords one of those lines.
- **Not built yet:** a PIN, routes from the search area, mines and `path.txt` on the map, RTK corner capture, and a safety-pilot check beyond the RC link.
- **Voice from a real phone hasn't been tried.** The recorder ran only as code; clips were sent with `curl`. The noise test has only seen synthetic speech and noise.
- **Every picked drone gets the same argument values.** A per-drone value, such as a different `path` for each, isn't possible from the phone yet.

## Picking a launch file

On the Mission tab, pick a folder in `src`, then a launch file in its `launch/` folder, then the drones to fly it. Every connected drone's own `src` is listed, so a file only one drone has shows up, and the other drones are marked "no such file".

- **Which files:** names ending `_launch.py` or `.launch.py`. Files starting `px4_sitl` are left out, since they only start the simulator. A folder is listed by its folder name, and launched by the package name in its `package.xml`, so `milestone3_multivehicle_anti-collision` runs as `milestone3_multivehicle`.
- **Arguments:** under Launch arguments, every argument the file declares, including ones from files it includes, unless the include sets them itself. Each shows its default as this drone works it out. An emptied field means the default, and only changed values are passed. Changing one runs the start checks again, and Start waits for them.
- **Set by the launcher:** `launch_sitl` is always `false`, and `topic_prefix` and `system_id` are the agent's `--topic-prefix` and `--system-id`. None is listed, and the hub refuses them from a phone.
- **Waypoints:** if the file declares `path`, the start checks read that waypoint file and refuse a drone too far from the first waypoint, as the offboard node would once it tried to arm. `relative_start:=true` or a negative `max_start_offset_m` turns that off.
- **One drone or several:** each picked drone runs the file itself, with its own `topic_prefix` and `system_id`. Milestone 2's files fly one vehicle each, so they work for any number of drones, as does milestone 3's `collision_launch.py` for its two. Milestone 3's other files still start nodes for several PX4 instances from one launch, so pick only one drone for those.
- **Landed drones stay up for the others:** a drone that lands first keeps its launch running until the whole run is complete, as it would inside one launch that flies them all. Milestone 3's yield rule needs this: it holds a lower-priority drone wherever it is while a peer's position goes stale. The hub tells every drone to stop at Complete. Without a hub, a drone stops its launch 10 minutes after landing.
- **Safety:** the phone only sends a package, a file and argument values. Each agent checks the file is in its own `src` and built, and that every argument is one the file declares, before anything runs. Values go to `ros2 launch` as plain arguments, never through a shell.

After a run, New mission keeps the pick. Check again, then Start, to fly it again.

## Tests

```bash
cd webapp
pip3 install pytest requests
python3 -m pytest test
```

On Ubuntu 22.04, set `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1` first: the `anyio` package's pytest plugin doesn't work with that release's older pytest.

They need no ROS install. `test_flight_ros.py` runs against stand-ins for `rclpy` and `px4_msgs`, so it catches wiring mistakes but says nothing about real PX4 behaviour. The SITL table above is the evidence for that. One test reads a real launch file with ROS's `launch` package, and is skipped unless ROS 2 is sourced. The Vosk tests run when the model is at `~/vosk-model-small-en-us-0.15`, or wherever `VOSK_MODEL` says, and are skipped otherwise.

Run files go to `~/launcher_runs/<run id>/`: `run.json`, `events.jsonl` and `log.jsonl` from the hub, and the mission package from each agent.
