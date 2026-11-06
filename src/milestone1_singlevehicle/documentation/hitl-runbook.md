# Milestone 1 HITL / real-drone runbook (single vehicle)

Bench and field checklist for flying `milestone1_singlevehicle` on one real drone (or a HITL-connected PX4). Written without access to hardware: items marked **VERIFY** were not exercised on a real airframe and must be confirmed on the day. Background and pass criteria: [`milestone-1-single-vehicle.md`](milestone-1-single-vehicle.md); scoring detail: [`testing.md`](testing.md).

Roles: **safety pilot** (holds the RC, does nothing else, never rotates), **operator** (runs the laptop), **spotter** (watches the area and battery), **analyst** (logs, scoring).

## 0. Go / no-go before leaving the lab

- [ ] SITL passes 3 consecutive runs and F1 is recorded (see the main guide).
- [ ] `colcon build` + `colcon test --packages-select milestone1_singlevehicle` pass (32 tests).
- [ ] `pip install pyulog` on the scoring laptop.
- [ ] Drone's PX4 version known: ____. It must publish the same topic names as the SITL build: on the bench, `ros2 topic list | grep -E "vehicle_local_position|vehicle_command_ack"` must show `..._v1` versions. If not, the node and `m1_wait_ready` need the names changed (two files).
- [ ] The `px4_msgs` in this workspace matches the drone's firmware message definitions (a mismatch shows as `RTPS_READER_HISTORY Error` or no data). It currently comes from PX4 `v1.18.0-beta1-1182-ga99287097a`. **VERIFY**
- [ ] Site, weather (wind low enough to hold hover), at least 6 charged batteries, safety pilot confirmed, 15 m x 15 m clear area (path is 5 m square at 3-6 m altitude, plus 5 m margin).

## 1. Drone parameters (set in QGroundControl, then reboot PX4)

Parameters live on the drone, not in the launch file (`launch_sitl:=false` sets none). Record the values you actually used in the results log.

| Parameter | Value | Why |
|---|---|---|
| `MPC_XY_VEL_MAX` | 2 | Same as SITL, so results are comparable |
| `MPC_Z_VEL_MAX_UP` / `MPC_Z_VEL_MAX_DN` | 1 / 1 | Same as SITL |
| `MPC_YAWRAUTO_MAX` | 45 | Matches the node's yaw ramp (45 deg/s) |
| `MPC_TILTMAX_AIR` | check (scorer limit is 45 deg) | Tilt row |
| `COM_OBL_RC_ACT` | Hold (or Land); record it | What PX4 does when offboard setpoints stop. F1 depends on this |
| `COM_OF_LOSS_T` | 1 s (default) | Offboard-loss timeout |
| `COM_RCL_EXCEPT` | do **not** set the offboard bit while a safety pilot is present | RC loss must still trigger a failsafe |
| `NAV_RCL_ACT` | Return or Land | RC-loss action |
| `NAV_DLL_ACT` | Disabled for bench; for field set per team decision | PX4 refuses to arm with "No connection to the GCS" if a GCS link failsafe is set and QGC isn't connected. Keep QGC connected |
| `GF_ACTION`, `GF_MAX_HOR_DIST` 20, `GF_MAX_VER_DIST` 10 | Geofence on (Return/Land) | Backstop if the path is wrong |
| `COM_DISARM_LAND` | default | So landing auto-disarms (L1) |
| `COM_RC_IN_MODE` | RC required (default) | Do not fly with RC disabled |
| `CBRK_USB_CHK`, `CBRK_SUPPLY_CHK`, other circuit breakers | **leave at default** | Never bypass safety checks to make a test run |

RC mode switch: configure a switch for **Position** (or Stabilized) so the pilot can leave Offboard instantly, and a separate **kill switch** (`RC_MAP_KILL_SW`). Confirm both channels respond in QGC's radio screen. **VERIFY** the Offboard-to-Position takeover by flipping the switch while armed on the bench with props off.

## 2. Connections

Pick one link. The drone's PX4 must have the XRCE-DDS client enabled (`UXRCE_DDS_CFG` set to the port you use; `UXRCE_DDS_DOM_ID` = ROS domain, 0 unless changed). **VERIFY** for the airframe's flight controller.

**A. Serial (companion computer or USB-serial adapter to the FC's TELEM/ XRCE port)**

```
# set UXRCE_DDS_CFG = the serial port (e.g. TELEM 2); baud via SER_TEL2_BAUD (921600)
MicroXRCEAgent serial --dev /dev/ttyUSB0 -b 921600
```

**B. UDP (FC with ethernet, or via companion/MAVLink-bridge IP)**

```
MicroXRCEAgent udp4 -p 8888      # FC's XRCE client must target this host's IP:8888
```

Run the agent in its own terminal and leave it running. **The agent must not already be running as part of SITL** (`pgrep -x px4; pgrep -x MicroXRCEAgent` should show nothing you don't expect).

Confirm the link before touching the node:

```
source install/setup.bash
ros2 topic list | grep /fmu/            # topics appear within ~10 s of the agent connecting
ros2 topic echo --once /fmu/out/vehicle_local_position_v1   # xy_valid / z_valid true
```

If the drone's PX4 uses a namespace (fleet instance N), add `topic_prefix:=/px4_N` to the launch command; topics then appear under `/px4_N/fmu/...`.

QGroundControl must also be connected (MAVLink telemetry radio or USB), for parameters, the log download, and live monitoring.

## 3. Bench run, props OFF

Do this first on every new airframe and after any firmware, parameter or path change.

1. Remove props. Power drone, connect QGC and the agent. Wait for a GPS / position lock in QGC.
2. Place the drone **at its power-on position**. Waypoints are absolute local NED metres from the EKF origin, which is where PX4 started its estimator (normally where it was powered on). The node refuses to arm if the drone is more than 2 m (x/y) from the first waypoint (0, 0). If it prints `not arming: drone is N m (x/y) from the first waypoint`, move the drone back or reboot PX4 where it stands.
3. Launch with a countdown:

   ```
   ros2 launch milestone1_singlevehicle offboard_solution.launch.py launch_sitl:=false arm_delay_s:=10
   ```

4. Expected log lines: `PX4 is ready`, `loaded 22 waypoints`, then `arm command sent`, `offboard mode command sent`, `arm and offboard mode accepted by PX4`. In QGC: Armed, mode Offboard, motors spinning (idle).
5. `ros2 topic hz /fmu/in/trajectory_setpoint` shows about 10 Hz.
6. Flip the RC mode switch away from Offboard: PX4 must leave Offboard. **Ctrl-C the node immediately** (see section 6). Disarm with the RC/kill switch.
7. Negative checks: start the node with the drone moved 5 m away (must not arm); start with the agent stopped (`m1_wait_ready` must wait, then the launch ends after 300 s with `PX4 never became ready`).

## 4. Field checklist (every flight)

- [ ] Area clear, spotter in place, nobody within 10 m of the path. Safety pilot stands where they can see the drone and has the transmitter on.
- [ ] Battery fully charged and secured, props on and tight, props spin the right way. Motors tested by pilot in Manual/Stabilized only if the team's normal procedure allows.
- [ ] Drone on the ground at its power-on position (EKF origin), nose pointing **north** (path starts at yaw 0), north (+x) and east (+y) clear for 5 m + margin.
- [ ] QGC shows good GPS lock (>=10 satellites, HDOP low), no preflight warnings, home position set.
- [ ] RC link strong; kill switch and mode switch tested within the last 5 minutes.
- [ ] Wind below the team limit. Parameters match section 1 (re-check after any reflash).
- [ ] XRCE agent running; topics present and `xy_valid`/`z_valid` true.
- [ ] Log recording enabled (`SDLOG_MODE`, default logs from arming). Note the time and flight number.
- [ ] First flight of the day or new firmware: fly the short path first (`path:=` pointing at `milestone1_updown.yaml`, 2 waypoints), then the square, then the full path.
- [ ] Operator announces "arming in 10" and starts:

  ```
  ros2 launch milestone1_singlevehicle offboard_solution.launch.py launch_sitl:=false arm_delay_s:=10
  ```

- [ ] Hands-on: safety pilot's thumb on the mode switch (Offboard to Position) and kill switch is within reach for the entire flight.

## 5. Flight plan (suggested order)

1. Flight 1: `milestone1_updown.yaml` (climb/descend), confirms S1, L1, hover.
2. Flight 2: `milestone1_square.yaml`.
3. Flight 3: full `milestone1_path.yaml`.
4. Flight 4: F1: with the full path running, Ctrl-C the node mid-path with the pilot ready. Record which failsafe PX4 ran (`COM_OBL_RC_ACT`). It must match the expected action from section 1.
5. Repeat the full path if time allows. Land and swap batteries between flights.

Pass criteria are the HITL list in the main guide plus HITL thresholds in the path file.

## 6. Abort criteria and procedures

**Take over immediately (flip out of Offboard to Position, and Ctrl-C the node) if:**

- unexpected motion: drift beyond about 2 m from the path, oscillation, flip tendency, tilt above about 35 deg;
- yaw spinning when no yaw change is commanded;
- altitude loss, or drone not holding altitude on a hold;
- telemetry/RC link loss warnings, GPS loss, low battery warning, or any PX4 failsafe the plan didn't expect;
- a person or animal enters the area.

**Kill (kill switch; the drone falls) only for a flyaway or imminent injury** that Position mode can't fix.

**IMPORTANT: stop the node after any takeover.** The node cannot see the pilot's mode change (it does not read `vehicle_status`). If left running after a takeover it will keep publishing setpoints and, after finishing the path, send a LAND command that overrides the pilot. The operator must Ctrl-C the launch the moment the pilot flips the switch. Flying through `swarm_launcher`, the agent sees the mode change and stops the launch itself. **VERIFY** this behaviour on the bench (step 6 of section 3).

**Abort the session** (from the main guide) if wind affects the hover, position lock is poor, or the safety pilot has to take over twice.

If the node reports `giving up: no arm + offboard within 30 s`, PX4 refused: read the QGC message (preflight check, no GCS link, geofence, no position). The node disarms if arm was accepted but offboard was not. If it reports `not arming: ...`, fix the stated cause.

## 7. After the flight: pull and score the log

1. Disarm, power down props-safe, then in QGC: Analyze Tools, Log Download, select the log for each flight, save as `YYYY-MM-DD_flightN_<path>.ulg`. Or pull from the SD card (`log/<date>/`). Note: file date on a downloaded log is the download date, check the report's date field.
2. Copy the `.ulg`, the path YAML used and a short note (parameters, wind, battery, who was pilot, any takeover) into one folder.
3. Score with the HITL thresholds, using the **same path file the flight used**:

   ```
   source install/setup.bash
   ros2 run milestone1_singlevehicle m1_analyze ~/flights/2026-xx-xx_flight3_full.ulg \
     --stage hitl --path src/milestone1_singlevehicle/resource/milestone1_path.yaml
   ```

   Reports (Markdown + JSON) go to `~/px4_ros_com_ws/log/m1_reports/`. Exit code 1 means FAIL or INCOMPLETE, not a crash.
4. Read the report: S1 (armed into Offboard), L1 (landed and disarmed), R1 (tilt), T1-T9, Y1-Y2, H1, F1. A pilot takeover shows up as a nav-state change out of Offboard and fails the run. F1 is only scored on the failsafe flight.
5. Cross-check one leg by eye in PlotJuggler or PX4 Flight Review. Put worst-case numbers in the results log (main guide), with the log file name.

## 8. Known gaps and risks

- No hardware test of this node was possible when this was written. Everything marked VERIFY is untested on hardware.
- The node cannot detect a pilot takeover or a mid-flight PX4 failsafe (section 6). The pilot and the operator's Ctrl-C are the safeguard.
- The pre-arm check uses only the local position estimate and distance to the first waypoint. It does not check battery, GPS quality or wind.
- Waypoints are relative to the EKF origin. Moving the drone after power-on without rebooting PX4 changes the real-world path; the start-offset check catches large offsets only.
- Firmware drift: the node uses `..._v1` topic names and `px4_msgs` from this workspace. A different PX4 version can silently break subscriptions.
- The launch file does not start the XRCE agent for hardware; start it yourself (section 2).
- Single instance only. Fleet drones need `topic_prefix` and, for MAVLink command targeting, a non-1 system id (the node always targets system 1, component 1). Milestone 2 must address this.
