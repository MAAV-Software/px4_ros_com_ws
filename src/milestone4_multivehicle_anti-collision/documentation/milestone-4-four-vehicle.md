# Milestone 4: four-vehicle SITL and anti-collision

This is the milestone 3 package extended to four vehicles. Milestone 4 is
sim-only in PLAN.md; real-airframe approval is not implied. Earlier milestone
guides are retained as historical references; this guide and README describe
the active package and override their two-vehicle commands.

## Focused changes

- Rename the active package, entry points and common position topic to milestone 4.
- Add launch arguments for vehicles 3/4, with default `vehicle_count:=4`.
- Start one XRCE agent and four PX4 instances sequentially: instance 0 creates
  the world; each later standalone instance starts after the preceding one's
  local position is ready. Separate rootfs/log folders and cleanup are retained.
- Validate every namespace/system-ID pair, every path and all origins before
  launching controllers. All four readiness checks must succeed before any
  controller starts. A readiness failure aborts the fleet launch.
- Expand avoidance to IDs 0–3. Keep the latest XY for all peers, compare against
  every higher-priority peer, and never release while any higher peer is within
  5 m or missing/stale. Instance 3 has no higher peers and does not wait for them.
- Permit instances 2/3 in `m4_sitl_process` and vehicle labels 2/3 in `m4_analyze`.
- Add four diagonal head-on YAMLs and use them in `collision_launch.py`.
- Lower the built-in SITL XY speed cap to 0.5 m/s for the requested slower tests.

No changes to baseline waypoint coordinates, scoring math/thresholds, 5 m
radius, one-time 5 m climb, braking criterion (<=0.2 m/s for 0.5 s), 10 Hz loop,
command routing semantics, arm retry logic, yaw ramp or landing procedure.
The existing two-drone offset-pass resources are retained for regression tests.

**Since 2026-11-11** the package has only `collision_launch.py`, flying one
vehicle per launch. `offboard_solution.launch.py`, `px4_sitl.launch.py`,
`square_launch.py`, `updown_launch.py`, `m4_sitl_process`, the `m4_analyze`
scorer and the baseline path/square/updown YAMLs were removed. SITL drones are
started from the web launcher's Fleet tab (20 m square layout). The list above
describes the change as first made.

## Mapping and arguments

| Instance | Path argument | World origin East, North (m) | DDS namespace | MAV_SYS_ID |
|---|---|---|---|---|
| 0 | path_1 | 0, 0 | / | 1 |
| 1 | path_2 | 20, 0 | /px4_1 | 2 |
| 2 | path_3 | 20, 20 | /px4_2 | 3 |
| 3 | path_4 | 0, 20 | /px4_3 | 4 |

Arguments `spawn_x_N`, `spawn_y_N`, `namespace_N`, `system_id_N`, `path_N`
exist for N=1..4. `path` remains a common fallback. `vehicle_count` accepts
2, 3 or 4 (default 4). All nodes in a fleet must have the same vehicle_count.
Changing namespaces/system IDs does not change instance priority.
Built-in SITL requires MAV_SYS_ID=instance+1.

The controller declares `vehicle_count` (default 4), `vehicle_instance`, and
`initial_global_x/y` (defaults from the table). The launch uses the same origin
values for simulator poses and controller parameters. Shared XY is
`origin_x + local.y, origin_y + local.x`, converting local NED to East/North.
Origins stay fixed; estimator origin resets/drift are not corrected.

The common topic is `/milestone4/global_positions`, type
`std_msgs/msg/Float64MultiArray`, data `[instance, global_x, global_y]`, empty
layout. Best-effort, volatile, depth 10, published at 10 Hz when own position
is fresh/valid, including after the land request. Receivers track freshness
locally (1 s); no timestamp, Z or mission state is transmitted.

A lower drone needs live messages from *every* configured higher instance to
arm or resume. On peer loss it holds; a missing peer is never assumed clear.
A latched encounter finishes its fixed climb and waits for fresh clearance.
Own invalid/stale position pauses progress without rebroadcasting stale data.
During avoidance no waypoint is consumed, and its continuous hold is reset.
After a land request, PX4 owns descent; avoidance does not override Land.
A landed higher-priority peer therefore still occupies its XY radius.

## Running and recording

Build/run commands are in README. Start four SITL drones from the web launcher's
Fleet tab in the **20 m square** layout. To record from before mission startup,
in a sourced terminal (records all four PX4 streams and the shared topic):

```bash
ros2 bag record -o m4_run01 -e '^(/milestone4/global_positions|/rosout|/fmu/.*|/px4_[123]/fmu/.*)$'
```

Then start the mission from the web launcher's Mission tab, or launch the
controllers by hand, one vehicle per sourced terminal:

```bash
ros2 launch milestone4_multivehicle collision_launch.py system_id:=1
ros2 launch milestone4_multivehicle collision_launch.py topic_prefix:=/px4_1 system_id:=2
ros2 launch milestone4_multivehicle collision_launch.py topic_prefix:=/px4_2 system_id:=3
ros2 launch milestone4_multivehicle collision_launch.py topic_prefix:=/px4_3 system_id:=4
```

Each starts its controller once its own PX4 is ready, so there is no fleet-wide
readiness gate: start them close together.

With custom origins/count, pass identical values to every controller, matching
where the drones really started. The controller sets no PX4 speed parameters.
Head-on routes have two waypoints: climb/hold 10 s at 5 m, then fly the diagonal
and hold 5 s before land. Arrival times differ; no shared departure barrier
is introduced, preserving the three-field communications protocol.

## Acceptance tests (pending SITL)

1. Verify all four DDS namespaces, MAV_SYS_IDs and global origins against Gazebo.
   Confirm only three values per shared message and correct motion on both axes.
2. Verify independent path overrides, all actual land/disarm events and a
   distinct ULog under rootfs/0..3/fs/log.
3. Remove a drone's readiness input: its mission must not start. Supply an
   invalid path or system ID: that drone's launch must refuse before arming.
4. Exercise each priority pair. Only the lower instance should yield at <=5 m.
   Record actual velocities, fixed XY braking target, and one captured z-5 target.
5. Place two higher peers inside a lower drone's radius. Moving one out must
   not release the lower drone. Move the last one out; it should resume the
   original waypoint only after completing its climb and with all peers fresh.
6. Test concurrent conflicts, especially instances 0/1/2 yielding together.
   Measure actual minimum separation; inspect contact, climb interactions and
   stationary chains. Do not mark a stalled run as passed simply because no contact occurred.
7. In external SITL, stop one controller: higher-priority surviving drones
   continue; lower ones hold when its data expires. Verify stopped PX4 failsafe.
8. Repeat normal encounter runs three times; retain bag, four ULogs, exact
   paths/origins, versions, state transition logs and outcome notes.

| Run | Routes/origins | Routing/readiness | Priority pairs | Multi-peer wait/release | Peer loss | Min separation | All completed | Bag / four logs | Result |
|---|---|---|---|---|---|---|---|---|---|
| 1 (2026-11-11, `3091fcc`) | Head-on YAMLs; web launcher SITL, 20 m square layout, MPC_XY_VEL_MAX 2 m/s | All four namespaces, system IDs and origins correct; all ready before arming | 0, 1, 2 braked on higher peers; 3 never yielded, as designed | 0 and 2 climbed to the same ~10 m layer; 0 re-latched and climbed again to 12.8 m | Not tested | **0.09 m** (1–3, both at 5 m); also 0–2 2.7 m (2.1 m vertical), 2–3 3.1 m, 0–3 3.9 m at the same height | Yes, but only after 1 hit the ground | `~/m4_runs/m4_headon_run01` on the SITL VM; no ULogs kept | **FAIL**: mid-air collision |
| 2 (2026-11-12, `12bf939` + SITL cap change) | Same head-on YAMLs and square layout; MPC_XY_VEL_MAX **1 m/s** (measured max 1.05–1.06 m/s on all four) | All four correct; all armed and accepted offboard | 0, 1, 2 braked within 0.9 s of each other on higher peers and climbed; 3 never yielded | 0 and 1 both climbed to ~10 m and held 4.98 m apart at the same height; 0 re-latched at t+77.2 s and climbed again to 12.9 m | Not tested | **2.12 m** (1–3: 1.52 m horizontal, 1.47 m vertical, 1 climbing through 6.5 m); 0–2 3.78 m, 1–2 4.46 m, 0–3 4.21 m, 2–3 3.80 m, 0–1 4.98 m | Yes, all four landed; mission complete in 1:32 | `~/m4_runs/m4_headon_run02` on the SITL VM; no ULogs kept | **No contact, small margin** |
| 3 (2026-11-12, uncommitted ORCA) | Same head-on YAMLs and square layout, drones restarted at their spawns; MPC_XY_VEL_MAX 1 m/s; `avoidance:=orca`, `max_speed:=1.0` (measured max 1.04–1.05 m/s) | All four correct; all armed and accepted offboard | n/a: no priority; all four AVOIDING from t+84.8–85.1 s to t+91.5–91.9 s | No climbs: all four stayed at 5.0 m | Not tested | **2.99 m** (1–2), 0–1 3.03 m, 0–3 3.05 m, 2–3 3.08 m, 0–2 4.24 m, 1–3 4.36 m; all at the same height | Yes, all four reached waypoint 1 within 0.2 s of each other and landed; mission complete in 1:04 | `~/m4_runs/m4_headon_run03_orca` on the SITL VM; no ULogs kept | **No contact; every pair held the 3 m ORCA spacing** |
| 4 (2026-11-12, uncommitted ORCA + scan) | `scan_launch.py` field scan; web launcher "South edge of the field" layout, test field placed from it; 1 m/s, `avoidance:=orca` (max 1.07–1.09 m/s) | All four correct; all armed | n/a; AVOIDING 11 times, 1–10 s each | No climbs: all at 5.0 m | Not tested | **3.01 m** (1–3 and 2–3, each with drone 3 crossing a strip); 0–3 4.63 m, 0–1 5.60 m | Drones 0–2 yes (airborne 410, 413 and 589 s). Drone 3 was aborted by the operator at about 10 min, about halfway through its 46 rows | `~/m4_runs/m4_scan_run01_orca` (positions and /rosout only) | **No contact, but strip coverage only 79%** |
| 5 (2026-11-12, uncommitted ORCA + scan, drift fix) | Same as run 4; ORCA now biases right only while it changes the velocity, and follows the leg from the previous waypoint | All four correct; all armed | n/a; AVOIDING 18 times, 1–10 s each | No climbs: all at 5.0 m | Not tested | **2.99 m** (1–3 and 2–3); 0–3 3.03 m, 0–1 5.88 m | Drones 0–2 yes. Drone 3 aborted by the operator once it was the only one flying | `~/m4_runs/m4_scan_run02_orca` (positions and /rosout only) | **No contact; strip coverage 99.8%** |

Run 1 notes: drones 1 and 3 fly the same diagonal head-on. They closed at about
4 m/s, so drone 1 started braking (t+68.3 s, 5.7 m apart) only about 1.3 s before
contact and never reached its climb. Drone 3 flew through its position at 2 m/s
(centres 0.09 m apart at t+70.5 s). Drone 1 fell from 5 m to the ground within
1 s, recovered, and still finished its route; drone 3 was knocked to 3.8 m/s and
dropped to 4.1 m. The launcher reported the run as Complete. The 5 m trigger
needs about 5 s at this closing speed (brake, 0.5 s stop, 5 m climb at 1 m/s),
so at the web launcher's 2 m/s cap the policy cannot clear a head-on pair.

Run 2 notes: the web launcher's SITL cap was lowered to 1 m/s. Drone 1 braked at
t+67.3 s and started climbing at t+70.3 s, after the 0.5 s stop hold. Drone 3
passed under it 1.5 s into the climb (t+71.8 s), 1.52 m apart horizontally and
1.47 m vertically. Halving the speed avoided contact only because the climb had
just begun; the policy still has no margin for a head-on pair. The same-layer climb
(0 and 1 at 10 m) and drone 0's second climb from run 1 repeated.

Run 3 notes: ORCA, with horizontal velocity setpoints. All four left waypoint 0
within 0.3 s of each other and met near (10, 10). Each sidestepped right for
about 7 s at its own 5 m height, and the closest pass was 2.99 m, the two
1.5 m radii. The crossing barely slowed them: 30 s from waypoint 0 to waypoint
1, against 28.3 s for the straight 28.3 m diagonal at 1 m/s, where run 2's
drone 0 took 51 s. In unit-test simulation the same four-way crossing also
clears at 0.5–3 m/s with velocity tracking lags of 0.2–0.8 s; at 2 m/s it needs
the roundabout bias, without which all four stopped in a 3 m square.

Run 4 notes: the first field scan. Separation was fine, but drones 0–2 covered only
79% of the field between them (all four, with drone 3's partial sweep, 89%), measured
from each track with a 2 m by 3 m footprint oriented by heading. Their cross-track error
was 0.7–0.8 m median and up to 4 m: going north they flew about 1.1 m east of the row
on average, and going south about 1 m west, so pairs of rows bowed toward each other
(coverage peaks at 6, 10, 14 and 18 m east). The cause is ORCA's 10 degree right bias,
meant to break head-on symmetry: it applies whenever any peer is within 10 m, and
neighbouring strip drones are 8 m apart for the whole scan. With the preferred velocity
pointed at the row's end point, the biased drone curves off the row rather than back
onto it.

Run 5 notes: with the fix, drones 0–2 covered 99.82% of the field between them (62
cells of 0.25 m missed, mostly at turns), against 79% in run 4. Cross-track error on the
strip rows was 0.01–0.02 m median, 0.03 m at the 90th percentile for drones 0 and 1 and
0.47 m for drone 2, whose rows drone 3 crossed most; the worst was 1.0 m during a
sidestep. Airborne times read 797, 794 and 976 s, but every drone's position stream
stopped at t+363 s for 387 s at once, with no drone moving: the SITL VM itself was paused
(its journal is empty from 16:47 to 16:58 UTC, then "Clock change detected"). Without that
gap drones 0 and 1 flew about 410 s, as in run 4, so the fix cost no time.

## Scope of validation and known policy limits

The ORCA policy (`avoidance:=orca`, the default since 2026-11-12) replaces the
rule this milestone specified. The README describes it. It shares two more
numbers per message (velocity), so it doesn't keep the three-field protocol
described above, and its limits differ: it is horizontal only, assumes every
peer runs ORCA, and has been flown once in SITL (run 3). The notes below are
about the priority rule, still available as `avoidance:=priority`.

Unit tests check decisions and launch wiring, not DDS transport or flight
physics. Head-on YAMLs are controller-only and are evaluated from fleet
recordings; the package no longer has a per-vehicle flight scorer.

The requested XY-only rule does not guarantee collision-free four-way convergence.
Braking and climbing take time. Multiple lower drones at the same initial height
all climb the same 5 m; no vertical separation is allocated between them. A chain
of nearby stationary higher peers can make lower vehicles wait indefinitely.
The policy cannot detect a vehicle above the climbing drone. These are retained
algorithm limitations, not resolved by launching four processes or slowing them.
No altitude lanes, Z sharing, prediction, or recovery procedure was added.

The package implements the requested four-instance behavior. Milestone flight
acceptance remains pending until the above simulator tests demonstrate acceptable
behavior on the selected predetermined routes.
