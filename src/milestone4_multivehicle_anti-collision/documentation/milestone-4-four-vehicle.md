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

Build/run commands are in README. Stop the previous simulator before changing
spawns. To record from before mission startup:

```bash
ros2 launch milestone4_multivehicle px4_sitl.launch.py headless:=false
```

In another sourced terminal (records all four PX4 streams and the shared topic):

```bash
ros2 bag record -o m4_run01 -e '^(/milestone4/global_positions|/rosout|/fmu/.*|/px4_[123]/fmu/.*)$'
```

Then launch controllers:

```bash
ros2 launch milestone4_multivehicle collision_launch.py launch_sitl:=false
```

With custom origins/count, pass identical values to simulator and controllers.
Controller-only launches do not apply the speed parameters to existing PX4s.
Head-on routes have two waypoints: climb/hold 10 s at 5 m, then fly the diagonal
and hold 5 s before land. Arrival times differ; no shared departure barrier
is introduced, preserving the three-field communications protocol.

## Acceptance tests (pending SITL)

1. Verify all four DDS namespaces, MAV_SYS_IDs and global origins against Gazebo.
   Confirm only three values per shared message and correct motion on both axes.
2. Verify four separated baseline flights, independent path overrides, all
   actual land/disarm events and a distinct ULog under rootfs/0..3/fs/log.
3. Remove any readiness input: no mission should start. Duplicate any namespace
   or system ID or supply an invalid fourth path: launch must reject the fleet.
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
| | | | | | | | | | |

## Scope of validation and known policy limits

Unit tests check decisions and launch wiring, not DDS transport or flight
physics. `m4_analyze` remains the single-vehicle baseline scorer with
`--vehicle 0`, `1`, `2` or `3`; its report is not an anti-collision certificate.
Baseline YAMLs retain thresholds; head-on YAMLs are controller-only and should
be evaluated from paired/fleet recordings, not passed to the baseline scorer.
Avoidance targets may be unmatched in baseline scoring; do not suppress them.

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
