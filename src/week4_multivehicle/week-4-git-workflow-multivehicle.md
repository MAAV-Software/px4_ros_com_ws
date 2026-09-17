# Week 4: Git Workflow + Multi-Vehicle SITL

**Dates:** Oct 6-11, 2026
**Outcome:** Team is functionally at the starting line for Milestone 1 (PLAN.md).

## Topics

- This repo's git workflow: branching, commit conventions, PR/review process the team actually uses
- Running two PX4 SITL instances simultaneously (multi-vehicle simulation basics)
- Namespacing topics/nodes per vehicle instance so multiple drones don't collide on the same topic names
- Recap and connect: how weeks 1-3 combine into the Milestone 1/2 work in PLAN.md

## Hands-on tasks

1. Practice the team's actual git workflow on a throwaway branch: branch, commit, open a PR against `dev-test` (or whatever the working branch is), get it reviewed.
2. Launch two SITL instances at once; confirm each drone's topics are distinguishable (namespace or instance ID).
3. Adapt the week 3 single-drone waypoint node to target one specific vehicle instance by namespace.
4. Group discussion: walk through the PLAN.md milestone table and make sure every new member understands what Milestone 1 and 2 actually require.

## Instructions

### Task 1 — Practice the git workflow

The team's git conventions weren't formally specified when this doc was drafted, so this uses the pattern this repo has actually been using. Revisit this section once real conventions are settled.

1. Branch off `dev-test`: `git checkout dev-test && git pull && git checkout -b <your-name>/week4-practice`
2. Make a small, real edit to [`src/week4_multivehicle/PRACTICE.md`](PRACTICE.md) — add your name under "Roster". (This file exists specifically for this exercise: `onboarding/` is gitignored, so edits there never produce a diff to practice with.)
3. Commit with a concise, imperative-mood summary line (matching this repo's existing commit style, e.g. `git log --oneline`) — e.g. `git commit -m "Add <your name> to week 4 practice roster"`.
4. Push your branch and open a PR against `dev-test`.
5. Get at least one other person (new or returning member) to review and approve before merging.

### Task 2 — Launch two SITL instances

Kill any single-instance SITL session from previous weeks first (this script does it for you). From `~/PX4-Autopilot`:
```
export PX4_SYS_AUTOSTART=4001
./Tools/simulation/sitl_multiple_run.sh 2 gz_x500 px4_sitl_default
```
This is PX4's own official multi-instance launcher. Instance 0 starts the Gazebo world and spawns `x500_0`; instance 1 detects the already-running world and spawns `x500_1` into it — you should see both in the Gazebo GUI.

Start the agent the same way as week 2 — **only one agent is needed for both instances**, since they share the same UDP port and are told apart by DDS namespace, not by port:
```
MicroXRCEAgent udp4 -p 8888
```

Confirm the namespacing:
```
ros2 topic list | grep fmu
```
You should see two sets of `/fmu/...` topics: instance 0's un-namespaced (`/fmu/out/vehicle_local_position`) and instance 1's prefixed (`/px4_1/fmu/out/vehicle_local_position`). This comes directly from PX4's own SITL startup script (`ROMFS/px4fmu_common/init.d-posix/rcS` in PX4-Autopilot): instance 0 gets no namespace, instance N gets `px4_N`.

### Task 3 — Target one vehicle instance by namespace

A `week4_multivehicle` package is provided at [`src/week4_multivehicle`](.), following the same stub + solution pattern as week 3:

- [`offboard_control_stub.py`](week4_multivehicle/offboard_control_stub.py) — your week 3 control logic (arm/takeoff/waypoints/land) is already filled back in, since that's last week's skill. What's new and left as a `TODO` is `topic()`, which needs to build the correct namespaced topic string for whichever `vehicle_instance` parameter it's given.
- [`offboard_control_solution.py`](week4_multivehicle/offboard_control_solution.py) — complete reference, including a working `topic()`.

Implement `topic()` in the stub: instance `0` → bare topic (`/fmu/...`), instance `N` → `/px4_N/fmu/...` (same rule you just observed in task 2). Then build and run it against instance 1 specifically:
```
colcon build --packages-select week4_multivehicle
source install/setup.bash
ros2 launch week4_multivehicle offboard_stub.launch.py instance:=1
```
Confirm vehicle 1 flies the waypoint path while vehicle 0 sits idle on the ground — that's the whole point of the exercise: one node, explicitly targeting one drone out of several by namespace, which is exactly the shape Milestone 4 (4 drones, anti-collision, predetermined routes) needs. You can pass `instance:=0` instead to confirm it also still works un-namespaced, and compare against `offboard_solution.launch.py` if you get stuck.

### Task 4 — PLAN.md milestone walkthrough

Group discussion, no code. Pull up [`PLAN.md`](../../PLAN.md)'s milestone table and go through it with the whole group (not just new members) until everyone can answer:
- What does Milestone 1 (1 drone autonomous) actually require that we haven't built yet, if anything, versus what onboarding weeks 1-3 already covered?
- What does Milestone 2 (2 drone) add on top of Milestone 1 — and how does this week's namespacing exercise (task 3) relate to it?
- Where does anti-collision (Milestone 3) start to matter, and why isn't it needed yet for Milestone 1/2?
- Which milestones are still blocked on external dependencies (object detection / path-finding integration, milestones 6-7), and how does that affect what Navigation can usefully work on next versus what's stuck waiting?

## Checkpoint

Each new member has landed at least one real PR through the team's workflow, and can launch + control a single vehicle within a 2-instance SITL setup. Team as a whole is ready to start Milestone 1 work the following week.

## Notes

Curriculum is a draft — this is also the natural point to fold in whatever the team's real git conventions turn out to be, since they weren't specified when this was drafted.
