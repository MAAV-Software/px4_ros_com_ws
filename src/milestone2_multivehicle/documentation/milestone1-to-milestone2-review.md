# Review: Milestone 1 → Milestone 2 package

This package copies `src/milestone1_singlevehicle` from branch
`worktree-three-launch-files` (baseline commit `9509c8b`) and adapts it to
milestone 2 in [PLAN.md](../../../PLAN.md). The original package is unchanged.

## Summary

- Two independent instances of the original controller replace one instance.
- Every command and subscription is routed to one explicit vehicle.
- The same full, square and up/down paths remain available, interpreted in
  each vehicle's own local NED frame.
- SITL starts two spaced x500s in one world with one UDP agent.
- Post-flight scoring is retained per vehicle. Manual pair-level evidence
  adds isolation, concurrency and independent-completion checks.
- Anti-collision and shared route coordination remain milestone 3 work.

## Changes by file

| File | Change |
|---|---|
| `package.xml`, `setup.py`, `setup.cfg`, resource marker | Rename package; install documentation and new process helper |
| `milestone2_multivehicle/offboard_control_solution.py` | Add vehicle routing parameters; prefix all five topics; target the correct MAV_SYS_ID |
| `milestone2_multivehicle/vehicle.py` | Shared validated namespace/system-ID resolution and duplicate pair checks |
| `milestone2_multivehicle/wait_for_px4.py` | Watch selected vehicle's local position; reject non-finite positions/heading |
| `launch/offboard_solution.launch.py` | Validate both paths and identities; wait for both vehicles before starting two named controllers |
| `launch/px4_sitl.launch.py` | One agent, two PX4 instances; start second after first readiness; explicit DDS namespaces and spaced spawns |
| `milestone2_multivehicle/sitl_process.py` | Isolated working folders/process groups; stop this run's descendants without machine-wide `pkill` |
| `launch/square_launch.py`, `launch/updown_launch.py` | Same wrapper pattern, now launching two controllers |
| `resource/milestone2_*.yaml` | Renamed copies; waypoint values and thresholds unchanged |
| `milestone2_multivehicle/analyze_flight.py` | `m2_analyze`, required vehicle label and separate report filenames |
| `milestone2_multivehicle/scoring.py` | Same scoring math; milestone 2/vehicle labels in report and results row |
| `test/test_scoring.py` | Copied metric regression tests with package/path names updated |
| `test/test_multivehicle.py` | Routing, independent controller state, readiness, and launch regression checks |
| `documentation/` | Updated test guide, test workflow and this review; stale milestone 1 implementation claims removed |

## What stays the same

The 10 Hz control loop, 0.5 m / 5° arrival criteria, continuous waypoint holds,
45°/s yaw ramp, arm/Offboard retry timing, position-only Offboard control,
QoS profile, message version suffixes and landing behavior are inherited.
Each node owns its own state. No controller subscribes to its peer or adjusts
its trajectory in response to the other aircraft.

## Runtime parameters

| Node parameter | Default | Meaning |
|---|---|---|
| `vehicle_instance` | 0 | Default routing index: 0 -> bare topics, N -> `/px4_N`; default system ID N+1 |
| `topic_namespace` | `auto` | Override with observed DDS prefix, or `/` for bare topics |
| `target_system_id` | 0 (auto) | Explicit hardware MAV_SYS_ID; auto resolves to instance+1, never broadcast |
| `waypoints_file` | Empty (required) | Per-controller local-NED path; controller only |

The pair launch uses fixed logical labels 0 and 1. Its `namespace_1/2` and
`system_id_1/2` arguments override routing for those labels on hardware.
The launch's built-in simulation uses PX4 instances 0/1, system IDs 1/2.

## Review and validation

- [ ] Build using `colcon build --packages-up-to milestone2_multivehicle`
- [ ] Run package tests; inspect both launch argument lists
- [ ] Verify both topic sets and explicit target IDs in the ROS graph
- [ ] Fly each alone, then concurrently, then with different path lengths
- [ ] Validate readiness failure and one-controller-loss response in SITL
- [ ] Save and score both logs per run; complete manual M1–M7 matrix
- [ ] Record actual airframe IDs, links, namespaces and separate physical routes
- [ ] Complete both real-airframe acceptance checklists before milestone sign-off

Python/unit checks can run without ROS. Full ROS launch, DDS, SITL and hardware
acceptance must run in the team's flight environment. Unit-test doubles do not
validate real DDS transport or physical flight behavior. No acceptance flight
or milestone completion is claimed by adding this package.

## References

- [PX4 multi-vehicle ROS 2 routing](https://docs.px4.io/main/en/ros2/multi_vehicle)
- [PX4 multi-vehicle Gazebo startup](https://docs.px4.io/main/en/sim_gazebo_gz/multi_vehicle_simulation)
- [Milestone 2 test guide](milestone-2-multi-vehicle.md)
- [Testing and scoring](testing.md)
