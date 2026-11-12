# Milestone 4 testing

Run from the ROS workspace:

```bash
colcon test --packages-select milestone4_multivehicle
colcon test-result --verbose
```

Without ROS, from this package directory:

```bash
python3 -m pip install pytest PyYAML
PYTHONPATH=. python3 -m pytest test -q
```

The two-vehicle regression tests remain, explicitly selecting vehicle_count=2.
`test_four_vehicles.py` adds every priority pair, multiple blockers, stale-peer
checks, four controllers sharing positions and invalid fleet configuration.
`test_multivehicle.py` also checks `collision_launch.py` for all four vehicles.
The full suite requires pytest and PyYAML. See the milestone 4 guide for SITL
acceptance, recording and stress-test limits.

## Checks performed when milestone 4 was first written

Python syntax, package metadata, installed resources and baseline waypoint/threshold
preservation were checked. An offline lightweight runner executed 114 cases from
the routing, avoidance and four-vehicle test modules: all passed. This runner
provided the small fixture/parameterization interface needed by those tests;
it was not pytest. pytest was unavailable and its installation timed out.
The full pytest/colcon suite, real ROS/DDS transport and PX4/Gazebo flights
remain unverified here. Run the commands above in the flight workspace.

On 2026-11-11 the baseline launch files, the SITL launch, the `m4_analyze` scorer
and the baseline path files were removed; `collision_launch.py` is the only launch
file. The pytest suite then had 76 cases, all passing.
