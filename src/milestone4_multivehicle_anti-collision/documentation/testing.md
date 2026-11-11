# Milestone 4 testing

Run from the ROS workspace:

```bash
colcon test --packages-select milestone4_multivehicle
colcon test-result --verbose
```

Without ROS, from this package directory:

```bash
python3 -m pip install pytest numpy PyYAML
PYTHONPATH=. python3 -m pytest test -q
```

Existing scoring and two-vehicle regression tests remain, explicitly selecting
vehicle_count=2. `test_four_vehicles.py` adds four-way routing, every priority
pair, all 24 readiness completion orders, any readiness failure, invalid fleet
configuration, sequential PX4 startup, multiple blockers and stale-peer checks.
The full suite requires pytest, numpy and PyYAML. See the milestone 4 guide for
SITL acceptance, recording, stress-test limits and baseline ULog scoring scope.

Score baseline logs with the actual file path, repeating for instances 0–3:

```bash
ros2 run milestone4_multivehicle m4_analyze /actual/path/flight.ulg \
  --vehicle 3 --stage sitl --out /tmp/m4_baseline
```

Omit --path only for the installed milestone4_path.yaml. --vehicle is a report
label, not verification of log identity. Output prefixes include the vehicle
number, so four identical log basenames do not overwrite each other. The
collision_launch.py routes have no scoring metadata; evaluate them from the
shared ROS bag and four ULogs instead.

## Checks performed during this edit

Python syntax, package metadata, installed resources and baseline waypoint/threshold
preservation were checked. An offline lightweight runner executed 114 cases from
the routing, avoidance and four-vehicle test modules: all passed. This runner
provided the small fixture/parameterization interface needed by those tests;
it was not pytest. pytest was unavailable and its installation timed out.
The full pytest/colcon suite, real ROS/DDS transport and PX4/Gazebo flights
remain unverified here. Run the commands above in the flight workspace.
