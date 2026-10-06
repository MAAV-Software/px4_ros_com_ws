# px4_launch_blank

Blank offboard control template. Copy it and rename everything below to draft
a new mission — don't edit this package in place for real mission work.

## Copying for a new mission

Given a new mission name `<mission>` (e.g. `week5_survey`):

1. Copy the directory: `cp -r px4_launch_blank <mission>`
2. Rename the inner Python package dir: `mv <mission>/px4_launch_blank <mission>/<mission>`
3. Rename the resource marker: `mv <mission>/resource/px4_launch_blank <mission>/resource/<mission>`
4. In `<mission>/package.xml`: change `<name>` to `<mission>` and update `<description>`
5. In `<mission>/setup.py`:
   - `package_name = '<mission>'`
   - update `description`
   - update the console_script entry point, e.g. `offboard_control_blank = <mission>.offboard_control_blank:main`
6. In `<mission>/launch/offboard_control_blank.launch.py`:
   - `package='<mission>'`
   - `executable=` matches the entry point name from step 5
7. In `<mission>/<mission>/offboard_control_blank.py`:
   - update the node name in `super().__init__('offboard_control_blank')` if you want it distinct from other missions running at once
   - fill in `WAYPOINTS` for the mission (defaults are a 5m square at -5m altitude)
8. Build: `colcon build --packages-select <mission>`, then source the workspace.

## What's already wired up

- Vehicle-instance targeting via the `instance` launch arg / `vehicle_instance`
  ROS parameter (0 = bare topics, N = `/px4_N/...`, matching PX4 SITL namespacing)
- Arm -> offboard -> fly waypoints -> land state machine
- QoS profile matched to PX4's uXRCE-DDS bridge
