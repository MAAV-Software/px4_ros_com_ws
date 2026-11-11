"""Unit tests for what the agent works out from PX4 data and the node's log lines."""
import re
from pathlib import Path
from types import SimpleNamespace

import pytest

from swarm_launcher import flight_state
from swarm_launcher.flight_state import (
    FlightTracker, clean_log_line, launch_argv, log_level, versioned_topic)

PACKAGES = Path(__file__).resolve().parent.parent.parent / 'src'
NODE = PACKAGES / 'milestone1_singlevehicle' / 'milestone1_singlevehicle' / 'offboard_control_solution.py'


@pytest.fixture
def tracker(clock):
    return FlightTracker(clock)


def message_value(message, name):
    path = PACKAGES / 'px4_msgs' / 'msg' / f'{message}.msg'
    if not path.exists():
        pytest.skip('px4_msgs is not next to this package')
    text = path.read_text()
    return int(re.search(rf'^\w+ {name}\s+=\s+(\d+)', text, re.MULTILINE).group(1))


@pytest.mark.parametrize('name', [
    'VEHICLE_CMD_NAV_RETURN_TO_LAUNCH', 'VEHICLE_CMD_NAV_LAND',
    'VEHICLE_CMD_DO_SET_MODE', 'VEHICLE_CMD_COMPONENT_ARM_DISARM'])
def test_command_numbers_match_px4_msgs(name):
    assert getattr(flight_state, name) == message_value('VehicleCommand', name)


@pytest.mark.parametrize('name', [
    'ARMING_STATE_ARMED', 'NAVIGATION_STATE_MANUAL', 'NAVIGATION_STATE_ALTCTL', 'NAVIGATION_STATE_POSCTL',
    'NAVIGATION_STATE_AUTO_LOITER', 'NAVIGATION_STATE_AUTO_RTL', 'NAVIGATION_STATE_POSITION_SLOW',
    'NAVIGATION_STATE_ALTITUDE_CRUISE', 'NAVIGATION_STATE_ACRO', 'NAVIGATION_STATE_DESCEND',
    'NAVIGATION_STATE_OFFBOARD', 'NAVIGATION_STATE_STAB', 'NAVIGATION_STATE_AUTO_LAND',
    'NAVIGATION_STATE_AUTO_PRECLAND'])
def test_status_numbers_match_px4_msgs(name):
    assert getattr(flight_state, name) == message_value('VehicleStatus', name)


def test_accepted_result_matches_px4_msgs():
    assert flight_state.VEHICLE_CMD_RESULT_ACCEPTED == message_value(
        'VehicleCommandAck', 'VEHICLE_CMD_RESULT_ACCEPTED')


@pytest.mark.parametrize('phrase', [
    'loaded {', 'advancing to waypoint {', 'Final hold complete; landing',
    'arm and offboard mode accepted by PX4', 'not flying: no arm + offboard'])
def test_the_log_lines_parsed_here_still_exist_in_the_node(phrase):
    """The agent reads these lines until the node publishes real progress. If this
    fails, someone reworded a log line and flight_state.py has to follow."""
    assert phrase in NODE.read_text()


def test_launch_argv_passes_changed_arguments_and_the_launchers_own():
    launch = {'package': 'pkg', 'file': 'a.launch.py', 'args': {'record': 'true'},
              'declared': ['launch_sitl', 'record', 'system_id', 'topic_prefix']}
    argv = launch_argv(launch, '/px4_1', 2)
    assert argv == ['ros2', 'launch', 'pkg', 'a.launch.py', 'record:=true',
                    'launch_sitl:=false', 'topic_prefix:=/px4_1', 'system_id:=2']


def test_launch_argv_only_sets_what_the_file_declares():
    launch = {'package': 'pkg', 'file': 'a.launch.py', 'args': {}, 'declared': []}
    assert launch_argv(launch, '/px4_1', 2) == ['ros2', 'launch', 'pkg', 'a.launch.py']


def test_launch_argv_leaves_out_an_empty_prefix():
    """ros2 launch refuses `topic_prefix:=`, which SITL's instance 0 would otherwise send."""
    launch = {'package': 'pkg', 'file': 'a.launch.py', 'args': {}, 'declared': ['topic_prefix', 'system_id']}
    assert launch_argv(launch, '', 1) == ['ros2', 'launch', 'pkg', 'a.launch.py', 'system_id:=1']


def test_log_level_and_cleaning():
    assert log_level('[node-1] [ERROR] [1.0] [x]: bad') == 'ERROR'
    assert log_level('[node-1] [FATAL] [1.0] [x]: bad') == 'ERROR'
    assert log_level('[node-1] [WARN] [1.0] [x]: hm') == 'WARN'
    assert log_level('plain output') == 'INFO'
    assert clean_log_line('\x1b[31mred\x1b[0m\n') == 'red'


def test_land_detected_drives_airborne(tracker):
    tracker.on_land_detected(True)
    assert not tracker.airborne and not tracker.was_airborne
    tracker.on_land_detected(False)
    assert tracker.airborne and tracker.was_airborne
    tracker.on_land_detected(True)
    assert not tracker.airborne and tracker.landed_after_flight


def test_position_fallback_when_land_detected_never_arrives(tracker, clock):
    tracker.on_position(0.0, 0.0, True)
    assert not tracker.airborne
    tracker.on_position(-2.0, -1.0, True)
    assert tracker.airborne
    # Low, but still descending: not landed yet.
    tracker.on_position(-0.1, 0.5, True)
    clock.advance(flight_state.LANDED_SETTLE_S + 1)
    tracker.on_position(-0.1, 0.5, True)
    assert tracker.airborne
    # Low and still, but not for long enough.
    tracker.on_position(0.0, 0.0, True)
    clock.advance(1)
    tracker.on_position(0.0, 0.0, True)
    assert tracker.airborne
    clock.advance(flight_state.LANDED_SETTLE_S)
    tracker.on_position(0.0, 0.0, True)
    assert tracker.landed_after_flight


def test_invalid_position_is_ignored(tracker):
    tracker.on_position(-50.0, 0.0, False)
    assert tracker.start_z is None and not tracker.airborne


def test_fresh_land_detected_beats_the_position_fallback(tracker):
    tracker.on_land_detected(True)
    tracker.on_position(0.0, 0.0, True)
    tracker.on_position(-5.0, 0.0, True)  # Estimator drift on the ground.
    assert not tracker.airborne


def test_armed_is_conservative_until_a_flight_has_ended(tracker):
    assert not tracker.armed
    tracker.on_arm_ack()
    assert tracker.armed  # Acknowledged, not yet flying.
    tracker.on_land_detected(False)
    assert tracker.armed
    tracker.on_land_detected(True)
    assert not tracker.armed


def test_begin_run_forgets_the_last_flight(tracker):
    tracker.on_arm_ack()
    tracker.on_land_detected(False)
    tracker.on_land_detected(True)
    tracker.begin_run()
    assert not tracker.was_airborne and not tracker.armed


def test_node_log_lines(tracker):
    tracker.on_log('[offboard_solution-2] [INFO] [1.0] [x]: loaded 22 waypoints from /a/b.yaml')
    assert tracker.waypoint_total == 22 and tracker.waypoint == 0
    tracker.on_log('[offboard_solution-2] [INFO] [1.0] [x]: advancing to waypoint 7')
    assert tracker.waypoint == 7
    tracker.on_log('[offboard_solution-2] [INFO] [1.0] [x]: Final hold complete; landing')
    assert tracker.landing


def test_giving_up_means_not_armed(tracker):
    tracker.on_arm_ack()
    tracker.on_log('[x] [ERROR] [1.0] [y]: not flying: no arm + offboard within 30 s; giving up (reason)')
    assert tracker.gave_up.startswith('Not flying: no arm')
    assert not tracker.armed


def test_offboard_node_death_is_seen_in_the_launch_output(tracker):
    """Real line from SITL. Launch itself keeps running while another process does."""
    tracker.on_log("[ERROR] [offboard_solution-3]: process has died [pid 9330, exit code -9, cmd '/x/offboard_solution']")
    assert tracker.node_died == 'offboard_solution died with exit code -9'


def test_recorder_death_is_not_a_flight_failure(tracker):
    tracker.on_log("[ERROR] [ros2-1]: process has died [pid 6228, exit code 2, cmd 'ros2 bag record']")
    assert tracker.node_died == ''


def test_versioned_topic_follows_the_message_version():
    assert versioned_topic('', 'vehicle_status', SimpleNamespace(MESSAGE_VERSION=4)) == '/fmu/out/vehicle_status_v4'
    assert versioned_topic('/px4_1', 'battery_status', SimpleNamespace(MESSAGE_VERSION=1)) == '/px4_1/fmu/out/battery_status_v1'
    assert versioned_topic('', 'vehicle_land_detected', SimpleNamespace(MESSAGE_VERSION=0)) == '/fmu/out/vehicle_land_detected'
    assert versioned_topic('', 'failsafe_flags', SimpleNamespace()) == '/fmu/out/failsafe_flags'


def test_px4_status_beats_inference_while_it_is_arriving(tracker, clock):
    tracker.on_land_detected(False)  # In the air, which would infer armed.
    tracker.on_status(False, flight_state.NAVIGATION_STATE_AUTO_LOITER)
    assert not tracker.armed
    tracker.on_status(True, flight_state.NAVIGATION_STATE_AUTO_LOITER)
    tracker.on_land_detected(True)  # Landed, which would infer disarmed.
    assert tracker.armed


def test_inference_takes_over_when_status_stops(tracker, clock):
    tracker.on_status(False, flight_state.NAVIGATION_STATE_AUTO_LOITER)
    tracker.on_land_detected(False)
    assert not tracker.armed
    clock.advance(flight_state.STATUS_MAX_AGE_S + 1)
    assert tracker.armed
    assert tracker.nav_state is None


@pytest.mark.parametrize('nav_state, expected', [
    (flight_state.NAVIGATION_STATE_OFFBOARD, 'Flying route'),
    (flight_state.NAVIGATION_STATE_AUTO_LOITER, 'Hold'),
    (flight_state.NAVIGATION_STATE_AUTO_RTL, 'Returning'),
    (flight_state.NAVIGATION_STATE_AUTO_LAND, 'Landing'),
    (flight_state.NAVIGATION_STATE_POSCTL, 'Pilot control, Position'),
    (flight_state.NAVIGATION_STATE_STAB, 'Pilot control, Stabilized'),
    (21, 'mode 21'),
])
def test_activity_in_the_air_comes_from_px4(tracker, nav_state, expected):
    tracker.on_land_detected(False)
    tracker.on_status(True, nav_state)
    # What the agent last commanded, or the launch state, must not change what PX4 says.
    assert tracker.activity(False, 'hold') == expected


def test_says_when_a_landed_drone_is_still_armed(tracker):
    tracker.on_land_detected(False)
    tracker.on_land_detected(True)
    tracker.on_status(True, flight_state.NAVIGATION_STATE_AUTO_LAND)
    assert tracker.activity(False, None) == 'Landed, still armed'
    tracker.on_status(False, flight_state.NAVIGATION_STATE_AUTO_LOITER)
    assert tracker.activity(False, None) == 'On ground'


def test_left_offboard_needs_to_have_been_in_offboard(tracker):
    tracker.on_status(False, flight_state.NAVIGATION_STATE_AUTO_LOITER)
    assert tracker.left_offboard == ''  # Hold before the node has switched modes.
    tracker.on_status(True, flight_state.NAVIGATION_STATE_OFFBOARD)
    assert tracker.left_offboard == ''
    tracker.on_status(True, flight_state.NAVIGATION_STATE_POSCTL)
    assert tracker.left_offboard == 'Position'
    assert tracker.pilot_control


def test_the_nodes_own_landing_is_not_leaving_offboard(tracker):
    tracker.on_status(True, flight_state.NAVIGATION_STATE_OFFBOARD)
    tracker.on_status(True, flight_state.NAVIGATION_STATE_AUTO_LAND)
    assert tracker.left_offboard == ''
    assert tracker.landing


def test_a_failsafe_return_is_leaving_offboard_not_landing(tracker):
    tracker.on_status(True, flight_state.NAVIGATION_STATE_OFFBOARD)
    tracker.on_status(True, flight_state.NAVIGATION_STATE_AUTO_RTL)
    assert tracker.left_offboard == 'Return'
    assert not tracker.landing
    assert tracker.coming_down


def test_begin_run_forgets_offboard(tracker):
    tracker.on_status(True, flight_state.NAVIGATION_STATE_OFFBOARD)
    tracker.begin_run()
    tracker.on_status(False, flight_state.NAVIGATION_STATE_AUTO_LOITER)
    assert tracker.left_offboard == ''


def test_px4_never_ready(tracker):
    tracker.on_log('[INFO] [launch.user]: PX4 never became ready; not starting the offboard node.')
    assert tracker.gave_up == 'PX4 never became ready'


@pytest.mark.parametrize('airborne, commanded, expected', [
    (True, 'hold', 'Hold'),
    (True, 'return', 'Returning'),
    (True, 'land', 'Landing'),
    (True, None, 'Flying route'),
])
def test_activity_in_the_air_without_status(tracker, airborne, commanded, expected):
    tracker.on_land_detected(not airborne)
    assert tracker.activity(True, commanded) == expected


def test_never_on_ground_while_in_the_air(tracker):
    """Seen in SITL: the node died, the launch stopped, and the card said On ground at 1.3 m."""
    tracker.on_land_detected(False)
    assert tracker.activity(False, None) == 'In the air, PX4 failsafe'
    tracker.on_land_detected(True)
    assert tracker.activity(False, None) == 'On ground'


def test_activity_on_the_ground(tracker):
    assert tracker.activity(False, None) == 'On ground'
    assert tracker.activity(True, None) == 'Starting'
    tracker.on_arm_ack()
    assert tracker.activity(True, None) == 'Armed'
