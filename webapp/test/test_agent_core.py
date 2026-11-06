"""Unit tests for the drone agent, using the pretend drone and a fake clock."""
import json

import pytest

from swarm_launcher import agent_core
from swarm_launcher.agent_core import ABORTED, DONE, FAILED, IDLE, LANDING, PREPARED, RUNNING, Agent
from swarm_launcher.flight_fake import CLIMB_S, FakeFlight
from swarm_launcher.flight_state import NAVIGATION_STATE_AUTO_LOITER, NAVIGATION_STATE_AUTO_RTL, NAVIGATION_STATE_POSCTL

LAUNCH = {'package': 'p', 'file': 'f.launch.py', 'args': {'arm_delay_s': '0'}, 'waypoints': None}


class Harness:
    def __init__(self, clock, runs_dir=None, **flight_options):
        self.clock = clock
        self.sent = []
        self.flight = FakeFlight(now=clock, **flight_options)
        self.agent = Agent('d1', self.flight, self.sent.append, runs_dir=runs_dir, now=clock)

    def of_type(self, kind):
        return [message for message in self.sent if message['type'] == kind]

    def package(self, run_id='run1'):
        self.agent.handle({'type': 'package', 'run_id': run_id, 'mission_id': 'm', 'launch': LAUNCH})

    def go(self, run_id='run1'):
        self.agent.handle({'type': 'go', 'run_id': run_id})

    def run_for(self, seconds, step=0.5):
        for _ in range(int(seconds / step)):
            self.clock.advance(step)
            self.agent.tick()

    def fly(self):
        self.package()
        self.go()
        self.run_for(CLIMB_S + 2)
        assert self.flight.snapshot()['airborne']


@pytest.fixture
def drone(clock):
    return Harness(clock)


def test_preflight_passes_problems_back(clock):
    harness = Harness(clock, problems=['no valid local position estimate yet'])
    harness.agent.handle({'type': 'preflight', 'mission_id': 'm', 'launch': LAUNCH})
    assert harness.of_type('preflight_result')[0]['problems'] == ['no valid local position estimate yet']


def test_package_is_saved_before_it_is_acknowledged(clock, tmp_path):
    harness = Harness(clock, runs_dir=tmp_path)
    harness.package()
    assert harness.of_type('package_ack')[0]['problems'] == []
    saved = json.loads((tmp_path / 'run1' / 'package_d1.json').read_text())
    assert saved['launch'] == LAUNCH
    assert harness.agent.run_state == PREPARED


def test_package_that_cannot_be_saved_is_refused(clock, tmp_path):
    blocker = tmp_path / 'file'
    blocker.write_text('')
    harness = Harness(clock, runs_dir=blocker)
    harness.package()
    assert 'could not save' in harness.of_type('package_ack')[0]['problems'][0]
    assert harness.agent.run_state == IDLE


def test_package_is_refused_when_checks_now_fail(clock):
    harness = Harness(clock, problems=['GPS lost'])
    harness.package()
    assert harness.of_type('package_ack')[0]['problems'] == ['GPS lost']


def test_repeated_package_and_go_are_harmless(drone):
    drone.package()
    drone.package()
    assert len(drone.of_type('package_ack')) == 2
    drone.go()
    started_at = drone.flight.started_at
    drone.clock.advance(1)
    drone.go()
    assert drone.flight.started_at == started_at
    assert len(drone.of_type('go_ack')) == 2


def test_go_for_another_run_is_ignored(drone):
    drone.package()
    drone.go('some_other_run')
    assert drone.agent.run_state == PREPARED
    assert drone.of_type('go_ack') == []


def test_cancel_clears_a_prepared_run(drone):
    drone.package()
    drone.agent.handle({'type': 'cancel', 'run_id': 'run1'})
    assert drone.agent.run_state == IDLE


def test_full_flight_with_no_further_messages_from_the_hub(drone):
    """After Go, nothing here needs the hub: the flight runs to landing on ticks alone."""
    drone.package()
    drone.go()
    assert drone.agent.run_state == RUNNING
    seen = set()
    for _ in range(200):
        drone.clock.advance(0.5)
        drone.agent.tick()
        seen.add(drone.agent.run_state)
    assert seen >= {RUNNING, LANDING, DONE}
    assert drone.agent.run_state == DONE
    assert not drone.flight.launch_running  # The launch never ends by itself.
    status = drone.of_type('status')[-1]
    assert status['airborne'] is False and status['armed'] is False
    assert status['waypoint_total'] == 6


def test_pilot_takeover_stops_the_launch(drone):
    """The node can't see this. Left running it would finish its route and send LAND over the pilot."""
    drone.fly()
    drone.flight.pilot_takes_over(NAVIGATION_STATE_POSCTL)
    drone.run_for(0.5)
    assert not drone.flight.launch_running
    assert drone.agent.run_state == FAILED
    assert drone.agent.failure == 'PX4 left Offboard for Position. Launch stopped.'
    assert drone.of_type('status')[-1]['activity'] == 'Pilot control, Position'
    assert drone.flight.commands_sent == []


def test_outside_hold_also_stops_the_launch(drone):
    drone.fly()
    drone.flight.pilot_takes_over(NAVIGATION_STATE_AUTO_LOITER)
    drone.run_for(0.5)
    assert not drone.flight.launch_running
    assert 'Hold' in drone.agent.failure


def test_failsafe_return_stops_the_launch(drone):
    drone.fly()
    drone.flight.pilot_takes_over(NAVIGATION_STATE_AUTO_RTL)
    drone.run_for(0.5)
    assert not drone.flight.launch_running
    assert 'Return' in drone.agent.failure


def test_abort_leaves_a_pilot_alone(drone):
    drone.fly()
    drone.flight.pilot_takes_over(NAVIGATION_STATE_POSCTL)
    drone.run_for(0.5)
    drone.agent.handle({'type': 'abort'})
    assert drone.of_type('abort_ack')[0]['result'] == 'pilot_control'
    assert drone.flight.commands_sent == []  # No Hold over the pilot.
    assert not drone.flight.launch_running
    drone.run_for(agent_core.HOLD_LIMIT_S + 5)
    assert drone.flight.commands_sent == []  # And no automatic Return later.


def test_pilot_taking_over_during_hold_cancels_the_automatic_return(drone):
    drone.fly()
    drone.agent.handle({'type': 'abort'})
    drone.run_for(0.5)
    assert drone.of_type('abort_ack')[0]['result'] == 'holding'
    drone.flight.pilot_takes_over(NAVIGATION_STATE_POSCTL)
    drone.run_for(agent_core.HOLD_LIMIT_S + 5)
    assert 'return' not in drone.flight.commands_sent
    assert drone.of_type('status')[-1]['hold_left_s'] is None


def test_operator_choice_cannot_override_a_pilot(drone):
    drone.fly()
    drone.agent.handle({'type': 'abort'})
    drone.run_for(0.5)
    drone.flight.pilot_takes_over(NAVIGATION_STATE_POSCTL)
    drone.run_for(0.5)
    drone.agent.handle({'type': 'recover', 'action': 'land'})
    assert 'land' not in drone.flight.commands_sent


def test_abort_leaves_a_drone_already_coming_down(drone):
    drone.fly()
    drone.flight.pilot_takes_over(NAVIGATION_STATE_AUTO_RTL)
    drone.run_for(0.5)
    drone.agent.handle({'type': 'abort'})
    assert drone.of_type('abort_ack')[0]['result'] == 'already_landing'
    assert drone.flight.commands_sent == []


def test_not_done_until_px4_disarms(drone):
    """Seen in SITL: Complete showed about 2 s before PX4 disarmed."""
    drone.package()
    drone.go()
    seen_landed_and_armed = False
    for _ in range(200):
        drone.clock.advance(0.5)
        drone.agent.tick()
        snapshot = drone.flight.snapshot()
        if snapshot['landed_after_flight'] and snapshot['armed']:
            seen_landed_and_armed = True
            assert drone.agent.run_state == LANDING
            assert snapshot['activity'] == 'Landed, still armed'
    assert seen_landed_and_armed
    assert drone.agent.run_state == DONE


def test_arming_that_lands_after_an_abort_is_undone(clock):
    """Seen in SITL: it armed after the abort, then flew a failsafe Return from the ground."""
    harness = Harness(clock)
    harness.package()
    harness.go()
    harness.flight.arm_delay_s = 100.0
    harness.agent.handle({'type': 'abort'})
    assert harness.of_type('abort_ack')[0]['result'] == 'not_armed'
    harness.flight.px4_armed = True  # The last arm command arrives after the launch has stopped.
    harness.run_for(0.5)
    assert harness.flight.commands_sent == ['disarm']
    assert not harness.flight.px4_armed


def test_a_drone_airborne_after_saying_not_armed_is_held(clock):
    harness = Harness(clock)
    harness.package()
    harness.go()
    harness.flight.arm_delay_s = 100.0
    harness.agent.handle({'type': 'abort'})
    harness.flight.px4_armed = True
    harness.flight.tracker.on_land_detected(False)
    harness.run_for(0.5)
    assert 'hold' in harness.flight.commands_sent
    assert harness.of_type('abort_ack')[-1]['result'] == 'holding'
    assert harness.of_type('status')[-1]['hold_left_s'] > 0


def test_abort_disarms_a_drone_armed_on_the_ground(clock):
    """Seen in SITL: abort just after arming left it armed in Hold on the ground."""
    harness = Harness(clock)
    harness.package()
    harness.go()
    harness.run_for(1)  # Armed, not yet climbing.
    snapshot = harness.flight.snapshot()
    assert snapshot['armed'] and not snapshot['airborne']
    harness.agent.handle({'type': 'abort'})
    assert not harness.flight.launch_running  # In the same step: no setpoints means a failsafe soon.
    assert harness.flight.commands_sent == ['disarm']
    assert harness.of_type('abort_ack') == []  # Not until PX4 confirms it.
    harness.run_for(0.5)
    assert harness.of_type('abort_ack')[0]['result'] == 'not_armed'
    assert not harness.flight.px4_armed
    assert harness.of_type('status')[-1]['hold_left_s'] is None


def test_refused_disarm_on_the_ground_is_reported(clock):
    harness = Harness(clock)
    harness.package()
    harness.go()
    harness.run_for(1)
    original = harness.flight.command
    harness.flight.command = lambda name: None if name == 'disarm' else original(name)
    harness.agent.handle({'type': 'abort'})
    harness.run_for(agent_core.HOLD_CONFIRM_S + 1)
    assert harness.of_type('abort_ack')[0]['result'] == 'hold_unconfirmed'


def test_without_status_a_drone_armed_on_the_ground_still_gets_hold(clock):
    """With no vehicle_status, armed is a guess, and PX4's mode can't confirm a disarm."""
    harness = Harness(clock, reports_status=False)
    harness.package()
    harness.go()
    harness.run_for(1)
    harness.agent.handle({'type': 'abort'})
    harness.run_for(0.5)
    assert harness.flight.commands_sent == ['hold']
    assert harness.of_type('abort_ack')[0]['result'] == 'holding'


def test_will_not_start_on_a_drone_that_is_already_armed(clock):
    """Seen in SITL: a mission was allowed to start while PX4 was armed and flying a Return."""
    harness = Harness(clock)
    harness.flight.px4_armed = True
    harness.agent.tick()
    harness.agent.handle({'type': 'preflight', 'mission_id': 'm', 'launch': LAUNCH})
    assert harness.of_type('preflight_result')[-1]['problems'] == ['PX4 is already armed']
    harness.package()
    assert harness.of_type('package_ack')[-1]['problems'] == ['PX4 is already armed']


def test_cautious_armed_guess_does_not_block_a_start(clock):
    """Without vehicle_status, armed stays true after an abort on the ground. It must not lock the drone out."""
    harness = Harness(clock, reports_status=False)
    harness.flight.tracker.on_arm_ack()
    harness.agent.handle({'type': 'preflight', 'mission_id': 'm', 'launch': LAUNCH})
    assert harness.of_type('preflight_result')[-1]['problems'] == []


def test_px4_is_taken_out_of_offboard_after_a_flight(drone):
    """Seen in SITL: left in Offboard, PX4's preflight fails, which blocked the next mission."""
    drone.package()
    drone.go()
    drone.run_for(100)
    assert drone.agent.run_state == DONE
    assert drone.flight.commands_sent == ['hold']
    assert drone.flight.snapshot()['nav_state'] == NAVIGATION_STATE_AUTO_LOITER


def test_offboard_is_left_alone_while_flying(drone):
    drone.fly()
    assert drone.flight.commands_sent == []


def test_without_status_abort_confirms_hold_from_the_ack(clock):
    harness = Harness(clock, reports_status=False)
    harness.fly()
    harness.agent.handle({'type': 'abort'})
    harness.run_for(0.5)
    assert harness.of_type('abort_ack')[0]['result'] == 'holding'


def test_without_status_the_flight_still_completes(clock):
    harness = Harness(clock, reports_status=False)
    harness.package()
    harness.go()
    harness.run_for(100)
    assert harness.agent.run_state == DONE


def test_status_says_what_the_drone_is_doing(clock):
    drone = Harness(clock, reports_status=False)
    drone.agent.tick()
    assert drone.of_type('status')[-1]['activity'] == 'On ground'
    drone.package()
    drone.flight.arm_delay_s = 0.0
    drone.go()
    drone.flight.arm_delay_s = 100.0
    drone.agent.tick()
    assert drone.of_type('status')[-1]['activity'] == 'Starting'
    drone.flight.tracker.on_arm_ack()
    drone.agent.tick()
    assert drone.of_type('status')[-1]['activity'] == 'Armed'
    drone.flight.tracker.on_land_detected(False)
    drone.agent.tick()
    assert drone.of_type('status')[-1]['activity'] == 'Flying route'


def test_status_lets_a_restarted_hub_rebuild_the_run(drone):
    drone.package()
    drone.go()
    drone.run_for(2)
    status = drone.of_type('status')[-1]
    assert status['run_id'] == 'run1' and status['mission_id'] == 'm'
    assert status['run_age_s'] == pytest.approx(2.0)


def test_log_lines_are_forwarded(drone):
    drone.package()
    drone.go()
    drone.run_for(1)
    texts = [line['text'] for message in drone.of_type('log') for line in message['lines']]
    assert any('loaded 6 waypoints' in text for text in texts)


def test_launch_failure_reports_the_reason(clock):
    harness = Harness(clock, launch_fails=True)
    harness.package()
    harness.go()
    harness.run_for(1)
    assert harness.agent.run_state == FAILED
    assert harness.of_type('status')[-1]['failure'] == 'PX4 never became ready'


def test_node_death_is_a_failure_even_while_the_launch_keeps_running(clock):
    harness = Harness(clock)
    harness.fly()
    harness.flight.tracker.on_log('[ERROR] [offboard_solution-3]: process has died [pid 1, exit code -9, cmd x]')
    harness.run_for(0.5)
    assert harness.agent.run_state == FAILED
    assert harness.agent.failure == 'offboard_solution died with exit code -9'
    assert not harness.flight.launch_running  # Stops whatever else the launch was running.


def test_run_age_freezes_when_the_flight_ends(drone):
    drone.package()
    drone.go()
    drone.run_for(100)
    assert drone.agent.run_state == DONE
    age = drone.of_type('status')[-1]['run_age_s']
    drone.run_for(5)
    assert drone.of_type('status')[-1]['run_age_s'] == age


def test_launch_exit_says_what_the_launch_last_printed(clock):
    harness = Harness(clock)
    harness.package()
    harness.go()
    harness.flight.logs.append(('INFO', "malformed launch argument 'x:='"))
    harness.flight.launch_running = False
    harness.flight.exit_code = 1
    harness.run_for(1)
    assert harness.agent.failure == "launch exited with code 1: malformed launch argument 'x:='"


def test_node_giving_up_stops_the_launch_and_reports(clock):
    harness = Harness(clock, gives_up=True)
    harness.package()
    harness.go()
    harness.run_for(1)
    assert harness.agent.run_state == FAILED
    assert 'Not flying' in harness.agent.failure
    assert not harness.flight.launch_running


def test_launch_that_cannot_start_reports_instead_of_crashing(drone):
    def broken_start(launch):
        raise OSError('ros2 not found')
    drone.flight.start = broken_start
    drone.package()
    drone.go()
    assert drone.agent.run_state == FAILED
    assert 'ros2 not found' in drone.agent.failure
    assert drone.of_type('go_ack')


def test_crash_in_flight_is_reported_while_the_drone_comes_down(clock):
    harness = Harness(clock, crashes_after_s=CLIMB_S + 1)
    harness.package()
    harness.go()
    harness.run_for(CLIMB_S + 2)
    assert harness.agent.run_state == FAILED
    assert harness.of_type('status')[-1]['airborne'] is True
    harness.run_for(10)
    assert harness.of_type('status')[-1]['airborne'] is False


def test_abort_when_not_armed_stops_the_launch_first(clock):
    harness = Harness(clock)
    harness.package()
    harness.agent.handle({'type': 'go', 'run_id': 'run1'})
    harness.flight.arm_delay_s = 100.0  # Still counting down.
    harness.agent.handle({'type': 'abort'})
    assert harness.of_type('abort_ack')[0]['result'] == 'not_armed'
    assert not harness.flight.launch_running
    assert harness.flight.commands_sent == []  # No Hold sent to a drone on the ground.


def test_abort_when_armed_holds_before_stopping_the_launch(drone):
    drone.fly()
    running_when_hold_sent = []
    original = drone.flight.command

    def spy(name):
        running_when_hold_sent.append(drone.flight.launch_running)
        original(name)
    drone.flight.command = spy

    drone.agent.handle({'type': 'abort'})
    assert drone.of_type('abort_ack') == []  # Not until PX4 confirms Hold.
    assert running_when_hold_sent == [True]
    drone.run_for(0.5)
    assert drone.of_type('abort_ack')[0]['result'] == 'holding'
    assert not drone.flight.launch_running
    assert drone.agent.run_state == ABORTED


def test_unconfirmed_hold_still_stops_the_launch_and_says_so(clock):
    harness = Harness(clock, ignores_hold=True)
    harness.fly()
    harness.agent.handle({'type': 'abort'})
    harness.run_for(agent_core.HOLD_CONFIRM_S + 1)
    assert harness.of_type('abort_ack')[0]['result'] == 'hold_unconfirmed'
    assert not harness.flight.launch_running
    assert len(harness.flight.commands_sent) > 1  # Hold was resent while waiting.


def test_repeated_abort_repeats_the_answer_without_redoing_the_work(drone):
    drone.fly()
    drone.agent.handle({'type': 'abort'})
    drone.run_for(0.5)
    sent = len(drone.flight.commands_sent)
    drone.agent.handle({'type': 'abort'})
    assert len(drone.of_type('abort_ack')) == 2
    assert len(drone.flight.commands_sent) == sent


def test_operator_choice_after_hold(drone):
    drone.fly()
    drone.agent.handle({'type': 'abort'})
    drone.run_for(0.5)
    assert drone.of_type('status')[-1]['hold_left_s'] > 0
    drone.agent.handle({'type': 'recover', 'action': 'land'})
    assert drone.flight.commands_sent[-1] == 'land'
    assert drone.agent.recovery == 'land'
    drone.run_for(1)
    assert drone.of_type('status')[-1]['hold_left_s'] is None


def test_unknown_recovery_action_is_ignored(drone):
    drone.fly()
    drone.agent.handle({'type': 'abort'})
    drone.run_for(0.5)
    drone.agent.handle({'type': 'recover', 'action': 'explode'})
    assert drone.agent.recovery is None


def test_held_drone_returns_by_itself_if_no_choice_arrives(drone):
    """A dropped link must not leave a drone hovering until the battery failsafe."""
    drone.fly()
    drone.agent.handle({'type': 'abort'})
    drone.run_for(0.5)
    drone.run_for(agent_core.HOLD_LIMIT_S - 2)
    assert 'return' not in drone.flight.commands_sent
    drone.run_for(3)
    assert 'return' in drone.flight.commands_sent
    drone.run_for(10)
    assert drone.of_type('status')[-1]['airborne'] is False


def test_automatic_return_survives_the_land_detector_flipping(drone):
    """Seen in SITL: one on-ground reading cancelled the timer, leaving it armed and hovering."""
    drone.fly()
    drone.agent.handle({'type': 'abort'})
    drone.run_for(0.5)
    drone.flight.tracker.on_land_detected(True)
    drone.run_for(1)
    assert drone.of_type('status')[-1]['hold_left_s'] is None
    drone.flight.tracker.on_land_detected(False)
    drone.run_for(0.5)
    assert drone.of_type('status')[-1]['hold_left_s'] > 0
    drone.run_for(agent_core.HOLD_LIMIT_S)
    assert 'return' in drone.flight.commands_sent


def test_recovery_command_is_resent_until_accepted(clock):
    harness = Harness(clock)
    harness.fly()
    harness.agent.handle({'type': 'abort'})
    harness.run_for(0.5)
    original_accepted = harness.flight.accepted
    harness.flight.accepted = lambda name: False if name == 'land' else original_accepted(name)
    harness.agent.handle({'type': 'recover', 'action': 'land'})
    before = harness.flight.commands_sent.count('land')
    harness.run_for(2)
    assert harness.flight.commands_sent.count('land') > before


def test_new_mission_refused_while_airborne_after_abort(drone):
    drone.fly()
    drone.agent.handle({'type': 'abort'})
    drone.run_for(0.5)
    drone.agent.handle({'type': 'preflight', 'mission_id': 'm', 'launch': LAUNCH})
    assert drone.of_type('preflight_result')[-1]['problems'] == ['drone is still airborne']


def test_finished_run_clears_on_next_preflight(drone):
    drone.package()
    drone.go()
    drone.run_for(100)
    assert drone.agent.run_state == DONE
    drone.agent.handle({'type': 'preflight', 'mission_id': 'm', 'launch': LAUNCH})
    assert drone.of_type('preflight_result')[-1]['problems'] == []
    assert drone.agent.run_state == IDLE


def test_a_second_run_flies_again(drone):
    drone.package('run1')
    drone.go('run1')
    drone.run_for(100)
    drone.agent.handle({'type': 'reset'})
    drone.package('run2')
    drone.go('run2')
    drone.run_for(100)
    assert drone.agent.run_state == DONE
    assert drone.agent.run_id == 'run2'
