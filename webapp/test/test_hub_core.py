"""Unit tests for the hub's state machine, driven with fake agents and a fake clock."""
import pytest

from swarm_launcher import hub_core
from swarm_launcher.hub_core import (
    ABORTED, ABORTING, COMPLETE, IDLE, LANDING, PREFLIGHT, READY, SEARCHING, STARTING, HubCore)

MISSION = {
    'id': 'm', 'name': 'M', 'spoken_name': 'm', 'inputs': [],
    'drones': {'d1': {'role': 'search'}, 'd2': {'role': 'verify'}},
    'launch': {'package': 'p', 'file': 'f.launch.py', 'args': {}, 'waypoints': None},
}


class FakeLink:
    def __init__(self):
        self.sent = []

    def __call__(self, message):
        self.sent.append(message)

    def kinds(self):
        return [message['type'] for message in self.sent]


@pytest.fixture
def core(clock):
    return HubCore({'m': MISSION}, now=clock)


@pytest.fixture
def links(core):
    result = {'d1': FakeLink(), 'd2': FakeLink()}
    for drone_id, link in result.items():
        core.agent_connected(drone_id, link)
    return result


def status(core, drone_id, **fields):
    core.agent_message(drone_id, {'type': 'status', **fields})


def to_ready(core):
    assert core.select('m')['ok']
    for drone_id in ('d1', 'd2'):
        core.agent_message(drone_id, {'type': 'preflight_result', 'mission_id': 'm', 'problems': []})
    assert core.state == READY


def to_committed(core):
    to_ready(core)
    assert core.start()['ok']
    for drone_id in ('d1', 'd2'):
        core.agent_message(drone_id, {'type': 'package_ack', 'run_id': core.run_id, 'problems': []})
    assert core.committed


def to_searching(core):
    to_committed(core)
    for drone_id in ('d1', 'd2'):
        core.agent_message(drone_id, {'type': 'go_ack', 'run_id': core.run_id})
        status(core, drone_id, run_state='running', airborne=True, armed=True)
    assert core.state == SEARCHING


def test_select_needs_every_drone_connected(core):
    core.agent_connected('d1', FakeLink())
    assert not core.select('m')['ok']
    assert core.state == IDLE
    assert 'd2' in core.reason


def test_unknown_mission_is_refused(core, links):
    assert not core.select('nope')['ok']


def test_preflight_passes_to_ready(core, links):
    core.select('m')
    assert core.state == PREFLIGHT
    assert 'preflight' in links['d1'].kinds()
    to_ready_result = core.agent_message('d1', {'type': 'preflight_result', 'mission_id': 'm', 'problems': []})
    assert to_ready_result is None and core.state == PREFLIGHT
    core.agent_message('d2', {'type': 'preflight_result', 'mission_id': 'm', 'problems': []})
    assert core.state == READY


def test_failed_check_returns_to_idle_and_says_why(core, links):
    core.select('m')
    core.agent_message('d1', {'type': 'preflight_result', 'mission_id': 'm', 'problems': []})
    core.agent_message('d2', {'type': 'preflight_result', 'mission_id': 'm', 'problems': ['no GPS']})
    assert core.state == IDLE
    assert 'd2' in core.reason
    assert core.snapshot()['checks']['d2'] == {'ok': False, 'problems': ['no GPS']}


def test_silent_preflight_times_out(core, links, clock):
    core.select('m')
    clock.advance(hub_core.PREFLIGHT_TIMEOUT_S + 1)
    core.tick()
    assert core.state == IDLE
    assert 'd1' in core.reason


def test_ready_falls_back_if_a_drone_disconnects(core, links):
    to_ready(core)
    core.agent_disconnected('d2', links['d2'])
    core.tick()
    assert core.state == IDLE


def test_start_is_refused_unless_ready(core, links):
    assert not core.start()['ok']


def test_nothing_flies_until_every_drone_holds_its_package(core, links):
    to_ready(core)
    core.start()
    assert core.state == STARTING and not core.committed
    core.agent_message('d1', {'type': 'package_ack', 'run_id': core.run_id, 'problems': []})
    assert 'go' not in links['d1'].kinds()
    core.agent_message('d2', {'type': 'package_ack', 'run_id': core.run_id, 'problems': []})
    assert core.committed
    assert 'go' in links['d1'].kinds() and 'go' in links['d2'].kinds()


def test_refused_package_cancels_and_nothing_flies(core, links):
    to_ready(core)
    core.start()
    core.agent_message('d1', {'type': 'package_ack', 'run_id': core.run_id, 'problems': ['disk full']})
    assert core.state == IDLE
    assert 'disk full' in core.reason
    assert 'go' not in links['d2'].kinds()
    assert 'cancel' in links['d2'].kinds()


def test_silent_package_times_out_and_nothing_flies(core, links, clock):
    to_ready(core)
    core.start()
    clock.advance(hub_core.PACKAGE_TIMEOUT_S + 1)
    core.tick()
    assert core.state == IDLE
    assert 'go' not in links['d1'].kinds()


def test_clock_starts_at_the_press_not_the_request(core, links, clock):
    to_ready(core)
    core.start(press_age_ms=3000)
    assert core.snapshot()['clock'] == {'running': True, 'elapsed_ms': 3000}
    clock.advance(2)
    assert core.snapshot()['clock']['elapsed_ms'] == 5000


def test_silly_press_age_is_clamped(core, links):
    to_ready(core)
    core.start(press_age_ms=10_000_000)
    assert core.snapshot()['clock']['elapsed_ms'] == hub_core.MAX_PRESS_AGE_S * 1000


def test_full_run_mirrors_what_the_drones_report(core, links, clock):
    to_searching(core)
    status(core, 'd1', run_state='landing', airborne=True)
    assert core.state == SEARCHING  # d2 is still searching
    status(core, 'd2', run_state='landing', airborne=True)
    assert core.state == LANDING
    clock.advance(10)
    assert core.snapshot()['clock']['running'] is False
    stopped_at = core.snapshot()['clock']['elapsed_ms']
    status(core, 'd1', run_state='done', airborne=False)
    assert core.state == LANDING
    status(core, 'd2', run_state='done', airborne=False)
    assert core.state == COMPLETE
    assert core.snapshot()['clock']['elapsed_ms'] == stopped_at


def test_launch_failure_before_arming_returns_to_idle_with_the_reason(core, links):
    to_committed(core)
    core.agent_message('d1', {'type': 'go_ack', 'run_id': core.run_id})
    status(core, 'd1', run_state='failed', failure='PX4 never became ready', armed=False, airborne=False)
    assert core.state == IDLE
    assert 'never became ready' in core.reason
    assert 'abort' in links['d2'].kinds()


def test_failure_with_a_drone_armed_aborts_the_rest(core, links):
    to_committed(core)
    status(core, 'd2', run_state='running', armed=True)
    status(core, 'd1', run_state='failed', failure='launch exited', armed=False, airborne=False)
    assert core.state == ABORTING


def test_crash_in_flight_lets_the_other_drone_finish(core, links):
    to_searching(core)
    status(core, 'd1', run_state='failed', failure='launch exited', airborne=True)
    assert core.state == SEARCHING
    status(core, 'd1', run_state='failed', airborne=False)
    status(core, 'd2', run_state='landing', airborne=True)
    assert core.state == LANDING
    status(core, 'd2', run_state='done', airborne=False)
    assert core.state == COMPLETE


def test_a_failed_run_is_not_complete_until_px4_disarms(core, links):
    """Seen in SITL: after a crash, Complete showed while PX4 was still armed on the ground."""
    to_searching(core)
    status(core, 'd1', run_state='failed', failure='node died', airborne=False, armed=True)
    status(core, 'd2', run_state='failed', failure='node died', airborne=False, armed=False)
    assert core.state != COMPLETE
    status(core, 'd1', run_state='failed', failure='node died', airborne=False, armed=False)
    status(core, 'd1', run_state='failed', failure='node died', airborne=False, armed=False)
    assert core.state == COMPLETE


def test_missing_go_ack_aborts(core, links, clock):
    to_committed(core)
    core.agent_message('d1', {'type': 'go_ack', 'run_id': core.run_id})
    clock.advance(hub_core.GO_TIMEOUT_S + 1)
    core.tick()
    assert core.state == ABORTING
    assert 'd2' in core.reason


def test_abort_is_never_refused(core, links):
    assert core.abort()['ok']
    assert core.state == IDLE


def test_abort_before_commit_cancels(core, links):
    to_ready(core)
    core.start()
    core.abort()
    assert core.state == IDLE
    assert 'cancel' in links['d1'].kinds()
    assert core.snapshot()['clock']['running'] is False


def test_abort_stops_the_clock_and_waits_for_every_drone(core, links, clock):
    to_searching(core)
    clock.advance(30)
    core.abort()
    assert core.state == ABORTING
    assert core.snapshot()['clock']['running'] is False
    core.agent_message('d1', {'type': 'abort_ack', 'result': 'holding'})
    assert core.state == ABORTING
    core.agent_message('d2', {'type': 'abort_ack', 'result': 'not_armed'})
    assert core.state == ABORTED
    assert core.snapshot()['abort_results'] == {'d1': 'holding', 'd2': 'not_armed'}


def test_a_later_answer_updates_the_abort_result(core, links):
    to_searching(core)
    core.abort()
    core.agent_message('d1', {'type': 'abort_ack', 'result': 'not_armed'})
    core.agent_message('d2', {'type': 'abort_ack', 'result': 'holding'})
    assert core.state == ABORTED
    core.agent_message('d1', {'type': 'abort_ack', 'result': 'holding'})
    assert core.snapshot()['abort_results']['d1'] == 'holding'


def test_abort_is_resent_until_answered(core, links, clock):
    to_searching(core)
    core.abort()
    core.agent_message('d1', {'type': 'abort_ack', 'result': 'holding'})
    d1_before = links['d1'].kinds().count('abort')
    d2_before = links['d2'].kinds().count('abort')
    clock.advance(hub_core.RESEND_S + 0.1)
    core.tick()
    assert links['d1'].kinds().count('abort') == d1_before
    assert links['d2'].kinds().count('abort') == d2_before + 1


def test_silent_drone_shows_no_response(core, links, clock):
    to_searching(core)
    core.abort()
    assert core.snapshot()['abort_results']['d1'] == 'sending'
    clock.advance(hub_core.ABORT_NO_RESPONSE_S + 0.1)
    assert core.snapshot()['abort_results']['d1'] == 'no_response'


def test_recover_only_after_abort(core, links):
    to_searching(core)
    assert not core.recover('return')['ok']
    core.abort()
    core.agent_message('d1', {'type': 'abort_ack', 'result': 'holding'})
    core.agent_message('d2', {'type': 'abort_ack', 'result': 'holding'})
    assert not core.recover('explode')['ok']
    assert core.recover('land')['ok']
    assert {'type': 'recover', 'action': 'land'} in links['d1'].sent


def test_reset_refused_while_a_drone_is_airborne(core, links):
    to_searching(core)
    core.abort()
    core.agent_message('d1', {'type': 'abort_ack', 'result': 'holding'})
    core.agent_message('d2', {'type': 'abort_ack', 'result': 'holding'})
    assert not core.reset()['ok']
    status(core, 'd1', airborne=False)
    status(core, 'd2', airborne=False)
    assert core.reset()['ok']
    assert core.state == IDLE
    assert 'reset' in links['d1'].kinds()


def test_reset_refused_while_a_connected_drone_has_not_answered(core, links):
    to_searching(core)
    core.abort()
    assert not core.reset()['ok']
    # Both gone: nothing more can be learned here, so the operator can move on.
    status(core, 'd1', airborne=False)
    status(core, 'd2', airborne=False)
    core.agent_disconnected('d1', links['d1'])
    core.agent_disconnected('d2', links['d2'])
    assert core.reset()['ok']


def test_command_id_stops_a_retry_firing_twice(core, links, clock):
    to_ready(core)
    first = core.run_command('abc', core.start)
    run_id = core.run_id
    clock.advance(1)
    second = core.run_command('abc', core.start)
    assert first is second
    assert core.run_id == run_id


def test_late_reconnect_keeps_the_new_link(core, links):
    new_link = FakeLink()
    core.agent_connected('d1', new_link)
    core.agent_disconnected('d1', links['d1'])  # The old socket's close arrives late.
    assert core.links['d1'] is new_link


def test_restarted_hub_picks_up_a_run_and_can_abort_it(core, links, clock):
    """A hub that comes back Idle with drones flying must not leave abort with nothing to act on."""
    status(core, 'd1', run_state='running', run_id='r1', mission_id='m', run_age_s=30.0, airborne=True, armed=True)
    assert core.state == SEARCHING
    assert core.committed and core.run_id == 'r1' and core.mission['id'] == 'm'
    assert core.snapshot()['clock']['elapsed_ms'] == 30000
    core.abort()
    assert core.state == ABORTING
    assert 'abort' in links['d1'].kinds() and 'abort' in links['d2'].kinds()
    core.agent_message('d1', {'type': 'abort_ack', 'result': 'holding'})
    core.agent_message('d2', {'type': 'abort_ack', 'result': 'not_armed'})
    assert core.state == ABORTED


def test_restarted_hub_picks_up_a_run_with_no_manifest(core, links):
    status(core, 'd1', run_state='running', run_id='r1', mission_id='gone', run_age_s=1.0, airborne=True)
    assert core.state == SEARCHING
    assert list(core.mission['drones']) == ['d1']


def test_restarted_hub_picks_up_a_failed_run_still_in_the_air(core, links):
    """Seen in SITL: the hub came back Idle over an airborne drone, with the abort bar disabled."""
    status(core, 'd1', run_state='failed', failure='node died', run_id='r1', mission_id='m', run_age_s=9.0, airborne=True)
    assert core.state == SEARCHING
    core.abort()
    assert core.state == ABORTING


def test_restarted_hub_offers_recovery_for_a_drone_still_holding(core, links):
    status(core, 'd1', run_state='aborted', run_id='r1', mission_id='gone', abort_result='holding', airborne=True)
    assert core.state == ABORTED
    assert core.snapshot()['abort_results'] == {'d1': 'holding'}
    assert core.recover('return')['ok']
    assert {'type': 'recover', 'action': 'return'} in links['d1'].sent


def test_abort_that_beats_the_drones_back_to_a_restarted_hub_still_counts(core, clock):
    assert core.abort()['ok']  # Hub just restarted: Idle, nothing connected yet.
    clock.advance(hub_core.ABORT_CARRY_S - 1)
    link = FakeLink()
    core.agent_connected('d1', link)
    status(core, 'd1', run_state='running', run_id='r1', mission_id='gone', run_age_s=9.0, airborne=True)
    assert core.state == ABORTING
    assert 'abort' in link.kinds()


def test_an_old_idle_abort_does_not_carry_over(core, clock):
    core.abort()
    clock.advance(hub_core.ABORT_CARRY_S + 1)
    core.agent_connected('d1', FakeLink())
    status(core, 'd1', run_state='running', run_id='r1', mission_id='gone', run_age_s=9.0, airborne=True)
    assert core.state == SEARCHING


def test_run_that_finished_while_the_hub_was_down_is_reported_complete(core, links, clock):
    status(core, 'd1', run_state='done', run_id='r1', mission_id='m', run_age_s=40.0, airborne=False)
    status(core, 'd2', run_state='done', run_id='r1', mission_id='m', run_age_s=41.0, airborne=False)
    assert core.state == COMPLETE
    assert core.run_id == 'r1'
    elapsed = core.snapshot()['clock']['elapsed_ms']
    assert core.snapshot()['clock']['running'] is False and elapsed == 40000
    clock.advance(100)
    assert core.snapshot()['clock']['elapsed_ms'] == elapsed


def test_flight_that_crashed_and_landed_while_the_hub_was_down_is_complete(core, links):
    status(core, 'd1', run_state='failed', failure='node died', run_id='r1', mission_id='gone',
           landed_after_flight=True, airborne=False)
    assert core.state == COMPLETE


def test_run_that_never_flew_is_not_picked_up(core, links):
    status(core, 'd1', run_state='failed', run_id='r1', mission_id='m', landed_after_flight=False, airborne=False)
    assert core.state == IDLE
    status(core, 'd1', run_state='aborted', run_id='r1', mission_id='m', airborne=False)
    assert core.state == IDLE


def test_reset_run_is_not_picked_up_from_a_late_status(core, links):
    status(core, 'd1', run_state='done', run_id='r1', mission_id='gone', run_age_s=1.0, airborne=False)
    assert core.state == COMPLETE
    assert core.reset()['ok']
    status(core, 'd1', run_state='done', run_id='r1', mission_id='gone', run_age_s=1.0, airborne=False)
    assert core.state == IDLE


def test_a_run_the_hub_closed_is_not_picked_up_again(core, links):
    """A status already on the wire when the hub gives up must not reopen the run."""
    to_committed(core)
    run_id = core.run_id
    status(core, 'd1', run_state='failed', failure='PX4 never became ready', armed=False, airborne=False, run_id=run_id)
    assert core.state == IDLE
    status(core, 'd2', run_state='running', run_id=run_id, mission_id='m', armed=False, airborne=False)
    assert core.state == IDLE


def test_logs_come_back_after_a_sequence_number(core):
    core.log('hub', 'INFO', 'one')
    seq = core.log_seq
    core.log('hub', 'INFO', 'two')
    assert [line['text'] for line in core.logs_after(seq)] == ['two']
