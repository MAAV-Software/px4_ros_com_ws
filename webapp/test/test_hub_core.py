"""Unit tests for the hub's state machine, driven with fake agents and a fake clock."""
import pytest

from swarm_launcher import hub_core
from swarm_launcher.hub_core import (
    ABORTED, ABORTING, COMPLETE, IDLE, LANDING, PREFLIGHT, READY, SEARCHING, STARTING, HubCore)

MISSION_ID = 'p/f.launch.py'


class FakeLink:
    def __init__(self):
        self.sent = []

    def __call__(self, message):
        self.sent.append(message)

    def kinds(self):
        return [message['type'] for message in self.sent]


@pytest.fixture
def core(clock):
    return HubCore(now=clock)


@pytest.fixture
def links(core):
    result = {'d1': FakeLink(), 'd2': FakeLink()}
    for drone_id, link in result.items():
        core.agent_connected(drone_id, link)
    return result


def status(core, drone_id, **fields):
    core.agent_message(drone_id, {'type': 'status', **fields})


def select(core, drones=('d1', 'd2'), args=None, file='f.launch.py'):
    return core.select('p', file, list(drones), args or {})


def answer(core, drone_id, problems=(), launch_args=None, check_id=None):
    core.agent_message(drone_id, {'type': 'preflight_result', 'mission_id': MISSION_ID,
                                  'check_id': core.check_id if check_id is None else check_id,
                                  'problems': list(problems), 'launch_args': launch_args})


def to_ready(core):
    assert select(core)['ok']
    for drone_id in ('d1', 'd2'):
        answer(core, drone_id)
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
    assert not select(core)['ok']
    assert core.state == IDLE
    assert 'd2' in core.reason


def test_preflight_passes_to_ready(core, links):
    select(core)
    assert core.state == PREFLIGHT
    assert links['d1'].sent[-1] == {'type': 'preflight', 'check_id': core.check_id, 'mission_id': MISSION_ID,
                                    'mode': 'hitl', 'launch': {'package': 'p', 'file': 'f.launch.py', 'args': {}}}
    answer(core, 'd1')
    assert core.state == PREFLIGHT
    answer(core, 'd2')
    assert core.state == READY


def test_failed_check_returns_to_idle_and_says_why(core, links):
    select(core)
    answer(core, 'd1')
    answer(core, 'd2', ['no GPS'])
    assert core.state == IDLE
    assert 'd2' in core.reason
    assert core.snapshot()['checks']['d2'] == {'ok': False, 'problems': ['no GPS']}


def test_silent_preflight_times_out(core, links, clock):
    select(core)
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
    status(core, 'd1', run_state='running', run_id='r1', mission_id=MISSION_ID, run_age_s=30.0, airborne=True, armed=True)
    assert core.state == SEARCHING
    assert core.committed and core.run_id == 'r1' and core.mission['id'] == MISSION_ID
    assert core.snapshot()['clock']['elapsed_ms'] == 30000
    # The hub learns who else is in the run from their own reports.
    status(core, 'd2', run_state='running', run_id='r1', mission_id=MISSION_ID, run_age_s=30.0, airborne=True)
    assert core.mission['drones'] == ['d1', 'd2']
    core.abort()
    assert core.state == ABORTING
    assert 'abort' in links['d1'].kinds() and 'abort' in links['d2'].kinds()
    core.agent_message('d1', {'type': 'abort_ack', 'result': 'holding'})
    core.agent_message('d2', {'type': 'abort_ack', 'result': 'not_armed'})
    assert core.state == ABORTED


def test_a_drone_that_reports_the_run_after_an_abort_is_aborted_too(core, links):
    status(core, 'd1', run_state='running', run_id='r1', mission_id=MISSION_ID, airborne=True, armed=True)
    core.abort()
    core.agent_message('d1', {'type': 'abort_ack', 'result': 'holding'})
    assert core.state == ABORTED
    status(core, 'd2', run_state='running', run_id='r1', mission_id=MISSION_ID, airborne=True, armed=True)
    assert core.state == ABORTING
    assert links['d2'].kinds()[-1] == 'abort'
    core.agent_message('d2', {'type': 'abort_ack', 'result': 'holding'})
    assert core.state == ABORTED and core.snapshot()['abort_results'] == {'d1': 'holding', 'd2': 'holding'}


def test_a_restarted_hub_picks_up_every_drone_already_reporting_the_run(core, links):
    status(core, 'd2', run_state='idle', run_id='r1', mission_id=MISSION_ID, airborne=True)
    status(core, 'd1', run_state='running', run_id='r1', mission_id=MISSION_ID, airborne=True)
    assert core.mission['drones'] == ['d1', 'd2']


def test_restarted_hub_picks_up_a_run_with_no_manifest(core, links):
    status(core, 'd1', run_state='running', run_id='r1', mission_id='gone', run_age_s=1.0, airborne=True)
    assert core.state == SEARCHING
    assert list(core.mission['drones']) == ['d1']


def test_restarted_hub_picks_up_a_failed_run_still_in_the_air(core, links):
    """Seen in SITL: the hub came back Idle over an airborne drone, with the abort bar disabled."""
    status(core, 'd1', run_state='failed', failure='node died', run_id='r1', mission_id=MISSION_ID, run_age_s=9.0, airborne=True)
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
    status(core, 'd1', run_state='done', run_id='r1', mission_id=MISSION_ID, run_age_s=40.0, airborne=False)
    status(core, 'd2', run_state='done', run_id='r1', mission_id=MISSION_ID, run_age_s=41.0, airborne=False)
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
    status(core, 'd1', run_state='failed', run_id='r1', mission_id=MISSION_ID, landed_after_flight=False, airborne=False)
    assert core.state == IDLE
    status(core, 'd1', run_state='aborted', run_id='r1', mission_id=MISSION_ID, airborne=False)
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
    status(core, 'd2', run_state='running', run_id=run_id, mission_id=MISSION_ID, armed=False, airborne=False)
    assert core.state == IDLE


def test_logs_come_back_after_a_sequence_number(core):
    core.log('hub', 'INFO', 'one')
    seq = core.log_seq
    core.log('hub', 'INFO', 'two')
    assert [line['text'] for line in core.logs_after(seq)] == ['two']


# --- Picking a launch file ---------------------------------------------------------

CATALOG = [{'folder': 'p_folder', 'package': 'p', 'files': [{'file': 'f.launch.py', 'built': True}]}]
ARGUMENTS = [{'name': 'record', 'default': 'false', 'description': '', 'choices': [], 'included': False}]


def test_each_drone_is_asked_for_its_catalog_and_the_phone_sees_it(core, links):
    assert links['d1'].kinds() == ['catalog_request']
    core.agent_message('d1', {'type': 'catalog', 'folders': CATALOG})
    assert core.snapshot()['catalogs'] == {'d1': CATALOG}
    core.agent_disconnected('d1', links['d1'])
    assert core.snapshot()['catalogs'] == {}


def test_the_folder_name_comes_from_the_catalog(core, links):
    core.agent_message('d1', {'type': 'catalog', 'folders': CATALOG})
    select(core)
    assert core.snapshot()['mission'] == {'id': MISSION_ID, 'name': 'f.launch.py', 'folder': 'p_folder',
                                          'package': 'p', 'file': 'f.launch.py', 'drones': ['d1', 'd2'],
                                          'args': {}}


@pytest.mark.parametrize('package, file, drones, args, reason', [
    ('', 'f.launch.py', ['d1'], {}, 'pick a folder and a launch file'),
    ('p', '../../bin/sh', ['d1'], {}, 'pick a folder and a launch file'),
    ('p', 'f.launch.py', [], {}, 'pick at least one drone'),
    ('p', 'f.launch.py', ['d1'], {'launch_sitl': 'true'}, 'launch_sitl is set by the launcher'),
    ('p', 'f.launch.py', ['d1'], {'record': 'a\nb'}, 'bad value for record'),
])
def test_a_malformed_pick_is_refused(core, links, package, file, drones, args, reason):
    result = core.select(package, file, drones, args)
    assert not result['ok'] and result['reason'] == reason
    assert core.state == IDLE and core.mission is None


def test_only_the_picked_drones_are_checked_and_flown(core, links):
    select(core, drones=['d2'])
    assert 'preflight' not in links['d1'].kinds()
    answer(core, 'd2')
    assert core.state == READY
    core.start()
    assert 'package' in links['d2'].kinds() and 'package' not in links['d1'].kinds()
    views = {drone['id']: drone['in_mission'] for drone in core.snapshot()['drones']}
    assert views == {'d1': False, 'd2': True}


def test_emptied_fields_mean_the_default_and_the_rest_go_to_the_drones(core, links):
    select(core, args={'record': 'true', 'path': ''})
    assert links['d1'].sent[-1]['launch']['args'] == {'record': 'true'}


def test_an_answer_to_an_older_check_is_ignored(core, links):
    select(core)
    old = core.check_id
    select(core, args={'record': 'true'})
    answer(core, 'd1', check_id=old)
    answer(core, 'd2', check_id=old)
    assert core.state == PREFLIGHT


def test_the_files_arguments_reach_the_phone_and_survive_a_recheck(core, links):
    select(core)
    answer(core, 'd2', launch_args=ARGUMENTS)
    answer(core, 'd1', launch_args=ARGUMENTS + [{'name': 'path', 'default': '/d1/x.yaml'}])
    # The first picked drone's view wins, since defaults that are paths can differ per drone.
    assert core.snapshot()['launch_args']['source'] == 'd1'
    select(core, args={'record': 'true'})
    assert core.snapshot()['launch_args']['source'] == 'd1'
    select(core, file='g_launch.py')
    assert core.snapshot()['launch_args'] is None


def test_the_run_id_is_named_after_the_file(core, links):
    to_ready(core)
    core.start()
    assert core.run_id.endswith('_f_launch')


def test_reset_keeps_the_pick_so_it_can_fly_again(core, links):
    to_searching(core)
    core.mission['args'] = {'record': 'true'}
    for _ in range(2):
        for drone_id in ('d1', 'd2'):
            status(core, drone_id, run_state='done', run_id=core.run_id, airborne=False, armed=False)
    assert core.state == COMPLETE
    assert core.reset()['ok']
    assert core.state == IDLE and core.run_id is None and core.checks == {}
    assert core.snapshot()['mission']['args'] == {'record': 'true'}


def test_reset_forgets_a_run_picked_up_after_a_restart(core, links):
    status(core, 'd1', run_state='done', run_id='r1', mission_id=MISSION_ID, airborne=False, armed=False)
    status(core, 'd2', run_state='done', run_id='r1', mission_id=MISSION_ID, airborne=False, armed=False)
    assert core.state == COMPLETE
    core.reset()
    assert core.mission is None


# --- SITL or HITL ------------------------------------------------------------------

from swarm_launcher.sitl_manager import FakeBackend, SitlManager  # noqa: E402


@pytest.fixture
def sitl_core(clock):
    saved = []
    core = HubCore(now=clock, mode='sitl', save_mode=saved.append, sitl=SitlManager(FakeBackend(), now=clock))
    core.saved = saved
    return core


def kind(core, drone_id, sim):
    core.agent_message(drone_id, {'type': 'catalog', 'folders': [], 'sim': sim})


def test_mode_is_in_every_check_and_package(core, links):
    to_ready(core)
    core.start()
    assert links['d1'].sent[-1]['type'] == 'package' and links['d1'].sent[-1]['mode'] == 'hitl'


def test_switching_mode_clears_the_pick_and_is_saved(sitl_core):
    links = {d: FakeLink() for d in ('d1', 'd2')}
    for drone_id, link in links.items():
        sitl_core.agent_connected(drone_id, link)
        kind(sitl_core, drone_id, True)
    select(sitl_core)
    assert sitl_core.state == PREFLIGHT
    assert sitl_core.set_mode('hitl')['ok']
    assert sitl_core.mode == 'hitl' and sitl_core.saved == ['hitl']
    assert sitl_core.state == IDLE and sitl_core.mission is None and sitl_core.checks == {}
    assert not sitl_core.set_mode('both')['ok']


def test_no_switching_mode_during_a_run(core, links):
    to_searching(core)
    result = core.set_mode('sitl')
    assert not result['ok'] and result['reason'] == 'cannot switch to SITL during a run'
    assert core.mode == 'hitl'


def test_drones_of_the_other_kind_cannot_be_picked(core, links):
    kind(core, 'd1', True)
    result = select(core)
    assert not result['ok'] and result['reason'] == 'd1: SITL drone. Switch to SITL to fly it'
    assert core.snapshot()['drones'][0]['sim'] is True
    kind(core, 'd1', False)
    assert select(core)['ok']


def test_sitl_drones_are_added_one_at_a_time(sitl_core, clock):
    assert sitl_core.sitl_add()['ok']
    assert sitl_core.snapshot()['sitl'] == {'available': True, 'drones': [
        {'drone_id': 'sim1', 'instance': 0, 'state': 'starting', 'detail': 'starting PX4'}]}
    assert sitl_core.sitl_add()['reason'] == 'wait for sim1 to finish starting'
    sitl_core.agent_connected('sim1', FakeLink())
    assert sitl_core._sitl_view()[0]['detail'] == 'waiting for a position'
    status(sitl_core, 'sim1', sim=True, position_ok=True)
    assert sitl_core._sitl_view()[0]['state'] == 'ready'
    assert sitl_core.sitl_add()['ok']
    clock.advance(hub_core.SITL_START_TIMEOUT_S + 1)
    assert sitl_core._sitl_view()[1]['state'] == 'stuck'


def test_sitl_drones_can_only_be_added_in_sitl(core):
    core.sitl = SitlManager(FakeBackend())
    assert core.sitl_add()['reason'] == 'switch to SITL to add SITL drones'
    core.mode, core.sitl = 'sitl', None
    assert core.sitl_add()['reason'] == 'this hub cannot start SITL drones'


def test_a_sitl_drone_in_the_air_or_in_a_run_stays(sitl_core):
    sitl_core.sitl_add()
    sitl_core.agent_connected('sim1', FakeLink())
    status(sitl_core, 'sim1', sim=True, position_ok=True, airborne=True)
    assert sitl_core.sitl_remove('sim1')['reason'] == 'sim1 still airborne'
    assert sitl_core.sitl_stop_all()['reason'] == 'sim1 still airborne'
    status(sitl_core, 'sim1', sim=True, position_ok=True, airborne=False)
    sitl_core.mission = {'drones': ['sim1']}
    sitl_core.state = SEARCHING
    assert 'a run is using it' in sitl_core.sitl_remove('sim1')['reason']
    sitl_core.state = IDLE
    assert sitl_core.sitl_remove('sim1')['ok']


def test_a_removed_sitl_drone_leaves_the_fleet_once_its_agent_goes(sitl_core):
    sitl_core.sitl_add()
    link = FakeLink()
    sitl_core.agent_connected('sim1', link)
    status(sitl_core, 'sim1', sim=True, position_ok=True)
    sitl_core.tick()
    sitl_core.sitl_remove('sim1')
    sitl_core.tick()
    assert [d['id'] for d in sitl_core.snapshot()['drones']] == ['sim1']  # Agent still connected.
    sitl_core.agent_disconnected('sim1', link)
    sitl_core.tick()
    assert sitl_core.snapshot()['drones'] == [] and sitl_core.snapshot()['sitl']['drones'] == []


def test_a_real_drone_that_drops_out_stays_in_the_fleet(sitl_core):
    link = FakeLink()
    sitl_core.agent_connected('drone1', link)
    sitl_core.agent_disconnected('drone1', link)
    sitl_core.tick()
    assert [d['id'] for d in sitl_core.snapshot()['drones']] == ['drone1']


def test_sitl_errors_reach_the_log(sitl_core):
    sitl_core.sitl_add()
    px4 = sitl_core.sitl.entries[0]['px4']
    px4.alive, px4.code = False, 1
    sitl_core.tick()
    assert any(line['source'] == 'sim1' and line['text'] == 'PX4 exited with code 1'
               for line in sitl_core.logs)


# --- Search area -------------------------------------------------------------------

from swarm_launcher.grid import fit_around  # noqa: E402

HOME = (47.397971057728974, 8.546163739800146)


def test_the_area_puts_each_drone_on_the_grid(clock):
    saved = []
    core = HubCore(now=clock, save_area=saved.append)
    core.agent_connected('d1', FakeLink())
    status(core, 'd1', **{'global': {'lat': HOME[0], 'lon': HOME[1]}})
    assert core.snapshot()['drones'][0]['grid'] is None
    assert core.set_area(fit_around(*HOME))['ok']
    view = core.snapshot()
    assert view['area']['flags'] == [] and set(view['area']['corners']) == {
        'south_west', 'south_east', 'north_east', 'north_west'}
    assert view['drones'][0]['grid'] == {'col': 3.28, 'row': 3.28, 'on_field': True}
    assert saved[-1] == view['area']['corners']
    assert core.set_area(None)['ok'] and core.snapshot()['area'] is None and saved[-1] is None


def test_a_bad_area_is_refused_and_the_old_one_kept(core):
    core.set_area(fit_around(*HOME))
    result = core.set_area({'south_west': {'lat': 1, 'lon': 2}})
    assert not result['ok'] and 'South-east corner is missing' in result['reason']
    assert core.snapshot()['area'] is not None


def test_the_area_cannot_change_during_a_run(core, links):
    to_searching(core)
    assert core.set_area(fit_around(*HOME))['reason'] == 'cannot change the search area during a run'


def test_a_saved_area_is_loaded_and_a_broken_one_ignored(clock):
    assert HubCore(now=clock, area=fit_around(*HOME)).area is not None
    assert HubCore(now=clock, area={'south_west': 'x'}).area is None


def test_fitting_the_area_is_for_sitl_drones_only(sitl_core, core):
    sitl_core.agent_connected('sim1', FakeLink())
    status(sitl_core, 'sim1', sim=True, **{'global': {'lat': HOME[0], 'lon': HOME[1]}})
    assert sitl_core.fit_area('sim1')['ok']
    assert sitl_core.snapshot()['drones'][0]['grid']['on_field']
    core.agent_connected('d1', FakeLink())
    status(core, 'd1', sim=False, **{'global': {'lat': HOME[0], 'lon': HOME[1]}})
    assert 'only for SITL drones' in core.fit_area('d1')['reason']
    status(sitl_core, 'sim1', sim=True, **{'global': None})
    assert sitl_core.fit_area('sim1')['reason'] == 'sim1 has no global position yet'


# --- Trails and colours ------------------------------------------------------------

from swarm_launcher.grid import local_to_global  # noqa: E402


def at(north, east):
    lat, lon = local_to_global(*HOME, north, east)
    return {'global': {'lat': lat, 'lon': lon}}


def flying_with_area(core):
    core.set_area(fit_around(*HOME, east_m=0, north_m=0))
    to_searching(core)


def test_a_trail_follows_each_picked_drone_through_its_run(core, links):
    status(core, 'd1', **at(0, 0))  # Before the run: not recorded.
    flying_with_area(core)
    for north in (0, 1, 1.1, 2, 3):
        status(core, 'd1', run_state='running', airborne=True, **at(north, 0))
    status(core, 'd2', run_state='running', airborne=True, **at(0, 5))
    # 1.1 m is within 0.3 m of the point before, so it adds nothing.
    assert len(core.trails['d1']) == 4 and len(core.trails['d2']) == 1
    message, version, counts = core.trail_update(None, {})
    assert message['full'] and version == core.trail_version
    assert [round(row) for _, row in message['points']['d1']] == [0, 2, 3, 5]
    assert counts == {'d1': 4, 'd2': 1}


def test_phones_get_only_new_points_after_the_first_send(core, links):
    flying_with_area(core)
    status(core, 'd1', run_state='running', airborne=True, **at(1, 1))
    _, version, counts = core.trail_update(None, {})
    assert core.trail_update(version, counts)[0] is None
    status(core, 'd1', run_state='running', airborne=True, **at(2, 1))
    message, version, counts = core.trail_update(version, counts)
    assert not message['full'] and len(message['points']['d1']) == 1 and counts['d1'] == 2


def test_trails_last_until_new_mission_and_restart_at_the_next_start(core, links):
    flying_with_area(core)
    status(core, 'd1', run_state='running', airborne=True, **at(1, 1))
    for _ in range(2):
        for drone_id in ('d1', 'd2'):
            status(core, drone_id, run_state='done', run_id=core.run_id, airborne=False, armed=False)
    assert core.state == COMPLETE and core.trails['d1']
    status(core, 'd1', run_state='done', airborne=False, **at(9, 9))  # After the run: not recorded.
    assert len(core.trails['d1']) == 1
    version = core.trail_version
    core.reset()
    assert core.trails == {} and core.trail_version > version
    message, _, _ = core.trail_update(version, {'d1': 1})
    assert message['full'] and message['points'] == {}


def test_drones_not_picked_leave_no_trail(core, links):
    core.agent_connected('d3', FakeLink())
    flying_with_area(core)
    status(core, 'd3', airborne=True, **at(1, 1))
    assert 'd3' not in core.trails


def test_without_an_area_there_are_no_points_to_send(core, links):
    to_searching(core)
    status(core, 'd1', run_state='running', airborne=True, **at(1, 1))
    message, version, counts = core.trail_update(None, {})
    assert message['points'] == {}
    assert core.trail_update(version, counts)[0] is None


def test_each_drone_keeps_its_own_colour(sitl_core):
    for drone_id in ('sim1', 'sim2', 'drone1'):
        sitl_core.agent_connected(drone_id, FakeLink())
    colours = {d['id']: d['color'] for d in sitl_core.snapshot()['drones']}
    assert colours == {'sim1': 0, 'sim2': 1, 'drone1': 2}
    # A drone leaving the fleet frees its colour for the next one.
    sitl_core._forget_drone('sim2')
    sitl_core.agent_connected('sim3', FakeLink())
    assert {d['id']: d['color'] for d in sitl_core.snapshot()['drones']}['sim3'] == 1
    for number in range(4, 9):
        sitl_core.agent_connected(f'sim{number}', FakeLink())
    # Past the palette, a drone is drawn in ink rather than reusing a colour.
    assert {d['id']: d['color'] for d in sitl_core.snapshot()['drones']}['sim8'] is None
