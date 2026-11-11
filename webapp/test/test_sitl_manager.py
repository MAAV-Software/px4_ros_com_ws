"""SITL drones from the Fleet tab: order, removal, crashes, and the processes they start."""
import json

import pytest

from swarm_launcher import sitl_manager
from swarm_launcher.sitl_manager import (
    FakeBackend, FakeHandle, ProcessBackend, SitlError, SitlManager, line_level)


@pytest.fixture
def backend():
    return FakeBackend()


@pytest.fixture
def manager(backend, clock):
    return SitlManager(backend, now=clock)


def test_drones_get_consecutive_instances_and_share_one_xrce_agent(manager, backend):
    assert manager.add() == 'sim1'
    assert manager.add() == 'sim2'
    assert backend.started == ['xrce', 'px4 0', 'agent sim1', 'px4 1', 'agent sim2']
    assert [(d['drone_id'], d['instance'], d['state']) for d in manager.view()] == [
        ('sim1', 0, 'running'), ('sim2', 1, 'running')]


def test_backend_problems_refuse_the_add(clock):
    manager = SitlManager(FakeBackend(problems=['no PX4 SITL build']), now=clock)
    with pytest.raises(SitlError, match='no PX4 SITL build'):
        manager.add()
    assert manager.view() == []


def test_there_is_a_limit(manager):
    for _ in range(sitl_manager.MAX_DRONES):
        manager.add()
    with pytest.raises(SitlError, match='at most'):
        manager.add()


def test_removal_is_newest_first_because_the_first_runs_the_world(manager):
    manager.add()
    manager.add()
    with pytest.raises(SitlError, match='remove sim2 first'):
        manager.remove('sim1')
    with pytest.raises(SitlError, match='no SITL drone sim9'):
        manager.remove('sim9')
    manager.remove('sim2')
    assert manager.view()[1]['state'] == 'stopping'
    manager.poll()
    assert [d['drone_id'] for d in manager.view()] == ['sim1']


def test_the_agent_stops_before_its_px4(manager):
    manager.add()
    entry = manager.entries[0]
    order = []
    entry['agent'].stop = lambda: order.append('agent')
    entry['px4'].stop = lambda: order.append('px4')
    manager.remove('sim1')
    assert order == ['agent', 'px4']


def test_no_add_while_one_is_still_stopping(manager):
    manager.add()
    manager.remove('sim1')
    manager.entries[0]['px4'].alive = True  # Still shutting down.
    with pytest.raises(SitlError, match='wait for the last SITL drone to stop'):
        manager.add()


def test_xrce_agent_stops_with_the_last_drone(manager):
    manager.add()
    xrce = manager.xrce
    manager.remove('sim1')
    manager.poll()
    assert not xrce.running()
    manager.poll()
    assert manager.xrce is None


def test_a_crashed_px4_is_reported_once_and_kept_until_removed(manager):
    manager.add()
    px4 = manager.entries[0]['px4']
    px4.alive, px4.code = False, -11
    lines = manager.poll()
    assert lines == [('sim1', 'ERROR', 'PX4 exited with code -11')]
    assert manager.poll() == []
    assert manager.view()[0] == {'drone_id': 'sim1', 'instance': 0, 'started_at': 1000.0,
                                 'state': 'exited', 'detail': 'PX4 exited with code -11'}
    manager.remove('sim1')
    manager.poll()
    assert manager.view() == []


def test_only_errors_and_warnings_reach_the_hub_log(manager):
    manager.add()
    manager.entries[0]['px4'].lines = [
        'INFO  [px4] startup script: /bin/sh etc/init.d-posix/rcS 0\n',
        '\x1b[1;31mERROR [gz_bridge] Service call timed out\x1b[0m\n',
        'WARN  [health_and_arming_checks] Preflight Fail: No valid data from Accel 0\n',
    ]
    assert manager.poll() == [
        ('sim1', 'ERROR', 'ERROR [gz_bridge] Service call timed out'),
        ('sim1', 'WARN', 'WARN  [health_and_arming_checks] Preflight Fail: No valid data from Accel 0'),
    ]


def test_stop_all_stops_every_drone_and_any_strays(manager, backend):
    manager.add()
    manager.add()
    manager.stop_all()
    assert all(d['state'] == 'stopping' for d in manager.view())
    assert backend.strays_stopped == 1
    manager.poll()
    assert manager.view() == []


@pytest.mark.parametrize('line, level', [
    ('ERROR [uxrce_dds_client] init failed', 'ERROR'),
    ('Traceback (most recent call last):', 'ERROR'),
    ('WARN  [commander] Disarmed by auto preflight disarming', 'WARN'),
    ('INFO  [commander] Ready for takeoff!', None),
])
def test_line_level(line, level):
    assert line_level(line) == level


# --- The real processes, without starting them -------------------------------------

class Recorder:
    started = []

    def __init__(self, argv, now=None, env=None, cwd=None):
        self.argv, self.env, self.cwd = argv, env, cwd
        self.process = type('P', (), {'pid': 4242})()

    def start(self):
        Recorder.started.append(self)


@pytest.fixture
def real(monkeypatch, tmp_path):
    Recorder.started = []
    monkeypatch.setattr(sitl_manager, 'LaunchProcess', Recorder)
    return ProcessBackend(tmp_path / 'PX4', 'ws://127.0.0.1:8080/agent', '~/ws', tmp_path / 'sitl.json')


def test_px4_instances_follow_milestone_2s_sitl_layout(real, tmp_path):
    real.start_px4(0)
    real.start_px4(2)
    first, third = Recorder.started
    build = (tmp_path / 'PX4').resolve() / 'build/px4_sitl_default'
    assert first.argv[1:] == [str(build / 'etc'), '-i', '0', '-d', '-w', str(build / 'rootfs/0')]
    assert first.env['PX4_GZ_STANDALONE'] == '' and first.env['PX4_UXRCE_DDS_NS'] == ''
    assert third.env['PX4_GZ_STANDALONE'] == '1' and third.env['PX4_UXRCE_DDS_NS'] == 'px4_2'
    assert third.env['PX4_GZ_MODEL_POSE'] == '40,0,0'
    assert third.env['HEADLESS'] == '1' and third.env['PX4_PARAM_NAV_DLL_ACT'] == '0'


def test_each_sitl_agent_is_told_who_it_is(real):
    real.start_agent(1, 'sim2')
    argv = Recorder.started[0].argv
    assert argv[1:] == ['-m', 'swarm_launcher.agent', '--hub', 'ws://127.0.0.1:8080/agent', '--drone-id', 'sim2',
                        '--topic-prefix', '/px4_1', '--system-id', '2', '--workspace',
                        argv[argv.index('--workspace') + 1], '--sim']
    assert not argv[argv.index('--workspace') + 1].startswith('~')


def test_started_processes_are_recorded_for_a_later_stop_all(real, tmp_path):
    real.start_xrce()
    records = json.loads((tmp_path / 'sitl.json').read_text())
    assert records == [{'pid': 4242, 'argv0': 'MicroXRCEAgent', 'arg': '8888'}]


def test_stop_all_only_signals_processes_that_are_still_ours(real, tmp_path, monkeypatch):
    (tmp_path / 'sitl.json').write_text(json.dumps([{'pid': 1, 'argv0': 'px4', 'arg': '/x'}]))
    signalled = []
    monkeypatch.setattr(ProcessBackend, '_signal', staticmethod(lambda pid, number: signalled.append(pid)))
    monkeypatch.setattr(ProcessBackend, '_is_ours', staticmethod(lambda record: False))
    real.stop_strays()
    assert signalled == []
    monkeypatch.setattr(ProcessBackend, '_is_ours', staticmethod(lambda record: True))
    real.stop_strays()
    assert signalled == [1]


def test_missing_px4_build_is_a_problem(real):
    problems = real.problems(first=True)
    assert any('no PX4 SITL build' in problem for problem in problems)


def test_fake_handle():
    handle = FakeHandle()
    assert handle.running() and handle.returncode is None
    handle.stop()
    assert not handle.running() and handle.returncode == 0


class Leftovers(FakeHandle):
    """A PX4 that exits when asked but leaves Gazebo running in its group, as the real one does."""

    def __init__(self):
        super().__init__()
        self.group = True
        self.killed = False

    def group_alive(self):
        return self.group

    def kill_group(self):
        self.killed = True
        self.group = False


def test_what_a_stopped_px4_leaves_behind_is_killed(manager, clock):
    manager.add()
    px4 = manager.entries[0]['px4'] = Leftovers()
    manager.remove('sim1')
    manager.poll()
    assert not px4.running() and not px4.killed
    assert [d['state'] for d in manager.view()] == ['stopping']  # Not forgotten while Gazebo lives.
    clock.advance(sitl_manager.LEFTOVER_KILL_AFTER_S + 1)
    manager.poll()
    assert px4.killed and manager.view() == []
