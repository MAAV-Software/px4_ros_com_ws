"""Four-vehicle launch and all-peer priority tests (no ROS transport/dynamics)."""
from itertools import permutations
from types import SimpleNamespace
import pytest
from milestone4_multivehicle.avoidance import Avoidance, POSITION_TOPIC
from test_multivehicle import api, make_controller, position, launch_context, gate_actions, PATH, ROOT


def fleet_context(**overrides):
    return launch_context(**{'vehicle_count': '4',
        'namespace_3': '/px4_2', 'namespace_4': '/px4_3',
        'system_id_3': '3', 'system_id_4': '4', 'path_3': PATH, 'path_4': PATH,
        'spawn_x_3': '20.0', 'spawn_y_3': '20.0',
        'spawn_x_4': '0.0', 'spawn_y_4': '20.0', **overrides})


@pytest.mark.parametrize('order', list(permutations(range(4))))
def test_all_four_ready_before_any_controller(api, order):
    ctx = fleet_context()
    _, handlers = gate_actions(api, ctx)
    assert len(handlers) == 4
    for i in order[:-1]:
        assert handlers[i].on_exit(SimpleNamespace(returncode=0), ctx) == []
    nodes = [a for a in handlers[order[-1]].on_exit(SimpleNamespace(returncode=0), ctx)
             if type(a).__name__ == 'Node']
    assert [n.parameters[0]['vehicle_instance'] for n in nodes] == list(range(4))
    assert [n.parameters[0]['target_system_id'] for n in nodes] == [1, 2, 3, 4]
    assert all(n.parameters[0]['vehicle_count'] == 4 for n in nodes)
    assert [(n.parameters[0]['initial_global_x'], n.parameters[0]['initial_global_y'])
            for n in nodes] == [(0, 0), (20, 0), (20, 20), (0, 20)]


@pytest.mark.parametrize('failed', range(4))
def test_any_readiness_failure_blocks_entire_fleet(api, failed):
    ctx = fleet_context()
    _, hs = gate_actions(api, ctx)
    result = hs[failed].on_exit(SimpleNamespace(returncode=1), ctx)
    assert any(type(a).__name__ == 'EmitEvent' for a in result)
    for i in range(4):
        assert hs[i].on_exit(SimpleNamespace(returncode=0), ctx) == []


@pytest.mark.parametrize('bad', [{'namespace_4': '/px4_1'}, {'system_id_3': '1'},
    {'path_4': '/does-not-exist'}, {'spawn_y_4': 'nan'}, {'vehicle_count': '5'}])
def test_invalid_last_vehicle_rejected_before_start(api, bad):
    with pytest.raises((ValueError, OSError)):
        gate_actions(api, fleet_context(**bad))


def test_four_sitl_processes_start_in_sequence(api, tmp_path, monkeypatch):
    launch = api('launch/px4_sitl.launch.py')
    binary = tmp_path / 'build/px4_sitl_default/bin/px4'
    binary.parent.mkdir(parents=True)
    binary.touch(); binary.chmod(0o755)
    monkeypatch.setattr(launch.subprocess, 'run', lambda *a, **k: SimpleNamespace(returncode=1))
    ctx = fleet_context(px4_dir=str(tmp_path))
    actions = launch.start_sitl(ctx)
    assert len([a for a in actions if type(a).__name__ == 'ExecuteProcess']) == 1
    nodes = [a for a in actions if type(a).__name__ == 'Node']
    assert [n.name for n in nodes] == ['m4_px4_0', 'm4_wait_world_0']
    processes = [nodes[0]]
    for i in range(3):
        h = actions[i].args[0]
        result = h.on_exit(SimpleNamespace(returncode=0), ctx)
        processes.append(result[0])
        assert result[0].name == f'm4_px4_{i + 1}'
        assert len(result) == (2 if i < 2 else 1)
        assert any(type(a).__name__ == 'EmitEvent' for a in
                   h.on_exit(SimpleNamespace(returncode=1), ctx))
    assert [p.additional_env['PX4_GZ_MODEL_POSE'] for p in processes] == [
        '0,0,0', '20,0,0', '20,20,0', '0,20,0']
    assert [p.additional_env['PX4_UXRCE_DDS_NS'] for p in processes] == ['', 'px4_1', 'px4_2', 'px4_3']
    assert all(p.additional_env['PX4_GZ_STANDALONE'] == '1' for p in processes[1:])


@pytest.mark.parametrize('own,peer', list(permutations(range(4), 2)))
def test_all_priority_pairs(own, peer):
    a = Avoidance(own, 0., 0.)
    for i in range(4):
        a.receive([i, 100., 100.], 0.)
    a.receive([peer, 3., 4.], 0.)
    assert a.conflicts(position()) == (peer > own)
    target = a.update(position(), 0.)
    assert (target is not None) == (peer > own)


def test_multiple_blockers_and_stale_peers_cannot_release_wait():
    a = Avoidance(0, 0., 0.)
    for i in (1, 2, 3): a.receive([i, 1., 0.], 0.)
    a.update(position(), 0.)
    a.update(position(), .5)
    a.update(position(z=-10), .6)
    for i in (2, 3): a.receive([i, 20., 0.], .7)
    assert a.update(position(z=-10), .7) is not None  # instance 1 still near
    a.receive([1, 20., 0.], 2.)
    assert a.update(position(z=-10), 2.) is not None  # other peers stale
    for i in (2, 3): a.receive([i, 20., 0.], 2.)
    assert a.update(position(z=-10), 2.) is None


def test_four_controllers_share_only_position_and_route_commands(api):
    nodes = [make_controller(api, i, vehicle_count=4) for i in range(4)]
    for node in nodes:
        node.on_local_position(position())
        node.publish_global_position()
    for i, node in enumerate(nodes):
        for peer in nodes: node.on_global_position(peer.global_position_pub.messages[-1])
        assert len(node.avoidance.peers) == 3
        node.on_timer(); node.arm()
        assert node.avoidance.state == 'NORMAL'
        assert node.vehicle_command_pub.messages[-1].target_system == i + 1
        assert node.global_position_pub.messages[-1].data[0] == i
    for a, b in permutations(nodes, 2):
        assert set(a.publishers) & set(b.publishers) == {POSITION_TOPIC}


def test_all_four_paths_and_origins_forwarded(api):
    paths = {f'path_{i + 1}': str(ROOT / f'resource/milestone4_headon_{i}.yaml') for i in range(4)}
    ctx = fleet_context(launch_sitl='true', **paths)
    actions, hs = gate_actions(api, ctx)
    include = next(a for a in actions if type(a).__name__ == 'IncludeLaunchDescription')
    args = dict(include.launch_arguments)
    assert args['vehicle_count'] == '4'
    for h in hs[:-1]: h.on_exit(SimpleNamespace(returncode=0), ctx)
    nodes = [a for a in hs[-1].on_exit(SimpleNamespace(returncode=0), ctx) if type(a).__name__ == 'Node']
    for i, n in enumerate(nodes, 1):
        assert n.parameters[0]['waypoints_file'] == paths[f'path_{i}']
        for axis in ('x', 'y'):
            assert n.parameters[0][f'initial_global_{axis}'] == float(args[f'spawn_{axis}_{i}'])


@pytest.mark.parametrize('instance,count', [(4,4), (-1,4), (1.5,4), (0,1), (0,5)])
def test_invalid_fleet_identity(instance,count):
    with pytest.raises(ValueError): Avoidance(instance, 0., 0., count)
